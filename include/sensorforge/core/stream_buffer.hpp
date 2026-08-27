/*
==============================================================================
SensorForge - Per-stream bounded buffer with byte accounting
Part of the SensorForge AV HIL validation platform.

The audit found that the ring bounded MESSAGE COUNT but not BYTES: 1024 slots
holding SensorFrames whose payloads are heap vectors means a full camera or
LiDAR ring is hundreds of megabytes, and at the 4 MiB frame ceiling it is 4 GB.
The buffer was bounded in the one dimension that does not protect the process.

StreamBuffer wraps SPSCRing and adds the three things the bridge actually needs:

  1. BYTE accounting. queued_bytes() is maintained on every push and pop, and a
     runtime max_bytes limit is enforced alongside the frame limit. Whichever
     limit binds first triggers the stream's backpressure policy.
  2. RUNTIME limits. The ring's slot count is still a compile-time power of two
     (that is what makes the index masking cheap), but max_frames and max_bytes
     are runtime values, so per-stream configuration is real configuration
     rather than a recompile.
  3. COUNTERS. enqueued / dropped / overwritten / bytes / peak occupancy, all
     readable without disturbing the producer.

Concurrency contract: one producer thread calls push(); one consumer thread
calls pop(). Counters are atomics with relaxed ordering -- they are metrics,
not synchronisation. queued_bytes_ is an atomic because both sides adjust it.
==============================================================================
*/

#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <utility>

#include "sensorforge/core/backpressure_policy.hpp"
#include "sensorforge/core/sensor_frame.hpp"
#include "sensorforge/core/spsc_ring.hpp"

namespace sensorforge::core {

/// Runtime, per-stream buffer limits.
struct StreamLimits
{
  size_t max_frames = 512;                 // <= SPSCRing usable capacity
  size_t max_bytes = 64u * 1024u * 1024u;  // 64 MiB per stream by default
  std::chrono::microseconds block_budget{2000};
};

struct StreamCounters
{
  uint64_t enqueued = 0;
  uint64_t dropped = 0;
  uint64_t overwritten = 0;
  uint64_t blocked = 0;
  uint64_t bytes_enqueued = 0;
  uint64_t bytes_dropped = 0;
  size_t queued_frames = 0;
  size_t queued_bytes = 0;
  size_t peak_queued_frames = 0;
  size_t peak_queued_bytes = 0;
};

template<size_t Capacity = 1024>
class StreamBufferT
{
public:
  static constexpr size_t kSlotCapacity = Capacity;

  StreamBufferT() = default;

  explicit StreamBufferT(BackpressurePolicy policy, StreamLimits limits = {})
  : policy_(policy), limits_(clamp(limits)) {}

  void configure(BackpressurePolicy policy, const StreamLimits & limits)
  {
    policy_ = policy;
    limits_ = clamp(limits);
  }

  BackpressurePolicy policy() const {return policy_;}
  const StreamLimits & limits() const {return limits_;}

  /**
   * @brief Producer: enqueue @p frame under this stream's policy and limits.
   *
   * The frame is moved into the ring on success. On a drop the frame is left
   * untouched so the caller can inspect its size for accounting.
   */
  PushResult push(SensorFrame && frame)
  {
    const size_t sz = frame.byte_size();

    // Byte and frame ceilings are checked before the ring so that a stream
    // configured well below the slot capacity behaves exactly as configured.
    if (at_limit(sz)) {
      switch (policy_) {
        case BackpressurePolicy::kDropNewest:
          return record(PushResult::kDroppedNewest, sz);

        case BackpressurePolicy::kOverwriteOldest: {
          // Evict oldest until the incoming frame fits, bounded.
          for (int i = 0; i < 8 && at_limit(sz); ++i) {
            SensorFrame victim;
            if (!ring_.try_pop(victim)) {
              break;
            }
            release_bytes(victim.byte_size());
            ++overwritten_;
          }
          if (at_limit(sz)) {
            return record(PushResult::kDroppedNewest, sz);
          }
          break;
        }

        case BackpressurePolicy::kNeverDropBlock: {
          const auto deadline = std::chrono::steady_clock::now() + limits_.block_budget;
          while (at_limit(sz) && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
          }
          if (at_limit(sz)) {
            ++blocked_;
            return record(PushResult::kDroppedAfterBlock, sz);
          }
          ++blocked_;
          break;
        }
      }
    }

    // Eviction (if any) already happened above, with exact byte accounting, so
    // the ring only ever needs a plain push here. Note that the eviction loop
    // calls ring_.try_pop() from the PRODUCER thread: that is sound under the
    // CAS-claimed tail, which is precisely what the ring fix buys -- two
    // callers of try_pop each claim a distinct slot, and the two-slot reserve
    // keeps the producer from writing a slot still being copied out.
    if (!ring_.try_push(std::move(frame))) {
      return record(PushResult::kDroppedNewest, sz);
    }
    reserve_bytes(sz);
    ++enqueued_;
    bytes_enqueued_ += sz;
    return PushResult::kEnqueued;
  }

  /// Consumer: dequeue the oldest frame. Returns false when empty.
  bool pop(SensorFrame & out)
  {
    if (!ring_.try_pop(out)) {
      return false;
    }
    release_bytes(out.byte_size());
    return true;
  }

  size_t queued_frames() const {return ring_.size_approx();}
  size_t queued_bytes() const {return queued_bytes_.load(std::memory_order_relaxed);}
  bool empty() const {return ring_.empty_approx();}

  StreamCounters counters() const
  {
    StreamCounters c;
    c.enqueued = enqueued_.load(std::memory_order_relaxed);
    c.dropped = dropped_.load(std::memory_order_relaxed);
    c.overwritten = overwritten_.load(std::memory_order_relaxed);
    c.blocked = blocked_.load(std::memory_order_relaxed);
    c.bytes_enqueued = bytes_enqueued_.load(std::memory_order_relaxed);
    c.bytes_dropped = bytes_dropped_.load(std::memory_order_relaxed);
    c.queued_frames = ring_.size_approx();
    c.queued_bytes = queued_bytes_.load(std::memory_order_relaxed);
    c.peak_queued_frames = peak_frames_.load(std::memory_order_relaxed);
    c.peak_queued_bytes = peak_bytes_.load(std::memory_order_relaxed);
    return c;
  }

private:
  static StreamLimits clamp(StreamLimits l)
  {
    const size_t usable = SPSCRing<SensorFrame, Capacity>::capacity();
    if (l.max_frames == 0 || l.max_frames > usable) {
      l.max_frames = usable;
    }
    if (l.max_bytes == 0) {
      l.max_bytes = 64u * 1024u * 1024u;
    }
    return l;
  }

  bool at_limit(size_t incoming) const
  {
    if (ring_.size_approx() >= limits_.max_frames) {
      return true;
    }
    return queued_bytes_.load(std::memory_order_relaxed) + incoming > limits_.max_bytes;
  }

  PushResult record(PushResult r, size_t sz)
  {
    ++dropped_;
    bytes_dropped_ += sz;
    return r;
  }

  void reserve_bytes(size_t n)
  {
    const size_t now = queued_bytes_.fetch_add(n, std::memory_order_relaxed) + n;
    bump_peak(peak_bytes_, now);
    bump_peak(peak_frames_, ring_.size_approx());
  }

  void release_bytes(size_t n)
  {
    // fetch_sub is safe even if a concurrent push raced the read above; the
    // value is a metric, and pushes/pops are each single-threaded.
    queued_bytes_.fetch_sub(n, std::memory_order_relaxed);
  }

  static void bump_peak(std::atomic<size_t> & peak, size_t candidate)
  {
    size_t seen = peak.load(std::memory_order_relaxed);
    while (candidate > seen &&
      !peak.compare_exchange_weak(seen, candidate, std::memory_order_relaxed))
    {
    }
  }

  SPSCRing<SensorFrame, Capacity> ring_;
  BackpressurePolicy policy_ = BackpressurePolicy::kDropNewest;
  StreamLimits limits_{};

  std::atomic<uint64_t> enqueued_{0};
  std::atomic<uint64_t> dropped_{0};
  std::atomic<uint64_t> overwritten_{0};
  std::atomic<uint64_t> blocked_{0};
  std::atomic<uint64_t> bytes_enqueued_{0};
  std::atomic<uint64_t> bytes_dropped_{0};
  std::atomic<size_t> queued_bytes_{0};
  std::atomic<size_t> peak_frames_{0};
  std::atomic<size_t> peak_bytes_{0};
};

using StreamBuffer = StreamBufferT<1024>;

}  // namespace sensorforge::core
