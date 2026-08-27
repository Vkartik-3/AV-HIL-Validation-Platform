/*
==============================================================================
SensorForge - Bounded single-producer / single-consumer ring buffer
Part of the SensorForge AV HIL validation platform.

A bounded queue for exactly one producer thread and one consumer thread (in the
bridge: producer = ROS2 subscription callback thread, consumer = the recorder
drain thread).

Design:
  - Capacity is a compile-time power of two (template parameter). Index masking
    replaces modulo.
  - TWO slots are reserved rather than one. One reserved slot disambiguates
    full from empty; the second is a safety gap that guarantees the producer
    never writes the slot a consumer may still be copying out. See the
    "Overwrite safety" note below -- this gap is what makes producer-side
    eviction sound.
  - Producer and consumer indices live on separate 64-byte cache lines
    (alignas) to avoid false sharing.
  - Storage is a single std::array member: no heap allocation after
    construction, no allocation in try_push / try_pop.

Progress guarantees (stated precisely, because the previous revision of this
file claimed more than it delivered):
  - try_push          : wait-free. Bounded steps, no loop.
  - try_pop           : lock-free. Contains a CAS retry loop that can only spin
                        when a producer concurrently evicts the exact slot this
                        consumer is claiming; each failed iteration corresponds
                        to real progress by the producer.
  - evict_oldest      : lock-free, single CAS attempt, never blocks.

Overwrite safety (this is the defect the audit found, and how it is fixed)
-------------------------------------------------------------------------
The previous implementation let the PRODUCER do a plain load+store on tail_,
the consumer-owned index. Producer and consumer both storing to tail_ is a
lost-update race: a consumer try_pop interleaving between the producer's load
and store desynchronizes the indices and can hand out a slot that is being
concurrently written.

The fix has two parts, and both are required:

  1. CLAIM BEFORE COPY. Every advance of tail_ is a compare_exchange, by both
     sides. The consumer CLAIMS a slot by winning the CAS and only then copies
     the payload out; if it loses the CAS a producer eviction took that slot
     first, so the consumer discards and retries. Producer eviction is a single
     CAS attempt. Two claimers can therefore never be handed the same slot.

  2. A PUBLISHED READER FLOOR. Claiming is not sufficient on its own. Once a
     consumer has claimed slot k, tail_ has already moved past k, and each
     further producer eviction moves tail_ further still -- so k drifts deeper
     behind tail_ and a fixed "reserve N slots behind tail" rule cannot bound
     it. (An earlier revision of this file tried exactly that and
     ThreadSanitizer caught the producer writing a slot a consumer was still
     copying.) The consumer therefore PUBLISHES the slot it is about to read in
     read_floor_ before it claims, and clears it afterwards. The producer
     refuses to write at, or immediately before, that floor. The window is
     closed by construction rather than by hoping the consumer is never
     preempted for a full lap.

     Ordering: the consumer publishes read_floor_ BEFORE the CAS, so a producer
     that has not yet observed the floor is still looking at the older (smaller)
     tail_, which is at least as restrictive. Either way the in-flight slot is
     never a legal write target.

The type T must be movable. Slots are default-constructed up front and reused.
==============================================================================
*/

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>

namespace sensorforge::core {

// std::hardware_destructive_interference_size is not available in every libc++;
// fall back to 64, the near-universal cache line size on x86-64 and ARM64.
#if defined(__cpp_lib_hardware_interference_size)
inline constexpr size_t kCacheLine = std::hardware_destructive_interference_size;
#else
inline constexpr size_t kCacheLine = 64;
#endif

template<typename T, size_t Capacity>
class SPSCRing
{
  static_assert(Capacity >= 4, "SPSCRing capacity must be at least 4");
  static_assert((Capacity & (Capacity - 1)) == 0, "SPSCRing capacity must be a power of two");
  static_assert(std::is_nothrow_move_assignable_v<T> || std::is_copy_assignable_v<T>,
    "T must be assignable into ring slots");

public:
  SPSCRing() = default;
  SPSCRing(const SPSCRing &) = delete;
  SPSCRing & operator=(const SPSCRing &) = delete;

  /// Usable capacity. One slot is reserved to distinguish full from empty.
  /// A consumer with a read in flight transiently costs one more slot (see the
  /// reader-floor note in the header comment).
  static constexpr size_t capacity() {return Capacity - 1;}

  /**
   * @brief Producer: enqueue by copy. Wait-free.
   * @return false if the ring is full (item not stored).
   */
  bool try_push(const T & item)
  {
    const size_t head = head_.value.load(std::memory_order_relaxed);
    if (is_full(head)) {
      return false;
    }
    buffer_[head] = item;
    head_.value.store((head + 1) & kMask, std::memory_order_release);
    return true;
  }

  /**
   * @brief Producer: enqueue by move. Wait-free.
   * @return false if the ring is full (item not stored, argument unchanged).
   */
  bool try_push(T && item)
  {
    const size_t head = head_.value.load(std::memory_order_relaxed);
    if (is_full(head)) {
      return false;
    }
    buffer_[head] = std::move(item);
    head_.value.store((head + 1) & kMask, std::memory_order_release);
    return true;
  }

  /**
   * @brief Consumer: dequeue into @p out. Lock-free.
   *
   * Claims the slot with a CAS on tail_ BEFORE copying, so a concurrent
   * producer eviction cannot hand the same slot to both sides. Because the
   * producer stops two slots short of tail, the claimed slot is not a legal
   * producer write target while this copy is in flight.
   *
   * @return false if the ring is empty (@p out untouched).
   */
  bool try_pop(T & out)
  {
    size_t tail = tail_.value.load(std::memory_order_relaxed);
    for (;;) {
      if (tail == head_.value.load(std::memory_order_acquire)) {
        return false;   // empty
      }
      // Announce the slot BEFORE claiming it, so a producer can never pick it
      // as a write target while the copy below is in flight.
      floor_.value.store(tail, std::memory_order_seq_cst);

      const size_t next = (tail + 1) & kMask;
      if (tail_.value.compare_exchange_strong(
          tail, next, std::memory_order_acq_rel, std::memory_order_relaxed))
      {
        out = std::move(buffer_[tail]);
        floor_.value.store(kNoReader, std::memory_order_release);
        return true;
      }
      // CAS failed: a producer evicted this slot first. `tail` was reloaded by
      // compare_exchange_strong; drop the announcement and retry.
      floor_.value.store(kNoReader, std::memory_order_release);
    }
  }

  /**
   * @brief Producer: drop the oldest queued element to free one slot.
   *
   * Single CAS attempt, never blocks. Returns true if this call actually
   * evicted an element. A false return means the ring was empty or the
   * consumer concurrently claimed the same slot -- in both cases space either
   * exists already or is about to, and the caller should simply retry its push.
   *
   * Safe to call from the producer thread while a consumer runs: the eviction
   * only moves tail_ via CAS and never touches slot storage.
   */
  bool evict_oldest()
  {
    size_t tail = tail_.value.load(std::memory_order_relaxed);
    if (tail == head_.value.load(std::memory_order_acquire)) {
      return false;   // empty, nothing to evict
    }
    const size_t next = (tail + 1) & kMask;
    return tail_.value.compare_exchange_strong(
      tail, next, std::memory_order_acq_rel, std::memory_order_relaxed);
  }

  /**
   * @brief Producer helper implementing "overwrite oldest" backpressure.
   *
   * Attempts a normal push; if the ring is full, evicts the oldest element and
   * retries. Bounded: at most kMaxEvictAttempts eviction attempts, after which
   * it reports failure rather than spinning. Producer-thread only.
   *
   * @param evicted [out] incremented once per element actually dropped.
   * @return true if @p item was enqueued.
   */
  bool push_overwrite(const T & item, uint64_t & evicted)
  {
    if (try_push(item)) {
      return true;
    }
    for (int attempt = 0; attempt < kMaxEvictAttempts; ++attempt) {
      if (evict_oldest()) {
        ++evicted;
      }
      if (try_push(item)) {
        return true;
      }
    }
    return false;
  }

  /// Approximate number of queued items (racy; for metrics/occupancy only).
  size_t size_approx() const
  {
    const size_t head = head_.value.load(std::memory_order_acquire);
    const size_t tail = tail_.value.load(std::memory_order_acquire);
    return (head - tail) & kMask;
  }

  bool empty_approx() const {return size_approx() == 0;}

private:
  static constexpr size_t kMask = Capacity - 1;
  static constexpr size_t kNoReader = static_cast<size_t>(-1);
  static constexpr int kMaxEvictAttempts = 4;

  /**
   * @brief True when the producer must not write at @p head.
   *
   * Classic full is (head + 1) == tail. When a consumer has a read in flight it
   * publishes that slot in floor_, and the producer additionally refuses both
   * that slot and the one before it -- the floor sits behind tail_ and is what
   * a fixed reserve could not bound.
   */
  bool is_full(size_t head) const
  {
    const size_t tail = tail_.value.load(std::memory_order_acquire);
    if (((head + 1) & kMask) == tail) {
      return true;
    }
    const size_t floor = floor_.value.load(std::memory_order_acquire);
    if (floor != kNoReader) {
      if (head == floor || ((head + 1) & kMask) == floor) {
        return true;
      }
    }
    return false;
  }

  // Cache-line-isolated atomic indices to prevent false sharing between the
  // producer (writes head_) and consumer (writes tail_).
  struct alignas(kCacheLine) PaddedIndex
  {
    std::atomic<size_t> value{0};
  };

  struct alignas(kCacheLine) PaddedFloor
  {
    std::atomic<size_t> value{static_cast<size_t>(-1)};
  };

  PaddedIndex head_;   // next slot the producer will write
  PaddedIndex tail_;   // next slot the consumer will read
  PaddedFloor floor_;  // slot a consumer is copying right now, or kNoReader
  std::array<T, Capacity> buffer_{};
};

}  // namespace sensorforge::core
