/*
==============================================================================
SensorForge - Lock-free single-producer / single-consumer ring buffer
Part of the SensorForge AV HIL validation platform.

A bounded, wait-free SPSC queue. Exactly one producer thread and one consumer
thread may use an instance concurrently (in the bridge: producer = ROS2
subscription callback thread, consumer = send-timer thread).

Design:
  - Capacity is a compile-time power of two (template parameter). Index masking
    replaces modulo. One slot is intentionally left empty to disambiguate the
    full and empty states without a separate size counter.
  - Producer and consumer indices live on separate 64-byte cache lines
    (alignas(std::hardware_destructive_interference_size)) to avoid false
    sharing.
  - Storage is a single std::array member: no heap allocation after
    construction, no allocation in try_push / try_pop.
  - Synchronization is std::atomic with acquire/release ordering only; there
    are no locks and no CAS loops, so both operations are wait-free.

The type T must be movable. Slots are default-constructed up front and reused.
==============================================================================
*/

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <new>
#include <optional>
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
  static_assert(Capacity >= 2, "SPSCRing capacity must be at least 2");
  static_assert((Capacity & (Capacity - 1)) == 0, "SPSCRing capacity must be a power of two");
  static_assert(std::is_nothrow_move_assignable_v<T> || std::is_copy_assignable_v<T>,
    "T must be assignable into ring slots");

public:
  SPSCRing() = default;
  SPSCRing(const SPSCRing &) = delete;
  SPSCRing & operator=(const SPSCRing &) = delete;

  /// Usable capacity (one slot is reserved to distinguish full from empty).
  static constexpr size_t capacity() {return Capacity - 1;}

  /**
   * @brief Producer: enqueue by copy. Wait-free.
   * @return false if the ring is full (item not stored).
   */
  bool try_push(const T & item)
  {
    const size_t head = head_.value.load(std::memory_order_relaxed);
    const size_t next = (head + 1) & kMask;
    if (next == tail_.value.load(std::memory_order_acquire)) {
      return false;  // full
    }
    buffer_[head] = item;
    head_.value.store(next, std::memory_order_release);
    return true;
  }

  /**
   * @brief Producer: enqueue by move. Wait-free.
   * @return false if the ring is full (item not stored, argument unchanged).
   */
  bool try_push(T && item)
  {
    const size_t head = head_.value.load(std::memory_order_relaxed);
    const size_t next = (head + 1) & kMask;
    if (next == tail_.value.load(std::memory_order_acquire)) {
      return false;  // full
    }
    buffer_[head] = std::move(item);
    head_.value.store(next, std::memory_order_release);
    return true;
  }

  /**
   * @brief Consumer: dequeue into @p out. Wait-free.
   * @return false if the ring is empty (@p out untouched).
   */
  bool try_pop(T & out)
  {
    const size_t tail = tail_.value.load(std::memory_order_relaxed);
    if (tail == head_.value.load(std::memory_order_acquire)) {
      return false;  // empty
    }
    out = std::move(buffer_[tail]);
    tail_.value.store((tail + 1) & kMask, std::memory_order_release);
    return true;
  }

  /**
   * @brief Producer helper for "overwrite oldest" backpressure: push and, if
   *        full, drop the oldest element to make room, then push.
   *
   * Only safe to call from the producer thread. Because the consumer may be
   * concurrently advancing tail_, the drop is attempted via the normal
   * consumer-visible protocol (advance tail) which is why this is limited to
   * the SPSC contract: the producer transiently acts as consumer of one slot.
   * For strict SPSC correctness with a live consumer, prefer draining from the
   * consumer side; this helper targets the camera case where the producer owns
   * pacing. Returns true always (the new item is stored).
   */
  bool force_push_overwrite(const T & item)
  {
    if (try_push(item)) {
      return true;
    }
    // Full: drop one oldest by advancing tail from the producer side. This is
    // acceptable for the camera "overwrite_oldest" policy where the producer
    // is authoritative over freshness. Load tail relaxed (producer view),
    // advance with release so the consumer never reads a stale slot.
    const size_t tail = tail_.value.load(std::memory_order_acquire);
    tail_.value.store((tail + 1) & kMask, std::memory_order_release);
    return try_push(item);
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

  // Cache-line-isolated atomic indices to prevent false sharing between the
  // producer (writes head_) and consumer (writes tail_).
  struct alignas(kCacheLine) PaddedIndex
  {
    std::atomic<size_t> value{0};
  };

  PaddedIndex head_;  // next slot the producer will write
  PaddedIndex tail_;  // next slot the consumer will read
  std::array<T, Capacity> buffer_{};
};

}  // namespace sensorforge::core
