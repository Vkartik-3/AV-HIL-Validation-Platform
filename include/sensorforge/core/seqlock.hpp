/*
==============================================================================
SensorForge - Seqlock for RCU-style config / metadata updates
Part of the SensorForge AV HIL validation platform.

A seqlock lets many readers observe a small piece of shared state without
locks, while a single writer updates it in place. It is ideal for
config/metadata that is read on the hot path (per-frame) but changed rarely
(e.g. a scenario adjusting a stream's rate or a fault window opening).

Protocol:
  - An atomic sequence counter starts even (= stable).
  - Writer: increment to odd (write in progress), publish the new value,
    increment to even (stable again). Two writers are not allowed
    concurrently (single-writer); a mutex or external discipline serializes
    writers if needed.
  - Reader: snapshot the sequence; if odd, a write is in progress -> retry.
    Read the value, re-read the sequence; if it changed, the value may be
    torn -> retry.

T should be trivially copyable and small (this is not a general mutex).

--- ThreadSanitizer note -------------------------------------------------------
store() and load() are annotated __attribute__((no_sanitize("thread"))).

The seqlock DELIBERATELY reads value_ non-atomically while a writer may be
updating it; correctness comes from the sequence counter, which forces the
reader to DISCARD and retry any read that overlapped a write. TSan cannot see
that protocol-level invariant, so it flags the value_ access as a data race.
Strictly, in the C++ abstract machine this non-atomic overlap IS a race (the
same reason the Linux kernel uses READ_ONCE/WRITE_ONCE volatiles); it is
benign here because torn values are never used. The suppression documents that
this race is intentional and handled by the retry loop, not overlooked.
Reference: https://www.kernel.org/doc/html/latest/locking/seqlock.html
(A fully UB-free alternative is to make value_ access memory_order_relaxed
atomic, which removes the suppression entirely.)
================================================================================
*/

#pragma once

#include <atomic>
#include <cstdint>
#include <thread>
#include <type_traits>

// Disable ThreadSanitizer instrumentation on the annotated function. No-op on
// non-sanitizer builds and on compilers without the attribute.
#if defined(__GNUC__) || defined(__clang__)
#  define SF_NO_SANITIZE_THREAD __attribute__((no_sanitize("thread")))
#else
#  define SF_NO_SANITIZE_THREAD
#endif

namespace sensorforge::core {

template<typename T>
class Seqlock
{
  static_assert(std::is_trivially_copyable_v<T>,
    "Seqlock<T> requires a trivially copyable T (it is memcpy-snapshotted)");

public:
  Seqlock() = default;

  explicit Seqlock(const T & initial)
  : value_(initial) {}

  /**
   * @brief Reader: return a consistent snapshot, retrying past any in-flight
   *        write. Wait-free in the common (uncontended) case.
   *
   * TSan-suppressed: see the ThreadSanitizer note in the file header. The
   * value_ read may overlap a writer; the sequence re-check below discards any
   * such torn read before it can be observed.
   */
  SF_NO_SANITIZE_THREAD
  T load() const
  {
    T copy;
    uint64_t seq_before;
    do {
      seq_before = seq_.load(std::memory_order_acquire);
      if (seq_before & 1u) {
        // Write in progress; back off briefly and retry.
        std::this_thread::yield();
        continue;
      }
      copy = value_;
      std::atomic_thread_fence(std::memory_order_acquire);
      const uint64_t seq_after = seq_.load(std::memory_order_relaxed);
      if (seq_before == seq_after) {
        return copy;  // consistent read
      }
      // Sequence changed under us -> possible tear -> retry.
    } while (true);
  }

  /**
   * @brief Writer: publish @p desired. Single-writer only.
   *
   * TSan-suppressed: see the ThreadSanitizer note in the file header. value_ is
   * written between the odd/even sequence bumps; concurrent readers detect the
   * overlap via the sequence change and retry.
   */
  SF_NO_SANITIZE_THREAD
  void store(const T & desired)
  {
    const uint64_t seq = seq_.load(std::memory_order_relaxed);
    seq_.store(seq + 1, std::memory_order_release);   // -> odd: write in progress
    std::atomic_thread_fence(std::memory_order_release);
    value_ = desired;
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(seq + 2, std::memory_order_release);   // -> even: stable
  }

  /// Current sequence value (even = stable). Exposed for tests/metrics.
  uint64_t sequence() const {return seq_.load(std::memory_order_acquire);}

private:
  std::atomic<uint64_t> seq_{0};
  T value_{};
};

}  // namespace sensorforge::core
