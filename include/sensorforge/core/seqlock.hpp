/*
==============================================================================
SensorForge - Seqlock for RCU-style config / metadata updates
Part of the SensorForge AV HIL validation platform.

A seqlock lets many readers observe a small piece of shared state without
locks, while a single writer updates it in place. It is ideal for
config/metadata that is read on the hot path (per-frame) but changed rarely
(e.g. a scenario adjusting a stream's rate or a fault window opening).

Seqlock implementation using relaxed atomics for data fields to avoid UB in
the C++ abstract machine while maintaining seqlock semantics. Sequence counter
uses acquire/release for synchronization. Data fields use relaxed atomics --
torn reads are safe because load() retries if seq changes.

Because T is an arbitrary trivially-copyable type, the payload is stored as a
byte array of std::atomic<unsigned char> and copied in/out with per-byte
memory_order_relaxed operations. Every access to shared memory is therefore an
atomic access: there is no data race in the C++ memory model, so the code is
UB-free and ThreadSanitizer-clean with ZERO suppressions. The relaxed ordering
carries no synchronization by itself; all happens-before ordering comes from
the acquire/release sequence counter, exactly as in a classic seqlock.

Protocol:
  - An atomic sequence counter starts even (= stable).
  - Writer: bump to odd (write in progress), publish the new bytes, bump to
    even (stable again). Single-writer only; serialize writers externally if
    there can be more than one.
  - Reader: snapshot the sequence; if odd, a write is in progress -> retry.
    Read the bytes, re-read the sequence; if it changed, the snapshot may be
    torn -> retry.

Reference: https://www.kernel.org/doc/html/latest/locking/seqlock.html

T should be trivially copyable and small (this is not a general mutex).
==============================================================================
*/

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <thread>
#include <type_traits>

namespace sensorforge::core {

template<typename T>
class Seqlock
{
  static_assert(std::is_trivially_copyable_v<T>,
    "Seqlock<T> requires a trivially copyable T (it is byte-copied)");

public:
  Seqlock()
  {
    write_relaxed(T{});
  }

  explicit Seqlock(const T & initial)
  {
    // Constructor runs before any reader exists: seq_ stays 0 (even/stable).
    write_relaxed(initial);
  }

  Seqlock(const Seqlock &) = delete;
  Seqlock & operator=(const Seqlock &) = delete;

  /**
   * @brief Reader: return a consistent snapshot, retrying past any in-flight
   *        write. Wait-free in the common (uncontended) case.
   *
   * All byte reads are memory_order_relaxed atomics, so no access is a data
   * race even when they overlap a writer. The acquire load of seq0 synchronizes
   * with the writer's release publish, and the acquire fence prevents the data
   * loads from being reordered after the seq1 re-check; if the sequence changed
   * (or was odd), the snapshot is discarded and retried.
   */
  T load() const
  {
    T out;
    uint64_t seq0;
    uint64_t seq1;
    do {
      seq0 = seq_.load(std::memory_order_acquire);
      if (seq0 & 1u) {
        // Write in progress; back off briefly and retry.
        std::this_thread::yield();
        continue;
      }
      read_relaxed(out);
      std::atomic_thread_fence(std::memory_order_acquire);
      seq1 = seq_.load(std::memory_order_relaxed);
    } while ((seq0 & 1u) || seq0 != seq1);
    return out;
  }

  /**
   * @brief Writer: publish @p desired. Single-writer only.
   *
   * Bumps seq_ to odd, writes the bytes with relaxed atomics between two
   * release fences, then bumps seq_ to even with a release store so readers'
   * acquire loads observe the completed update.
   */
  void store(const T & desired)
  {
    const uint64_t seq = seq_.load(std::memory_order_relaxed);
    seq_.store(seq + 1, std::memory_order_relaxed);   // -> odd: write in progress
    std::atomic_thread_fence(std::memory_order_release);
    write_relaxed(desired);
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(seq + 2, std::memory_order_release);    // -> even: stable/published
  }

  /// Current sequence value (even = stable). Exposed for tests/metrics.
  uint64_t sequence() const {return seq_.load(std::memory_order_acquire);}

private:
  void write_relaxed(const T & v)
  {
    unsigned char tmp[sizeof(T)];
    std::memcpy(tmp, &v, sizeof(T));
    for (size_t i = 0; i < sizeof(T); ++i) {
      data_[i].store(tmp[i], std::memory_order_relaxed);
    }
  }

  void read_relaxed(T & out) const
  {
    unsigned char tmp[sizeof(T)];
    for (size_t i = 0; i < sizeof(T); ++i) {
      tmp[i] = data_[i].load(std::memory_order_relaxed);
    }
    std::memcpy(&out, tmp, sizeof(T));
  }

  std::atomic<uint64_t> seq_{0};
  std::array<std::atomic<unsigned char>, sizeof(T)> data_{};
};

}  // namespace sensorforge::core
