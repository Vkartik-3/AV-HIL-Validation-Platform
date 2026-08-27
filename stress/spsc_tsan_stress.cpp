/*
==============================================================================
SensorForge - SPSC ring / seqlock concurrency stress test (Extension A)
Part of the SensorForge AV HIL validation platform.

Standalone (no ROS2, no GoogleTest) so it can be compiled with
-fsanitize=thread and run in the TSan CI job.

Three concurrent workloads run simultaneously:

  1. SPSCRing try_push / try_pop  -- the plain queue path.
  2. SPSCRing push_overwrite      -- the PRODUCER-SIDE EVICTION path. The audit
     found that the previous force_push_overwrite let the producer do a plain
     load+store on the consumer-owned tail (a lost-update race) and that this
     one racy method was the one method TSan never ran. It is now driven here,
     concurrently with a live consumer, with a per-item checksum so a torn or
     recycled slot is detected as corruption in addition to whatever TSan sees.
  3. Seqlock writer / reader      -- RCU-style config updates.

On success it prints a summary and exits 0; a data race, if present, is
reported by ThreadSanitizer and fails the run.

Build (on EC2):
  clang++ -std=c++20 -O1 -g -fsanitize=thread -Iinclude \
      stress/spsc_tsan_stress.cpp -o spsc_tsan_stress
Run:
  SENSORFORGE_STRESS_SECONDS=60 ./spsc_tsan_stress
==============================================================================
*/

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <thread>

#include "sensorforge/core/spsc_ring.hpp"
#include "sensorforge/core/seqlock.hpp"

using sensorforge::core::SPSCRing;
using sensorforge::core::Seqlock;

namespace {

struct Item
{
  uint64_t seq = 0;
  uint64_t checksum = 0;
};

int stress_seconds()
{
  if (const char * s = std::getenv("SENSORFORGE_STRESS_SECONDS")) {
    const int v = std::atoi(s);
    if (v > 0) {
      return v;
    }
  }
  return 60;
}

}  // namespace

int main()
{
  const auto duration = std::chrono::seconds(stress_seconds());
  const auto deadline = std::chrono::steady_clock::now() + duration;

  SPSCRing<Item, 1024> ring;
  std::atomic<uint64_t> produced{0};
  std::atomic<uint64_t> consumed{0};
  std::atomic<uint64_t> mismatches{0};
  std::atomic<bool> stop{false};

  // Producer: monotonically increasing seq with a derived checksum.
  std::thread producer([&]() {
    uint64_t seq = 0;
    while (!stop.load(std::memory_order_relaxed)) {
      Item it{seq, seq * 2654435761u};
      if (ring.try_push(it)) {
        ++seq;
        produced.store(seq, std::memory_order_relaxed);
      }
    }
  });

  // Consumer: verify each popped item is internally consistent.
  std::thread consumer([&]() {
    Item it;
    while (!stop.load(std::memory_order_relaxed)) {
      if (ring.try_pop(it)) {
        if (it.checksum != it.seq * 2654435761u) {
          mismatches.fetch_add(1, std::memory_order_relaxed);
        }
        consumed.fetch_add(1, std::memory_order_relaxed);
      }
    }
  });

  // Seqlock: one writer updating a config value, one reader snapshotting it.
  struct Config { uint64_t rate_hz; uint64_t delay_ns; uint64_t guard; };
  Seqlock<Config> cfg(Config{100, 0, 100 + 0});
  std::atomic<uint64_t> torn_reads{0};
  std::thread cfg_writer([&]() {
    uint64_t r = 100;
    while (!stop.load(std::memory_order_relaxed)) {
      cfg.store(Config{r, r * 3, r + r * 3});
      ++r;
    }
  });
  std::thread cfg_reader([&]() {
    while (!stop.load(std::memory_order_relaxed)) {
      const Config c = cfg.load();
      if (c.guard != c.rate_hz + c.delay_ns) {
        torn_reads.fetch_add(1, std::memory_order_relaxed);
      }
    }
  });

  // ---- overwrite / saturation path -----------------------------------------
  // A SEPARATE ring whose producer deliberately outruns its consumer, so the
  // ring is permanently full and push_overwrite() evicts on essentially every
  // push. This is the path the audit flagged as racy and untested.
  SPSCRing<Item, 64> ow_ring;
  std::atomic<uint64_t> ow_produced{0};
  std::atomic<uint64_t> ow_consumed{0};
  std::atomic<uint64_t> ow_evicted{0};
  std::atomic<uint64_t> ow_mismatches{0};
  std::atomic<uint64_t> ow_regressions{0};

  std::thread ow_producer([&]() {
    uint64_t seq = 0;
    uint64_t evicted = 0;
    while (!stop.load(std::memory_order_relaxed)) {
      Item it{seq, seq * 2654435761u};
      ow_ring.push_overwrite(it, evicted);
      ++seq;
      ow_produced.store(seq, std::memory_order_relaxed);
      ow_evicted.store(evicted, std::memory_order_relaxed);
    }
  });

  std::thread ow_consumer([&]() {
    Item it;
    uint64_t last = 0;
    bool have_last = false;
    while (!stop.load(std::memory_order_relaxed)) {
      if (ow_ring.try_pop(it)) {
        // Content integrity: a torn or recycled slot breaks the checksum.
        if (it.checksum != it.seq * 2654435761u) {
          ow_mismatches.fetch_add(1, std::memory_order_relaxed);
        }
        // Ordering integrity: overwrite may SKIP sequences (that is the point)
        // but must never deliver one backwards, which is what a lost update on
        // the tail index would produce.
        if (have_last && it.seq < last) {
          ow_regressions.fetch_add(1, std::memory_order_relaxed);
        }
        last = it.seq;
        have_last = true;
        ow_consumed.fetch_add(1, std::memory_order_relaxed);
      }
      // Deliberately slow: keeps the ring saturated so eviction dominates.
      std::this_thread::sleep_for(std::chrono::microseconds(20));
    }
  });

  std::this_thread::sleep_until(deadline);
  stop.store(true, std::memory_order_relaxed);
  producer.join();
  consumer.join();
  cfg_writer.join();
  cfg_reader.join();
  ow_producer.join();
  ow_consumer.join();

  std::printf(
    "[STRESS] produced=%llu consumed=%llu ring_mismatches=%llu torn_seqlock_reads=%llu\n",
    static_cast<unsigned long long>(produced.load()),
    static_cast<unsigned long long>(consumed.load()),
    static_cast<unsigned long long>(mismatches.load()),
    static_cast<unsigned long long>(torn_reads.load()));
  std::printf(
    "[STRESS] overwrite produced=%llu consumed=%llu evicted=%llu "
    "mismatches=%llu seq_regressions=%llu\n",
    static_cast<unsigned long long>(ow_produced.load()),
    static_cast<unsigned long long>(ow_consumed.load()),
    static_cast<unsigned long long>(ow_evicted.load()),
    static_cast<unsigned long long>(ow_mismatches.load()),
    static_cast<unsigned long long>(ow_regressions.load()));
  std::printf(
    "[STRESS] total_ring_ops=%llu\n",
    static_cast<unsigned long long>(
      produced.load() + consumed.load() + ow_produced.load() + ow_consumed.load()));

  if (mismatches.load() != 0 || torn_reads.load() != 0 ||
    ow_mismatches.load() != 0 || ow_regressions.load() != 0)
  {
    std::printf("[STRESS] FAIL: data corruption detected\n");
    return 1;
  }
  if (ow_evicted.load() == 0) {
    std::printf("[STRESS] FAIL: overwrite path never evicted; test is not exercising it\n");
    return 1;
  }
  std::printf("[STRESS] PASS: no corruption (run under TSan to check for races)\n");
  return 0;
}
