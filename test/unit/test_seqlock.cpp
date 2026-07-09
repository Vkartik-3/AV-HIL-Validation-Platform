/*
==============================================================================
SensorForge - Seqlock tests (Extension G)
Single-threaded correctness, sequence parity, and concurrent consistency.
==============================================================================
*/

#include <gtest/gtest.h>

#include "test_support.hpp"

#include <atomic>
#include <thread>

#include "sensorforge/core/seqlock.hpp"

using sensorforge::core::Seqlock;

struct Config
{
  uint32_t rate_hz;
  uint32_t delay_ns;
  uint64_t guard;   // invariant: guard == rate_hz + delay_ns
};

TEST(Seqlock, StoreLoadRoundTrip)
{
  Seqlock<Config> sl(Config{10, 20, 30});
  for (uint32_t i = 0; i < 2000; ++i) {
    Config c{i, i * 2, static_cast<uint64_t>(i) + i * 2};
    sl.store(c);
    const Config got = sl.load();
    SF_EXPECT_EQ(got.rate_hz, i);
    SF_EXPECT_EQ(got.delay_ns, i * 2);
    SF_EXPECT_EQ(got.guard, got.rate_hz + got.delay_ns);
  }
}

TEST(Seqlock, SequenceIsEvenWhenStable)
{
  Seqlock<int> sl(0);
  for (int i = 0; i < 1000; ++i) {
    sl.store(i);
    SF_EXPECT_EQ(sl.sequence() % 2, 0u);  // stable => even
    (void)sl.load();
    SF_EXPECT_EQ(sl.sequence() % 2, 0u);
  }
}

TEST(Seqlock, ConcurrentReadsAreConsistent)
{
  Seqlock<Config> sl(Config{0, 0, 0});
  std::atomic<bool> stop{false};
  std::atomic<uint64_t> torn{0};
  std::atomic<uint64_t> reads{0};

  std::thread writer([&]() {
    uint32_t r = 0;
    while (!stop.load(std::memory_order_relaxed)) {
      sl.store(Config{r, r * 3, static_cast<uint64_t>(r) + r * 3});
      ++r;
    }
  });

  // Reader validates the guard invariant on every snapshot. 50k contended
  // reads is plenty to exercise the retry path without dragging out the
  // Debug/-O0 CI run (a hot writer makes each read retry repeatedly).
  for (int i = 0; i < 50000; ++i) {
    const Config c = sl.load();
    if (c.guard != static_cast<uint64_t>(c.rate_hz) + c.delay_ns) {
      torn.fetch_add(1);
    }
    reads.fetch_add(1);
  }
  stop.store(true);
  writer.join();

  SF_EXPECT_EQ(torn.load(), 0u);
  SF_EXPECT_GT(reads.load(), 0u);
}
