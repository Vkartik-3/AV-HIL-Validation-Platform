/*
==============================================================================
SensorForge - SPSC ring tests (Extension G)
FIFO ordering, full/empty behavior, wraparound, and concurrent stress.
==============================================================================
*/

#include <gtest/gtest.h>

#include "test_support.hpp"

#include <atomic>
#include <thread>

#include "sensorforge/core/spsc_ring.hpp"

using sensorforge::core::SPSCRing;

TEST(SpscRing, EmptyAndFull)
{
  SPSCRing<int, 8> ring;
  SF_EXPECT_TRUE(ring.empty_approx());
  SF_EXPECT_EQ(ring.capacity(), 7u);  // one slot reserved

  int out = 0;
  SF_EXPECT_FALSE(ring.try_pop(out));  // empty pop fails

  // Fill to capacity.
  for (int i = 0; i < 7; ++i) {
    SF_EXPECT_TRUE(ring.try_push(i));
  }
  SF_EXPECT_FALSE(ring.try_push(999));  // full push fails
  SF_EXPECT_EQ(ring.size_approx(), 7u);
}

TEST(SpscRing, FifoOrderAndWraparound)
{
  SPSCRing<int, 16> ring;
  int expected_pop = 0;
  int next_push = 0;
  // Push/pop many more than capacity to force wraparound repeatedly.
  for (int round = 0; round < 10000; ++round) {
    const int batch = (round % 7) + 1;
    for (int i = 0; i < batch; ++i) {
      if (ring.try_push(next_push)) {
        ++next_push;
      }
    }
    int out = 0;
    while (ring.try_pop(out)) {
      SF_EXPECT_EQ(out, expected_pop);
      ++expected_pop;
    }
  }
  SF_EXPECT_EQ(expected_pop, next_push);
}

TEST(SpscRing, OverwriteOldestPolicyHelper)
{
  SPSCRing<int, 4> ring;  // capacity 3
  for (int i = 0; i < 3; ++i) {SF_EXPECT_TRUE(ring.try_push(i));}
  SF_EXPECT_FALSE(ring.try_push(100));   // full
  SF_EXPECT_TRUE(ring.force_push_overwrite(100));  // drops oldest, inserts new
  // Oldest (0) should be gone; first pop is 1.
  int out = 0;
  SF_ASSERT_TRUE(ring.try_pop(out));
  SF_EXPECT_EQ(out, 1);
}

TEST(SpscRing, ConcurrentProducerConsumerNoLoss)
{
  SPSCRing<uint64_t, 1024> ring;
  const uint64_t N = 300000;   // bounded so the Debug/-O0 CI run stays fast
  std::atomic<bool> done{false};
  uint64_t consumed = 0;
  uint64_t expected = 0;
  bool ordered = true;

  std::thread producer([&]() {
    uint64_t i = 0;
    while (i < N) {
      if (ring.try_push(i)) {
        ++i;
      }
    }
    done.store(true);
  });

  uint64_t out = 0;
  while (consumed < N) {
    if (ring.try_pop(out)) {
      if (out != expected) {ordered = false;}
      ++expected;
      ++consumed;
    }
  }
  producer.join();

  SF_EXPECT_TRUE(ordered);
  SF_EXPECT_EQ(consumed, N);
}
