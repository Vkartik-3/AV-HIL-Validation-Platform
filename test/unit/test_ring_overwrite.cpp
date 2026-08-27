/*
==============================================================================
SensorForge - Concurrent overwrite / eviction tests for SPSCRing
The audit found force_push_overwrite raced (producer stored to the
consumer-owned tail) and was exercised SINGLE-THREADED only, never under TSan.
These tests drive the corrected eviction path with a live consumer.
==============================================================================
*/

#include <atomic>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "sensorforge/core/spsc_ring.hpp"
#include "test_support.hpp"

using sensorforge::core::SPSCRing;

TEST(RingOverwrite, ReservesTwoSlots)
{
  SPSCRing<int, 8> ring;
  SF_EXPECT_EQ(ring.capacity(), 7u);
  int n = 0;
  while (ring.try_push(n)) {
    ++n;
  }
  SF_EXPECT_EQ(static_cast<size_t>(n), ring.capacity());
}

TEST(RingOverwrite, EvictOldestFreesExactlyOneSlot)
{
  SPSCRing<int, 8> ring;
  for (int i = 0; i < 7; ++i) {
    SF_ASSERT_TRUE(ring.try_push(i));
  }
  SF_EXPECT_FALSE(ring.try_push(99));
  SF_EXPECT_TRUE(ring.evict_oldest());
  SF_EXPECT_TRUE(ring.try_push(99));

  int out = 0;
  SF_ASSERT_TRUE(ring.try_pop(out));
  SF_EXPECT_EQ(out, 1);   // 0 was evicted
}

TEST(RingOverwrite, EvictOnEmptyReturnsFalse)
{
  SPSCRing<int, 8> ring;
  SF_EXPECT_FALSE(ring.evict_oldest());
}

TEST(RingOverwrite, PushOverwriteAlwaysAdmitsNewest)
{
  SPSCRing<int, 8> ring;
  uint64_t evicted = 0;
  for (int i = 0; i < 200; ++i) {
    SF_EXPECT_TRUE(ring.push_overwrite(i, evicted));
  }
  SF_EXPECT_GT(evicted, 0u);
  // Everything still in the ring must be from the tail end of the sequence.
  int out = 0;
  int min_seen = 1 << 30;
  while (ring.try_pop(out)) {
    min_seen = out < min_seen ? out : min_seen;
  }
  SF_EXPECT_GE(min_seen, 200 - static_cast<int>(ring.capacity()) - 1);
}

// The core regression: producer evicting while a consumer pops. Every value the
// consumer sees must be one the producer actually wrote (no torn or recycled
// slots), and the indices must not desynchronise.
TEST(RingOverwrite, ConcurrentEvictAndPopStaysConsistent)
{
  struct Item
  {
    uint64_t seq = 0;
    uint64_t check = 0;
  };
  SPSCRing<Item, 64> ring;
  std::atomic<bool> stop{false};
  std::atomic<uint64_t> mismatches{0};
  std::atomic<uint64_t> consumed{0};

  std::thread consumer([&]() {
      Item it;
      while (!stop.load(std::memory_order_relaxed)) {
        if (ring.try_pop(it)) {
          if (it.check != it.seq * 2654435761u) {
            mismatches.fetch_add(1, std::memory_order_relaxed);
          }
          consumed.fetch_add(1, std::memory_order_relaxed);
        }
      }
      while (ring.try_pop(it)) {
        if (it.check != it.seq * 2654435761u) {
          mismatches.fetch_add(1, std::memory_order_relaxed);
        }
        consumed.fetch_add(1, std::memory_order_relaxed);
      }
    });

  uint64_t evicted = 0;
  uint64_t produced = 0;
  uint64_t push_failed = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
  while (std::chrono::steady_clock::now() < deadline) {
    Item it{produced, produced * 2654435761u};
    // push_overwrite is BOUNDED: it gives up after kMaxEvictAttempts rather
    // than spinning, so a rejected push is a legitimate outcome under a
    // consumer that is concurrently claiming slots.
    if (!ring.push_overwrite(it, evicted)) {
      ++push_failed;
    }
    ++produced;
  }
  stop.store(true);
  consumer.join();

  SF_EXPECT_EQ(mismatches.load(), 0u);
  SF_EXPECT_GT(produced, 0u);
  SF_EXPECT_GT(evicted, 0u);          // the eviction path really was exercised
  // Full accounting: every produced item was consumed, deliberately evicted,
  // rejected at push, or is still resident. Nothing vanishes unexplained and
  // nothing is delivered twice.
  SF_EXPECT_LE(consumed.load(), produced);
  SF_EXPECT_GE(
    consumed.load() + evicted + push_failed + ring.capacity(), produced);
}
