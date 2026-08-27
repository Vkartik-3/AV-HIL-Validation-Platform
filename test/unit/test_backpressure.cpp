/*
==============================================================================
SensorForge - Backpressure policy tests (Extension G)
Verifies each per-sensor policy's behavior when the ring is full, and the
default policy mapping.
==============================================================================
*/

#include <chrono>
#include <string>

#include <gtest/gtest.h>

#include "test_support.hpp"

#include "sensorforge/core/backpressure_policy.hpp"
#include "sensorforge/core/spsc_ring.hpp"

using namespace sensorforge::core;
using sensorforge::protocol::SensorType;

TEST(Backpressure, DefaultPolicyMapping)
{
  SF_EXPECT_EQ(default_policy_for(SensorType::kCamera), BackpressurePolicy::kOverwriteOldest);
  SF_EXPECT_EQ(default_policy_for(SensorType::kLidar), BackpressurePolicy::kDropNewest);
  // kBatchAccumulate was DELETED: it shared a case label with kDropNewest and
  // implemented no batching whatsoever, so IMU now declares the drop-newest
  // behaviour it always actually had.
  SF_EXPECT_EQ(default_policy_for(SensorType::kImu), BackpressurePolicy::kDropNewest);
  SF_EXPECT_EQ(default_policy_for(SensorType::kGps), BackpressurePolicy::kDropNewest);
  SF_EXPECT_EQ(default_policy_for(SensorType::kCan), BackpressurePolicy::kNeverDropBlock);
}

TEST(Backpressure, PolicyNamesRoundTrip)
{
  SF_EXPECT_EQ(policy_from_string("overwrite_oldest"), BackpressurePolicy::kOverwriteOldest);
  SF_EXPECT_EQ(policy_from_string("block"), BackpressurePolicy::kNeverDropBlock);
  SF_EXPECT_EQ(policy_from_string("drop_newest"), BackpressurePolicy::kDropNewest);
  SF_EXPECT_EQ(policy_from_string("nonsense"), BackpressurePolicy::kDropNewest);
  SF_EXPECT_EQ(std::string(to_string(BackpressurePolicy::kOverwriteOldest)), "overwrite_oldest");
}

TEST(Backpressure, DropNewestWhenFull)
{
  SPSCRing<int, 8> ring;  // capacity 7
  uint64_t evicted = 0;
  for (int i = 0; i < 7; ++i) {
    SF_EXPECT_EQ(
      apply_policy(ring, i, BackpressurePolicy::kDropNewest, evicted), PushResult::kEnqueued);
  }
  // Now full: newest is dropped.
  for (int i = 0; i < 50; ++i) {
    SF_EXPECT_EQ(
      apply_policy(ring, 999, BackpressurePolicy::kDropNewest, evicted),
      PushResult::kDroppedNewest);
  }
  SF_EXPECT_EQ(evicted, 0u);
  int out = 0;
  SF_ASSERT_TRUE(ring.try_pop(out));
  SF_EXPECT_EQ(out, 0);  // oldest preserved
}

TEST(Backpressure, OverwriteOldestWhenFull)
{
  SPSCRing<int, 8> ring;  // capacity 7
  uint64_t evicted = 0;
  for (int i = 0; i < 7; ++i) {
    SF_EXPECT_EQ(
      apply_policy(ring, i, BackpressurePolicy::kOverwriteOldest, evicted),
      PushResult::kEnqueued);
  }
  SF_EXPECT_EQ(
    apply_policy(ring, 100, BackpressurePolicy::kOverwriteOldest, evicted),
    PushResult::kOverwrote);
  SF_EXPECT_EQ(evicted, 1u);
  int out = 0;
  SF_ASSERT_TRUE(ring.try_pop(out));
  SF_EXPECT_EQ(out, 1);  // 0 was overwritten out
}

TEST(Backpressure, NeverDropBlockEnqueuesWhenSpace)
{
  SPSCRing<int, 8> ring;
  uint64_t evicted = 0;
  for (int i = 0; i < 7; ++i) {
    SF_EXPECT_EQ(
      apply_policy(ring, i, BackpressurePolicy::kNeverDropBlock, evicted),
      PushResult::kEnqueued);
  }
  // Drain one, then a blocking push should succeed immediately.
  int out = 0;
  SF_ASSERT_TRUE(ring.try_pop(out));
  SF_EXPECT_EQ(
    apply_policy(ring, 42, BackpressurePolicy::kNeverDropBlock, evicted),
    PushResult::kEnqueued);
}

// The old kNeverDropBlock spun FOREVER, so a stalled consumer could hang a ROS
// executor thread permanently. It is now bounded.
TEST(Backpressure, NeverDropBlockIsBoundedWhenConsumerIsStalled)
{
  SPSCRing<int, 8> ring;
  uint64_t evicted = 0;
  for (int i = 0; i < 7; ++i) {
    apply_policy(ring, i, BackpressurePolicy::kNeverDropBlock, evicted);
  }
  const auto t0 = std::chrono::steady_clock::now();
  const auto r = apply_policy(
    ring, 7, BackpressurePolicy::kNeverDropBlock, evicted, std::chrono::microseconds(1000));
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  SF_EXPECT_EQ(r, PushResult::kDroppedAfterBlock);
  SF_EXPECT_LT(ms, 500);
}
