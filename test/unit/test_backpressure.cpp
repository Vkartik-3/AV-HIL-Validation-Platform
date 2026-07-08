/*
==============================================================================
SensorForge - Backpressure policy tests (Extension G)
Verifies each per-sensor policy's behavior when the ring is full, and the
default policy mapping.
==============================================================================
*/

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
  SF_EXPECT_EQ(default_policy_for(SensorType::kImu), BackpressurePolicy::kBatchAccumulate);
  SF_EXPECT_EQ(default_policy_for(SensorType::kGps), BackpressurePolicy::kDropNewest);
  SF_EXPECT_EQ(default_policy_for(SensorType::kCan), BackpressurePolicy::kNeverDropBlock);
}

TEST(Backpressure, DropNewestWhenFull)
{
  SPSCRing<int, 4> ring;  // capacity 3
  for (int i = 0; i < 3; ++i) {
    SF_EXPECT_EQ(apply_policy(ring, i, BackpressurePolicy::kDropNewest), PushResult::kEnqueued);
  }
  // Now full: newest is dropped.
  for (int i = 0; i < 50; ++i) {
    SF_EXPECT_EQ(apply_policy(ring, 999, BackpressurePolicy::kDropNewest), PushResult::kDroppedNewest);
  }
  int out = 0;
  SF_ASSERT_TRUE(ring.try_pop(out));
  SF_EXPECT_EQ(out, 0);  // oldest preserved
}

TEST(Backpressure, OverwriteOldestWhenFull)
{
  SPSCRing<int, 4> ring;  // capacity 3
  for (int i = 0; i < 3; ++i) {
    SF_EXPECT_EQ(apply_policy(ring, i, BackpressurePolicy::kOverwriteOldest), PushResult::kEnqueued);
  }
  SF_EXPECT_EQ(apply_policy(ring, 100, BackpressurePolicy::kOverwriteOldest), PushResult::kOverwrote);
  int out = 0;
  SF_ASSERT_TRUE(ring.try_pop(out));
  SF_EXPECT_EQ(out, 1);  // 0 was overwritten out
}

TEST(Backpressure, NeverDropBlockEnqueuesWhenSpace)
{
  SPSCRing<int, 8> ring;
  for (int i = 0; i < 7; ++i) {
    SF_EXPECT_EQ(apply_policy(ring, i, BackpressurePolicy::kNeverDropBlock), PushResult::kEnqueued);
  }
  // Drain one, then a blocking push should succeed immediately.
  int out = 0;
  SF_ASSERT_TRUE(ring.try_pop(out));
  SF_EXPECT_EQ(apply_policy(ring, 42, BackpressurePolicy::kNeverDropBlock), PushResult::kEnqueued);
}
