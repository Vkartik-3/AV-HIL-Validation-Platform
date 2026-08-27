/*
==============================================================================
SensorForge - Resource monitoring / budget tests
==============================================================================
*/

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "sensorforge/core/resource_monitor.hpp"
#include "test_support.hpp"

using namespace sensorforge::core;

TEST(ResourceMonitorTest, SamplerReportsPlausibleRss)
{
  ResourceSampler s;
  const auto a = s.sample();
  if (!ResourceSampler::platform_supported()) {
    SF_EXPECT_FALSE(a.available);
    return;
  }
  SF_EXPECT_TRUE(a.available);
  SF_EXPECT_GT(a.rss_bytes, 0u);
  // A test process is somewhere between 1 MiB and 16 GiB resident.
  SF_EXPECT_GT(a.rss_bytes, 1024u * 1024u);
  SF_EXPECT_LT(a.rss_bytes, 16ull * 1024 * 1024 * 1024);
}

TEST(ResourceMonitorTest, RssTracksAnAllocation)
{
  if (!ResourceSampler::platform_supported()) {
    SF_EXPECT_TRUE(true);
    return;
  }
  ResourceSampler s;
  const auto before = s.sample();
  std::vector<uint8_t> big(64u * 1024u * 1024u, 0xAB);   // touch the pages
  uint64_t touched = 0;
  for (size_t i = 0; i < big.size(); i += 4096) {
    big[i] = static_cast<uint8_t>(i / 4096 + 1);
    touched += big[i];
  }
  const auto after = s.sample();
  SF_EXPECT_GT(after.rss_bytes, before.rss_bytes);
  // Keep the allocation alive past the second sample so it cannot be optimised
  // away or reclaimed before RSS is read.
  SF_EXPECT_GT(touched, 0u);
}

TEST(ResourceMonitorTest, CpuPercentIsNonNegative)
{
  ResourceSampler s;
  s.sample();
  volatile double x = 0;
  for (int i = 0; i < 2000000; ++i) {
    x += i * 0.5;
  }
  const auto a = s.sample();
  SF_EXPECT_GE(a.cpu_percent, 0.0);
  SF_EXPECT_GT(x, 0.0);
}

TEST(ResourceMonitorTest, UnsetBudgetNeverBreaches)
{
  ResourceBudget b;
  SF_EXPECT_FALSE(b.any());
  SF_EXPECT_EQ(evaluate_budget(b, 1ull << 40, 1ull << 40), BudgetState::kOk);
}

TEST(ResourceMonitorTest, SoftAndHardBudgetsEvaluate)
{
  ResourceBudget b;
  b.soft_queue_bytes = 100;
  b.hard_queue_bytes = 200;
  SF_EXPECT_EQ(evaluate_budget(b, 0, 50), BudgetState::kOk);
  SF_EXPECT_EQ(evaluate_budget(b, 0, 150), BudgetState::kSoftBreach);
  SF_EXPECT_EQ(evaluate_budget(b, 0, 250), BudgetState::kHardBreach);
}

TEST(ResourceMonitorTest, RssBudgetEvaluatesIndependently)
{
  ResourceBudget b;
  b.soft_rss_bytes = 1000;
  b.hard_rss_bytes = 2000;
  SF_EXPECT_EQ(evaluate_budget(b, 500, 0), BudgetState::kOk);
  SF_EXPECT_EQ(evaluate_budget(b, 1500, 0), BudgetState::kSoftBreach);
  SF_EXPECT_EQ(evaluate_budget(b, 5000, 0), BudgetState::kHardBreach);
}

TEST(ResourceMonitorTest, HardBeatsSoftWhenBothBreached)
{
  ResourceBudget b;
  b.soft_rss_bytes = 100;
  b.hard_rss_bytes = 200;
  b.soft_queue_bytes = 100;
  b.hard_queue_bytes = 200;
  SF_EXPECT_EQ(evaluate_budget(b, 300, 300), BudgetState::kHardBreach);
}

TEST(ResourceMonitorTest, PlatformNameIsReported)
{
  const std::string n = ResourceSampler::platform_name();
  SF_EXPECT_FALSE(n.empty());
}
