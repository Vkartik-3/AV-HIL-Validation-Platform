/*
==============================================================================
SensorForge - Parallel aggregation + namespace tests (Extension G/O)
Non-ROS: verifies namespace vending, port assignment, and result aggregation.
==============================================================================
*/

#include <gtest/gtest.h>

#include "test_support.hpp"

#include "scenario/namespace_manager.hpp"

using namespace sensorforge::scenario;

TEST(NamespaceManager, VendsUniqueNamespaces)
{
  NamespaceManager m("/scenario");
  SF_EXPECT_EQ(m.next_namespace(), "/scenario_0");
  SF_EXPECT_EQ(m.next_namespace(), "/scenario_1");
  SF_EXPECT_EQ(m.next_namespace(), "/scenario_2");
  SF_EXPECT_EQ(m.count(), 3u);
}

TEST(NamespaceManager, MetricsPortsDisabledWhenBaseZero)
{
  NamespaceManager m("/scenario", 0);
  SF_EXPECT_EQ(m.metrics_port(0), 0);
  SF_EXPECT_EQ(m.metrics_port(5), 0);
}

TEST(NamespaceManager, MetricsPortsIncrement)
{
  NamespaceManager m("/scenario", 9090);
  SF_EXPECT_EQ(m.metrics_port(0), 9090);
  SF_EXPECT_EQ(m.metrics_port(3), 9093);
}

namespace {
ScenarioResult mk(const std::string & name, bool passed, int n_assert, int n_pass)
{
  ScenarioResult r;
  r.scenario_name = name;
  r.passed = passed;
  for (int i = 0; i < n_assert; ++i) {
    AssertionResult a;
    a.passed = i < n_pass;
    r.assertions.push_back(a);
  }
  return r;
}
}  // namespace

TEST(Aggregate, CountsScenariosAndChecks)
{
  std::vector<ScenarioResult> results = {
    mk("a", true, 4, 4),
    mk("b", false, 4, 2),
    mk("c", true, 4, 4),
  };
  const AggregateResult agg = aggregate(results);
  SF_EXPECT_EQ(agg.total_scenarios, 3u);
  SF_EXPECT_EQ(agg.scenarios_passed, 2u);
  SF_EXPECT_EQ(agg.scenarios_failed, 1u);
  SF_EXPECT_EQ(agg.total_assertions, 12u);   // N=3 x M=4
  SF_EXPECT_EQ(agg.assertions_passed, 10u);
  SF_EXPECT_FALSE(agg.all_passed());
  SF_ASSERT_EQ(agg.per_scenario.size(), 3u);
  SF_EXPECT_EQ(agg.per_scenario[1].first, "b");
  SF_EXPECT_FALSE(agg.per_scenario[1].second);
}

TEST(Aggregate, AllPassed)
{
  std::vector<ScenarioResult> results = {mk("a", true, 2, 2), mk("b", true, 3, 3)};
  const AggregateResult agg = aggregate(results);
  SF_EXPECT_TRUE(agg.all_passed());
  SF_EXPECT_EQ(agg.total_assertions, 5u);
  SF_EXPECT_EQ(agg.assertions_passed, 5u);
}
