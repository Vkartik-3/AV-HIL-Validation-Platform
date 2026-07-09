/*
==============================================================================
SensorForge - Scenario parser / assertion / metrics tests (Extension G/J)
Non-ROS: covers the YAML parser, assertion evaluation and metric derivation.
==============================================================================
*/

#include <gtest/gtest.h>

#include "test_support.hpp"

#include "scenario/assertion.hpp"
#include "scenario/metrics.hpp"
#include "scenario/scenario.hpp"

using namespace sensorforge::scenario;

TEST(ScenarioParser, ParsesFullScenario)
{
  const std::string yaml = R"(
name: delayed_imu
duration_seconds: 30
streams:
  camera: {rate_hz: 30}
  imu: {rate_hz: 200}
faults:
  - stream: imu
    start_seconds: 8
    duration_seconds: 3
    type: delay
    params: {delay_ms: 25}
assertions:
  - metric: imu_latency_p99_ms
    operator: less_than
    value: 30
  - metric: camera_drop_rate_pct
    operator: less_than
    value: 0.1
)";
  const Scenario s = parse_scenario_string(yaml);
  SF_EXPECT_EQ(s.name, "delayed_imu");
  SF_EXPECT_EQ(s.duration_seconds, 30.0);
  SF_ASSERT_EQ(s.streams.size(), 2u);
  SF_ASSERT_EQ(s.faults.size(), 1u);
  SF_EXPECT_EQ(s.faults[0].type, FaultType::kDelay);
  SF_EXPECT_EQ(s.faults[0].stream, "imu");
  SF_EXPECT_EQ(s.faults[0].param("delay_ms"), 25.0);
  SF_ASSERT_EQ(s.assertions.size(), 2u);
  SF_EXPECT_EQ(s.assertions[0].op, CompareOp::kLessThan);
}

TEST(ScenarioParser, RejectsMalformed)
{
  SF_EXPECT_TRUE(([] {
      try {parse_scenario_string("duration_seconds: 5"); return false;}
      catch (const std::exception &) {return true;}
    })());
  SF_EXPECT_TRUE(([] {
      try {parse_scenario_string("name: x\nduration_seconds: 0"); return false;}
      catch (const std::exception &) {return true;}
    })());
  SF_EXPECT_TRUE(([] {
      try {
        parse_scenario_string(
          "name: x\nduration_seconds: 1\nassertions:\n  - metric: m\n    operator: bogus\n    value: 1");
        return false;
      } catch (const std::exception &) {return true;}
    })());
}

TEST(ScenarioParser, FaultTypeRoundTrip)
{
  const char * names[] = {"delay", "drop", "corrupt", "reorder", "duplicate",
    "burst_loss", "bandwidth_limit", "ecu_disconnect", "process_restart"};
  for (const char * n : names) {
    const FaultType t = fault_type_from_string(n);
    SF_EXPECT_NE(t, FaultType::kUnknown);
    SF_EXPECT_EQ(to_string(t), std::string(n));
  }
  SF_EXPECT_EQ(fault_type_from_string("nonsense"), FaultType::kUnknown);
}

// ---- assertion evaluation over all operators -------------------------------
class AssertionOps : public ::testing::TestWithParam<int> {};

TEST_P(AssertionOps, EvaluatesCorrectly)
{
  const auto op = static_cast<CompareOp>(GetParam());
  MetricMap m;
  for (double actual = 0.0; actual <= 20.0; actual += 0.5) {
    m["x"] = actual;
    Assertion a;
    a.metric = "x";
    a.op = op;
    a.value = 10.0;
    const AssertionResult r = evaluate(a, m);
    SF_EXPECT_TRUE(r.found);
    bool expected = false;
    switch (op) {
      case CompareOp::kLessThan: expected = actual < 10.0; break;
      case CompareOp::kGreaterThan: expected = actual > 10.0; break;
      case CompareOp::kEquals: expected = std::fabs(actual - 10.0) <= 1e-9; break;
      case CompareOp::kNotEquals: expected = std::fabs(actual - 10.0) > 1e-9; break;
      default: break;
    }
    SF_EXPECT_EQ(r.passed, expected);
  }
}

INSTANTIATE_TEST_SUITE_P(AllOps, AssertionOps, ::testing::Range(0, 4));

TEST(Assertion, MissingMetricFails)
{
  MetricMap m;
  Assertion a;
  a.metric = "absent";
  a.op = CompareOp::kLessThan;
  a.value = 1.0;
  const AssertionResult r = evaluate(a, m);
  SF_EXPECT_FALSE(r.found);
  SF_EXPECT_FALSE(r.passed);  // missing metric must fail, not silently pass
}

// ---- metric derivation -----------------------------------------------------
TEST(StreamMetrics, DerivesPercentilesAndRates)
{
  StreamMetrics sm("imu", 200.0, 10.0);  // expect 2000 msgs
  for (int i = 0; i < 2000; ++i) {
    sm.record(1.0 + (i % 50) * 0.1);   // latencies 1.0 .. 5.9 ms
  }
  MetricMap m;
  sm.emit(m);
  SF_EXPECT_EQ(m["imu_received"], 2000.0);
  SF_EXPECT_NEAR(m["imu_msgs_per_sec"], 200.0, 1e-6);
  SF_EXPECT_NEAR(m["imu_drop_rate_pct"], 0.0, 1e-6);
  SF_EXPECT_GE(m["imu_latency_p99_ms"], m["imu_latency_p50_ms"]);
  SF_EXPECT_EQ(m["imu_sequence_gaps"], 0.0);
}

TEST(StreamMetrics, DropRateReflectsMissingMessages)
{
  StreamMetrics sm("camera", 30.0, 10.0);  // expect 300 over full duration
  for (int i = 0; i < 150; ++i) {sm.record(5.0);}  // no arrival -> full-duration fallback
  MetricMap m;
  sm.emit(m);
  SF_EXPECT_NEAR(m["camera_drop_rate_pct"], 50.0, 1e-6);
}

TEST(StreamMetrics, SteadyStateWindowExcludesStartup)
{
  // 200 Hz stream that only STARTS at t=4s (e.g. slow `ros2 run` launch), then
  // delivers cleanly for 1s. Full-duration math would call this ~99% "drop";
  // steady-state over [4,5] correctly reports ~0%.
  StreamMetrics sm("imu", 200.0, 30.0);
  for (int i = 0; i <= 200; ++i) {
    sm.record(1.0, 4.0 + i * (1.0 / 200.0));   // 201 msgs -> 200 intervals over 1s
  }
  MetricMap m;
  sm.emit(m);
  SF_EXPECT_NEAR(m["imu_drop_rate_pct"], 0.0, 0.5);
}

TEST(StreamMetrics, SteadyStateDetectsRealLoss)
{
  // Over a 1s window at 200 Hz we expect 200 intervals; deliver only 100 -> 50%.
  StreamMetrics sm("imu", 200.0, 30.0);
  sm.record(1.0, 4.0);
  for (int i = 1; i < 100; ++i) {sm.record(1.0, 4.0 + i * (1.0 / 200.0));}
  sm.record(1.0, 5.0);   // 101 msgs -> 100 observed intervals in a 1s window
  MetricMap m;
  sm.emit(m);
  SF_EXPECT_NEAR(m["imu_drop_rate_pct"], 50.0, 2.0);
}
