/*
==============================================================================
SensorForge - Report generation tests (Extension G/L)
Non-ROS: verifies JSON schema + HTML rendering of scenario reports.
==============================================================================
*/

#include <gtest/gtest.h>

#include "test_support.hpp"

#include <nlohmann/json.hpp>

#include "report/report_types.hpp"
#include "report/json_report.hpp"
#include "report/html_report.hpp"

using namespace sensorforge;

namespace {
report::ScenarioReport sample(bool pass)
{
  scenario::ScenarioResult res;
  res.scenario_name = "delayed_imu";
  res.passed = pass;
  scenario::AssertionResult a;
  a.metric = "imu_latency_p99_ms";
  a.op = scenario::CompareOp::kLessThan;
  a.threshold = 30.0;
  a.actual = 12.4;
  a.found = true;
  a.passed = pass;
  res.assertions.push_back(a);

  scenario::MetricMap m;
  m["imu_msgs_per_sec"] = 200.1;
  m["imu_drop_rate_pct"] = 0.0;
  m["imu_latency_p99_ms"] = 12.4;
  m["imu_received"] = 6003;
  m["camera_msgs_per_sec"] = 30.0;
  return report::build_report(res, m, {"imu", "camera"}, 30.0);
}
}  // namespace

TEST(Report, JsonSchemaMatchesSpec)
{
  const auto rep = sample(true);
  const auto js = report::to_json(rep);
  const auto j = nlohmann::json::parse(js);

  SF_EXPECT_EQ(j["scenario"], "delayed_imu");
  SF_EXPECT_EQ(j["result"], "PASS");
  SF_EXPECT_EQ(j["duration_seconds"], 30.0);
  SF_ASSERT_TRUE(j.contains("assertions"));
  SF_ASSERT_EQ(j["assertions"].size(), 1u);
  SF_EXPECT_EQ(j["assertions"][0]["metric"], "imu_latency_p99_ms");
  SF_EXPECT_EQ(j["assertions"][0]["operator"], "less_than");
  SF_EXPECT_EQ(j["assertions"][0]["threshold"], 30.0);
  SF_EXPECT_EQ(j["assertions"][0]["actual"], 12.4);
  SF_EXPECT_EQ(j["assertions"][0]["result"], "PASS");

  SF_ASSERT_TRUE(j["metrics"].contains("per_sensor"));
  SF_EXPECT_EQ(j["metrics"]["per_sensor"]["imu"]["msgs_sec"], 200.1);
  SF_EXPECT_EQ(j["metrics"]["per_sensor"]["imu"]["latency_p99_ms"], 12.4);
  SF_ASSERT_TRUE(j["metrics"].contains("transport"));
  SF_EXPECT_TRUE(j["metrics"]["transport"].contains("crc_failures"));
}

TEST(Report, FailResultReflected)
{
  const auto j = nlohmann::json::parse(report::to_json(sample(false)));
  SF_EXPECT_EQ(j["result"], "FAIL");
  SF_EXPECT_EQ(j["assertions"][0]["result"], "FAIL");
}

TEST(Report, HtmlIsSelfContained)
{
  const auto html = report::to_html(sample(true));
  SF_EXPECT_NE(html.find("<!doctype html>"), std::string::npos);
  SF_EXPECT_NE(html.find("delayed_imu"), std::string::npos);
  SF_EXPECT_NE(html.find("imu_latency_p99_ms"), std::string::npos);
  // No external asset references (fully self-contained).
  SF_EXPECT_EQ(html.find("http://"), std::string::npos);
  SF_EXPECT_EQ(html.find("https://"), std::string::npos);
  SF_EXPECT_EQ(html.find("<script"), std::string::npos);
}
