/*
==============================================================================
SensorForge - Prometheus registry tests (Extension G/M)
Non-ROS: verifies the text exposition format (gauges, counters, labels, help).
==============================================================================
*/

#include <gtest/gtest.h>

#include "test_support.hpp"

#include "metrics/registry.hpp"

using namespace sensorforge::metrics;

TEST(Registry, GaugeWithLabelRenders)
{
  Registry r;
  r.set_help("sensorforge_msgs_per_sec", "msgs/sec per sensor");
  r.set_sensor_gauge("sensorforge_msgs_per_sec", "imu", 200.0);
  const std::string out = r.render();
  SF_EXPECT_NE(out.find("# HELP sensorforge_msgs_per_sec msgs/sec per sensor"), std::string::npos);
  SF_EXPECT_NE(out.find("# TYPE sensorforge_msgs_per_sec gauge"), std::string::npos);
  SF_EXPECT_NE(out.find("sensorforge_msgs_per_sec{sensor=\"imu\"} 200"), std::string::npos);
}

TEST(Registry, GaugeIsOverwrittenNotAccumulated)
{
  Registry r;
  r.set_sensor_gauge("g", "imu", 1.0);
  r.set_sensor_gauge("g", "imu", 5.0);
  const std::string out = r.render();
  SF_EXPECT_NE(out.find("g{sensor=\"imu\"} 5"), std::string::npos);
  SF_EXPECT_EQ(out.find("g{sensor=\"imu\"} 1\n"), std::string::npos);
}

TEST(Registry, CounterAccumulates)
{
  Registry r;
  r.add_counter("sensorforge_crc_failures_total", 2.0);
  r.add_counter("sensorforge_crc_failures_total", 3.0);
  const std::string out = r.render();
  SF_EXPECT_NE(out.find("# TYPE sensorforge_crc_failures_total counter"), std::string::npos);
  SF_EXPECT_NE(out.find("sensorforge_crc_failures_total 5"), std::string::npos);
}

TEST(Registry, MultipleSensorSeries)
{
  Registry r;
  for (const char * s : {"lidar", "camera", "imu", "gps", "can"}) {
    r.set_sensor_gauge("sensorforge_latency_p99_ms", s, 10.0);
  }
  const std::string out = r.render();
  for (const char * s : {"lidar", "camera", "imu", "gps", "can"}) {
    const std::string needle =
      std::string("sensorforge_latency_p99_ms{sensor=\"") + s + "\"} 10";
    SF_EXPECT_NE(out.find(needle), std::string::npos);
  }
}

TEST(Registry, LabelValueEscaped)
{
  Registry r;
  r.set_gauge("g", 1.0, Labels{{"path", "a\"b"}});
  const std::string out = r.render();
  SF_EXPECT_NE(out.find("a\\\"b"), std::string::npos);
}
