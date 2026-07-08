/*
==============================================================================
SensorForge - Scenario runner (implementation, Extension J)
Part of the SensorForge AV HIL validation platform.
==============================================================================
*/

#include "scenario/scenario_runner.hpp"

#include <algorithm>
#include <cmath>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#if defined(__linux__)
#include <csignal>
#include <spawn.h>
#include <sys/wait.h>
extern char ** environ;
#endif

namespace sensorforge::scenario {

namespace {
// Map a stream name to its publisher executable and message type.
std::string publisher_executable(const std::string & stream)
{
  return stream + "_publisher";
}
}  // namespace

ScenarioRunner::ScenarioRunner(Scenario scenario, std::string ns)
: rclcpp::Node("scenario_runner"),
  scenario_(std::move(scenario)),
  ns_(std::move(ns)),
  start_time_(this->now())
{
}

ScenarioRunner::~ScenarioRunner()
{
#if defined(__linux__)
  for (int pid : child_pids_) {
    if (pid > 0) {
      ::kill(pid, SIGINT);
      int status = 0;
      ::waitpid(pid, &status, 0);
    }
  }
#endif
}

void ScenarioRunner::start()
{
  start_time_ = this->now();
  for (const auto & s : scenario_.streams) {
    metrics_.push_back(std::make_unique<StreamMetrics>(
        s.name, s.rate_hz, scenario_.duration_seconds));
    subscribe_stream(s);
    spawn_publisher(s);
  }
  RCLCPP_INFO(
    this->get_logger(), "Scenario '%s' started: %zu streams, %.0fs",
    scenario_.name.c_str(), scenario_.streams.size(), scenario_.duration_seconds);
}

void ScenarioRunner::spawn_publisher(const StreamConfig & s)
{
#if defined(__linux__)
  const std::string exe = publisher_executable(s.name);
  const std::string topic = ns_ + "/" + s.name;
  const std::string rate_arg = "rate_hz:=" + std::to_string(s.rate_hz);
  const std::string topic_arg = "topic:=" + topic;
  const std::string ns_arg = std::string("__ns:=") + ns_;

  std::vector<std::string> args = {
    "ros2", "run", "network_bridge", exe, "--ros-args",
    "-p", rate_arg, "-p", topic_arg, "-r", ns_arg};
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (auto & a : args) {argv.push_back(const_cast<char *>(a.c_str()));}
  argv.push_back(nullptr);

  pid_t pid = 0;
  const int rc = ::posix_spawnp(&pid, "ros2", nullptr, nullptr, argv.data(), environ);
  if (rc == 0) {
    child_pids_.push_back(pid);
    RCLCPP_INFO(this->get_logger(), "Launched %s -> %s @ %.0f Hz", exe.c_str(), topic.c_str(), s.rate_hz);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to launch %s (rc=%d)", exe.c_str(), rc);
  }
#else
  (void)s;
  RCLCPP_WARN(this->get_logger(), "Publisher spawning is Linux-only");
#endif
}

void ScenarioRunner::record(const std::string & stream, const rclcpp::Time & stamp)
{
  const double latency_ms = (this->now() - stamp).seconds() * 1000.0;
  for (auto & m : metrics_) {
    if (m->name() == stream) {
      m->record(latency_ms);
      return;
    }
  }
}

void ScenarioRunner::subscribe_stream(const StreamConfig & s)
{
  const std::string topic = ns_ + "/" + s.name;
  const auto qos = rclcpp::SensorDataQoS();
  const std::string name = s.name;

  if (s.name == "lidar") {
    subs_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos, [this, name](const sensor_msgs::msg::PointCloud2 & m) {
          record(name, m.header.stamp);
        }));
  } else if (s.name == "camera") {
    subs_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
        topic, qos, [this, name](const sensor_msgs::msg::Image & m) {
          record(name, m.header.stamp);
        }));
  } else if (s.name == "imu") {
    subs_.push_back(this->create_subscription<sensor_msgs::msg::Imu>(
        topic, qos, [this, name](const sensor_msgs::msg::Imu & m) {
          record(name, m.header.stamp);
        }));
  } else if (s.name == "gps") {
    subs_.push_back(this->create_subscription<sensor_msgs::msg::NavSatFix>(
        topic, qos, [this, name](const sensor_msgs::msg::NavSatFix & m) {
          record(name, m.header.stamp);
        }));
  } else if (s.name == "can") {
    // CAN has no ROS topic (it is a raw vcan bus). Its metrics are collected by
    // a dedicated CAN consumer when available; here we register the stream so
    // drop_rate is computed against the expected rate.
    RCLCPP_INFO(this->get_logger(), "CAN stream registered (measured on vcan bus)");
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown stream '%s' - no subscription", s.name.c_str());
  }
}

ScenarioResult ScenarioRunner::finish()
{
#if defined(__linux__)
  for (int pid : child_pids_) {
    if (pid > 0) {
      ::kill(pid, SIGINT);
    }
  }
  for (int pid : child_pids_) {
    if (pid > 0) {
      int status = 0;
      ::waitpid(pid, &status, 0);
    }
  }
  child_pids_.clear();
#endif

  MetricMap metrics;
  for (const auto & m : metrics_) {
    m->emit(metrics);
  }

  ScenarioResult result = evaluate_all(scenario_.name, scenario_.assertions, metrics);

  RCLCPP_INFO(
    this->get_logger(), "Scenario '%s' result: %s",
    scenario_.name.c_str(), result.passed ? "PASS" : "FAIL");
  for (const auto & a : result.assertions) {
    RCLCPP_INFO(
      this->get_logger(), "  [%s] %s %s %.3f (actual=%.3f found=%d)",
      a.passed ? "PASS" : "FAIL", a.metric.c_str(), to_string(a.op).c_str(),
      a.threshold, a.actual, a.found);
  }
  return result;
}

}  // namespace sensorforge::scenario
