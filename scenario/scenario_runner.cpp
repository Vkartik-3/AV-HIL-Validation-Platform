/*
==============================================================================
SensorForge - Scenario runner (implementation, Extension J)
Part of the SensorForge AV HIL validation platform.
==============================================================================
*/

#include "scenario/scenario_runner.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

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

// Map a parsed scenario fault to a fault-engine rule.
faults::FaultRule to_fault_rule(const FaultSpec & f)
{
  faults::FaultRule r;
  r.kind = faults::fault_kind_from_string(to_string(f.type));
  r.stream = f.stream;
  r.start_s = f.start_seconds;
  r.duration_s = f.duration_seconds;
  r.delay_ms = f.param("delay_ms");
  r.drop_rate = f.param("drop_rate");
  r.bandwidth_kbps = f.param("bandwidth_kbps");
  return r;
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

    // Build a fault engine for this stream if any fault targets it. Its write
    // callback records the (possibly delayed/duplicated) arrival's latency.
    const std::string stream = s.name;
    bool has_fault = false;
    for (const auto & f : scenario_.faults) {
      if (f.stream == stream) {has_fault = true;}
    }
    if (has_fault) {
      auto * sm = metrics_.back().get();
      auto engine = std::make_unique<faults::FaultEngine>(
        [sm](const std::vector<uint8_t> & buf) {
          if (buf.size() >= sizeof(double)) {
            double latency_ms = 0.0;
            std::memcpy(&latency_ms, buf.data(), sizeof(double));
            sm->record(latency_ms);
          }
        });
      for (const auto & f : scenario_.faults) {
        if (f.stream == stream) {engine->add_rule(to_fault_rule(f));}
      }
      stream_faults_.emplace(stream, std::move(engine));
    }

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

StreamMetrics * ScenarioRunner::metrics_for(const std::string & stream)
{
  for (auto & m : metrics_) {
    if (m->name() == stream) {
      return m.get();
    }
  }
  return nullptr;
}

void ScenarioRunner::record(const std::string & stream, const rclcpp::Time & stamp)
{
  const double latency_ms = (this->now() - stamp).seconds() * 1000.0;
  const auto it = stream_faults_.find(stream);
  if (it != stream_faults_.end()) {
    // Route the arrival through the fault engine (drop/delay/duplicate/...);
    // the engine's callback records the surviving arrivals.
    const double t_s = (this->now() - start_time_).seconds();
    std::vector<uint8_t> buf(sizeof(double));
    std::memcpy(buf.data(), &latency_ms, sizeof(double));
    it->second->process(buf, t_s, stream);
    return;
  }
  if (auto * sm = metrics_for(stream)) {
    sm->record(latency_ms);
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

  // Release any frames still held by fault-engine delay queues.
  for (auto & [stream, engine] : stream_faults_) {
    (void)stream;
    engine->flush();
  }

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
