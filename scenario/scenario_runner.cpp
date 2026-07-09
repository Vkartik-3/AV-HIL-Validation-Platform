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

#include <filesystem>

#include "report/report_types.hpp"
#include "report/json_report.hpp"
#include "report/html_report.hpp"

#if defined(__linux__)
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <spawn.h>
#include <sys/wait.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
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
  stop_can_reader();
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
          if (buf.size() >= 2 * sizeof(double)) {
            double latency_ms = 0.0;
            double arrival_s = 0.0;
            std::memcpy(&latency_ms, buf.data(), sizeof(double));
            std::memcpy(&arrival_s, buf.data() + sizeof(double), sizeof(double));
            sm->record(latency_ms, arrival_s);
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
  // Optional Prometheus exporter + 1 Hz gauge refresh.
  if (metrics_port_ != 0) {
    registry_.set_help("sensorforge_msgs_per_sec", "Messages per second per sensor");
    registry_.set_help("sensorforge_latency_p99_ms", "p99 end-to-end latency (ms) per sensor");
    exporter_ = std::make_unique<metrics::PrometheusExporter>(registry_, metrics_port_);
    if (exporter_->start()) {
      RCLCPP_INFO(this->get_logger(), "Prometheus /metrics on :%u", metrics_port_);
      metrics_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), [this]() {update_registry();});
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to start metrics exporter on :%u", metrics_port_);
      exporter_.reset();
    }
  }

  RCLCPP_INFO(
    this->get_logger(), "Scenario '%s' started: %zu streams, %.0fs",
    scenario_.name.c_str(), scenario_.streams.size(), scenario_.duration_seconds);
}

void ScenarioRunner::update_registry()
{
  for (const auto & m : metrics_) {
    MetricMap mm;
    m->emit(mm);
    const std::string & s = m->name();
    registry_.set_sensor_gauge("sensorforge_msgs_per_sec", s, mm[s + "_msgs_per_sec"]);
    registry_.set_sensor_gauge("sensorforge_drop_rate_pct", s, mm[s + "_drop_rate_pct"]);
    registry_.set_sensor_gauge("sensorforge_latency_p50_ms", s, mm[s + "_latency_p50_ms"]);
    registry_.set_sensor_gauge("sensorforge_latency_p99_ms", s, mm[s + "_latency_p99_ms"]);
    registry_.set_sensor_gauge("sensorforge_queue_occupancy", s, mm[s + "_received"]);
    registry_.set_sensor_gauge("sensorforge_sequence_gaps", s, mm[s + "_sequence_gaps"]);
  }
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
  const double now_s = (this->now() - start_time_).seconds();
  const double latency_ms = (this->now() - stamp).seconds() * 1000.0;
  const auto it = stream_faults_.find(stream);
  if (it != stream_faults_.end()) {
    // Route the arrival through the fault engine (drop/delay/duplicate/...);
    // the engine's callback records the surviving arrivals. Carry both the
    // latency and the arrival time so the consumer tracks the steady-state
    // window.
    std::vector<uint8_t> buf(2 * sizeof(double));
    std::memcpy(buf.data(), &latency_ms, sizeof(double));
    std::memcpy(buf.data() + sizeof(double), &now_s, sizeof(double));
    it->second->process(buf, now_s, stream);
    return;
  }
  if (auto * sm = metrics_for(stream)) {
    sm->record(latency_ms, now_s);
  }
}

void ScenarioRunner::record_direct(
  const std::string & stream, double latency_ms, double arrival_s)
{
  if (auto * sm = metrics_for(stream)) {
    sm->record(latency_ms, arrival_s);
  }
}

void ScenarioRunner::start_can_reader(const std::string & stream, const std::string & ifname)
{
#if defined(__linux__)
  // Resolve the target StreamMetrics on the main thread (no concurrent walk of
  // metrics_ from the reader thread).
  StreamMetrics * sm = metrics_for(stream);
  if (sm == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "CAN reader: no metrics for stream '%s'", stream.c_str());
    return;
  }
  can_reader_stop_.store(false);
  can_reader_thread_ = std::thread(
    [this, ifname, sm]() {can_reader_loop(ifname, sm);});
  RCLCPP_INFO(this->get_logger(), "CAN reader started on %s", ifname.c_str());
#else
  (void)stream;
  (void)ifname;
  RCLCPP_WARN(this->get_logger(), "SocketCAN reader is Linux-only");
#endif
}

void ScenarioRunner::stop_can_reader()
{
#if defined(__linux__)
  can_reader_stop_.store(true);
  if (can_fd_ >= 0) {
    ::shutdown(can_fd_, SHUT_RDWR);
  }
  if (can_reader_thread_.joinable()) {
    can_reader_thread_.join();
  }
  if (can_fd_ >= 0) {
    ::close(can_fd_);
    can_fd_ = -1;
  }
#endif
}

void ScenarioRunner::can_reader_loop(std::string ifname, StreamMetrics * metrics)
{
#if defined(__linux__)
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    RCLCPP_ERROR(this->get_logger(), "CAN reader: socket() failed: %s", std::strerror(errno));
    return;
  }
  can_fd_ = fd;

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "CAN reader: interface %s not found: %s",
      ifname.c_str(), std::strerror(errno));
    ::close(fd);
    can_fd_ = -1;
    return;
  }

  // Ensure locally-transmitted frames are looped back to this (separate)
  // socket, so we see the can_publisher's frames on vcan. Loopback is on by
  // default; set it explicitly for robustness across environments.
  const int loopback = 1;
  ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "CAN reader: bind failed on %s: %s",
      ifname.c_str(), std::strerror(errno));
    ::close(fd);
    can_fd_ = -1;
    return;
  }

  // Read timeout so the loop periodically re-checks can_reader_stop_.
  struct timeval tv{0, 100000};   // 100 ms
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  RCLCPP_INFO(
    this->get_logger(), "CAN reader listening on %s (ifindex=%d)",
    ifname.c_str(), ifr.ifr_ifindex);

  bool have_prev_seq = false;
  uint16_t prev_seq = 0;
  uint64_t frames = 0;
  while (!can_reader_stop_.load()) {
    struct can_frame frame;
    const ssize_t n = ::read(fd, &frame, sizeof(frame));
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;   // read timeout / interrupted -> re-check stop flag
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "CAN reader: read error: %s", std::strerror(errno));
      continue;
    }
    if (n < static_cast<ssize_t>(sizeof(frame))) {
      continue;   // short read (should not happen for classic CAN)
    }
    ++frames;

    // Decode embedded seq (data[2..3]) + monotonic-us timestamp (data[4..7]).
    const uint16_t seq = static_cast<uint16_t>(frame.data[2] | (frame.data[3] << 8));
    const uint32_t sent_us = static_cast<uint32_t>(frame.data[4]) |
      (static_cast<uint32_t>(frame.data[5]) << 8) |
      (static_cast<uint32_t>(frame.data[6]) << 16) |
      (static_cast<uint32_t>(frame.data[7]) << 24);
    const uint32_t now_us = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() & 0xFFFFFFFF);
    const uint32_t latency_us = now_us - sent_us;   // wraps correctly (uint32)
    const double latency_ms = static_cast<double>(latency_us) / 1000.0;
    const double arrival_s = (this->now() - start_time_).seconds();

    bool gap = false;
    if (have_prev_seq) {
      const uint16_t expected = static_cast<uint16_t>(prev_seq + 1);
      if (seq != expected) {
        gap = true;
      }
    }
    have_prev_seq = true;
    prev_seq = seq;

    metrics->record(latency_ms, arrival_s, gap);   // captured pointer, no lookup
  }
  RCLCPP_INFO(this->get_logger(), "CAN reader stopped: %llu frames received",
    static_cast<unsigned long long>(frames));
#else
  (void)ifname;
  (void)metrics;
#endif
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
    // CAN has no ROS topic (it is a raw vcan bus). Start a SocketCAN reader
    // thread that measures the bus directly.
    start_can_reader(s.name, "vcan0");
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown stream '%s' - no subscription", s.name.c_str());
  }
}

ScenarioResult ScenarioRunner::finish()
{
  // Stop the CAN reader first so the CAN metrics are complete + race-free
  // before we read them.
  stop_can_reader();

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

  // Final metric refresh + scenario pass/fail counters (Extension M).
  if (exporter_) {
    update_registry();
    registry_.add_counter(
      result.passed ? "sensorforge_scenarios_passed_total" : "sensorforge_scenarios_failed_total",
      1.0);
  }

  // Structured reports (Extension L).
  if (!report_dir_.empty()) {
    std::vector<std::string> stream_names;
    for (const auto & s : scenario_.streams) {stream_names.push_back(s.name);}
    const auto rep = report::build_report(
      result, metrics, stream_names, scenario_.duration_seconds);
    std::error_code ec;
    std::filesystem::create_directories(report_dir_, ec);
    const std::string base = report_dir_ + "/" + scenario_.name;
    report::write_json(rep, base + ".json");
    report::write_html(rep, base + ".html");
    RCLCPP_INFO(this->get_logger(), "Wrote report %s.{json,html}", base.c_str());
  }

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
