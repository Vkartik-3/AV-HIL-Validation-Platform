/*
==============================================================================
SensorForge - Scenario runner (Extension J)
Part of the SensorForge AV HIL validation platform.

A ROS2 node that executes a parsed Scenario:
  1. launches the configured synthetic sensor publishers as child processes
     (in a per-run namespace so scenarios can run in parallel -- Extension O),
  2. subscribes to each stream and measures latency / count / gaps,
  3. runs for duration_seconds,
  4. derives metrics and evaluates the scenario assertions.

Fault injection (Extension K) is applied at ingest via an optional FaultEngine
hook set with set_fault_engine(); without it, streams are measured unperturbed.
==============================================================================
*/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "scenario/assertion.hpp"
#include "scenario/metrics.hpp"
#include "scenario/scenario.hpp"
#include "faults/fault_engine.hpp"

namespace sensorforge::scenario {

class ScenarioRunner : public rclcpp::Node
{
public:
  ScenarioRunner(Scenario scenario, std::string ns = "/scenario");
  ~ScenarioRunner() override;

  /// Launch publishers + subscriptions and record the start time.
  void start();

  /// Stop publishers, compute metrics, evaluate assertions.
  ScenarioResult finish();

  double duration_seconds() const {return scenario_.duration_seconds;}
  const Scenario & scenario() const {return scenario_;}

  /// Directory to write JSON + HTML reports into (empty = disabled).
  void set_report_dir(std::string dir) {report_dir_ = std::move(dir);}

private:
  void spawn_publisher(const StreamConfig & s);
  void subscribe_stream(const StreamConfig & s);
  void record(const std::string & stream, const rclcpp::Time & stamp);

  Scenario scenario_;
  std::string ns_;
  rclcpp::Time start_time_;

  StreamMetrics * metrics_for(const std::string & stream);

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::vector<std::unique_ptr<StreamMetrics>> metrics_;
  std::vector<int> child_pids_;

  // Per-stream fault engines (Extension K), built from scenario_.faults. A
  // received message is routed through its stream's engine, which may drop /
  // delay / duplicate it before the metric is recorded -- simulating the
  // transport-layer fault from the consumer's perspective.
  std::map<std::string, std::unique_ptr<faults::FaultEngine>> stream_faults_;

  std::string report_dir_;
};

}  // namespace sensorforge::scenario
