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

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "scenario/assertion.hpp"
#include "scenario/metrics.hpp"
#include "scenario/scenario.hpp"

namespace sensorforge::scenario {

class FaultEngine;  // Extension K (optional; forward-declared)

class ScenarioRunner : public rclcpp::Node
{
public:
  ScenarioRunner(Scenario scenario, std::string ns = "/scenario");
  ~ScenarioRunner() override;

  /// Launch publishers + subscriptions and record the start time.
  void start();

  /// Stop publishers, compute metrics, evaluate assertions.
  ScenarioResult finish();

  /// Optional fault engine (Extension K) applied to received messages.
  void set_fault_engine(std::shared_ptr<FaultEngine> engine) {fault_engine_ = std::move(engine);}

  double duration_seconds() const {return scenario_.duration_seconds;}
  const Scenario & scenario() const {return scenario_;}

private:
  void spawn_publisher(const StreamConfig & s);
  void subscribe_stream(const StreamConfig & s);
  void record(const std::string & stream, const rclcpp::Time & stamp);

  Scenario scenario_;
  std::string ns_;
  rclcpp::Time start_time_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::vector<std::unique_ptr<StreamMetrics>> metrics_;
  std::vector<int> child_pids_;
  std::shared_ptr<FaultEngine> fault_engine_;
};

}  // namespace sensorforge::scenario
