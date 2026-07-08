/*
==============================================================================
SensorForge - Scenario runner executable (Extension J)
Part of the SensorForge AV HIL validation platform.

Usage:
  ros2 run network_bridge scenario_runner --scenario <path.yaml> [--ns /scenario]

Parses the scenario, runs it for duration_seconds, prints the pass/fail summary,
and exits non-zero if any assertion failed (so CI can gate on it).
==============================================================================
*/

#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "scenario/scenario.hpp"
#include "scenario/scenario_runner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string scenario_path;
  std::string ns = "/scenario";
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
      scenario_path = argv[++i];
    } else if (std::strcmp(argv[i], "--ns") == 0 && i + 1 < argc) {
      ns = argv[++i];
    }
  }
  if (scenario_path.empty()) {
    std::fprintf(stderr, "usage: scenario_runner --scenario <path.yaml> [--ns /scenario]\n");
    rclcpp::shutdown();
    return 2;
  }

  sensorforge::scenario::Scenario scenario;
  try {
    scenario = sensorforge::scenario::parse_scenario_file(scenario_path);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "Failed to parse scenario: %s\n", e.what());
    rclcpp::shutdown();
    return 2;
  }

  auto runner = std::make_shared<sensorforge::scenario::ScenarioRunner>(scenario, ns);
  runner->start();

  // Spin until the scenario duration elapses.
  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(runner->duration_seconds()));
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(runner);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some(std::chrono::milliseconds(10));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const auto result = runner->finish();
  std::printf(
    "[SCENARIO] %s : %s\n", result.scenario_name.c_str(),
    result.passed ? "PASS" : "FAIL");

  runner.reset();
  rclcpp::shutdown();
  return result.passed ? 0 : 1;
}
