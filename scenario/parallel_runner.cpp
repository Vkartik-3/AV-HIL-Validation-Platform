/*
==============================================================================
SensorForge - Parallel scenario runner (Extension O)
Part of the SensorForge AV HIL validation platform.

Runs N scenarios simultaneously, each in its own ROS2 namespace
(/scenario_0, /scenario_1, ...), under a multi-threaded executor, then
aggregates the results: total_pass / total_fail / per-scenario, and the total
check count (N scenarios x M assertions). Exits non-zero if any scenario failed.

Usage:
  ros2 run network_bridge parallel_runner \
      --scenario a.yaml --scenario b.yaml [--count K] \
      [--report-dir DIR] [--metrics-base-port 9090]

  --count K repeats the given scenario list K times (K*len(list) runs total).
==============================================================================
*/

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "scenario/namespace_manager.hpp"
#include "scenario/scenario.hpp"
#include "scenario/scenario_runner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::vector<std::string> scenario_paths;
  std::string report_dir;
  uint16_t metrics_base_port = 0;
  int count = 1;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
      scenario_paths.emplace_back(argv[++i]);
    } else if (std::strcmp(argv[i], "--report-dir") == 0 && i + 1 < argc) {
      report_dir = argv[++i];
    } else if (std::strcmp(argv[i], "--metrics-base-port") == 0 && i + 1 < argc) {
      metrics_base_port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--count") == 0 && i + 1 < argc) {
      count = std::max(1, std::atoi(argv[++i]));
    }
  }
  if (scenario_paths.empty()) {
    std::fprintf(stderr, "usage: parallel_runner --scenario <p.yaml> [--scenario ...] "
      "[--count K] [--report-dir DIR] [--metrics-base-port N]\n");
    rclcpp::shutdown();
    return 2;
  }

  using namespace sensorforge::scenario;
  NamespaceManager ns_mgr("/scenario", metrics_base_port);

  // Parse each scenario once; build N runners across distinct namespaces.
  std::vector<std::shared_ptr<ScenarioRunner>> runners;
  double max_duration = 0.0;
  for (int rep = 0; rep < count; ++rep) {
    for (const auto & path : scenario_paths) {
      Scenario s;
      try {
        s = parse_scenario_file(path);
      } catch (const std::exception & e) {
        std::fprintf(stderr, "Failed to parse %s: %s\n", path.c_str(), e.what());
        rclcpp::shutdown();
        return 2;
      }
      const size_t idx = runners.size();
      auto runner = std::make_shared<ScenarioRunner>(s, ns_mgr.next_namespace());
      if (!report_dir.empty()) {runner->set_report_dir(report_dir);}
      const uint16_t port = ns_mgr.metrics_port(idx);
      if (port != 0) {runner->set_metrics_port(port);}
      max_duration = std::max(max_duration, s.duration_seconds);
      runners.push_back(std::move(runner));
    }
  }

  std::printf("[PARALLEL] launching %zu scenarios concurrently\n", runners.size());

  rclcpp::executors::MultiThreadedExecutor exec;
  for (auto & r : runners) {
    r->start();
    exec.add_node(r);
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(max_duration));
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some(std::chrono::milliseconds(20));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::vector<ScenarioResult> results;
  for (auto & r : runners) {
    results.push_back(r->finish());
  }
  const AggregateResult agg = aggregate(results);

  std::printf("\n===== SensorForge parallel run summary =====\n");
  std::printf("scenarios: %zu   passed: %zu   failed: %zu\n",
    agg.total_scenarios, agg.scenarios_passed, agg.scenarios_failed);
  std::printf("total checks: %zu (%zu passed)   [N scenarios x M assertions]\n",
    agg.total_assertions, agg.assertions_passed);
  for (const auto & [name, passed] : agg.per_scenario) {
    std::printf("  [%s] %s\n", passed ? "PASS" : "FAIL", name.c_str());
  }

  for (auto & r : runners) {exec.remove_node(r);}
  runners.clear();
  rclcpp::shutdown();
  return agg.all_passed() ? 0 : 1;
}
