/*
==============================================================================
SensorForge - Namespace manager + result aggregation (Extension O)
Part of the SensorForge AV HIL validation platform.

Vends unique ROS2 namespaces (/scenario_0, /scenario_1, ...) and unique metrics
ports for parallel scenario execution, and aggregates the per-run results into a
single summary (the "scale" signal: N scenarios x M assertions = N*M checks).
No ROS2 dependency; unit-testable standalone.
==============================================================================
*/

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "scenario/assertion.hpp"

namespace sensorforge::scenario {

class NamespaceManager
{
public:
  explicit NamespaceManager(std::string prefix = "/scenario", uint16_t base_port = 0)
  : prefix_(std::move(prefix)), base_port_(base_port) {}

  std::string next_namespace() {return prefix_ + "_" + std::to_string(counter_++);}

  /// Unique metrics port for run @p index, or 0 if metrics disabled (base_port 0).
  uint16_t metrics_port(size_t index) const
  {
    return base_port_ == 0 ? 0 : static_cast<uint16_t>(base_port_ + index);
  }

  size_t count() const {return counter_;}

private:
  std::string prefix_;
  uint16_t base_port_;
  size_t counter_ = 0;
};

struct AggregateResult
{
  size_t total_scenarios = 0;
  size_t scenarios_passed = 0;
  size_t scenarios_failed = 0;
  size_t total_assertions = 0;     // N x M
  size_t assertions_passed = 0;
  std::vector<std::pair<std::string, bool>> per_scenario;

  bool all_passed() const {return scenarios_failed == 0 && total_scenarios > 0;}
};

/// Aggregate a set of per-run results into a single summary.
inline AggregateResult aggregate(const std::vector<ScenarioResult> & results)
{
  AggregateResult agg;
  agg.total_scenarios = results.size();
  for (const auto & r : results) {
    if (r.passed) {
      ++agg.scenarios_passed;
    } else {
      ++agg.scenarios_failed;
    }
    agg.total_assertions += r.assertions.size();
    for (const auto & a : r.assertions) {
      if (a.passed) {
        ++agg.assertions_passed;
      }
    }
    agg.per_scenario.emplace_back(r.scenario_name, r.passed);
  }
  return agg;
}

}  // namespace sensorforge::scenario
