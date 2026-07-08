/*
==============================================================================
SensorForge - Assertion evaluation (Extension J)
Part of the SensorForge AV HIL validation platform.

Evaluates scenario assertions against a collected metric map. No ROS2
dependency; unit-testable standalone.
==============================================================================
*/

#pragma once

#include <map>
#include <string>
#include <vector>

#include "scenario/scenario.hpp"

namespace sensorforge::scenario {

using MetricMap = std::map<std::string, double>;

struct AssertionResult
{
  std::string metric;
  CompareOp op = CompareOp::kUnknown;
  double threshold = 0.0;
  double actual = 0.0;
  bool found = false;   // was the metric present in the map?
  bool passed = false;
};

/// Evaluate one assertion against @p metrics. A missing metric fails.
AssertionResult evaluate(const Assertion & a, const MetricMap & metrics);

/// Evaluate all assertions; overall pass = all individual passes.
struct ScenarioResult
{
  std::string scenario_name;
  bool passed = false;
  std::vector<AssertionResult> assertions;
};

ScenarioResult evaluate_all(
  const std::string & scenario_name,
  const std::vector<Assertion> & assertions, const MetricMap & metrics);

}  // namespace sensorforge::scenario
