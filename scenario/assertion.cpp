/*
==============================================================================
SensorForge - Assertion evaluation (implementation, Extension J)
Part of the SensorForge AV HIL validation platform.
==============================================================================
*/

#include "scenario/assertion.hpp"

#include <cmath>

namespace sensorforge::scenario {

namespace {
// Tolerance for the equals/not_equals comparisons on floating metrics.
constexpr double kEpsilon = 1e-9;
}  // namespace

AssertionResult evaluate(const Assertion & a, const MetricMap & metrics)
{
  AssertionResult r;
  r.metric = a.metric;
  r.op = a.op;
  r.threshold = a.value;

  const auto it = metrics.find(a.metric);
  if (it == metrics.end()) {
    r.found = false;
    r.passed = false;   // a missing metric is a failure, not a silent pass
    return r;
  }
  r.found = true;
  r.actual = it->second;

  switch (a.op) {
    case CompareOp::kLessThan:
      r.passed = r.actual < a.value;
      break;
    case CompareOp::kGreaterThan:
      r.passed = r.actual > a.value;
      break;
    case CompareOp::kEquals:
      r.passed = std::fabs(r.actual - a.value) <= kEpsilon;
      break;
    case CompareOp::kNotEquals:
      r.passed = std::fabs(r.actual - a.value) > kEpsilon;
      break;
    case CompareOp::kUnknown:
      r.passed = false;
      break;
  }
  return r;
}

ScenarioResult evaluate_all(
  const std::string & scenario_name,
  const std::vector<Assertion> & assertions, const MetricMap & metrics)
{
  ScenarioResult sr;
  sr.scenario_name = scenario_name;
  sr.passed = true;
  for (const auto & a : assertions) {
    AssertionResult ar = evaluate(a, metrics);
    if (!ar.passed) {
      sr.passed = false;
    }
    sr.assertions.push_back(std::move(ar));
  }
  return sr;
}

}  // namespace sensorforge::scenario
