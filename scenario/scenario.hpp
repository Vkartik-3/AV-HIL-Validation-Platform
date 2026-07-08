/*
==============================================================================
SensorForge - Scenario data model + parser (Extension J)
Part of the SensorForge AV HIL validation platform.

Data model for a YAML scenario and a yaml-cpp parser. No ROS2 dependency, so
the model, parser and assertion evaluator are unit-testable standalone.

Scenario YAML:
  name: string
  duration_seconds: float
  streams:
    <sensor>: {rate_hz: float}
  faults:
    - stream: string
      start_seconds: float
      duration_seconds: float
      type: delay|drop|corrupt|reorder|duplicate|burst
      params: {delay_ms: float, drop_rate: float, ...}
  assertions:
    - metric: string
      operator: less_than|greater_than|equals|not_equals
      value: float
      window_seconds: float   # optional
==============================================================================
*/

#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace sensorforge::scenario {

struct StreamConfig
{
  std::string name;
  double rate_hz = 0.0;
};

enum class FaultType {
  kDelay,
  kDrop,
  kCorrupt,
  kReorder,
  kDuplicate,
  kBurstLoss,
  kBandwidthLimit,
  kEcuDisconnect,
  kProcessRestart,
  kUnknown,
};

FaultType fault_type_from_string(const std::string & s);
std::string to_string(FaultType t);

struct FaultSpec
{
  std::string stream;
  double start_seconds = 0.0;
  double duration_seconds = 0.0;
  FaultType type = FaultType::kUnknown;
  std::map<std::string, double> params;

  double param(const std::string & key, double fallback = 0.0) const
  {
    const auto it = params.find(key);
    return it == params.end() ? fallback : it->second;
  }
};

enum class CompareOp {
  kLessThan,
  kGreaterThan,
  kEquals,
  kNotEquals,
  kUnknown,
};

CompareOp compare_op_from_string(const std::string & s);
std::string to_string(CompareOp op);

struct Assertion
{
  std::string metric;
  CompareOp op = CompareOp::kUnknown;
  double value = 0.0;
  std::optional<double> window_seconds;
};

struct Scenario
{
  std::string name;
  double duration_seconds = 0.0;
  std::vector<StreamConfig> streams;
  std::vector<FaultSpec> faults;
  std::vector<Assertion> assertions;
};

/// Parse a scenario from a YAML file. Throws std::runtime_error on malformed
/// input (missing name, bad operator, etc.).
Scenario parse_scenario_file(const std::string & path);

/// Parse a scenario from a YAML string (used by tests/fuzzers).
Scenario parse_scenario_string(const std::string & yaml);

}  // namespace sensorforge::scenario
