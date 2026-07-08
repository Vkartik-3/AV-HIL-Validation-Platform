/*
==============================================================================
SensorForge - Scenario parser (implementation, Extension J)
Part of the SensorForge AV HIL validation platform.
==============================================================================
*/

#include "scenario/scenario.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace sensorforge::scenario {

FaultType fault_type_from_string(const std::string & s)
{
  if (s == "delay") {return FaultType::kDelay;}
  if (s == "drop") {return FaultType::kDrop;}
  if (s == "corrupt") {return FaultType::kCorrupt;}
  if (s == "reorder") {return FaultType::kReorder;}
  if (s == "duplicate") {return FaultType::kDuplicate;}
  if (s == "burst" || s == "burst_loss") {return FaultType::kBurstLoss;}
  if (s == "bandwidth_limit") {return FaultType::kBandwidthLimit;}
  if (s == "ecu_disconnect") {return FaultType::kEcuDisconnect;}
  if (s == "process_restart") {return FaultType::kProcessRestart;}
  return FaultType::kUnknown;
}

std::string to_string(FaultType t)
{
  switch (t) {
    case FaultType::kDelay: return "delay";
    case FaultType::kDrop: return "drop";
    case FaultType::kCorrupt: return "corrupt";
    case FaultType::kReorder: return "reorder";
    case FaultType::kDuplicate: return "duplicate";
    case FaultType::kBurstLoss: return "burst_loss";
    case FaultType::kBandwidthLimit: return "bandwidth_limit";
    case FaultType::kEcuDisconnect: return "ecu_disconnect";
    case FaultType::kProcessRestart: return "process_restart";
    case FaultType::kUnknown: return "unknown";
  }
  return "unknown";
}

CompareOp compare_op_from_string(const std::string & s)
{
  if (s == "less_than") {return CompareOp::kLessThan;}
  if (s == "greater_than") {return CompareOp::kGreaterThan;}
  if (s == "equals") {return CompareOp::kEquals;}
  if (s == "not_equals") {return CompareOp::kNotEquals;}
  return CompareOp::kUnknown;
}

std::string to_string(CompareOp op)
{
  switch (op) {
    case CompareOp::kLessThan: return "less_than";
    case CompareOp::kGreaterThan: return "greater_than";
    case CompareOp::kEquals: return "equals";
    case CompareOp::kNotEquals: return "not_equals";
    case CompareOp::kUnknown: return "unknown";
  }
  return "unknown";
}

namespace {

Scenario parse_node(const YAML::Node & root)
{
  Scenario s;
  if (!root["name"]) {
    throw std::runtime_error("scenario: missing required 'name'");
  }
  s.name = root["name"].as<std::string>();
  s.duration_seconds = root["duration_seconds"] ? root["duration_seconds"].as<double>() : 0.0;
  if (s.duration_seconds <= 0.0) {
    throw std::runtime_error("scenario: duration_seconds must be > 0");
  }

  if (root["streams"]) {
    for (const auto & kv : root["streams"]) {
      StreamConfig sc;
      sc.name = kv.first.as<std::string>();
      const YAML::Node & body = kv.second;
      sc.rate_hz = body["rate_hz"] ? body["rate_hz"].as<double>() : 0.0;
      if (sc.rate_hz <= 0.0) {
        throw std::runtime_error("scenario: stream '" + sc.name + "' rate_hz must be > 0");
      }
      s.streams.push_back(std::move(sc));
    }
  }

  if (root["faults"]) {
    for (const auto & f : root["faults"]) {
      FaultSpec fs;
      fs.stream = f["stream"] ? f["stream"].as<std::string>() : "";
      fs.start_seconds = f["start_seconds"] ? f["start_seconds"].as<double>() : 0.0;
      fs.duration_seconds = f["duration_seconds"] ? f["duration_seconds"].as<double>() : 0.0;
      const std::string type_str = f["type"] ? f["type"].as<std::string>() : "";
      fs.type = fault_type_from_string(type_str);
      if (fs.type == FaultType::kUnknown) {
        throw std::runtime_error("scenario: unknown fault type '" + type_str + "'");
      }
      // Accept both a nested 'params:' map and inline keys (delay_ms, etc).
      if (f["params"]) {
        for (const auto & p : f["params"]) {
          fs.params[p.first.as<std::string>()] = p.second.as<double>();
        }
      }
      for (const char * key : {"delay_ms", "drop_rate", "bandwidth_kbps", "burst_ms"}) {
        if (f[key]) {
          fs.params[key] = f[key].as<double>();
        }
      }
      s.faults.push_back(std::move(fs));
    }
  }

  if (root["assertions"]) {
    for (const auto & a : root["assertions"]) {
      Assertion as;
      as.metric = a["metric"] ? a["metric"].as<std::string>() : "";
      const std::string op_str = a["operator"] ? a["operator"].as<std::string>() : "";
      as.op = compare_op_from_string(op_str);
      if (as.metric.empty() || as.op == CompareOp::kUnknown) {
        throw std::runtime_error("scenario: assertion needs metric + valid operator");
      }
      as.value = a["value"] ? a["value"].as<double>() : 0.0;
      if (a["window_seconds"]) {
        as.window_seconds = a["window_seconds"].as<double>();
      }
      s.assertions.push_back(std::move(as));
    }
  }

  return s;
}

}  // namespace

Scenario parse_scenario_file(const std::string & path)
{
  return parse_node(YAML::LoadFile(path));
}

Scenario parse_scenario_string(const std::string & yaml)
{
  return parse_node(YAML::Load(yaml));
}

}  // namespace sensorforge::scenario
