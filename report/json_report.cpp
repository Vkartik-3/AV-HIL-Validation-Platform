/*
==============================================================================
SensorForge - JSON report serializer (implementation, Extension L)
==============================================================================
*/

#include "report/json_report.hpp"

#include <fstream>

#include <nlohmann/json.hpp>

namespace sensorforge::report {

using nlohmann::json;

std::string to_json(const ScenarioReport & r)
{
  json j;
  j["scenario"] = r.scenario;
  j["timestamp"] = r.timestamp;
  j["duration_seconds"] = r.duration_seconds;
  j["result"] = r.passed ? "PASS" : "FAIL";

  j["assertions"] = json::array();
  for (const auto & a : r.assertions) {
    j["assertions"].push_back({
        {"metric", a.metric},
        {"operator", a.op},
        {"threshold", a.threshold},
        {"actual", a.actual},
        {"found", a.found},
        {"result", a.passed ? "PASS" : "FAIL"},
      });
  }

  json per_sensor = json::object();
  for (const auto & [name, s] : r.per_sensor) {
    per_sensor[name] = {
      {"msgs_sec", s.msgs_sec},
      {"drop_rate_pct", s.drop_rate_pct},
      {"latency_p50_ms", s.latency_p50_ms},
      {"latency_p99_ms", s.latency_p99_ms},
      {"latency_p999_ms", s.latency_p999_ms},
      {"sequence_gaps", s.sequence_gaps},
      {"received", s.received},
    };
  }
  j["metrics"]["per_sensor"] = per_sensor;
  j["metrics"]["transport"] = {
    {"bytes_sec", r.transport.bytes_sec},
    {"crc_failures", r.transport.crc_failures},
    {"sequence_gaps", r.transport.sequence_gaps},
    {"retransmits", r.transport.retransmits},
  };
  j["replay_recovery_ms"] = r.replay_recovery_ms;

  if (!r.latency_histograms.empty()) {
    json hists = json::object();
    for (const auto & [name, buckets] : r.latency_histograms) {
      json arr = json::array();
      for (const auto & [upper, count] : buckets) {
        arr.push_back({{"le_ms", upper}, {"count", count}});
      }
      hists[name] = arr;
    }
    j["latency_histograms"] = hists;
  }

  return j.dump(2);
}

bool write_json(const ScenarioReport & r, const std::string & path)
{
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << to_json(r);
  return static_cast<bool>(out);
}

}  // namespace sensorforge::report
