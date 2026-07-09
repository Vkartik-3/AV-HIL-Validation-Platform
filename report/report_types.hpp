/*
==============================================================================
SensorForge - Structured test report types (Extension L)
Part of the SensorForge AV HIL validation platform.

The data model for a per-scenario report, plus a builder that assembles it from
a scenario result + metric map. Serializers live in json_report / html_report.
No ROS2 dependency; unit-testable standalone.
==============================================================================
*/

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "scenario/assertion.hpp"

namespace sensorforge::report {

struct AssertionReport
{
  std::string metric;
  std::string op;
  double threshold = 0.0;
  double actual = 0.0;
  bool found = false;
  bool passed = false;
};

struct SensorMetrics
{
  double msgs_sec = 0.0;
  double drop_rate_pct = 0.0;
  double latency_p50_ms = 0.0;
  double latency_p99_ms = 0.0;
  double latency_p999_ms = 0.0;
  uint64_t sequence_gaps = 0;
  uint64_t received = 0;
};

struct TransportMetrics
{
  double bytes_sec = 0.0;
  uint64_t crc_failures = 0;
  uint64_t sequence_gaps = 0;
  uint64_t retransmits = 0;
};

struct ScenarioReport
{
  std::string scenario;
  std::string timestamp;          // ISO-8601 UTC
  double duration_seconds = 0.0;
  bool passed = false;

  std::vector<AssertionReport> assertions;
  std::map<std::string, SensorMetrics> per_sensor;   // keyed by stream name
  TransportMetrics transport;

  // Per-sensor latency histogram: (bucket_upper_ms, count).
  std::map<std::string, std::vector<std::pair<double, uint64_t>>> latency_histograms;

  double replay_recovery_ms = 0.0;
};

/// Build a report from a scenario result + collected metric map. @p streams is
/// the list of stream names to extract per-sensor metrics for.
ScenarioReport build_report(
  const scenario::ScenarioResult & result,
  const scenario::MetricMap & metrics,
  const std::vector<std::string> & streams,
  double duration_seconds);

/// Current UTC time as ISO-8601 (e.g. 2026-07-08T10:00:00Z).
std::string iso8601_now();

}  // namespace sensorforge::report
