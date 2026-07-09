/*
==============================================================================
SensorForge - Report builder (Extension L)
Assembles a ScenarioReport from a scenario result + metric map.
==============================================================================
*/

#include "report/report_types.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>

namespace sensorforge::report {

namespace {
double metric_or(const scenario::MetricMap & m, const std::string & key, double fallback = 0.0)
{
  const auto it = m.find(key);
  return it == m.end() ? fallback : it->second;
}
}  // namespace

std::string iso8601_now()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_utc{};
#if defined(_WIN32)
  gmtime_s(&tm_utc, &t);
#else
  gmtime_r(&t, &tm_utc);
#endif
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
  return buf;
}

ScenarioReport build_report(
  const scenario::ScenarioResult & result,
  const scenario::MetricMap & metrics,
  const std::vector<std::string> & streams,
  double duration_seconds)
{
  ScenarioReport r;
  r.scenario = result.scenario_name;
  r.timestamp = iso8601_now();
  r.duration_seconds = duration_seconds;
  r.passed = result.passed;

  for (const auto & a : result.assertions) {
    AssertionReport ar;
    ar.metric = a.metric;
    ar.op = scenario::to_string(a.op);
    ar.threshold = a.threshold;
    ar.actual = a.actual;
    ar.found = a.found;
    ar.passed = a.passed;
    r.assertions.push_back(std::move(ar));
  }

  for (const auto & s : streams) {
    SensorMetrics sm;
    sm.msgs_sec = metric_or(metrics, s + "_msgs_per_sec");
    sm.drop_rate_pct = metric_or(metrics, s + "_drop_rate_pct");
    sm.latency_p50_ms = metric_or(metrics, s + "_latency_p50_ms");
    sm.latency_p99_ms = metric_or(metrics, s + "_latency_p99_ms");
    sm.latency_p999_ms = metric_or(metrics, s + "_latency_p999_ms");
    sm.sequence_gaps = static_cast<uint64_t>(metric_or(metrics, s + "_sequence_gaps"));
    sm.received = static_cast<uint64_t>(metric_or(metrics, s + "_received"));
    r.per_sensor[s] = sm;
  }

  r.transport.bytes_sec = metric_or(metrics, "transport_bytes_sec");
  r.transport.crc_failures = static_cast<uint64_t>(metric_or(metrics, "transport_crc_failures"));
  r.transport.sequence_gaps = static_cast<uint64_t>(metric_or(metrics, "transport_sequence_gaps"));
  r.transport.retransmits = static_cast<uint64_t>(metric_or(metrics, "transport_retransmits"));
  r.replay_recovery_ms = metric_or(metrics, "replay_recovery_ms");

  return r;
}

}  // namespace sensorforge::report
