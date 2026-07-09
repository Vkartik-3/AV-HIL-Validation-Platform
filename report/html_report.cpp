/*
==============================================================================
SensorForge - HTML report generator (implementation, Extension L)
Self-contained HTML: a summary banner, an assertions table, a per-sensor
metrics table, and inline-SVG latency bars. No external assets or JS.
==============================================================================
*/

#include "report/html_report.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace sensorforge::report {

namespace {

std::string esc(const std::string & s)
{
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      default: out += c;
    }
  }
  return out;
}

std::string fmt(double v)
{
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(3);
  os << v;
  return os.str();
}

}  // namespace

std::string to_html(const ScenarioReport & r)
{
  std::ostringstream h;
  const std::string status = r.passed ? "PASS" : "FAIL";
  const std::string color = r.passed ? "#1b8a3a" : "#c0392b";

  h << "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">"
    << "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    << "<title>SensorForge report - " << esc(r.scenario) << "</title><style>"
    << "body{font-family:-apple-system,Segoe UI,Roboto,sans-serif;margin:0;background:#0f1115;color:#e6e6e6}"
    << ".wrap{max-width:900px;margin:0 auto;padding:24px}"
    << ".banner{padding:16px 20px;border-radius:10px;color:#fff;font-weight:600;font-size:20px;background:"
    << color << "}"
    << "h1{font-size:22px;margin:8px 0}h2{font-size:16px;margin:24px 0 8px;color:#9fb0c0}"
    << "table{width:100%;border-collapse:collapse;font-size:14px;margin-top:6px}"
    << "th,td{text-align:left;padding:8px 10px;border-bottom:1px solid #232833}"
    << "th{color:#9fb0c0;font-weight:600}"
    << ".pass{color:#38d160}.fail{color:#ff6b6b}.muted{color:#8a94a3}"
    << ".bar{height:10px;background:#38d160;border-radius:3px}"
    << "</style></head><body><div class=\"wrap\">";

  h << "<h1>SensorForge Scenario Report</h1>"
    << "<div class=\"banner\">" << esc(r.scenario) << " &mdash; " << status << "</div>"
    << "<p class=\"muted\">" << esc(r.timestamp) << " &middot; duration "
    << fmt(r.duration_seconds) << " s</p>";

  // Assertions.
  h << "<h2>Assertions</h2><table><tr><th>Metric</th><th>Operator</th>"
    << "<th>Threshold</th><th>Actual</th><th>Result</th></tr>";
  for (const auto & a : r.assertions) {
    h << "<tr><td>" << esc(a.metric) << "</td><td>" << esc(a.op) << "</td><td>"
      << fmt(a.threshold) << "</td><td>" << (a.found ? fmt(a.actual) : "&mdash;")
      << "</td><td class=\"" << (a.passed ? "pass\">PASS" : "fail\">FAIL") << "</td></tr>";
  }
  h << "</table>";

  // Per-sensor metrics with a p99 latency bar.
  double max_p99 = 1.0;
  for (const auto & [name, s] : r.per_sensor) {max_p99 = std::max(max_p99, s.latency_p99_ms);}
  h << "<h2>Per-sensor metrics</h2><table><tr><th>Sensor</th><th>msgs/s</th>"
    << "<th>drop %</th><th>p50 ms</th><th>p99 ms</th><th>gaps</th><th>p99 latency</th></tr>";
  for (const auto & [name, s] : r.per_sensor) {
    const int w = static_cast<int>(100.0 * s.latency_p99_ms / max_p99);
    h << "<tr><td>" << esc(name) << "</td><td>" << fmt(s.msgs_sec) << "</td><td>"
      << fmt(s.drop_rate_pct) << "</td><td>" << fmt(s.latency_p50_ms) << "</td><td>"
      << fmt(s.latency_p99_ms) << "</td><td>" << s.sequence_gaps << "</td>"
      << "<td><div class=\"bar\" style=\"width:" << w << "%\"></div></td></tr>";
  }
  h << "</table>";

  // Transport.
  h << "<h2>Transport</h2><table>"
    << "<tr><th>bytes/s</th><th>CRC failures</th><th>sequence gaps</th><th>retransmits</th></tr><tr><td>"
    << fmt(r.transport.bytes_sec) << "</td><td>" << r.transport.crc_failures << "</td><td>"
    << r.transport.sequence_gaps << "</td><td>" << r.transport.retransmits << "</td></tr></table>";

  h << "</div></body></html>";
  return h.str();
}

bool write_html(const ScenarioReport & r, const std::string & path)
{
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << to_html(r);
  return static_cast<bool>(out);
}

}  // namespace sensorforge::report
