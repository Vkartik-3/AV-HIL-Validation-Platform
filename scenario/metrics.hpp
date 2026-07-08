/*
==============================================================================
SensorForge - Per-stream metric accumulator (Extension J)
Part of the SensorForge AV HIL validation platform.

Collects latency samples and counts for one stream and derives the scenario
metrics (msgs/sec, drop_rate_pct, latency_p50/p99/p999_ms, sequence_gaps).
No ROS2 dependency; unit-testable standalone.
==============================================================================
*/

#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "scenario/assertion.hpp"   // MetricMap

namespace sensorforge::scenario {

class StreamMetrics
{
public:
  StreamMetrics(std::string name, double expected_rate_hz, double duration_seconds)
  : name_(std::move(name)),
    expected_rate_hz_(expected_rate_hz),
    duration_seconds_(duration_seconds) {}

  /// Record one received message: its end-to-end latency (ms) and a flag for
  /// whether a sequence gap was detected before it.
  void record(double latency_ms, bool gap_before = false)
  {
    latencies_ms_.push_back(latency_ms);
    ++received_;
    if (gap_before) {
      ++sequence_gaps_;
    }
  }

  void add_sequence_gaps(uint64_t n) {sequence_gaps_ += n;}

  double pct(double p) const
  {
    if (latencies_ms_.empty()) {
      return 0.0;
    }
    std::vector<double> s = latencies_ms_;
    std::sort(s.begin(), s.end());
    const size_t idx = static_cast<size_t>(p * (s.size() - 1));
    return s[idx];
  }

  double msgs_per_sec() const
  {
    return duration_seconds_ > 0 ? static_cast<double>(received_) / duration_seconds_ : 0.0;
  }

  double drop_rate_pct() const
  {
    const double expected = expected_rate_hz_ * duration_seconds_;
    if (expected <= 0.0) {
      return 0.0;
    }
    const double dropped = expected - static_cast<double>(received_);
    const double pct_dropped = 100.0 * (dropped > 0 ? dropped : 0.0) / expected;
    return pct_dropped;
  }

  /// Emit this stream's metrics into @p out under keys like
  /// "imu_latency_p99_ms", "camera_drop_rate_pct", "<name>_sequence_gaps".
  void emit(MetricMap & out) const
  {
    out[name_ + "_msgs_per_sec"] = msgs_per_sec();
    out[name_ + "_drop_rate_pct"] = drop_rate_pct();
    out[name_ + "_latency_p50_ms"] = pct(0.50);
    out[name_ + "_latency_p99_ms"] = pct(0.99);
    out[name_ + "_latency_p999_ms"] = pct(0.999);
    out[name_ + "_sequence_gaps"] = static_cast<double>(sequence_gaps_);
    out[name_ + "_received"] = static_cast<double>(received_);
  }

  const std::string & name() const {return name_;}
  uint64_t received() const {return received_;}
  uint64_t sequence_gaps() const {return sequence_gaps_;}

private:
  std::string name_;
  double expected_rate_hz_;
  double duration_seconds_;
  std::vector<double> latencies_ms_;
  uint64_t received_ = 0;
  uint64_t sequence_gaps_ = 0;
};

}  // namespace sensorforge::scenario
