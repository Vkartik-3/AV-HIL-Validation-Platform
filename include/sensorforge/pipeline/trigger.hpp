/*
==============================================================================
SensorForge - Event-triggered selective recording
Part of the SensorForge AV HIL validation platform.

Baseline: record each stream decimated (every Nth frame). On a trigger:
flush a bounded PRE-ROLL of recently seen frames, then record every frame for a
POST-ROLL window.

Trigger sources are events the system already produces -- no perception model,
no anomaly detector:

  kSequenceGap     a receiver reported missing sequence numbers
  kCrcFailure      a frame failed CRC validation
  kLatencyHigh     capture-to-record latency crossed a threshold
  kQueuePressure   a stream hit its byte/frame ceiling
  kExternal        an operator or test raised it explicitly

Determinism under overlap: a trigger arriving while a window is already open
EXTENDS the window to max(current_end, now + post_roll). It never restarts,
never nests, and never shortens an open window, so any interleaving of triggers
produces the same window end for the same arrival times.

Pre-roll memory counts against the recorder's byte budget: the buffer is capped
in BYTES as well as frames, and its current size is reported so the recorder can
include it in the total queued bytes.
==============================================================================
*/

#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "sensorforge/core/sensor_frame.hpp"
#include "sensorforge/protocol/frame.hpp"

namespace sensorforge::pipeline {

enum class TriggerReason {
  kNone,
  kSequenceGap,
  kCrcFailure,
  kLatencyHigh,
  kQueuePressure,
  kExternal,
};

inline const char * to_string(TriggerReason r)
{
  switch (r) {
    case TriggerReason::kNone: return "none";
    case TriggerReason::kSequenceGap: return "sequence_gap";
    case TriggerReason::kCrcFailure: return "crc_failure";
    case TriggerReason::kLatencyHigh: return "latency_high";
    case TriggerReason::kQueuePressure: return "queue_pressure";
    case TriggerReason::kExternal: return "external";
  }
  return "unknown";
}

struct TriggerConfig
{
  double pre_roll_seconds = 2.0;
  double post_roll_seconds = 5.0;
  uint64_t pre_roll_max_bytes = 16u * 1024u * 1024u;
  size_t pre_roll_max_frames = 4096;
  /// Latency threshold (microseconds) for kLatencyHigh. 0 = disabled.
  uint64_t latency_trigger_us = 0;
};

struct TriggerEvent
{
  TriggerReason reason = TriggerReason::kNone;
  uint64_t mono_ns = 0;
  std::string detail;
};

/// One buffered pre-roll entry: the encoded frame plus what the WAL needs.
struct PreRollEntry
{
  size_t stream_id = 0;
  uint64_t timestamp_ns = 0;
  uint64_t sequence = 0;
  protocol::SensorType sensor_type = protocol::SensorType::kUnknown;
  uint64_t capture_mono_ns = 0;
  std::vector<uint8_t> payload;
};

/**
 * @brief Window state machine plus the bounded pre-roll ring.
 *
 * All methods are mutex-guarded and called from the drain thread plus (for
 * fire()) arbitrary threads. The mutex is NOT on the per-message capture path.
 */
class TriggerEngine
{
public:
  explicit TriggerEngine(TriggerConfig cfg)
  : cfg_(cfg) {}

  /// Raise a trigger. Extends an open window; never restarts or shortens it.
  void fire(TriggerReason reason, uint64_t mono_ns, const std::string & detail = "")
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const uint64_t end = mono_ns + static_cast<uint64_t>(cfg_.post_roll_seconds * 1e9);
    if (end > window_end_ns_) {
      window_end_ns_ = end;
    }
    ++activations_;
    last_ = TriggerEvent{reason, mono_ns, detail};
  }

  bool window_open(uint64_t mono_ns) const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return mono_ns < window_end_ns_;
  }

  /// Buffer a frame that baseline decimation would otherwise discard.
  void push_preroll(PreRollEntry && e)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    bytes_ += e.payload.size();
    buf_.push_back(std::move(e));
    while (buf_.size() > cfg_.pre_roll_max_frames ||
      bytes_ > cfg_.pre_roll_max_bytes)
    {
      if (buf_.empty()) {
        break;
      }
      bytes_ -= buf_.front().payload.size();
      buf_.pop_front();
      ++evicted_;
    }
  }

  /// Remove entries older than the pre-roll horizon relative to @p mono_ns.
  void expire(uint64_t mono_ns)
  {
    const uint64_t horizon = static_cast<uint64_t>(cfg_.pre_roll_seconds * 1e9);
    std::lock_guard<std::mutex> lk(mtx_);
    while (!buf_.empty() && buf_.front().capture_mono_ns + horizon < mono_ns) {
      bytes_ -= buf_.front().payload.size();
      buf_.pop_front();
      ++evicted_;
    }
  }

  /// Take everything buffered (called once when a window opens).
  std::vector<PreRollEntry> take_preroll()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<PreRollEntry> out;
    out.reserve(buf_.size());
    for (auto & e : buf_) {
      out.push_back(std::move(e));
    }
    buf_.clear();
    bytes_ = 0;
    return out;
  }

  uint64_t preroll_bytes() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return bytes_;
  }
  size_t preroll_frames() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return buf_.size();
  }
  uint64_t activations() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return activations_;
  }
  uint64_t evicted() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return evicted_;
  }
  TriggerEvent last() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return last_;
  }
  const TriggerConfig & config() const {return cfg_;}

private:
  mutable std::mutex mtx_;
  TriggerConfig cfg_;
  std::deque<PreRollEntry> buf_;
  uint64_t bytes_ = 0;
  uint64_t window_end_ns_ = 0;
  uint64_t activations_ = 0;
  uint64_t evicted_ = 0;
  TriggerEvent last_{};
};

}  // namespace sensorforge::pipeline
