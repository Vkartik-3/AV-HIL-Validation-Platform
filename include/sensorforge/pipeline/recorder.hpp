/*
==============================================================================
SensorForge - Multi-stream recorder
Part of the SensorForge AV HIL validation platform.

THE GAP THIS CLOSES
-------------------
The audit's central finding was that the repository contained two systems that
barely touched: a well-tested ROS-free core (ring, frame codec, CRC, WAL) and a
ROS2 HIL layer whose measurement path never traversed it. Sensor data went
publisher -> DDS -> scenario runner, so no frame was ever encoded, no CRC ever
computed, no ring ever pushed and no WAL record ever written on the path that
was actually measured.

Recorder is the missing assembly. It owns N sensor streams and drives every one
of them through the real path:

    capture(stream, bytes)                     [producer thread, per stream]
      -> per-stream sequence + monotonised timestamp
      -> recording policy (allow/deny/metadata-only)
      -> StreamBuffer  (bounded by FRAMES and BYTES, per-stream policy)
      ...
    drain thread
      -> pop
      -> encode_frame()  (magic/version/seq/timestamp + CRC32C header+payload)
      -> selective-recording gate (pre-roll / trigger / post-roll)
      -> WalWriter::append()   (segmented, CRC'd, restart-safe)
      -> optional sink (fault engine / transport)

Every stream carries its own sensor type, sequence counter, buffering policy,
byte accounting and recording counters, which is what the brief asked for and
what the previous SubscriptionManager could not express (it hard-coded one
policy, one compile-time capacity, and never called set_sensor_type at all).

PERFORMANCE CHARACTER
--------------------
This is a performance-sensitive pipeline, not a hard real-time one. The ring is
allocation-free and the capture path never blocks unboundedly, but the pipeline
around it does allocate (the captured payload copy and the encoded frame), the
drain thread is an ordinary thread with no priority, and no deadline is
enforced anywhere. What IS bounded is queue depth, queued bytes and memory --
by frames, by bytes, and by the resource budget, all observable at run time.
Latency figures in the artifacts are measured typical behaviour, not guarantees.

THREADING
---------
capture() is called from that stream's producer thread (one per stream). A
single drain thread consumes all streams. Counters are atomics; the drain loop
takes no lock on the capture path.
==============================================================================
*/

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "sensorforge/core/backpressure_policy.hpp"
#include "sensorforge/core/clock.hpp"
#include "sensorforge/core/resource_monitor.hpp"
#include "sensorforge/core/sensor_frame.hpp"
#include "sensorforge/core/stream_buffer.hpp"
#include "sensorforge/pipeline/recording_policy.hpp"
#include "sensorforge/pipeline/trigger.hpp"
#include "sensorforge/protocol/frame.hpp"
#include "sensorforge/replay/wal_writer.hpp"

namespace sensorforge::pipeline {

using core::BackpressurePolicy;
using core::SensorFrame;
using core::StreamLimits;
using protocol::SensorType;

/// Per-stream configuration. All of it is runtime, not compile-time.
struct StreamSpec
{
  std::string name;
  SensorType sensor_type = SensorType::kUnknown;
  BackpressurePolicy policy = BackpressurePolicy::kDropNewest;
  StreamLimits limits{};
  /// Fraction of frames recorded when no trigger is active (1 = every frame,
  /// 10 = every 10th). Selective recording; see trigger.hpp.
  uint32_t baseline_decimation = 1;
};

struct RecorderConfig
{
  std::string wal_dir;                       // empty = do not record to disk
  replay::WalConfig wal{};
  core::ResourceBudget budget{};
  RecordingPolicy recording_policy{};
  TriggerConfig trigger{};
  bool enable_triggers = false;
};

/// Latency histogram over capture -> durable, in microseconds.
class LatencyHistogram
{
public:
  void add(uint64_t us)
  {
    ++count_;
    sum_us_ += us;
    if (us > max_us_) {max_us_ = us;}
    samples_.push_back(us);
  }
  uint64_t count() const {return count_;}
  double mean_us() const {return count_ ? static_cast<double>(sum_us_) / count_ : 0.0;}
  uint64_t max_us() const {return max_us_;}
  /// Percentile in microseconds. Sorts a copy; call off the hot path.
  double pct_us(double p) const;
  void clear() {samples_.clear(); count_ = 0; sum_us_ = 0; max_us_ = 0;}

private:
  std::vector<uint64_t> samples_;
  uint64_t count_ = 0;
  uint64_t sum_us_ = 0;
  uint64_t max_us_ = 0;
};

struct StreamStats
{
  std::string name;
  SensorType sensor_type = SensorType::kUnknown;
  std::string policy;
  core::StreamCounters buffer;
  uint64_t captured = 0;
  uint64_t framed = 0;          // frames encoded by the drain thread
  uint64_t recorded = 0;        // records appended to the WAL
  uint64_t skipped_decimation = 0;
  uint64_t denied_by_policy = 0;
  uint64_t metadata_only = 0;
  uint64_t frame_bytes = 0;     // bytes handed to the transport/WAL
  uint64_t sequence = 0;        // next sequence to be assigned
  uint64_t clock_regressions = 0;
};

struct RecorderStats
{
  std::vector<StreamStats> streams;
  uint64_t total_captured = 0;
  uint64_t total_recorded = 0;
  uint64_t total_dropped = 0;
  uint64_t total_overwritten = 0;
  uint64_t total_frame_bytes = 0;
  uint64_t wal_records = 0;
  uint64_t wal_bytes = 0;
  uint64_t wal_fsyncs = 0;
  uint64_t queued_bytes = 0;
  uint64_t queued_frames = 0;
  uint64_t peak_queued_bytes = 0;
  uint64_t shed_events = 0;       // hard-budget breaches that forced shedding
  uint64_t soft_breaches = 0;
  uint64_t trigger_activations = 0;
  uint64_t preroll_flushed = 0;
  uint64_t bytes_saved_vs_always_on = 0;
  core::ResourceSample resources;
  uint64_t peak_rss_bytes = 0;
  double peak_cpu_percent = 0.0;
  core::BudgetState budget_state = core::BudgetState::kOk;
  double p50_capture_to_record_us = 0.0;
  double p99_capture_to_record_us = 0.0;
  double p999_capture_to_record_us = 0.0;
  uint64_t latency_samples = 0;
};

class Recorder
{
public:
  /// Sink invoked for each encoded frame (transport write / fault engine).
  /// Called on the drain thread. Optional.
  using FrameSink = std::function<void (const std::string & stream,
      SensorType type, const std::vector<uint8_t> & frame)>;

  explicit Recorder(RecorderConfig config);
  ~Recorder();

  Recorder(const Recorder &) = delete;
  Recorder & operator=(const Recorder &) = delete;

  /// Register a stream. Must be called before start(). Returns the stream id.
  size_t add_stream(const StreamSpec & spec);

  /// Look up a stream id by name; SIZE_MAX if unknown.
  size_t stream_id(const std::string & name) const;

  void set_frame_sink(FrameSink sink) {sink_ = std::move(sink);}

  /**
   * @brief Hook called with each WAL segment as it is SEALED.
   *
   * This is the offload integration point: a sealed segment is immutable,
   * CRC-verified and named by a monotonic id, so handing its path to an
   * Uploader is all offload needs. Must be set before start(); the hook runs on
   * the drain thread and must not block (Uploader::enqueue does not).
   */
  void set_segment_sealed_hook(std::function<void (const std::string &, uint32_t)> fn)
  {
    sealed_hook_ = std::move(fn);
  }

  /// Start the drain thread (and the WAL, if configured).
  void start();

  /// Drain everything still queued, stop the thread, close the WAL.
  void stop();

  /**
   * @brief Producer: submit one captured message for @p id.
   *
   * Assigns the stream's next sequence and a monotonised wall-clock timestamp,
   * then applies the stream's backpressure policy. Never blocks unbounded.
   */
  core::PushResult capture(size_t id, const uint8_t * data, size_t len);

  /// Convenience overload.
  core::PushResult capture(size_t id, const std::vector<uint8_t> & data)
  {
    return capture(id, data.data(), data.size());
  }

  /// Raise an external trigger (see trigger.hpp). Safe from any thread.
  void fire_trigger(TriggerReason reason, const std::string & detail = "");

  /// Snapshot of all counters. Safe to call while running.
  RecorderStats stats() const;

  /// Sample resources and re-evaluate the budget. Call ~1 Hz, not per message.
  void poll_resources();

  const replay::RecoveryReport * wal_recovery() const;

private:
  struct Stream;

  void drain_loop();
  /// Returns true if the frame should be written to the WAL right now.
  bool should_record(Stream & s, uint64_t seq);
  void handle_frame(Stream & s, SensorFrame & frame);
  void write_record(Stream & s, const SensorFrame & frame,
    const std::vector<uint8_t> & encoded);

  RecorderConfig config_;
  std::vector<std::unique_ptr<Stream>> streams_;
  std::unique_ptr<replay::WalWriter> wal_;
  FrameSink sink_;
  std::function<void (const std::string &, uint32_t)> sealed_hook_;

  std::thread drain_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};

  // Budget / shedding.
  std::atomic<uint64_t> shed_events_{0};
  std::atomic<uint64_t> soft_breaches_{0};
  std::atomic<int> budget_state_{static_cast<int>(core::BudgetState::kOk)};
  core::ResourceSampler sampler_;
  core::ResourceSample last_sample_{};
  // Peaks across all polls: a single final sample reads ~0% CPU because it sits
  // microseconds after the previous one, so the peak is the meaningful figure.
  std::atomic<uint64_t> peak_rss_{0};
  std::atomic<uint64_t> peak_cpu_milli_{0};

  // Selective recording.
  std::unique_ptr<TriggerEngine> triggers_;

  // Latency is accumulated on the drain thread only.
  mutable std::mutex latency_mtx_;
  LatencyHistogram latency_;
  std::atomic<uint64_t> bytes_saved_{0};
};

}  // namespace sensorforge::pipeline
