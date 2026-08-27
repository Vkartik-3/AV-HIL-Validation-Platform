/*
==============================================================================
SensorForge - Multi-stream recorder (implementation)
Part of the SensorForge AV HIL validation platform.
==============================================================================
*/

#include "sensorforge/pipeline/recorder.hpp"

#include <algorithm>
#include <chrono>
#include <mutex>

#include "sensorforge/protocol/frame_codec.hpp"

namespace sensorforge::pipeline {

namespace {
constexpr size_t kNoStream = static_cast<size_t>(-1);
}

double LatencyHistogram::pct_us(double p) const
{
  if (samples_.empty()) {
    return 0.0;
  }
  std::vector<uint64_t> s = samples_;
  std::sort(s.begin(), s.end());
  const size_t idx = static_cast<size_t>(p * static_cast<double>(s.size() - 1));
  return static_cast<double>(s[idx]);
}

// ---------------------------------------------------------------------------
// Stream
// ---------------------------------------------------------------------------
struct Recorder::Stream
{
  StreamSpec spec;
  core::StreamBuffer buffer;
  core::MonotonicWallClock clock;
  uint64_t sequence = 0;
  PolicyDecision policy_decision = PolicyDecision::kRecordFull;

  std::atomic<uint64_t> captured{0};
  std::atomic<uint64_t> framed{0};
  std::atomic<uint64_t> recorded{0};
  std::atomic<uint64_t> skipped_decimation{0};
  std::atomic<uint64_t> denied{0};
  std::atomic<uint64_t> metadata_only{0};
  std::atomic<uint64_t> frame_bytes{0};

  explicit Stream(const StreamSpec & s)
  : spec(s), buffer(s.policy, s.limits) {}
};

// ---------------------------------------------------------------------------
// Recorder
// ---------------------------------------------------------------------------
Recorder::Recorder(RecorderConfig config)
: config_(std::move(config))
{
  if (config_.enable_triggers) {
    triggers_ = std::make_unique<TriggerEngine>(config_.trigger);
  }
}

Recorder::~Recorder()
{
  stop();
}

size_t Recorder::add_stream(const StreamSpec & spec)
{
  StreamSpec s = spec;
  if (s.sensor_type != SensorType::kUnknown &&
    s.policy == BackpressurePolicy::kDropNewest)
  {
    // Only override when the caller left the default in place, so an explicit
    // policy in config always wins.
    s.policy = core::default_policy_for(s.sensor_type);
  }
  auto stream = std::make_unique<Stream>(s);
  stream->policy_decision = config_.recording_policy.decide(s.name);
  streams_.push_back(std::move(stream));
  return streams_.size() - 1;
}

size_t Recorder::stream_id(const std::string & name) const
{
  for (size_t i = 0; i < streams_.size(); ++i) {
    if (streams_[i]->spec.name == name) {
      return i;
    }
  }
  return kNoStream;
}

void Recorder::start()
{
  if (running_.load()) {
    return;
  }
  if (!config_.wal_dir.empty()) {
    wal_ = std::make_unique<replay::WalWriter>(config_.wal_dir, config_.wal);
    if (sealed_hook_) {
      wal_->set_segment_sealed_hook(sealed_hook_);
    }
  }
  stop_requested_.store(false);
  running_.store(true);
  drain_ = std::thread([this]() {drain_loop();});
}

void Recorder::stop()
{
  if (!running_.load()) {
    return;
  }
  stop_requested_.store(true);
  if (drain_.joinable()) {
    drain_.join();
  }
  running_.store(false);
  if (wal_) {
    wal_->close();
  }
}

const replay::RecoveryReport * Recorder::wal_recovery() const
{
  return wal_ ? &wal_->recovery() : nullptr;
}

core::PushResult Recorder::capture(size_t id, const uint8_t * data, size_t len)
{
  if (id >= streams_.size()) {
    return core::PushResult::kDroppedNewest;
  }
  Stream & s = *streams_[id];

  // A denied stream never allocates a frame and never enters the buffer, so
  // its payload cannot reach the WAL or any offload destination by any path.
  if (s.policy_decision == PolicyDecision::kDeny) {
    ++s.denied;
    return core::PushResult::kDroppedNewest;
  }

  SensorFrame frame;
  if (s.policy_decision == PolicyDecision::kMetadataOnly) {
    ++s.metadata_only;      // payload deliberately not copied
  } else if (len > 0 && data != nullptr) {
    frame.data.assign(data, data + len);
  }
  frame.sequence = s.sequence++;
  frame.timestamp_ns = s.clock.next();
  frame.capture_mono_ns = core::mono_now_ns();

  ++s.captured;

  // Hard budget breach: shed rather than grow. This is the documented
  // behaviour the audit found missing -- the budget now has teeth.
  if (budget_state_.load(std::memory_order_relaxed) ==
    static_cast<int>(core::BudgetState::kHardBreach))
  {
    shed_events_.fetch_add(1, std::memory_order_relaxed);
    if (s.spec.policy != BackpressurePolicy::kNeverDropBlock) {
      return core::PushResult::kDroppedNewest;
    }
  }

  const core::PushResult r = s.buffer.push(std::move(frame));
  if (triggers_ && (r == core::PushResult::kDroppedNewest ||
    r == core::PushResult::kDroppedAfterBlock))
  {
    triggers_->fire(TriggerReason::kQueuePressure, core::mono_now_ns(), s.spec.name);
  }
  return r;
}

void Recorder::fire_trigger(TriggerReason reason, const std::string & detail)
{
  if (triggers_) {
    triggers_->fire(reason, core::mono_now_ns(), detail);
  }
}

bool Recorder::should_record(Stream & s, uint64_t seq)
{
  const uint32_t d = s.spec.baseline_decimation;
  if (d <= 1) {
    return true;
  }
  return (seq % d) == 0;
}

void Recorder::write_record(
  Stream & s, const SensorFrame & frame, const std::vector<uint8_t> & encoded)
{
  if (!wal_) {
    return;
  }
  // The WAL stores the ENCODED frame, so a replayed record is byte-identical to
  // what went on the wire and can be re-validated with decode_header().
  if (wal_->append(
      frame.timestamp_ns, s.spec.sensor_type, frame.sequence,
      encoded.data(), encoded.size()))
  {
    ++s.recorded;
    const uint64_t lat_us =
      (core::mono_now_ns() - frame.capture_mono_ns) / 1000ull;
    std::lock_guard<std::mutex> lk(latency_mtx_);
    latency_.add(lat_us);
    if (triggers_ && config_.trigger.latency_trigger_us > 0 &&
      lat_us > config_.trigger.latency_trigger_us)
    {
      triggers_->fire(TriggerReason::kLatencyHigh, frame.capture_mono_ns, s.spec.name);
    }
  }
}

void Recorder::handle_frame(Stream & s, SensorFrame & frame)
{
  // --- encode: this is the real SensorForge framing path -------------------
  std::vector<uint8_t> encoded;
  try {
    encoded = protocol::encode_frame(
      s.spec.sensor_type, frame.sequence, frame.timestamp_ns,
      protocol::kFlagNone, frame.data.data(), frame.data.size());
  } catch (const std::exception &) {
    return;   // payload above kMaxPayload; counted as not framed
  }
  ++s.framed;
  s.frame_bytes += encoded.size();

  // --- selective recording gate --------------------------------------------
  bool record_now = true;
  if (triggers_) {
    const uint64_t now = core::mono_now_ns();
    triggers_->expire(now);
    if (triggers_->window_open(now)) {
      // Window just opened (or is open): flush any buffered pre-roll first so
      // records land in capture order.
      auto pre = triggers_->take_preroll();
      for (auto & e : pre) {
        if (wal_ && wal_->append(
            e.timestamp_ns, e.sensor_type, e.sequence,
            e.payload.data(), e.payload.size()))
        {
          ++streams_[e.stream_id]->recorded;
        }
      }
      record_now = true;
    } else if (!should_record(s, frame.sequence)) {
      // Baseline decimation would discard this frame: keep it in the bounded
      // pre-roll so a trigger in the next pre_roll_seconds can still recover it.
      PreRollEntry e;
      e.stream_id = stream_id(s.spec.name);
      e.timestamp_ns = frame.timestamp_ns;
      e.sequence = frame.sequence;
      e.sensor_type = s.spec.sensor_type;
      e.capture_mono_ns = frame.capture_mono_ns;
      e.payload = encoded;
      bytes_saved_.fetch_add(encoded.size(), std::memory_order_relaxed);
      triggers_->push_preroll(std::move(e));
      ++s.skipped_decimation;
      record_now = false;
    }
  } else if (!should_record(s, frame.sequence)) {
    bytes_saved_.fetch_add(encoded.size(), std::memory_order_relaxed);
    ++s.skipped_decimation;
    record_now = false;
  }

  if (record_now) {
    write_record(s, frame, encoded);
  }

  // --- downstream sink (transport / fault engine) ---------------------------
  if (sink_) {
    sink_(s.spec.name, s.spec.sensor_type, encoded);
  }
}

void Recorder::drain_loop()
{
  SensorFrame frame;
  while (true) {
    bool any = false;
    for (auto & sp : streams_) {
      // Fair round-robin: bounded burst per stream per pass, so one high-rate
      // stream cannot starve the others.
      for (int i = 0; i < 32; ++i) {
        if (!sp->buffer.pop(frame)) {
          break;
        }
        any = true;
        handle_frame(*sp, frame);
        frame.data.clear();
      }
    }
    if (!any) {
      if (stop_requested_.load(std::memory_order_relaxed)) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  }
}

void Recorder::poll_resources()
{
  last_sample_ = sampler_.sample();
  if (last_sample_.rss_bytes > peak_rss_.load(std::memory_order_relaxed)) {
    peak_rss_.store(last_sample_.rss_bytes, std::memory_order_relaxed);
  }
  const uint64_t cpu_milli = static_cast<uint64_t>(last_sample_.cpu_percent * 1000.0);
  if (cpu_milli > peak_cpu_milli_.load(std::memory_order_relaxed)) {
    peak_cpu_milli_.store(cpu_milli, std::memory_order_relaxed);
  }
  uint64_t queued = 0;
  for (const auto & s : streams_) {
    queued += s->buffer.queued_bytes();
  }
  if (triggers_) {
    queued += triggers_->preroll_bytes();   // pre-roll counts against the budget
  }
  const core::BudgetState st =
    core::evaluate_budget(config_.budget, last_sample_.rss_bytes, queued);
  const int prev = budget_state_.exchange(static_cast<int>(st));
  if (st == core::BudgetState::kSoftBreach &&
    prev != static_cast<int>(core::BudgetState::kSoftBreach))
  {
    soft_breaches_.fetch_add(1, std::memory_order_relaxed);
  }
}

RecorderStats Recorder::stats() const
{
  RecorderStats out;
  for (const auto & sp : streams_) {
    StreamStats ss;
    ss.name = sp->spec.name;
    ss.sensor_type = sp->spec.sensor_type;
    ss.policy = core::to_string(sp->spec.policy);
    ss.buffer = sp->buffer.counters();
    ss.captured = sp->captured.load(std::memory_order_relaxed);
    ss.framed = sp->framed.load(std::memory_order_relaxed);
    ss.recorded = sp->recorded.load(std::memory_order_relaxed);
    ss.skipped_decimation = sp->skipped_decimation.load(std::memory_order_relaxed);
    ss.denied_by_policy = sp->denied.load(std::memory_order_relaxed);
    ss.metadata_only = sp->metadata_only.load(std::memory_order_relaxed);
    ss.frame_bytes = sp->frame_bytes.load(std::memory_order_relaxed);
    ss.sequence = sp->sequence;
    ss.clock_regressions = sp->clock.regressions();

    out.total_captured += ss.captured;
    out.total_recorded += ss.recorded;
    out.total_dropped += ss.buffer.dropped;
    out.total_overwritten += ss.buffer.overwritten;
    out.total_frame_bytes += ss.frame_bytes;
    out.queued_bytes += ss.buffer.queued_bytes;
    out.queued_frames += ss.buffer.queued_frames;
    out.peak_queued_bytes += ss.buffer.peak_queued_bytes;
    out.streams.push_back(std::move(ss));
  }
  if (wal_) {
    out.wal_records = wal_->records_written();
    out.wal_bytes = wal_->bytes_written();
    out.wal_fsyncs = wal_->fsync_count();
  }
  out.shed_events = shed_events_.load(std::memory_order_relaxed);
  out.soft_breaches = soft_breaches_.load(std::memory_order_relaxed);
  out.resources = last_sample_;
  out.peak_rss_bytes = peak_rss_.load(std::memory_order_relaxed);
  out.peak_cpu_percent =
    static_cast<double>(peak_cpu_milli_.load(std::memory_order_relaxed)) / 1000.0;
  out.budget_state = static_cast<core::BudgetState>(budget_state_.load());
  out.bytes_saved_vs_always_on = bytes_saved_.load(std::memory_order_relaxed);
  if (triggers_) {
    out.trigger_activations = triggers_->activations();
    out.preroll_flushed = triggers_->preroll_frames();
    out.queued_bytes += triggers_->preroll_bytes();
  }
  {
    std::lock_guard<std::mutex> lk(latency_mtx_);
    out.latency_samples = latency_.count();
    out.p50_capture_to_record_us = latency_.pct_us(0.50);
    out.p99_capture_to_record_us = latency_.pct_us(0.99);
    out.p999_capture_to_record_us = latency_.pct_us(0.999);
  }
  return out;
}

}  // namespace sensorforge::pipeline
