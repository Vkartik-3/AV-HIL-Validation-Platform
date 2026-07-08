/*
==============================================================================
SensorForge - Fault injection engine (implementation, Extension K)
Part of the SensorForge AV HIL validation platform.
==============================================================================
*/

#include "faults/fault_engine.hpp"

#include <algorithm>
#include <chrono>

namespace sensorforge::faults {

FaultEngine::FaultEngine(WriteFn write, uint64_t seed)
: write_(std::move(write)), rng_(seed)
{
  worker_ = std::thread([this]() {worker_loop();});
}

FaultEngine::~FaultEngine()
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

double FaultEngine::now_ns()
{
  return static_cast<double>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

void FaultEngine::emit(const std::vector<uint8_t> & data)
{
  if (write_) {
    write_(data);
  }
  std::lock_guard<std::mutex> lk(stats_mtx_);
  ++stats_.forwarded;
}

void FaultEngine::process(
  const std::vector<uint8_t> & data, double t_s, const std::string & stream)
{
  // Collect active rules for this stream.
  std::vector<const FaultRule *> active;
  for (const auto & r : rules_) {
    if (r.matches(stream) && r.active_at(t_s)) {
      active.push_back(&r);
    }
  }

  // process_restart: fire the hook once per (stream) at first activation.
  for (const auto * r : active) {
    if (r->kind == FaultKind::kProcessRestart) {
      if (std::find(restarted_this_window_.begin(), restarted_this_window_.end(), stream) ==
        restarted_this_window_.end())
      {
        restarted_this_window_.push_back(stream);
        if (restart_hook_) {
          restart_hook_(stream);
        }
      }
    }
  }
  // Clear restart edge-tracking for streams with no active restart rule.
  if (active.empty()) {
    restarted_this_window_.erase(
      std::remove(restarted_this_window_.begin(), restarted_this_window_.end(), stream),
      restarted_this_window_.end());
  }

  // Hard suppression: disconnect or burst-loss drop everything.
  for (const auto * r : active) {
    if (r->kind == FaultKind::kEcuDisconnect || r->kind == FaultKind::kBurstLoss) {
      std::lock_guard<std::mutex> lk(stats_mtx_);
      ++stats_.dropped;
      return;
    }
  }

  // Probabilistic drop.
  for (const auto * r : active) {
    if (r->kind == FaultKind::kDrop && r->drop_rate > 0.0) {
      std::uniform_real_distribution<double> d(0.0, 1.0);
      if (d(rng_) < r->drop_rate) {
        std::lock_guard<std::mutex> lk(stats_mtx_);
        ++stats_.dropped;
        return;
      }
    }
  }

  // Bandwidth limit (token bucket over bytes).
  for (const auto * r : active) {
    if (r->kind == FaultKind::kBandwidthLimit && r->bandwidth_kbps > 0.0) {
      const double now = now_ns();
      const double cap_bytes = r->bandwidth_kbps * 1000.0 / 8.0;  // bytes per second
      if (last_refill_ns_ == 0.0) {
        last_refill_ns_ = now;
        tokens_bytes_ = cap_bytes;
      }
      const double elapsed_s = (now - last_refill_ns_) / 1e9;
      tokens_bytes_ = std::min(cap_bytes, tokens_bytes_ + elapsed_s * cap_bytes);
      last_refill_ns_ = now;
      if (tokens_bytes_ < static_cast<double>(data.size())) {
        std::lock_guard<std::mutex> lk(stats_mtx_);
        ++stats_.dropped;
        return;   // over budget -> drop
      }
      tokens_bytes_ -= static_cast<double>(data.size());
    }
  }

  // Build the payload to send (possibly corrupted).
  std::vector<uint8_t> payload = data;
  for (const auto * r : active) {
    if (r->kind == FaultKind::kCorrupt && !payload.empty()) {
      std::uniform_int_distribution<size_t> pos(0, payload.size() - 1);
      std::uniform_int_distribution<int> bit(0, 7);
      for (uint32_t i = 0; i < r->corrupt_bits; ++i) {
        payload[pos(rng_)] ^= static_cast<uint8_t>(1u << bit(rng_));
      }
      std::lock_guard<std::mutex> lk(stats_mtx_);
      ++stats_.corrupted;
    }
  }

  // Reorder: swap adjacent frames.
  bool reorder = false;
  for (const auto * r : active) {
    if (r->kind == FaultKind::kReorder) {reorder = true;}
  }
  if (reorder) {
    if (!has_held_) {
      has_held_ = true;
      held_ = std::move(payload);
      return;   // hold this one; it goes out after the next
    }
    // We have a held frame: emit THIS one first, then the held one.
    std::vector<uint8_t> current = std::move(payload);
    std::vector<uint8_t> prev = std::move(held_);
    has_held_ = false;
    dispatch_or_delay(current, active);
    dispatch_or_delay(prev, active);
    {
      std::lock_guard<std::mutex> lk(stats_mtx_);
      ++stats_.reordered;
    }
    return;
  }

  dispatch_or_delay(payload, active);

  // Duplicate: forward a second copy.
  for (const auto * r : active) {
    if (r->kind == FaultKind::kDuplicate) {
      dispatch_or_delay(payload, active);
      std::lock_guard<std::mutex> lk(stats_mtx_);
      ++stats_.duplicated;
    }
  }
}

void FaultEngine::dispatch_or_delay(
  const std::vector<uint8_t> & payload, const std::vector<const FaultRule *> & active)
{
  double delay_ms = 0.0;
  for (const auto * r : active) {
    if (r->kind == FaultKind::kDelay) {
      delay_ms = std::max(delay_ms, r->delay_ms);
    }
  }
  if (delay_ms <= 0.0) {
    emit(payload);
    return;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_);
    delayed_.push({now_ns() + delay_ms * 1e6, payload});
  }
  {
    std::lock_guard<std::mutex> lk(stats_mtx_);
    ++stats_.delayed;
  }
  cv_.notify_all();
}

void FaultEngine::worker_loop()
{
  std::unique_lock<std::mutex> lk(mtx_);
  while (!stop_) {
    if (delayed_.empty()) {
      cv_.wait(lk);
      continue;
    }
    const double due = delayed_.top().due_ns;
    const double now = now_ns();
    if (now < due) {
      cv_.wait_for(lk, std::chrono::nanoseconds(static_cast<int64_t>(due - now)));
      continue;
    }
    Delayed d = delayed_.top();
    delayed_.pop();
    lk.unlock();
    emit(d.data);
    lk.lock();
  }
}

void FaultEngine::flush()
{
  std::unique_lock<std::mutex> lk(mtx_);
  while (!delayed_.empty()) {
    Delayed d = delayed_.top();
    delayed_.pop();
    lk.unlock();
    emit(d.data);
    lk.lock();
  }
}

FaultStats FaultEngine::stats() const
{
  std::lock_guard<std::mutex> lk(stats_mtx_);
  return stats_;
}

}  // namespace sensorforge::faults
