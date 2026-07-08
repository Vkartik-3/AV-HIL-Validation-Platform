/*
==============================================================================
SensorForge - Fault injection engine (Extension K)
Part of the SensorForge AV HIL validation platform.

Applies per-stream, time-windowed faults to a stream of outgoing frames. The
engine is transport-agnostic: it is given a write callback (the real transport
write) and calls it zero, one, or many times per input frame depending on the
active faults. This lets it sit at a single call site -- the bridge's
network_interface_->write() -- as well as at the scenario runner's ingest.

Immediate faults (drop / burst_loss / disconnect / duplicate / corrupt /
reorder / bandwidth_limit) are applied synchronously. Delay is implemented with
an internal worker thread that releases held frames after delay_ms.
process_restart fires a caller-supplied hook once at window start.

Deterministic given the seed (except real wall-clock delay timing).
==============================================================================
*/

#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <queue>
#include <random>
#include <thread>
#include <vector>

#include "faults/fault_types.hpp"

namespace sensorforge::faults {

struct FaultStats
{
  uint64_t forwarded = 0;
  uint64_t dropped = 0;
  uint64_t duplicated = 0;
  uint64_t corrupted = 0;
  uint64_t reordered = 0;
  uint64_t delayed = 0;
};

class FaultEngine
{
public:
  using WriteFn = std::function<void (const std::vector<uint8_t> &)>;
  using RestartFn = std::function<void (const std::string & stream)>;

  explicit FaultEngine(WriteFn write, uint64_t seed = 42);
  ~FaultEngine();

  FaultEngine(const FaultEngine &) = delete;
  FaultEngine & operator=(const FaultEngine &) = delete;

  void add_rule(const FaultRule & r) {rules_.push_back(r);}
  void set_restart_hook(RestartFn fn) {restart_hook_ = std::move(fn);}

  /// Process one outgoing frame for @p stream at scenario time @p t_s.
  void process(const std::vector<uint8_t> & data, double t_s, const std::string & stream);

  /// Flush any delayed frames still queued (used at shutdown).
  void flush();

  FaultStats stats() const;

private:
  struct Delayed
  {
    double due_ns;
    std::vector<uint8_t> data;
    bool operator>(const Delayed & o) const {return due_ns > o.due_ns;}
  };

  void worker_loop();
  void emit(const std::vector<uint8_t> & data);   // final write + stats
  void dispatch_or_delay(
    const std::vector<uint8_t> & payload, const std::vector<const FaultRule *> & active);
  static double now_ns();

  WriteFn write_;
  RestartFn restart_hook_;
  std::vector<FaultRule> rules_;
  std::mt19937_64 rng_;

  // Reorder state (one held frame per engine; adjacent swap).
  bool has_held_ = false;
  std::vector<uint8_t> held_;

  // Token bucket for bandwidth limiting.
  double tokens_bytes_ = 0.0;
  double last_refill_ns_ = 0.0;

  // process_restart edge detection per stream.
  std::vector<std::string> restarted_this_window_;

  // Delay worker.
  std::thread worker_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::priority_queue<Delayed, std::vector<Delayed>, std::greater<Delayed>> delayed_;
  bool stop_ = false;

  mutable std::mutex stats_mtx_;
  FaultStats stats_;
};

}  // namespace sensorforge::faults
