/*
==============================================================================
SensorForge - Fault types (Extension K)
Part of the SensorForge AV HIL validation platform.

Standalone fault taxonomy + rule model (no ROS/yaml dependency). The scenario
layer maps scenario::FaultType -> faults::FaultKind at the integration boundary.
==============================================================================
*/

#pragma once

#include <cstdint>
#include <string>

namespace sensorforge::faults {

enum class FaultKind {
  kNone,
  kDelay,           // hold the frame delay_ms before forwarding
  kDrop,            // drop with probability drop_rate
  kDuplicate,       // forward the frame twice
  kCorrupt,         // flip random payload bits, then forward
  kReorder,         // swap adjacent frames
  kBurstLoss,       // drop ALL frames for the window
  kBandwidthLimit,  // token-bucket rate limit (excess dropped)
  kEcuDisconnect,   // suppress all frames for the window (link down)
  kProcessRestart,  // signal the orchestrator to restart the stream's process
};

inline FaultKind fault_kind_from_string(const std::string & s)
{
  if (s == "delay") {return FaultKind::kDelay;}
  if (s == "drop") {return FaultKind::kDrop;}
  if (s == "duplicate") {return FaultKind::kDuplicate;}
  if (s == "corrupt") {return FaultKind::kCorrupt;}
  if (s == "reorder") {return FaultKind::kReorder;}
  if (s == "burst" || s == "burst_loss") {return FaultKind::kBurstLoss;}
  if (s == "bandwidth_limit") {return FaultKind::kBandwidthLimit;}
  if (s == "ecu_disconnect") {return FaultKind::kEcuDisconnect;}
  if (s == "process_restart") {return FaultKind::kProcessRestart;}
  return FaultKind::kNone;
}

struct FaultRule
{
  FaultKind kind = FaultKind::kNone;
  std::string stream;          // empty = applies to every stream
  double start_s = 0.0;
  double duration_s = 0.0;

  // Parameters (used per-kind).
  double delay_ms = 0.0;
  double drop_rate = 0.0;       // 0..1
  double bandwidth_kbps = 0.0;  // for kBandwidthLimit
  uint32_t corrupt_bits = 4;    // bits flipped per corrupt event
  uint64_t seed = 1;

  bool active_at(double t_s) const
  {
    return t_s >= start_s && t_s < (start_s + duration_s);
  }

  bool matches(const std::string & s) const
  {
    return stream.empty() || stream == s;
  }
};

}  // namespace sensorforge::faults
