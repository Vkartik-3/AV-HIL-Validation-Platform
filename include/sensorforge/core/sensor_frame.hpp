/*
==============================================================================
SensorForge - In-process sensor frame
Part of the SensorForge AV HIL validation platform.

The unit that flows producer -> stream buffer -> consumer inside a process. It
carries the serialized message bytes plus the capture-time metadata that later
becomes the wire frame's sequence and timestamp_ns fields.

Two timestamps, deliberately (see core/clock.hpp):
  timestamp_ns      wall clock, monotonised per stream. Goes on the wire.
  capture_mono_ns   steady clock. Never leaves the process; used to compute
                    capture-to-record latency without wall-clock jumps
                    corrupting the measurement.
==============================================================================
*/

#pragma once

#include <cstdint>
#include <vector>

namespace sensorforge::core {

struct SensorFrame
{
  std::vector<uint8_t> data;      // serialized payload (pre-compression)
  uint64_t sequence = 0;          // per-stream monotonic, assigned at capture
  uint64_t timestamp_ns = 0;      // monotonised wall-clock capture time
  uint64_t capture_mono_ns = 0;   // steady-clock capture time (in-process only)

  size_t byte_size() const {return data.size();}
};

}  // namespace sensorforge::core
