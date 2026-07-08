/*
==============================================================================
SensorForge - In-process sensor frame
Part of the SensorForge AV HIL validation platform.

The unit that flows producer -> SPSC ring -> consumer inside a process. It
carries the serialized message bytes plus the capture-time metadata that later
becomes the wire frame's sequence and timestamp_ns fields. Kept in core/ so
the ring, benchmarks and the bridge can all share one definition.
==============================================================================
*/

#pragma once

#include <cstdint>
#include <vector>

namespace sensorforge::core {

struct SensorFrame
{
  std::vector<uint8_t> data;   // serialized payload (pre-compression)
  uint64_t sequence = 0;       // per-stream monotonic, assigned at capture
  uint64_t timestamp_ns = 0;   // wall-clock capture time (nanoseconds)
};

}  // namespace sensorforge::core
