/*
==============================================================================
SensorForge - WAL segment naming / layout (Extension D)
Part of the SensorForge AV HIL validation platform.

Segments are append-only files named segment-000001.sflog, segment-000002.sflog
in a WAL directory. A new segment is started when the current one would exceed
the configured maximum size, bounding per-file size and enabling incremental
recovery. No ROS2 dependency.
==============================================================================
*/

#pragma once

#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace sensorforge::replay {

inline constexpr const char * kSegmentPrefix = "segment-";
inline constexpr const char * kSegmentSuffix = ".sflog";
inline constexpr size_t kDefaultSegmentBytes = 64u * 1024u * 1024u;  // 64 MiB

/// Zero-padded segment file name, e.g. segment-000007.sflog (segment_id >= 1).
inline std::string segment_filename(uint32_t segment_id)
{
  std::ostringstream os;
  os << kSegmentPrefix << std::setw(6) << std::setfill('0') << segment_id << kSegmentSuffix;
  return os.str();
}

/// Parse a segment id from a file name; returns 0 if it is not a segment file.
inline uint32_t segment_id_from_filename(const std::string & name)
{
  const std::string prefix = kSegmentPrefix;
  const std::string suffix = kSegmentSuffix;
  if (name.size() <= prefix.size() + suffix.size()) {
    return 0;
  }
  if (name.compare(0, prefix.size(), prefix) != 0) {
    return 0;
  }
  if (name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) {
    return 0;
  }
  const std::string digits =
    name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
  if (digits.empty()) {
    return 0;
  }
  uint32_t id = 0;
  for (char c : digits) {
    if (c < '0' || c > '9') {
      return 0;
    }
    id = id * 10 + static_cast<uint32_t>(c - '0');
  }
  return id;
}

}  // namespace sensorforge::replay
