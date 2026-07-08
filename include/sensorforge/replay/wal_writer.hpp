/*
==============================================================================
SensorForge - WAL writer (Extension D)
Part of the SensorForge AV HIL validation platform.

Append-only, segmented writer. append() encodes a record (record.hpp), writes
it to the current segment, updates the in-memory index, and rolls over to a new
segment when the configured size limit would be exceeded. No ROS2 dependency.
==============================================================================
*/

#pragma once

#include <cstdint>
#include <fstream>
#include <string>

#include "sensorforge/protocol/frame.hpp"        // SensorType
#include "sensorforge/replay/replay_index.hpp"
#include "sensorforge/replay/segment.hpp"

namespace sensorforge::replay {

class WalWriter
{
public:
  /**
   * @brief Open (creating if needed) a WAL directory for appending.
   * @param dir              directory to hold segment files
   * @param segment_bytes    roll to a new segment before exceeding this size
   */
  explicit WalWriter(std::string dir, size_t segment_bytes = kDefaultSegmentBytes);
  ~WalWriter();

  WalWriter(const WalWriter &) = delete;
  WalWriter & operator=(const WalWriter &) = delete;

  /// Append one record. Returns false on I/O failure.
  bool append(
    uint64_t timestamp_ns, protocol::SensorType sensor_type, uint64_t sequence,
    const uint8_t * payload, size_t payload_len);

  /// Flush the current segment to the OS.
  void flush();

  /// Flush and close the current segment.
  void close();

  const ReplayIndex & index() const {return index_;}
  uint64_t records_written() const {return records_written_;}
  uint32_t current_segment_id() const {return current_segment_id_;}

private:
  void open_new_segment();

  std::string dir_;
  size_t segment_bytes_;
  std::ofstream out_;
  uint32_t current_segment_id_ = 0;
  uint64_t current_segment_offset_ = 0;
  uint64_t records_written_ = 0;
  ReplayIndex index_;
};

}  // namespace sensorforge::replay
