/*
==============================================================================
SensorForge - WAL reader / replayer (Extension D)
Part of the SensorForge AV HIL validation platform.

Loads all segments in a WAL directory, validating every record's CRC and
SKIPPING corrupt or truncated records (corruption recovery) so a partially
damaged log still replays its intact records. Records are sorted by timestamp
and can be replayed:

  - kDeterministic : invoke the callback for every record in order, no delays
                     (reproducible; used by tests / CI).
  - kRealTime      : reproduce the original inter-record timing (1x).
  - kFast          : reproduce timing scaled by a speed multiplier (Nx).

No ROS2 dependency.
==============================================================================
*/

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "sensorforge/replay/record.hpp"
#include "sensorforge/replay/replay_index.hpp"

namespace sensorforge::replay {

enum class ReplayMode {
  kDeterministic,
  kRealTime,
  kFast,
};

struct ReplayStats
{
  uint64_t records_ok = 0;
  uint64_t records_corrupt_skipped = 0;
  uint64_t bytes_skipped = 0;
  uint32_t segments_read = 0;
};

struct ReplayRecord
{
  RecordHeader header;
  std::vector<uint8_t> payload;
};

class WalReader
{
public:
  /// Load and validate every segment in @p dir.
  explicit WalReader(std::string dir);

  const std::vector<ReplayRecord> & records() const {return records_;}
  const ReplayIndex & index() const {return index_;}
  const ReplayStats & stats() const {return stats_;}
  size_t size() const {return records_.size();}

  /// Callback receives each replayed record in timestamp order.
  using Callback = std::function<void (const ReplayRecord &)>;

  /**
   * @brief Replay records with timestamp >= @p start_ns.
   * @param speed  multiplier for kFast (e.g. 2.0 = 2x). Ignored otherwise.
   * @return number of records delivered.
   */
  uint64_t replay(
    const Callback & cb, ReplayMode mode = ReplayMode::kDeterministic,
    double speed = 1.0, uint64_t start_ns = 0) const;

private:
  void load();

  std::string dir_;
  std::vector<ReplayRecord> records_;
  ReplayIndex index_;
  ReplayStats stats_;
};

}  // namespace sensorforge::replay
