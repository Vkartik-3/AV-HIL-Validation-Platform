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


/**
 * @brief Streaming replay: process one segment at a time, never holding the
 *        whole recording in memory.
 *
 * WalReader::load() materialises every record into a vector before anything can
 * be replayed, so a multi-segment recording costs its full size in RAM. This
 * path reads one segment, delivers its records, then frees it -- peak memory is
 * one segment plus one record. It carries the same per-record CRC validation
 * and the same corruption resync as load().
 *
 * ORDERING NOTE: records are delivered in (segment id, file offset) order,
 * which equals capture order for a single writer -- the only writer topology
 * this WAL supports. load() additionally sorts globally by timestamp; for a
 * single writer the two orders agree. If a future multi-writer topology is
 * added, this function would need a merge step.
 */
ReplayStats stream_replay(
  const std::string & dir,
  const std::function<void (const ReplayRecord &)> & cb,
  ReplayMode mode = ReplayMode::kDeterministic,
  double speed = 1.0,
  uint64_t start_ns = 0);

/**
 * @brief Order-sensitive digest of a replay, for record/replay regression tests.
 *
 * CRC32C chained over each delivered record's (timestamp_ns, sensor_type,
 * sequence, payload bytes) in delivery order. Two replays of the same intact
 * recording produce the same digest; any change to content, ordering or record
 * count changes it.
 */
struct ReplayDigest
{
  uint32_t digest = 0;
  uint64_t records = 0;
  uint64_t payload_bytes = 0;
  ReplayStats stats;
};

ReplayDigest compute_replay_digest(const std::string & dir, uint64_t start_ns = 0);

}  // namespace sensorforge::replay
