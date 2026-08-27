/*
==============================================================================
SensorForge - WAL writer
Part of the SensorForge AV HIL validation platform.

Append-only, segmented writer. append() encodes a record (record.hpp), writes
it to the current segment, updates the in-memory index, and rolls over to a new
segment when the configured size limit would be exceeded.

Two defects the audit found are fixed here:

  1. RESTART DESTROYED THE PREVIOUS RECORDING. The old constructor always
     started at segment id 1 and opened it with ios::trunc, so restarting the
     bridge against an existing directory truncated the first segment of the
     prior run. That is data loss labelled as recovery. The writer now
     discovers existing segments, validates the tail of the last one, discards
     any partially written trailing record, and appends from there.

  2. NO DURABILITY CONTROL. flush() pushed an ofstream buffer into the page
     cache and stopped; nothing ever called fsync, so a power loss or kernel
     panic lost everything not yet written back. FsyncPolicy now makes the
     durability/throughput trade explicit and configurable.

The writer uses a raw POSIX file descriptor rather than std::ofstream so that
fsync, ftruncate and append offsets are all directly controllable.
No ROS2 dependency.
==============================================================================
*/

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "sensorforge/protocol/frame.hpp"        // SensorType
#include "sensorforge/replay/replay_index.hpp"
#include "sensorforge/replay/segment.hpp"

namespace sensorforge::replay {

/// When to force data to stable storage.
enum class FsyncPolicy {
  kNever,        // rely on the OS. Fastest; survives process crash, not power loss.
  kInterval,     // fsync at most every fsync_interval_ms.
  kEveryRecord,  // fsync after every append. Slowest; strongest.
  kOnSegmentSeal,  // fsync only when a segment is closed.
};

FsyncPolicy fsync_policy_from_string(const std::string & s);
const char * to_string(FsyncPolicy p);

struct WalConfig
{
  size_t segment_bytes = kDefaultSegmentBytes;
  FsyncPolicy fsync_policy = FsyncPolicy::kOnSegmentSeal;
  uint32_t fsync_interval_ms = 1000;
};

/// Result of scanning an existing WAL directory at startup.
struct RecoveryReport
{
  uint32_t segments_found = 0;
  uint32_t resumed_segment_id = 0;
  uint64_t valid_records_in_tail = 0;
  uint64_t truncated_bytes = 0;     // bytes of partial/corrupt tail discarded
  bool recovered = false;           // true if an existing recording was resumed
  double scan_ms = 0.0;
};

class WalWriter
{
public:
  /// Open (creating if needed) a WAL directory for appending.
  explicit WalWriter(std::string dir, WalConfig config = {});

  /// Backwards-compatible form used by existing call sites and tests.
  WalWriter(std::string dir, size_t segment_bytes);

  ~WalWriter();

  WalWriter(const WalWriter &) = delete;
  WalWriter & operator=(const WalWriter &) = delete;

  /// Append one record. Returns false on I/O failure.
  bool append(
    uint64_t timestamp_ns, protocol::SensorType sensor_type, uint64_t sequence,
    const uint8_t * payload, size_t payload_len);

  /// Flush the current segment to the OS (and fsync if the policy demands it).
  void flush();

  /// Force data to stable storage regardless of policy.
  bool sync_now();

  /// Flush, sync per policy, and close the current segment.
  void close();

  /// Called with the path of each segment as it is sealed (rolled over or
  /// closed). Used by the offload uploader; never called with a live segment.
  void set_segment_sealed_hook(std::function<void(const std::string &, uint32_t)> fn)
  {
    sealed_hook_ = std::move(fn);
  }

  const ReplayIndex & index() const {return index_;}
  const RecoveryReport & recovery() const {return recovery_;}
  uint64_t records_written() const {return records_written_;}
  uint64_t bytes_written() const {return bytes_written_;}
  uint32_t current_segment_id() const {return current_segment_id_;}
  uint64_t fsync_count() const {return fsync_count_;}

private:
  void open_or_resume();
  void open_new_segment();
  void seal_current(bool sync);
  std::string segment_path(uint32_t id) const;
  /// Walk a segment file and return the offset just past the last valid record.
  static uint64_t valid_prefix_length(const std::string & path, uint64_t & records_ok);
  void maybe_fsync();

  std::string dir_;
  WalConfig config_{};
  int fd_ = -1;
  uint32_t current_segment_id_ = 0;
  uint64_t current_segment_offset_ = 0;
  uint64_t records_written_ = 0;
  uint64_t bytes_written_ = 0;
  uint64_t fsync_count_ = 0;
  uint64_t last_fsync_ms_ = 0;
  ReplayIndex index_;
  RecoveryReport recovery_{};
  std::function<void(const std::string &, uint32_t)> sealed_hook_;
};

}  // namespace sensorforge::replay
