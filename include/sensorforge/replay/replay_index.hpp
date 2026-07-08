/*
==============================================================================
SensorForge - In-memory replay index (Extension D)
Part of the SensorForge AV HIL validation platform.

Maps record timestamp_ns -> physical location {segment_id, offset}. Kept sorted
by timestamp so a replay can seek to a start time and iterate in order. Built
by the WAL reader while scanning segments; also updated by the writer as it
appends. No ROS2 dependency.
==============================================================================
*/

#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

namespace sensorforge::replay {

struct IndexEntry
{
  uint64_t timestamp_ns = 0;
  uint32_t segment_id = 0;
  uint64_t offset = 0;       // byte offset of the record within its segment
  uint32_t record_length = 0;
};

class ReplayIndex
{
public:
  void add(uint64_t timestamp_ns, uint32_t segment_id, uint64_t offset, uint32_t record_length)
  {
    entries_.push_back({timestamp_ns, segment_id, offset, record_length});
    // Cheap monotonic-append fast path; only mark unsorted if it regresses.
    if (entries_.size() > 1 && timestamp_ns < entries_[entries_.size() - 2].timestamp_ns) {
      sorted_ = false;
    }
  }

  /// Sort by timestamp (stable) so iteration is time-ordered.
  void finalize()
  {
    if (!sorted_) {
      std::stable_sort(
        entries_.begin(), entries_.end(),
        [](const IndexEntry & a, const IndexEntry & b) {
          return a.timestamp_ns < b.timestamp_ns;
        });
      sorted_ = true;
    }
  }

  /// First index position whose timestamp >= @p start_ns (call finalize first).
  size_t lower_bound(uint64_t start_ns) const
  {
    const auto it = std::lower_bound(
      entries_.begin(), entries_.end(), start_ns,
      [](const IndexEntry & e, uint64_t t) {return e.timestamp_ns < t;});
    return static_cast<size_t>(it - entries_.begin());
  }

  const std::vector<IndexEntry> & entries() const {return entries_;}
  size_t size() const {return entries_.size();}
  bool empty() const {return entries_.empty();}
  void clear() {entries_.clear(); sorted_ = true;}

private:
  std::vector<IndexEntry> entries_;
  bool sorted_ = true;
};

}  // namespace sensorforge::replay
