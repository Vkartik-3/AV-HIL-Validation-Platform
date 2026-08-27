/*
==============================================================================
SensorForge - WAL reader / replayer (implementation, Extension D)
Part of the SensorForge AV HIL validation platform.
==============================================================================
*/

#include "sensorforge/replay/wal_reader.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include "sensorforge/protocol/crc32c.hpp"
#include "sensorforge/replay/segment.hpp"

namespace sensorforge::replay {

namespace fs = std::filesystem;

WalReader::WalReader(std::string dir)
: dir_(std::move(dir))
{
  load();
}

void WalReader::load()
{
  records_.clear();
  index_.clear();
  stats_ = ReplayStats{};

  if (!fs::exists(dir_) || !fs::is_directory(dir_)) {
    return;
  }

  // Collect segment files in ascending id order.
  std::vector<std::pair<uint32_t, std::string>> segments;
  for (const auto & entry : fs::directory_iterator(dir_)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const uint32_t id = segment_id_from_filename(entry.path().filename().string());
    if (id != 0) {
      segments.emplace_back(id, entry.path().string());
    }
  }
  std::sort(segments.begin(), segments.end());

  for (const auto & [seg_id, path] : segments) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) {
      continue;
    }
    const std::streamsize sz = in.tellg();
    if (sz <= 0) {
      ++stats_.segments_read;
      continue;
    }
    in.seekg(0);
    std::vector<uint8_t> buf(static_cast<size_t>(sz));
    in.read(reinterpret_cast<char *>(buf.data()), sz);
    ++stats_.segments_read;

    // Scan with corruption recovery: on a clean record advance by its length;
    // on any error resync forward one byte until a valid record is found. A
    // contiguous run of skipped bytes is counted as one corrupt record.
    size_t pos = 0;
    bool in_skip_run = false;
    while (pos + kRecordHeaderSize <= buf.size()) {
      RecordHeader hdr;
      const RecordError err = decode_record(buf.data() + pos, buf.size() - pos, hdr);
      if (err == RecordError::kOk) {
        ReplayRecord rr;
        rr.header = hdr;
        rr.payload.assign(
          buf.data() + pos + kRecordHeaderSize,
          buf.data() + pos + kRecordHeaderSize + hdr.payload_size);
        index_.add(hdr.timestamp_ns, seg_id, pos, hdr.record_length);
        records_.push_back(std::move(rr));
        ++stats_.records_ok;
        pos += hdr.record_length;
        in_skip_run = false;
      } else {
        // Corrupt or truncated: skip one byte and try to resync.
        if (!in_skip_run) {
          ++stats_.records_corrupt_skipped;
          in_skip_run = true;
        }
        ++pos;
        ++stats_.bytes_skipped;
      }
    }
    // Any trailing bytes (< header size) are an incomplete tail write.
    if (pos < buf.size()) {
      if (!in_skip_run) {
        ++stats_.records_corrupt_skipped;
      }
      stats_.bytes_skipped += (buf.size() - pos);
    }
  }

  // Time-order for replay.
  std::stable_sort(
    records_.begin(), records_.end(),
    [](const ReplayRecord & a, const ReplayRecord & b) {
      if (a.header.timestamp_ns != b.header.timestamp_ns) {
        return a.header.timestamp_ns < b.header.timestamp_ns;
      }
      return a.header.sequence < b.header.sequence;
    });
  index_.finalize();
}

uint64_t WalReader::replay(
  const Callback & cb, ReplayMode mode, double speed, uint64_t start_ns) const
{
  if (records_.empty()) {
    return 0;
  }
  if (speed <= 0.0) {
    speed = 1.0;
  }

  // Find first record >= start_ns (records_ is time-sorted).
  const auto first = std::lower_bound(
    records_.begin(), records_.end(), start_ns,
    [](const ReplayRecord & r, uint64_t t) {return r.header.timestamp_ns < t;});

  uint64_t delivered = 0;
  const auto wall_start = std::chrono::steady_clock::now();
  const uint64_t base_ts = first != records_.end() ? first->header.timestamp_ns : 0;

  for (auto it = first; it != records_.end(); ++it) {
    if (mode != ReplayMode::kDeterministic) {
      // Sleep until this record's scheduled wall time.
      const double elapsed_log_ns =
        static_cast<double>(it->header.timestamp_ns - base_ts) / speed;
      const auto target = wall_start +
        std::chrono::nanoseconds(static_cast<int64_t>(elapsed_log_ns));
      std::this_thread::sleep_until(target);
    }
    cb(*it);
    ++delivered;
  }
  return delivered;
}


namespace {

// Shared segment-file scan used by both the streaming replay and the digest.
// Returns false from `on_record` to stop early.
ReplayStats scan_segments(
  const std::string & dir,
  const std::function<bool (const ReplayRecord &)> & on_record)
{
  ReplayStats stats;
  if (!fs::exists(dir) || !fs::is_directory(dir)) {
    return stats;
  }
  std::vector<std::pair<uint32_t, std::string>> segments;
  for (const auto & entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const uint32_t id = segment_id_from_filename(entry.path().filename().string());
    if (id != 0) {
      segments.emplace_back(id, entry.path().string());
    }
  }
  std::sort(segments.begin(), segments.end());

  for (const auto & [seg_id, path] : segments) {
    (void)seg_id;
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) {
      continue;
    }
    const std::streamsize sz = in.tellg();
    ++stats.segments_read;
    if (sz <= 0) {
      continue;
    }
    in.seekg(0);
    std::vector<uint8_t> buf(static_cast<size_t>(sz));
    in.read(reinterpret_cast<char *>(buf.data()), sz);

    size_t pos = 0;
    bool in_skip_run = false;
    while (pos + kRecordHeaderSize <= buf.size()) {
      RecordHeader hdr;
      const RecordError err = decode_record(buf.data() + pos, buf.size() - pos, hdr);
      if (err == RecordError::kOk) {
        ReplayRecord rr;
        rr.header = hdr;
        rr.payload.assign(
          buf.data() + pos + kRecordHeaderSize,
          buf.data() + pos + kRecordHeaderSize + hdr.payload_size);
        ++stats.records_ok;
        if (!on_record(rr)) {
          return stats;
        }
        pos += hdr.record_length;
        in_skip_run = false;
      } else {
        if (!in_skip_run) {
          ++stats.records_corrupt_skipped;
          in_skip_run = true;
        }
        ++pos;
        ++stats.bytes_skipped;
      }
    }
    if (pos < buf.size()) {
      if (!in_skip_run) {
        ++stats.records_corrupt_skipped;
      }
      stats.bytes_skipped += (buf.size() - pos);
    }
    // buf goes out of scope here: peak memory is one segment, not the whole log.
  }
  return stats;
}

}  // namespace

ReplayStats stream_replay(
  const std::string & dir,
  const std::function<void (const ReplayRecord &)> & cb,
  ReplayMode mode, double speed, uint64_t start_ns)
{
  if (speed <= 0.0) {
    speed = 1.0;
  }
  const auto wall_start = std::chrono::steady_clock::now();
  bool have_base = false;
  uint64_t base_ts = 0;

  return scan_segments(
    dir,
    [&](const ReplayRecord & rr) {
      if (rr.header.timestamp_ns < start_ns) {
        return true;   // seek past
      }
      if (!have_base) {
        base_ts = rr.header.timestamp_ns;
        have_base = true;
      }
      if (mode != ReplayMode::kDeterministic) {
        const double elapsed_log_ns =
          static_cast<double>(rr.header.timestamp_ns - base_ts) / speed;
        std::this_thread::sleep_until(
          wall_start + std::chrono::nanoseconds(static_cast<int64_t>(elapsed_log_ns)));
      }
      cb(rr);
      return true;
    });
}

ReplayDigest compute_replay_digest(const std::string & dir, uint64_t start_ns)
{
  ReplayDigest d;
  uint32_t running = 0;
  d.stats = scan_segments(
    dir,
    [&](const ReplayRecord & rr) {
      if (rr.header.timestamp_ns < start_ns) {
        return true;
      }
      // Chain the record's identity and payload into the running CRC so both
      // content and delivery ORDER are covered.
      uint8_t hdr[18];
      for (int i = 0; i < 8; ++i) {
        hdr[i] = static_cast<uint8_t>((rr.header.timestamp_ns >> (8 * i)) & 0xFF);
      }
      const uint16_t st = static_cast<uint16_t>(rr.header.sensor_type);
      hdr[8] = static_cast<uint8_t>(st & 0xFF);
      hdr[9] = static_cast<uint8_t>((st >> 8) & 0xFF);
      for (int i = 0; i < 8; ++i) {
        hdr[10 + i] = static_cast<uint8_t>((rr.header.sequence >> (8 * i)) & 0xFF);
      }
      running = crc::crc32c_update(running, hdr, sizeof(hdr));
      if (!rr.payload.empty()) {
        running = crc::crc32c_update(running, rr.payload.data(), rr.payload.size());
      }
      ++d.records;
      d.payload_bytes += rr.payload.size();
      return true;
    });
  d.digest = running;
  return d;
}

}  // namespace sensorforge::replay

