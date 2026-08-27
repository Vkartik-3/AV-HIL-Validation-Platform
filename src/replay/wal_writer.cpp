/*
==============================================================================
SensorForge - WAL writer (implementation)
Part of the SensorForge AV HIL validation platform.
==============================================================================
*/

#include "sensorforge/replay/wal_writer.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include "sensorforge/replay/record.hpp"

namespace sensorforge::replay {

namespace fs = std::filesystem;

namespace {
uint64_t now_ms()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}
}  // namespace

FsyncPolicy fsync_policy_from_string(const std::string & s)
{
  if (s == "never") {return FsyncPolicy::kNever;}
  if (s == "interval") {return FsyncPolicy::kInterval;}
  if (s == "every_record" || s == "every") {return FsyncPolicy::kEveryRecord;}
  return FsyncPolicy::kOnSegmentSeal;
}

const char * to_string(FsyncPolicy p)
{
  switch (p) {
    case FsyncPolicy::kNever: return "never";
    case FsyncPolicy::kInterval: return "interval";
    case FsyncPolicy::kEveryRecord: return "every_record";
    case FsyncPolicy::kOnSegmentSeal: return "on_segment_seal";
  }
  return "unknown";
}

WalWriter::WalWriter(std::string dir, WalConfig config)
: dir_(std::move(dir)), config_(config)
{
  if (config_.segment_bytes == 0) {
    config_.segment_bytes = kDefaultSegmentBytes;
  }
  fs::create_directories(dir_);
  open_or_resume();
}

WalWriter::WalWriter(std::string dir, size_t segment_bytes)
: WalWriter(std::move(dir), WalConfig{segment_bytes, FsyncPolicy::kOnSegmentSeal, 1000})
{
}

WalWriter::~WalWriter()
{
  close();
}

std::string WalWriter::segment_path(uint32_t id) const
{
  return (fs::path(dir_) / segment_filename(id)).string();
}

uint64_t WalWriter::valid_prefix_length(const std::string & path, uint64_t & records_ok)
{
  records_ok = 0;
  std::error_code ec;
  const auto size = fs::file_size(path, ec);
  if (ec || size == 0) {
    return 0;
  }
  const int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    return 0;
  }
  std::vector<uint8_t> buf(static_cast<size_t>(size));
  const ssize_t got = ::read(fd, buf.data(), buf.size());
  ::close(fd);
  if (got <= 0) {
    return 0;
  }
  const size_t n = static_cast<size_t>(got);

  // Walk forward record by record. Unlike the reader's recovery scan (which
  // resyncs past corruption to salvage later records), the WRITER must stop at
  // the first bad record: everything after it would be appended behind a hole,
  // and the resume point has to be a clean boundary.
  size_t pos = 0;
  while (pos + kRecordHeaderSize <= n) {
    RecordHeader hdr;
    if (decode_record(buf.data() + pos, n - pos, hdr) != RecordError::kOk) {
      break;
    }
    pos += hdr.record_length;
    ++records_ok;
  }
  return pos;
}

void WalWriter::open_or_resume()
{
  const auto t0 = std::chrono::steady_clock::now();

  uint32_t max_id = 0;
  uint32_t count = 0;
  for (const auto & entry : fs::directory_iterator(dir_)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const uint32_t id = segment_id_from_filename(entry.path().filename().string());
    if (id != 0) {
      ++count;
      max_id = std::max(max_id, id);
    }
  }
  recovery_.segments_found = count;

  if (max_id == 0) {
    // Fresh directory: start a new recording at segment 1.
    current_segment_id_ = 0;
    open_new_segment();
    recovery_.resumed_segment_id = current_segment_id_;
    recovery_.scan_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    return;
  }

  // Existing recording: validate the tail of the highest segment, discard any
  // partial trailing record, and resume appending after it.
  const std::string path = segment_path(max_id);
  uint64_t records_ok = 0;
  const uint64_t good = valid_prefix_length(path, records_ok);

  std::error_code ec;
  const auto actual = fs::file_size(path, ec);
  const uint64_t actual_size = ec ? 0 : static_cast<uint64_t>(actual);
  recovery_.truncated_bytes = actual_size > good ? actual_size - good : 0;
  recovery_.valid_records_in_tail = records_ok;
  recovery_.recovered = true;

  if (recovery_.truncated_bytes > 0) {
    // Cut the partial record off so the next append lands on a clean boundary.
    ::truncate(path.c_str(), static_cast<off_t>(good));
  }

  if (good >= config_.segment_bytes) {
    // Tail segment is already full: seal it and start the next one.
    current_segment_id_ = max_id;
    open_new_segment();
  } else {
    fd_ = ::open(path.c_str(), O_WRONLY | O_APPEND | O_CREAT, 0644);
    current_segment_id_ = max_id;
    current_segment_offset_ = good;
  }
  recovery_.resumed_segment_id = current_segment_id_;
  recovery_.scan_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
}

void WalWriter::seal_current(bool sync)
{
  if (fd_ < 0) {
    return;
  }
  if (sync) {
    ::fsync(fd_);
    ++fsync_count_;
  }
  const uint32_t sealed_id = current_segment_id_;
  ::close(fd_);
  fd_ = -1;
  if (sealed_hook_ && sealed_id != 0) {
    sealed_hook_(segment_path(sealed_id), sealed_id);
  }
}

void WalWriter::open_new_segment()
{
  seal_current(config_.fsync_policy != FsyncPolicy::kNever);
  ++current_segment_id_;
  current_segment_offset_ = 0;
  const std::string path = segment_path(current_segment_id_);
  fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
}

void WalWriter::maybe_fsync()
{
  switch (config_.fsync_policy) {
    case FsyncPolicy::kEveryRecord:
      if (fd_ >= 0) {
        ::fsync(fd_);
        ++fsync_count_;
      }
      break;
    case FsyncPolicy::kInterval: {
      const uint64_t t = now_ms();
      if (last_fsync_ms_ == 0 || t - last_fsync_ms_ >= config_.fsync_interval_ms) {
        if (fd_ >= 0) {
          ::fsync(fd_);
          ++fsync_count_;
        }
        last_fsync_ms_ = t;
      }
      break;
    }
    case FsyncPolicy::kNever:
    case FsyncPolicy::kOnSegmentSeal:
      break;
  }
}

bool WalWriter::append(
  uint64_t timestamp_ns, protocol::SensorType sensor_type, uint64_t sequence,
  const uint8_t * payload, size_t payload_len)
{
  if (payload_len > kMaxRecordPayload) {
    return false;
  }
  const std::vector<uint8_t> rec =
    encode_record(timestamp_ns, sensor_type, sequence, payload, payload_len);

  // Roll to a new segment if this record would push the current one past the
  // limit (but always allow at least one record per segment).
  if (current_segment_offset_ > 0 &&
    current_segment_offset_ + rec.size() > config_.segment_bytes)
  {
    open_new_segment();
  }

  if (fd_ < 0) {
    return false;
  }
  const uint64_t offset = current_segment_offset_;

  // Write the whole record or fail; a short write would leave a torn record,
  // which the reader would then have to resync past.
  size_t written = 0;
  while (written < rec.size()) {
    const ssize_t n = ::write(fd_, rec.data() + written, rec.size() - written);
    if (n <= 0) {
      return false;
    }
    written += static_cast<size_t>(n);
  }

  index_.add(timestamp_ns, current_segment_id_, offset, static_cast<uint32_t>(rec.size()));
  current_segment_offset_ += rec.size();
  bytes_written_ += rec.size();
  ++records_written_;
  maybe_fsync();
  return true;
}

void WalWriter::flush()
{
  // Writes go straight to the fd (no user-space buffer), so "flush" only has
  // meaning as a durability request.
  maybe_fsync();
}

bool WalWriter::sync_now()
{
  if (fd_ < 0) {
    return false;
  }
  const bool ok = ::fsync(fd_) == 0;
  ++fsync_count_;
  return ok;
}

void WalWriter::close()
{
  seal_current(config_.fsync_policy != FsyncPolicy::kNever);
}

}  // namespace sensorforge::replay
