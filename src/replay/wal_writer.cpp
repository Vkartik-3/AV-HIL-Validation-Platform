/*
==============================================================================
SensorForge - WAL writer (implementation, Extension D)
Part of the SensorForge AV HIL validation platform.
==============================================================================
*/

#include "sensorforge/replay/wal_writer.hpp"

#include <filesystem>

#include "sensorforge/replay/record.hpp"

namespace sensorforge::replay {

namespace fs = std::filesystem;

WalWriter::WalWriter(std::string dir, size_t segment_bytes)
: dir_(std::move(dir)),
  segment_bytes_(segment_bytes == 0 ? kDefaultSegmentBytes : segment_bytes)
{
  fs::create_directories(dir_);
  open_new_segment();
}

WalWriter::~WalWriter()
{
  close();
}

void WalWriter::open_new_segment()
{
  if (out_.is_open()) {
    out_.flush();
    out_.close();
  }
  ++current_segment_id_;
  current_segment_offset_ = 0;
  const std::string path = (fs::path(dir_) / segment_filename(current_segment_id_)).string();
  out_.open(path, std::ios::binary | std::ios::trunc);
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
    current_segment_offset_ + rec.size() > segment_bytes_)
  {
    open_new_segment();
  }

  if (!out_.is_open()) {
    return false;
  }
  const uint64_t offset = current_segment_offset_;
  out_.write(reinterpret_cast<const char *>(rec.data()), static_cast<std::streamsize>(rec.size()));
  if (!out_.good()) {
    return false;
  }
  index_.add(timestamp_ns, current_segment_id_, offset, static_cast<uint32_t>(rec.size()));
  current_segment_offset_ += rec.size();
  ++records_written_;
  return true;
}

void WalWriter::flush()
{
  if (out_.is_open()) {
    out_.flush();
  }
}

void WalWriter::close()
{
  if (out_.is_open()) {
    out_.flush();
    out_.close();
  }
}

}  // namespace sensorforge::replay
