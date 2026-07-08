/*
==============================================================================
SensorForge - WAL record codec (Extension D)
Part of the SensorForge AV HIL validation platform.

On-disk record layout (all multi-byte fields little-endian):

  Offset  Size  Field
  ------  ----  -----------------------------------------------------------
     0     4    record_length   total bytes of this record = 30 + N + 4
     4     8    timestamp_ns    capture wall-clock time
    12     2    sensor_type     SensorType enum
    14     8    sequence        per-stream monotonic
    22     4    payload_size    N
    26     4    payload_crc32c  CRC32C of payload bytes only
  ------  ----
    30          <-- kRecordHeaderSize
    30     N    payload
  ------  ----
    30+N   4    record_crc32c   CRC32C of bytes [0, 30+N) (header + payload)
  ------  ----
    34+N        <-- record_length

decode_record() is a pure function safe to call on fully attacker-controlled
bytes; it is the libFuzzer entry point for the replay-log reader. No ROS2
dependency.
==============================================================================
*/

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <vector>

#include "sensorforge/protocol/crc32c.hpp"
#include "sensorforge/protocol/frame.hpp"  // SensorType

namespace sensorforge::replay {

using protocol::SensorType;

inline constexpr size_t kRecordHeaderSize = 30;
inline constexpr size_t kRecordTrailerSize = 4;               // record_crc32c
inline constexpr size_t kRecordOverhead = kRecordHeaderSize + kRecordTrailerSize;  // 34
inline constexpr uint32_t kMaxRecordPayload = 4u * 1024u * 1024u;  // 4 MiB, matches frames
inline constexpr uint64_t kMaxRecordLength = kRecordOverhead + kMaxRecordPayload;

/// Parsed record header (semantic view; not the on-wire byte layout).
struct RecordHeader
{
  uint32_t record_length = 0;
  uint64_t timestamp_ns = 0;
  SensorType sensor_type = SensorType::kUnknown;
  uint64_t sequence = 0;
  uint32_t payload_size = 0;
  uint32_t payload_crc32c = 0;
};

enum class RecordError {
  kOk = 0,
  kTruncatedHeader,     // fewer than kRecordHeaderSize bytes available
  kBadLength,           // record_length < overhead or > kMaxRecordLength
  kLengthMismatch,      // record_length != kRecordOverhead + payload_size
  kPayloadTooLarge,     // payload_size > kMaxRecordPayload
  kTruncatedRecord,     // buffer smaller than record_length
  kPayloadCrcMismatch,
  kRecordCrcMismatch,
};

inline std::string_view to_string(RecordError e)
{
  switch (e) {
    case RecordError::kOk: return "ok";
    case RecordError::kTruncatedHeader: return "truncated_header";
    case RecordError::kBadLength: return "bad_length";
    case RecordError::kLengthMismatch: return "length_mismatch";
    case RecordError::kPayloadTooLarge: return "payload_too_large";
    case RecordError::kTruncatedRecord: return "truncated_record";
    case RecordError::kPayloadCrcMismatch: return "payload_crc_mismatch";
    case RecordError::kRecordCrcMismatch: return "record_crc_mismatch";
  }
  return "unknown";
}

// ---- little-endian helpers -------------------------------------------------
namespace detail {

inline void put_u16(std::vector<uint8_t> & v, uint16_t x)
{
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
}
inline void put_u32(std::vector<uint8_t> & v, uint32_t x)
{
  for (int i = 0; i < 4; ++i) {v.push_back(static_cast<uint8_t>((x >> (8 * i)) & 0xFF));}
}
inline void put_u64(std::vector<uint8_t> & v, uint64_t x)
{
  for (int i = 0; i < 8; ++i) {v.push_back(static_cast<uint8_t>((x >> (8 * i)) & 0xFF));}
}
inline uint16_t get_u16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
inline uint32_t get_u32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
inline uint64_t get_u64(const uint8_t * p)
{
  uint64_t v = 0;
  for (int i = 0; i < 8; ++i) {v |= static_cast<uint64_t>(p[i]) << (8 * i);}
  return v;
}

}  // namespace detail

/**
 * @brief Encode one complete record (header + payload + record CRC).
 */
inline std::vector<uint8_t> encode_record(
  uint64_t timestamp_ns, SensorType sensor_type, uint64_t sequence,
  const uint8_t * payload, size_t payload_len)
{
  const uint32_t n = static_cast<uint32_t>(payload_len);
  const uint32_t record_length = static_cast<uint32_t>(kRecordOverhead + payload_len);

  std::vector<uint8_t> out;
  out.reserve(record_length);
  detail::put_u32(out, record_length);
  detail::put_u64(out, timestamp_ns);
  detail::put_u16(out, static_cast<uint16_t>(sensor_type));
  detail::put_u64(out, sequence);
  detail::put_u32(out, n);
  detail::put_u32(out, payload_len ? crc::crc32c(payload, payload_len) : crc::crc32c(payload, 0));
  if (payload_len) {
    out.insert(out.end(), payload, payload + payload_len);
  }
  // record CRC over everything written so far (header + payload).
  const uint32_t record_crc = crc::crc32c(out.data(), out.size());
  detail::put_u32(out, record_crc);
  return out;
}

/**
 * @brief Pure decode + validation of one record at the start of @p data.
 *
 * On success @p out is filled and the payload occupies
 * data[kRecordHeaderSize, kRecordHeaderSize + out.payload_size). Every check
 * is bounds-safe on arbitrary input.
 */
inline RecordError decode_record(const uint8_t * data, size_t size, RecordHeader & out)
{
  if (data == nullptr || size < kRecordHeaderSize) {
    return RecordError::kTruncatedHeader;
  }
  out.record_length = detail::get_u32(data + 0);
  out.timestamp_ns = detail::get_u64(data + 4);
  out.sensor_type = static_cast<SensorType>(detail::get_u16(data + 12));
  out.sequence = detail::get_u64(data + 14);
  out.payload_size = detail::get_u32(data + 22);
  out.payload_crc32c = detail::get_u32(data + 26);

  if (out.record_length < kRecordOverhead || out.record_length > kMaxRecordLength) {
    return RecordError::kBadLength;
  }
  if (out.payload_size > kMaxRecordPayload) {
    return RecordError::kPayloadTooLarge;
  }
  if (static_cast<uint64_t>(out.record_length) !=
    kRecordOverhead + static_cast<uint64_t>(out.payload_size))
  {
    return RecordError::kLengthMismatch;
  }
  if (size < out.record_length) {
    return RecordError::kTruncatedRecord;
  }

  const uint8_t * payload = data + kRecordHeaderSize;
  const uint32_t pc = out.payload_size
    ? crc::crc32c(payload, out.payload_size) : crc::crc32c(payload, 0);
  if (pc != out.payload_crc32c) {
    return RecordError::kPayloadCrcMismatch;
  }

  const size_t crc_off = kRecordHeaderSize + out.payload_size;
  const uint32_t stored_record_crc = detail::get_u32(data + crc_off);
  if (crc::crc32c(data, crc_off) != stored_record_crc) {
    return RecordError::kRecordCrcMismatch;
  }
  return RecordError::kOk;
}

/// Pointer to the payload of a validated record.
inline const uint8_t * record_payload(const uint8_t * data)
{
  return data + kRecordHeaderSize;
}

}  // namespace sensorforge::replay
