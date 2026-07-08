/*
==============================================================================
SensorForge - Binary frame protocol
Part of the SensorForge AV HIL validation platform.

Wire format (all multi-byte fields little-endian):

  Offset  Size  Field
  ------  ----  -----------------------------------------------------------
     0     4    magic          0x53464844 ("SFHD")
     4     2    version        current = 0x0001
     6     2    sensor_type    SensorType enum (LIDAR/CAMERA/IMU/GPS/CAN/CTRL)
     8     2    flags          bit0 = fragmented, bit1 = compressed
    10     2    header_size    == kHeaderSize (36); self-describing
    12     8    sequence       per-stream, monotonically increasing
    20     8    timestamp_ns   wall-clock nanoseconds, monotonic per stream
    28     4    payload_size   payload length in bytes (<= kMaxPayload)
    32     4    payload_crc32c CRC32C of the payload bytes only
  ------  ----
    36          <-- kHeaderSize
    36     4    header_crc32c  CRC32C of header bytes [0, 36)
  ------  ----
    40          <-- kFrameOverhead (fixed overhead before payload)
    40     N    payload

SPEC DISCREPANCY (flagged honestly): the original spec text listed the
header as "34 bytes" and total overhead as "38 bytes", but the field list
it gave (4+2+2+2+2+8+8+4+4) sums to 36, not 34 -- the 2-byte header_size
field was omitted from the hand arithmetic. We use the internally consistent
value: kHeaderSize = 36, kFrameOverhead = 40. Validation checks
header_size == kHeaderSize, so old/new frames can never be confused, and a
future version bump can change the layout cleanly.

This header is intentionally free of ROS2 dependencies.
==============================================================================
*/

#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <vector>
#include <string_view>

namespace sensorforge::protocol {

/// ASCII "SFHD" (SensorForge HeaDer) read little-endian.
inline constexpr uint32_t kMagic = 0x53464844u;
inline constexpr uint16_t kVersion = 0x0001u;
inline constexpr uint16_t kHeaderSize = 36u;
inline constexpr size_t kFrameOverhead = kHeaderSize + 4u;  // + header_crc32c
inline constexpr uint32_t kMaxPayload = 4u * 1024u * 1024u;  // 4 MiB guard

/// Frame flag bits.
enum FrameFlags : uint16_t {
  kFlagNone = 0,
  kFlagFragmented = 1u << 0,
  kFlagCompressed = 1u << 1,
};

/// Logical stream classification carried in the frame header.
enum class SensorType : uint16_t {
  kUnknown = 0,
  kLidar = 1,
  kCamera = 2,
  kImu = 3,
  kGps = 4,
  kCan = 5,
  kControl = 6,  // CTRL: bridged non-sensor ROS2 topics
};

/// Parsed / to-be-encoded header fields (not the on-wire layout; see codec).
struct FrameHeader
{
  uint32_t magic = kMagic;
  uint16_t version = kVersion;
  SensorType sensor_type = SensorType::kUnknown;
  uint16_t flags = kFlagNone;
  uint16_t header_size = kHeaderSize;
  uint64_t sequence = 0;
  uint64_t timestamp_ns = 0;
  uint32_t payload_size = 0;
  uint32_t payload_crc32c = 0;
};

/// Result of validating/decoding a frame.
enum class FrameError {
  kOk = 0,
  kTruncatedHeader,      // fewer than kFrameOverhead bytes available
  kBadMagic,
  kUnsupportedVersion,   // version > kVersion
  kBadHeaderSize,        // header_size != kHeaderSize
  kPayloadTooLarge,      // payload_size > kMaxPayload
  kLengthOverflow,       // header_size + payload_size overflow / exceeds buffer
  kTruncatedPayload,     // buffer smaller than overhead + payload_size
  kHeaderCrcMismatch,
  kPayloadCrcMismatch,
  kSequenceRegression,   // sequence < last seen for this stream
  kTimestampRegression,  // timestamp_ns < last seen for this stream
};

inline std::string_view to_string(FrameError e)
{
  switch (e) {
    case FrameError::kOk: return "ok";
    case FrameError::kTruncatedHeader: return "truncated_header";
    case FrameError::kBadMagic: return "bad_magic";
    case FrameError::kUnsupportedVersion: return "unsupported_version";
    case FrameError::kBadHeaderSize: return "bad_header_size";
    case FrameError::kPayloadTooLarge: return "payload_too_large";
    case FrameError::kLengthOverflow: return "length_overflow";
    case FrameError::kTruncatedPayload: return "truncated_payload";
    case FrameError::kHeaderCrcMismatch: return "header_crc_mismatch";
    case FrameError::kPayloadCrcMismatch: return "payload_crc_mismatch";
    case FrameError::kSequenceRegression: return "sequence_regression";
    case FrameError::kTimestampRegression: return "timestamp_regression";
  }
  return "unknown";
}

inline std::string_view to_string(SensorType t)
{
  switch (t) {
    case SensorType::kUnknown: return "unknown";
    case SensorType::kLidar: return "lidar";
    case SensorType::kCamera: return "camera";
    case SensorType::kImu: return "imu";
    case SensorType::kGps: return "gps";
    case SensorType::kCan: return "can";
    case SensorType::kControl: return "ctrl";
  }
  return "unknown";
}

}  // namespace sensorforge::protocol
