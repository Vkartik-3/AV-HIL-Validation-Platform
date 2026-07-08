/*
==============================================================================
SensorForge - Binary frame protocol codec (implementation)
Part of the SensorForge AV HIL validation platform.
See sensorforge/protocol/frame_codec.hpp for the contract.
==============================================================================
*/

#include "sensorforge/protocol/frame_codec.hpp"

#include <cstring>
#include <stdexcept>

#include "sensorforge/protocol/crc32c.hpp"

namespace sensorforge::protocol {

namespace {

// ---- little-endian write helpers -------------------------------------------
inline void put_u16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline void put_u32(uint8_t * p, uint32_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

inline void put_u64(uint8_t * p, uint64_t v)
{
  for (int i = 0; i < 8; ++i) {
    p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
  }
}

// ---- little-endian read helpers --------------------------------------------
inline uint16_t get_u16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline uint32_t get_u32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

inline uint64_t get_u64(const uint8_t * p)
{
  uint64_t v = 0;
  for (int i = 0; i < 8; ++i) {
    v |= static_cast<uint64_t>(p[i]) << (8 * i);
  }
  return v;
}

// Field offsets within the header (must match frame.hpp documentation).
constexpr size_t kOffMagic = 0;
constexpr size_t kOffVersion = 4;
constexpr size_t kOffSensorType = 6;
constexpr size_t kOffFlags = 8;
constexpr size_t kOffHeaderSize = 10;
constexpr size_t kOffSequence = 12;
constexpr size_t kOffTimestamp = 20;
constexpr size_t kOffPayloadSize = 28;
constexpr size_t kOffPayloadCrc = 32;
constexpr size_t kOffHeaderCrc = kHeaderSize;  // 36

}  // namespace

std::vector<uint8_t> encode_frame(
  SensorType sensor_type,
  uint64_t sequence,
  uint64_t timestamp_ns,
  uint16_t flags,
  const uint8_t * payload,
  size_t payload_len)
{
  if (payload_len > kMaxPayload) {
    throw std::length_error("SensorForge frame payload exceeds kMaxPayload");
  }

  std::vector<uint8_t> out(kFrameOverhead + payload_len);
  uint8_t * p = out.data();

  put_u32(p + kOffMagic, kMagic);
  put_u16(p + kOffVersion, kVersion);
  put_u16(p + kOffSensorType, static_cast<uint16_t>(sensor_type));
  put_u16(p + kOffFlags, flags);
  put_u16(p + kOffHeaderSize, kHeaderSize);
  put_u64(p + kOffSequence, sequence);
  put_u64(p + kOffTimestamp, timestamp_ns);
  put_u32(p + kOffPayloadSize, static_cast<uint32_t>(payload_len));

  const uint32_t payload_crc =
    payload_len ? crc::crc32c(payload, payload_len) : crc::crc32c(payload, 0);
  put_u32(p + kOffPayloadCrc, payload_crc);

  // Header CRC covers bytes [0, kHeaderSize).
  const uint32_t header_crc = crc::crc32c(p, kHeaderSize);
  put_u32(p + kOffHeaderCrc, header_crc);

  if (payload_len) {
    std::memcpy(p + kFrameOverhead, payload, payload_len);
  }
  return out;
}

FrameError decode_header(const uint8_t * data, size_t size, FrameHeader & out)
{
  // Need at least the fixed overhead (header + header CRC) present.
  if (data == nullptr || size < kFrameOverhead) {
    return FrameError::kTruncatedHeader;
  }

  out.magic = get_u32(data + kOffMagic);
  if (out.magic != kMagic) {
    return FrameError::kBadMagic;
  }

  out.version = get_u16(data + kOffVersion);
  if (out.version > kVersion) {
    return FrameError::kUnsupportedVersion;
  }

  out.sensor_type = static_cast<SensorType>(get_u16(data + kOffSensorType));
  out.flags = get_u16(data + kOffFlags);

  out.header_size = get_u16(data + kOffHeaderSize);
  if (out.header_size != kHeaderSize) {
    return FrameError::kBadHeaderSize;
  }

  out.sequence = get_u64(data + kOffSequence);
  out.timestamp_ns = get_u64(data + kOffTimestamp);
  out.payload_size = get_u32(data + kOffPayloadSize);
  out.payload_crc32c = get_u32(data + kOffPayloadCrc);

  if (out.payload_size > kMaxPayload) {
    return FrameError::kPayloadTooLarge;
  }

  // Overflow-safe total-length check: kFrameOverhead + payload_size must not
  // wrap and must fit inside the provided buffer. payload_size is already
  // bounded by kMaxPayload above, so the addition cannot overflow size_t on
  // any supported platform, but we compute defensively regardless.
  const size_t need = kFrameOverhead + static_cast<size_t>(out.payload_size);
  if (need < kFrameOverhead) {
    return FrameError::kLengthOverflow;
  }
  if (size < need) {
    return FrameError::kTruncatedPayload;
  }

  // Header CRC over [0, kHeaderSize).
  const uint32_t stored_header_crc = get_u32(data + kOffHeaderCrc);
  if (crc::crc32c(data, kHeaderSize) != stored_header_crc) {
    return FrameError::kHeaderCrcMismatch;
  }

  // Payload CRC over [kFrameOverhead, kFrameOverhead + payload_size).
  const uint32_t payload_crc = out.payload_size
    ? crc::crc32c(data + kFrameOverhead, out.payload_size)
    : crc::crc32c(data + kFrameOverhead, 0);
  if (payload_crc != out.payload_crc32c) {
    return FrameError::kPayloadCrcMismatch;
  }

  return FrameError::kOk;
}

FrameError FrameDecoder::decode(
  const uint8_t * data, size_t size, uint64_t stream_key, FrameHeader & out)
{
  const FrameError err = decode_header(data, size, out);
  if (err != FrameError::kOk) {
    return err;
  }

  StreamState & st = last_[stream_key];
  if (st.seen) {
    if (out.sequence < st.last_sequence) {
      return FrameError::kSequenceRegression;
    }
    if (out.timestamp_ns < st.last_timestamp_ns) {
      return FrameError::kTimestampRegression;
    }
  }
  st.seen = true;
  st.last_sequence = out.sequence;
  st.last_timestamp_ns = out.timestamp_ns;
  return FrameError::kOk;
}

}  // namespace sensorforge::protocol
