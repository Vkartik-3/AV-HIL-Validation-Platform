/*
==============================================================================
SensorForge - Binary frame protocol codec (declarations)
Part of the SensorForge AV HIL validation platform.

Split into a stateless layer (pure, fuzzable) and a stateful decoder:

  encode_frame()   builds a complete frame (header + header CRC + payload).
  decode_header()  pure validation of a single frame in a buffer. Checks
                   everything that does NOT require history: magic, version,
                   header_size, payload bounds, integer overflow, truncation,
                   header CRC and payload CRC. Safe to call on fully attacker-
                   controlled bytes -> this is the libFuzzer entry point.
  FrameDecoder     wraps decode_header() and additionally enforces per-stream
                   sequence and timestamp monotonicity (stateful).

No ROS2 dependency.
==============================================================================
*/

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <unordered_map>

#include "sensorforge/protocol/frame.hpp"

namespace sensorforge::protocol {

/**
 * @brief Encode a complete frame into a byte vector.
 *
 * Computes payload_size, payload_crc32c and header_crc32c. The caller supplies
 * the semantic fields. Throws std::length_error if payload_len > kMaxPayload.
 *
 * @note The payload is written verbatim; if the caller has already compressed
 *       it, they should set kFlagCompressed in @p flags.
 */
std::vector<uint8_t> encode_frame(
  SensorType sensor_type,
  uint64_t sequence,
  uint64_t timestamp_ns,
  uint16_t flags,
  const uint8_t * payload,
  size_t payload_len);

/**
 * @brief Stateless decode + validation of one frame at the start of @p data.
 *
 * On success @p out is filled and the payload occupies
 * data[kFrameOverhead, kFrameOverhead + out.payload_size). Performs every
 * check except sequence/timestamp regression (those need history).
 *
 * @return FrameError::kOk on success, otherwise the first check that failed.
 */
FrameError decode_header(const uint8_t * data, size_t size, FrameHeader & out);

/// Pointer to the payload region of a validated frame (no bounds recheck).
inline const uint8_t * payload_ptr(const uint8_t * data)
{
  return data + kFrameOverhead;
}

/**
 * @brief Stateful decoder enforcing per-stream monotonic sequence & timestamp.
 *
 * @p stream_key identifies the logical stream (the bridge uses one key per
 * topic). The first frame seen for a key is always accepted; subsequent frames
 * must have sequence >= last and timestamp_ns >= last for that key.
 */
class FrameDecoder
{
public:
  FrameError decode(
    const uint8_t * data, size_t size, uint64_t stream_key, FrameHeader & out);

  /// Forget all per-stream history (e.g. on reconnect).
  void reset() {last_.clear();}

private:
  struct StreamState
  {
    uint64_t last_sequence = 0;
    uint64_t last_timestamp_ns = 0;
    bool seen = false;
  };
  std::unordered_map<uint64_t, StreamState> last_;
};

}  // namespace sensorforge::protocol
