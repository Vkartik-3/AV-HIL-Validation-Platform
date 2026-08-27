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

/// Per-stream integrity accounting produced by FrameDecoder.
struct StreamIntegrity
{
  uint64_t frames_ok = 0;
  uint64_t sequence_gaps = 0;        // number of gap EVENTS observed
  uint64_t missing_sequences = 0;    // total count of skipped sequence numbers
  uint64_t sequence_regressions = 0; // out-of-order / replayed sequences
  uint64_t timestamp_regressions = 0;
  uint64_t resyncs = 0;              // times this stream's state was re-based
  uint64_t last_sequence = 0;
  uint64_t last_timestamp_ns = 0;
};

/// Aggregate integrity across every tracked stream.
struct DecoderStats
{
  uint64_t frames_ok = 0;
  uint64_t sequence_gaps = 0;
  uint64_t missing_sequences = 0;
  uint64_t sequence_regressions = 0;
  uint64_t timestamp_regressions = 0;
  uint64_t resyncs = 0;
  uint64_t streams_tracked = 0;
  uint64_t streams_evicted = 0;
};

/**
 * @brief Stateful decoder tracking per-stream sequence and timestamp integrity.
 *
 * Three behaviours changed here relative to the original implementation, each
 * fixing a defect the audit identified:
 *
 *  1. RECOVERY INSTEAD OF WEDGING. A timestamp that goes backwards is COUNTED,
 *     not rejected. The original returned kTimestampRegression and the bridge
 *     dropped the frame, so a single NTP step backwards rejected every
 *     subsequent frame until the wall clock caught up -- permanently, since
 *     nothing ever called reset(). Senders now emit monotonised timestamps
 *     (core/clock.hpp), so a regression here means the peer's clock moved and
 *     the correct response is to record it and keep going.
 *
 *  2. GAPS ARE COUNTED, NOT JUST REGRESSIONS. A forward jump in sequence means
 *     frames were lost in transit; the decoder now reports how many. A
 *     BACKWARDS jump is a regression, and after kResyncThreshold consecutive
 *     regressions the stream re-bases rather than rejecting forever (the peer
 *     restarted and its sequence counter went back to zero).
 *
 *  3. STATE IS BOUNDED. stream_key is remote-influenced, so the map is capped
 *     at kMaxStreams entries; further keys are folded onto an overflow bucket
 *     and counted, so an adversarial or buggy peer cannot grow memory without
 *     limit.
 */
class FrameDecoder
{
public:
  static constexpr size_t kMaxStreams = 256;
  /// Consecutive regressions after which a stream re-bases on the new value.
  static constexpr uint64_t kResyncThreshold = 8;

  /**
   * @brief Validate one frame and update @p stream_key's integrity state.
   *
   * Returns kOk for any frame that passes structural validation. Sequence and
   * timestamp anomalies are reported through stats(), not by rejecting the
   * frame, except for a sequence regression that has not yet exceeded the
   * resync threshold -- those are genuine duplicates/reorders and are rejected
   * with kSequenceRegression so the caller does not republish stale data.
   */
  FrameError decode(
    const uint8_t * data, size_t size, uint64_t stream_key, FrameHeader & out);

  /// Integrity for one stream (all-zero if the key is unknown).
  StreamIntegrity stream(uint64_t stream_key) const;

  /// Aggregate across all tracked streams.
  DecoderStats stats() const;

  /// Forget all per-stream history (e.g. on an explicit link reset).
  void reset() {last_.clear(); evicted_ = 0;}

private:
  struct StreamState
  {
    uint64_t last_sequence = 0;
    uint64_t last_timestamp_ns = 0;
    bool seen = false;
    uint64_t consecutive_regressions = 0;
    StreamIntegrity integrity;
  };
  std::unordered_map<uint64_t, StreamState> last_;
  uint64_t evicted_ = 0;
};

}  // namespace sensorforge::protocol
