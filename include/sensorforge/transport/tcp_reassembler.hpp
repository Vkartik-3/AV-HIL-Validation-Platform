/*
==============================================================================
SensorForge - TCP stream reassembler (Extension F, pure/fuzzable)
Part of the SensorForge AV HIL validation platform.

A pure, push-based reassembler for the legacy TCP wire framing used by
TcpInterface::receive_thread:

    0xAB 0xCD | payload_size (uint32, big-endian) | payload[payload_size]

The original state machine lived inline in receive_thread() and read from a
blocking QueueStream, which is impossible to fuzz. This extracts the identical
logic into a byte-at-a-time state machine with:
  - a hard payload-size bound (rejects impossible lengths -> no OOM), and
  - resync on a bad start sequence or oversized length.

It has no ROS2 / Boost dependency, so it is the libFuzzer entry point for TCP
fragment reassembly and is the intended replacement for the inline logic.
==============================================================================
*/

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

namespace sensorforge::transport {

class TcpReassembler
{
public:
  using PayloadCallback = std::function<void (const uint8_t *, size_t)>;

  explicit TcpReassembler(PayloadCallback cb, uint32_t max_payload = 8u * 1024u * 1024u)
  : cb_(std::move(cb)), max_payload_(max_payload) {}

  /// Feed a chunk of received bytes; emits each complete payload via callback.
  void feed(const uint8_t * data, size_t n)
  {
    for (size_t i = 0; i < n; ++i) {
      feed_byte(data[i]);
    }
  }

  void reset()
  {
    state_ = 0;
    size_idx_ = 0;
    payload_size_ = 0;
    payload_.clear();
  }

  uint64_t payloads_emitted() const {return payloads_emitted_;}
  uint64_t resyncs() const {return resyncs_;}

private:
  void feed_byte(uint8_t b)
  {
    switch (state_) {
      case 0:  // expect first start byte
        if (b == 0xAB) {
          state_ = 1;
        }
        break;
      case 1:  // expect second start byte
        if (b == 0xCD) {
          state_ = 2;
          size_idx_ = 0;
        } else if (b == 0xAB) {
          state_ = 1;  // stay; could be the real start
        } else {
          state_ = 0;
          ++resyncs_;
        }
        break;
      case 2:  // 4 big-endian length bytes
        size_bytes_[size_idx_++] = b;
        if (size_idx_ == 4) {
          size_idx_ = 0;
          payload_size_ =
            (static_cast<uint32_t>(size_bytes_[0]) << 24) |
            (static_cast<uint32_t>(size_bytes_[1]) << 16) |
            (static_cast<uint32_t>(size_bytes_[2]) << 8) |
            static_cast<uint32_t>(size_bytes_[3]);
          if (payload_size_ > max_payload_) {
            // Impossible length -> drop and resync from scratch (no allocation).
            state_ = 0;
            payload_size_ = 0;
            ++resyncs_;
          } else if (payload_size_ == 0) {
            cb_(nullptr, 0);
            ++payloads_emitted_;
            state_ = 0;
          } else {
            payload_.clear();
            payload_.reserve(payload_size_);
            state_ = 3;
          }
        }
        break;
      case 3:  // payload bytes
        payload_.push_back(b);
        if (payload_.size() == payload_size_) {
          cb_(payload_.data(), payload_.size());
          ++payloads_emitted_;
          payload_.clear();
          state_ = 0;
        }
        break;
      default:
        state_ = 0;
    }
  }

  PayloadCallback cb_;
  uint32_t max_payload_;
  uint8_t state_ = 0;
  uint8_t size_bytes_[4] = {0, 0, 0, 0};
  uint8_t size_idx_ = 0;
  uint32_t payload_size_ = 0;
  std::vector<uint8_t> payload_;
  uint64_t payloads_emitted_ = 0;
  uint64_t resyncs_ = 0;
};

}  // namespace sensorforge::transport
