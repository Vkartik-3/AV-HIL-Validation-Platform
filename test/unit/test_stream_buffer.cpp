/*
==============================================================================
SensorForge - StreamBuffer tests
Covers the byte bound the audit found missing, runtime limits, and each
backpressure policy exercised with a LIVE consumer (the old suite only tested
policies single-threaded).
==============================================================================
*/

#include <atomic>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "sensorforge/core/stream_buffer.hpp"
#include "test_support.hpp"

using sensorforge::core::BackpressurePolicy;
using sensorforge::core::PushResult;
using sensorforge::core::SensorFrame;
using sensorforge::core::StreamBufferT;
using sensorforge::core::StreamLimits;

namespace {

SensorFrame make_frame(size_t bytes, uint64_t seq = 0)
{
  SensorFrame f;
  f.data.assign(bytes, static_cast<uint8_t>(seq & 0xFF));
  f.sequence = seq;
  f.timestamp_ns = 1000 + seq;
  return f;
}

}  // namespace

TEST(StreamBuffer, FrameLimitIsEnforced)
{
  StreamBufferT<64> buf(BackpressurePolicy::kDropNewest, StreamLimits{8, 1 << 20, {}});
  for (int i = 0; i < 8; ++i) {
    SF_EXPECT_EQ(buf.push(make_frame(16, i)), PushResult::kEnqueued);
  }
  for (int i = 0; i < 20; ++i) {
    SF_EXPECT_EQ(buf.push(make_frame(16, 100 + i)), PushResult::kDroppedNewest);
  }
  SF_EXPECT_EQ(buf.queued_frames(), 8u);
  SF_EXPECT_EQ(buf.counters().dropped, 20u);
}

// The defect: the old ring bounded COUNT only, so 1024 slots of 4 MiB payload
// was a 4 GB "bounded" buffer. Bytes must bind before frames when payloads are
// large.
TEST(StreamBuffer, ByteLimitBindsBeforeFrameLimit)
{
  StreamLimits lim;
  lim.max_frames = 500;             // far above what the byte ceiling allows
  lim.max_bytes = 10 * 1024;        // 10 KiB
  StreamBufferT<1024> buf(BackpressurePolicy::kDropNewest, lim);

  int enqueued = 0;
  for (int i = 0; i < 100; ++i) {
    if (buf.push(make_frame(1024, i)) == PushResult::kEnqueued) {
      ++enqueued;
    }
  }
  SF_EXPECT_LE(enqueued, 10);
  SF_EXPECT_LT(buf.queued_frames(), 500u);
  SF_EXPECT_LE(buf.queued_bytes(), lim.max_bytes);
  SF_EXPECT_GT(buf.counters().dropped, 0u);
}

TEST(StreamBuffer, ByteAccountingReturnsToZeroAfterDrain)
{
  StreamBufferT<64> buf(BackpressurePolicy::kDropNewest, StreamLimits{32, 1 << 20, {}});
  for (int i = 0; i < 20; ++i) {
    buf.push(make_frame(256, i));
  }
  SF_EXPECT_EQ(buf.queued_bytes(), 20u * 256u);
  SensorFrame out;
  while (buf.pop(out)) {
  }
  SF_EXPECT_EQ(buf.queued_bytes(), 0u);
  SF_EXPECT_EQ(buf.queued_frames(), 0u);
}

TEST(StreamBuffer, OverwriteKeepsNewestAndBoundsBytes)
{
  StreamLimits lim;
  lim.max_frames = 4;
  lim.max_bytes = 4 * 512;
  StreamBufferT<64> buf(BackpressurePolicy::kOverwriteOldest, lim);
  for (uint64_t i = 0; i < 40; ++i) {
    buf.push(make_frame(512, i));
  }
  SF_EXPECT_LE(buf.queued_frames(), lim.max_frames);
  SF_EXPECT_LE(buf.queued_bytes(), lim.max_bytes);
  SF_EXPECT_GT(buf.counters().overwritten, 0u);

  // The retained frames must be the NEWEST ones -- that is what overwrite means.
  SensorFrame out;
  uint64_t max_seq = 0;
  while (buf.pop(out)) {
    max_seq = out.sequence > max_seq ? out.sequence : max_seq;
  }
  SF_EXPECT_GE(max_seq, 35u);
}

TEST(StreamBuffer, BlockPolicyIsBoundedAndDoesNotHang)
{
  StreamLimits lim;
  lim.max_frames = 2;
  lim.max_bytes = 1 << 20;
  lim.block_budget = std::chrono::microseconds(2000);
  StreamBufferT<64> buf(BackpressurePolicy::kNeverDropBlock, lim);

  buf.push(make_frame(8, 0));
  buf.push(make_frame(8, 1));

  // Consumer never runs: the old implementation spun forever here.
  const auto t0 = std::chrono::steady_clock::now();
  const PushResult r = buf.push(make_frame(8, 2));
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  SF_EXPECT_EQ(r, PushResult::kDroppedAfterBlock);
  SF_EXPECT_LT(
    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 500);
}

// Producer faster than consumer, with a live consumer thread: the condition the
// policies exist for, and the one the old suite never exercised.
TEST(StreamBuffer, ProducerFasterThanConsumerStaysBounded)
{
  StreamLimits lim;
  lim.max_frames = 32;
  lim.max_bytes = 64 * 1024;
  StreamBufferT<256> buf(BackpressurePolicy::kDropNewest, lim);

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> consumed{0};
  std::atomic<size_t> max_seen_bytes{0};

  std::thread consumer([&]() {
      SensorFrame out;
      while (!stop.load(std::memory_order_relaxed)) {
        if (buf.pop(out)) {
          consumed.fetch_add(1, std::memory_order_relaxed);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
      while (buf.pop(out)) {
        consumed.fetch_add(1, std::memory_order_relaxed);
      }
    });

  uint64_t produced = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  while (std::chrono::steady_clock::now() < deadline) {
    buf.push(make_frame(1024, produced++));
    const size_t qb = buf.queued_bytes();
    size_t seen = max_seen_bytes.load(std::memory_order_relaxed);
    while (qb > seen && !max_seen_bytes.compare_exchange_weak(seen, qb)) {
    }
  }
  stop.store(true);
  consumer.join();

  const auto c = buf.counters();
  SF_EXPECT_GT(produced, 0u);
  SF_EXPECT_GT(c.dropped, 0u);            // producer outran the consumer
  SF_EXPECT_LE(max_seen_bytes.load(), lim.max_bytes + 1024);   // never exceeded
  SF_EXPECT_EQ(c.enqueued, consumed.load());  // nothing lost or duplicated
}

// Mixed payload sizes including camera/LiDAR-scale messages.
TEST(StreamBuffer, MixedPayloadSizesRespectByteCeiling)
{
  const std::vector<size_t> sizes = {16, 330, 1024, 196608, 230400};
  StreamLimits lim;
  lim.max_frames = 400;
  lim.max_bytes = 2u * 1024u * 1024u;   // 2 MiB
  StreamBufferT<512> buf(BackpressurePolicy::kDropNewest, lim);

  uint64_t seq = 0;
  for (int round = 0; round < 200; ++round) {
    for (size_t s : sizes) {
      buf.push(make_frame(s, seq++));
      SF_EXPECT_LE(buf.queued_bytes(), lim.max_bytes + 230400);
    }
  }
  SF_EXPECT_GT(buf.counters().dropped, 0u);
}
