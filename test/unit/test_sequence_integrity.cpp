/*
==============================================================================
SensorForge - Timestamp / sequence integrity tests
Pins the three defects the audit found in FrameDecoder:
  - a backwards wall-clock step permanently wedged the link
  - sequence GAPS were never counted (only regressions rejected)
  - per-stream state was keyed by a constant 0 and was unbounded
==============================================================================
*/

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "sensorforge/core/clock.hpp"
#include "sensorforge/protocol/frame_codec.hpp"
#include "test_support.hpp"

using namespace sensorforge::protocol;
using sensorforge::core::MonotonicWallClock;

namespace {

std::vector<uint8_t> frame_at(uint64_t seq, uint64_t ts, SensorType t = SensorType::kImu)
{
  const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  return encode_frame(t, seq, ts, kFlagNone, payload, sizeof(payload));
}

}  // namespace

// The wedge: previously every frame after a backwards clock step was rejected,
// forever, because nothing ever called reset().
TEST(SequenceIntegrity, BackwardsClockStepDoesNotWedgeTheLink)
{
  FrameDecoder dec;
  FrameHeader h;
  for (uint64_t i = 0; i < 100; ++i) {
    const auto f = frame_at(i, 1000000 + i * 1000);
    SF_ASSERT_EQ(dec.decode(f.data(), f.size(), 1, h), FrameError::kOk);
  }
  // NTP steps the wall clock back by a second; sequence keeps advancing.
  for (uint64_t i = 100; i < 200; ++i) {
    const auto f = frame_at(i, 500000 + i * 1000);
    SF_EXPECT_EQ(dec.decode(f.data(), f.size(), 1, h), FrameError::kOk);
  }
  const auto s = dec.stats();
  SF_EXPECT_GT(s.timestamp_regressions, 0u);   // observed
  SF_EXPECT_EQ(s.frames_ok, 200u);             // and recovered from
}

TEST(SequenceIntegrity, MissingSequenceRangeIsCounted)
{
  FrameDecoder dec;
  FrameHeader h;
  for (uint64_t i = 0; i < 10; ++i) {
    const auto f = frame_at(i, 1000 + i);
    SF_ASSERT_EQ(dec.decode(f.data(), f.size(), 7, h), FrameError::kOk);
  }
  // Sequences 10..24 are lost in transit; 25 arrives.
  const auto f = frame_at(25, 2000);
  SF_EXPECT_EQ(dec.decode(f.data(), f.size(), 7, h), FrameError::kOk);

  const auto st = dec.stream(7);
  SF_EXPECT_EQ(st.sequence_gaps, 1u);
  SF_EXPECT_EQ(st.missing_sequences, 15u);
}

TEST(SequenceIntegrity, NoGapsReportedOnContiguousStream)
{
  FrameDecoder dec;
  FrameHeader h;
  for (uint64_t i = 0; i < 2000; ++i) {
    const auto f = frame_at(i, 1000 + i);
    SF_ASSERT_EQ(dec.decode(f.data(), f.size(), 3, h), FrameError::kOk);
  }
  const auto st = dec.stream(3);
  SF_EXPECT_EQ(st.sequence_gaps, 0u);
  SF_EXPECT_EQ(st.missing_sequences, 0u);
  SF_EXPECT_EQ(st.frames_ok, 2000u);
}

TEST(SequenceIntegrity, InterleavedTopicsTrackIndependently)
{
  FrameDecoder dec;
  FrameHeader h;
  const uint64_t k_imu = 11, k_cam = 22, k_gps = 33;
  for (uint64_t i = 0; i < 50; ++i) {
    auto a = frame_at(i, 1000 + i, SensorType::kImu);
    auto b = frame_at(i * 2, 1000 + i, SensorType::kCamera);   // its own numbering
    auto c = frame_at(i, 1000 + i, SensorType::kGps);
    SF_EXPECT_EQ(dec.decode(a.data(), a.size(), k_imu, h), FrameError::kOk);
    SF_EXPECT_EQ(dec.decode(b.data(), b.size(), k_cam, h), FrameError::kOk);
    SF_EXPECT_EQ(dec.decode(c.data(), c.size(), k_gps, h), FrameError::kOk);
  }
  // IMU and GPS are contiguous; the camera stream advances by 2 each time, so
  // its gaps must be attributed to IT and not to the others.
  SF_EXPECT_EQ(dec.stream(k_imu).missing_sequences, 0u);
  SF_EXPECT_EQ(dec.stream(k_gps).missing_sequences, 0u);
  SF_EXPECT_EQ(dec.stream(k_cam).missing_sequences, 49u);
  SF_EXPECT_EQ(dec.stats().streams_tracked, 3u);
}

// A peer restart resets its sequence to 0. A handful of regressions are real
// reorders and must be rejected; a sustained run means a restart and must
// resync rather than reject forever.
TEST(SequenceIntegrity, PeerRestartResyncsInsteadOfWedging)
{
  FrameDecoder dec;
  FrameHeader h;
  for (uint64_t i = 0; i < 500; ++i) {
    const auto f = frame_at(i, 1000 + i);
    SF_ASSERT_EQ(dec.decode(f.data(), f.size(), 5, h), FrameError::kOk);
  }
  int rejected = 0, accepted = 0;
  for (uint64_t i = 0; i < 40; ++i) {   // peer restarted: sequence back to 0
    const auto f = frame_at(i, 2000 + i);
    if (dec.decode(f.data(), f.size(), 5, h) == FrameError::kOk) {
      ++accepted;
    } else {
      ++rejected;
    }
  }
  SF_EXPECT_GT(rejected, 0);                       // early ones look like reorders
  SF_EXPECT_GT(accepted, 0);                       // then it resyncs
  SF_EXPECT_GE(dec.stream(5).resyncs, 1u);
}

TEST(SequenceIntegrity, IsolatedReorderIsStillRejected)
{
  FrameDecoder dec;
  FrameHeader h;
  for (uint64_t i = 0; i < 100; ++i) {
    const auto f = frame_at(i, 1000 + i);
    SF_ASSERT_EQ(dec.decode(f.data(), f.size(), 9, h), FrameError::kOk);
  }
  const auto dup = frame_at(50, 1050);
  SF_EXPECT_EQ(dec.decode(dup.data(), dup.size(), 9, h), FrameError::kSequenceRegression);
}

// stream_key is remote-influenced; the map must not grow without bound.
TEST(SequenceIntegrity, DecoderStateIsBounded)
{
  FrameDecoder dec;
  FrameHeader h;
  for (uint64_t k = 0; k < 5000; ++k) {
    const auto f = frame_at(0, 1000);
    dec.decode(f.data(), f.size(), k, h);
  }
  SF_EXPECT_LE(dec.stats().streams_tracked, FrameDecoder::kMaxStreams + 1);
}

TEST(SequenceIntegrity, ResetClearsHistory)
{
  FrameDecoder dec;
  FrameHeader h;
  const auto f = frame_at(10, 1000);
  dec.decode(f.data(), f.size(), 1, h);
  SF_EXPECT_EQ(dec.stats().streams_tracked, 1u);
  dec.reset();
  SF_EXPECT_EQ(dec.stats().streams_tracked, 0u);
}

// The producer-side half of the fix.
TEST(MonotonicWallClockTest, NeverRegressesAndCountsClockSteps)
{
  MonotonicWallClock c;
  uint64_t prev = 0;
  for (int i = 0; i < 10000; ++i) {
    const uint64_t v = c.next();
    SF_EXPECT_GT(v, prev);
    prev = v;
  }
  SF_EXPECT_GT(c.last(), 0u);
}

TEST(MonotonicWallClockTest, StampsAreAcceptedByTheDecoder)
{
  MonotonicWallClock c;
  FrameDecoder dec;
  FrameHeader h;
  for (uint64_t i = 0; i < 500; ++i) {
    const auto f = frame_at(i, c.next());
    SF_EXPECT_EQ(dec.decode(f.data(), f.size(), 1, h), FrameError::kOk);
  }
  SF_EXPECT_EQ(dec.stats().timestamp_regressions, 0u);
}
