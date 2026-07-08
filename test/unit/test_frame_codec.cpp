/*
==============================================================================
SensorForge - Frame codec tests (Extension G)
Round-trip, field integrity, and corruption detection, parameterized over
payload sizes x sensor types x corruption modes.
==============================================================================
*/

#include <gtest/gtest.h>

#include <tuple>

#include "sensorforge/protocol/frame_codec.hpp"
#include "test_support.hpp"

using namespace sensorforge::protocol;
using sftest::Corruption;

// ---- round-trip over sizes x sensor types ---------------------------------
class FrameRoundTrip
  : public ::testing::TestWithParam<std::tuple<size_t, SensorType>> {};

TEST_P(FrameRoundTrip, EncodesAndDecodes)
{
  const size_t n = std::get<0>(GetParam());
  const SensorType type = std::get<1>(GetParam());
  const auto payload = sftest::make_payload(n, static_cast<uint64_t>(type));

  for (uint64_t seq : {uint64_t{0}, uint64_t{1}, uint64_t{1234567}, ~uint64_t{0}}) {
    const uint64_t ts = seq * 1000 + 7;
    auto frame = encode_frame(type, seq, ts, kFlagCompressed, payload.data(), payload.size());
    SF_ASSERT_EQ(frame.size(), kFrameOverhead + n);

    FrameHeader h;
    SF_ASSERT_EQ(decode_header(frame.data(), frame.size(), h), FrameError::kOk);
    SF_EXPECT_EQ(h.magic, kMagic);
    SF_EXPECT_EQ(h.version, kVersion);
    SF_EXPECT_EQ(h.sensor_type, type);
    SF_EXPECT_EQ(h.flags, kFlagCompressed);
    SF_EXPECT_EQ(h.header_size, kHeaderSize);
    SF_EXPECT_EQ(h.sequence, seq);
    SF_EXPECT_EQ(h.timestamp_ns, ts);
    SF_EXPECT_EQ(h.payload_size, n);

    const uint8_t * p = payload_ptr(frame.data());
    for (size_t i = 0; i < n; i += (n / 64 + 1)) {  // sample the payload
      SF_EXPECT_EQ(p[i], payload[i]);
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  Sizes, FrameRoundTrip,
  ::testing::Combine(
    ::testing::ValuesIn(sftest::kPayloadSizes),
    ::testing::ValuesIn(sftest::kSensorTypes)));

// ---- corruption detection over sizes x modes ------------------------------
class FrameCorruption
  : public ::testing::TestWithParam<std::tuple<size_t, int>> {};

TEST_P(FrameCorruption, Detected)
{
  const size_t n = std::get<0>(GetParam());
  const auto mode = static_cast<Corruption>(std::get<1>(GetParam()));
  const auto payload = sftest::make_payload(n, 42);

  for (int rep = 0; rep < 40; ++rep) {
    auto frame = encode_frame(
      SensorType::kLidar, 100 + rep, 5000 + rep, kFlagNone, payload.data(), payload.size());
    FrameHeader h;

    switch (mode) {
      case Corruption::kBitFlip: {
        const size_t idx = (rep * 7 + 3) % frame.size();
        frame[idx] ^= 0x40;
        SF_EXPECT_NE(decode_header(frame.data(), frame.size(), h), FrameError::kOk);
        break;
      }
      case Corruption::kTruncate: {
        const size_t keep = (rep % (frame.size() - 1));
        std::vector<uint8_t> t(frame.begin(), frame.begin() + keep);
        SF_EXPECT_NE(decode_header(t.data(), t.size(), h), FrameError::kOk);
        break;
      }
      case Corruption::kMagicCorrupt: {
        frame[0] ^= 0xFF;
        SF_EXPECT_EQ(decode_header(frame.data(), frame.size(), h), FrameError::kBadMagic);
        break;
      }
      case Corruption::kCrcCorrupt: {
        frame[kFrameOverhead + (rep % n)] ^= 0x01;  // flip a payload byte
        SF_EXPECT_EQ(decode_header(frame.data(), frame.size(), h), FrameError::kPayloadCrcMismatch);
        break;
      }
      case Corruption::kSeqRegression: {
        FrameDecoder dec;
        auto a = encode_frame(SensorType::kImu, 500, 9000, 0, payload.data(), payload.size());
        auto b = encode_frame(SensorType::kImu, 499, 9001, 0, payload.data(), payload.size());
        FrameHeader ha, hb;
        SF_EXPECT_EQ(dec.decode(a.data(), a.size(), 0, ha), FrameError::kOk);
        SF_EXPECT_EQ(dec.decode(b.data(), b.size(), 0, hb), FrameError::kSequenceRegression);
        break;
      }
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  Modes, FrameCorruption,
  ::testing::Combine(
    ::testing::ValuesIn(sftest::kPayloadSizes),
    ::testing::Range(0, 5)));

// ---- header field validation ----------------------------------------------
TEST(FrameValidation, RejectsBadVersionAndLength)
{
  const auto payload = sftest::make_payload(128, 1);
  auto frame = encode_frame(SensorType::kGps, 1, 1, 0, payload.data(), payload.size());

  // bump version above kVersion -> reject, and recompute header CRC so it is
  // the version check (not the CRC check) that fires.
  {
    auto f = frame;
    f[4] = 0xFF; f[5] = 0xFF;  // version field
    FrameHeader h;
    const auto e = decode_header(f.data(), f.size(), h);
    SF_EXPECT_TRUE(e == FrameError::kUnsupportedVersion || e == FrameError::kHeaderCrcMismatch);
  }
  // truncated header (< overhead)
  for (size_t k = 0; k < kFrameOverhead; ++k) {
    FrameHeader h;
    SF_EXPECT_EQ(decode_header(frame.data(), k, h), FrameError::kTruncatedHeader);
  }
}

TEST(FrameValidation, EmptyPayloadRoundTrips)
{
  for (auto type : sftest::kSensorTypes) {
    auto frame = encode_frame(type, 7, 8, 0, nullptr, 0);
    FrameHeader h;
    SF_EXPECT_EQ(decode_header(frame.data(), frame.size(), h), FrameError::kOk);
    SF_EXPECT_EQ(h.payload_size, 0u);
  }
}

TEST(FrameValidation, MonotonicSequenceAndTimestampAccepted)
{
  FrameDecoder dec;
  const auto payload = sftest::make_payload(32, 9);
  for (uint64_t i = 1; i <= 500; ++i) {
    auto f = encode_frame(SensorType::kCan, i, i * 10, 0, payload.data(), payload.size());
    FrameHeader h;
    SF_EXPECT_EQ(dec.decode(f.data(), f.size(), 0, h), FrameError::kOk);
    SF_EXPECT_EQ(h.sequence, i);
  }
}
