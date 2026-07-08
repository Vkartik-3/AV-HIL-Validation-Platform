/*
==============================================================================
SensorForge - WAL tests (Extension G)
Record codec, write/read round-trip, segment rollover, corruption recovery,
and replay modes.
==============================================================================
*/

#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

#include "sensorforge/replay/record.hpp"
#include "sensorforge/replay/wal_reader.hpp"
#include "sensorforge/replay/wal_writer.hpp"
#include "test_support.hpp"

namespace fs = std::filesystem;
using namespace sensorforge::replay;

namespace {
std::string temp_dir(const char * name)
{
  auto d = fs::temp_directory_path() / (std::string("sf_wal_") + name +
    std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
  fs::remove_all(d);
  return d.string();
}
}  // namespace

// ---- record codec: parameterized round-trip over sizes --------------------
class RecordRoundTrip : public ::testing::TestWithParam<size_t> {};

TEST_P(RecordRoundTrip, EncodeDecode)
{
  const size_t n = GetParam();
  const auto payload = sftest::make_payload(n, 7);
  for (uint64_t seq : {uint64_t{0}, uint64_t{999}, ~uint64_t{0}}) {
    auto rec = encode_record(seq * 3 + 1, SensorType::kLidar, seq, payload.data(), payload.size());
    SF_ASSERT_EQ(rec.size(), kRecordOverhead + n);
    RecordHeader h;
    SF_ASSERT_EQ(decode_record(rec.data(), rec.size(), h), RecordError::kOk);
    SF_EXPECT_EQ(h.sequence, seq);
    SF_EXPECT_EQ(h.payload_size, n);
    SF_EXPECT_EQ(h.timestamp_ns, seq * 3 + 1);
    const uint8_t * p = record_payload(rec.data());
    for (size_t i = 0; i < n; i += (n / 32 + 1)) {SF_EXPECT_EQ(p[i], payload[i]);}
  }
}

INSTANTIATE_TEST_SUITE_P(Sizes, RecordRoundTrip, ::testing::ValuesIn(sftest::kPayloadSizes));

TEST(RecordCodec, CorruptionDetected)
{
  const auto payload = sftest::make_payload(256, 3);
  for (int rep = 0; rep < 256; ++rep) {
    auto rec = encode_record(rep, SensorType::kImu, rep, payload.data(), payload.size());
    rec[rep % rec.size()] ^= 0x80;  // flip a bit somewhere
    RecordHeader h;
    SF_EXPECT_NE(decode_record(rec.data(), rec.size(), h), RecordError::kOk);
  }
}

// ---- writer/reader round-trip + rollover ----------------------------------
TEST(Wal, RoundTripWithRollover)
{
  const std::string dir = temp_dir("rt");
  const int N = 5000;
  {
    WalWriter w(dir, 32 * 1024);  // small segments -> many rollovers
    for (int i = 0; i < N; ++i) {
      char buf[48];
      const int len = std::snprintf(buf, sizeof(buf), "record-%d-payloaddata", i);
      SF_ASSERT_TRUE(
        w.append(1000ull * i, SensorType::kGps, static_cast<uint64_t>(i),
        reinterpret_cast<uint8_t *>(buf), static_cast<size_t>(len)));
    }
    SF_EXPECT_EQ(w.records_written(), static_cast<uint64_t>(N));
    SF_EXPECT_GT(w.current_segment_id(), 1u);  // rolled over
  }

  WalReader r(dir);
  SF_EXPECT_EQ(r.stats().records_ok, static_cast<uint64_t>(N));
  SF_EXPECT_EQ(r.stats().records_corrupt_skipped, 0u);
  SF_ASSERT_EQ(r.size(), static_cast<size_t>(N));

  uint64_t last_ts = 0;
  for (const auto & rec : r.records()) {
    SF_EXPECT_GE(rec.header.timestamp_ns, last_ts);  // time-ordered
    last_ts = rec.header.timestamp_ns;
    char expect[48];
    const int len = std::snprintf(
      expect, sizeof(expect), "record-%llu-payloaddata",
      static_cast<unsigned long long>(rec.header.sequence));
    SF_ASSERT_EQ(rec.payload.size(), static_cast<size_t>(len));
    SF_EXPECT_EQ(std::memcmp(rec.payload.data(), expect, rec.payload.size()), 0);
  }
  fs::remove_all(dir);
}

TEST(Wal, DeterministicReplayDeliversAll)
{
  const std::string dir = temp_dir("replay");
  const int N = 1000;
  {
    WalWriter w(dir, 1 << 20);
    for (int i = 0; i < N; ++i) {
      uint8_t b = static_cast<uint8_t>(i);
      w.append(1000ull * i, SensorType::kCan, i, &b, 1);
    }
  }
  WalReader r(dir);
  uint64_t count = 0;
  uint64_t last = 0;
  bool ordered = true;
  const uint64_t delivered = r.replay(
    [&](const ReplayRecord & rec) {
      if (rec.header.timestamp_ns < last) {ordered = false;}
      last = rec.header.timestamp_ns;
      ++count;
    },
    ReplayMode::kDeterministic);
  SF_EXPECT_EQ(delivered, static_cast<uint64_t>(N));
  SF_EXPECT_EQ(count, static_cast<uint64_t>(N));
  SF_EXPECT_TRUE(ordered);
  fs::remove_all(dir);
}

TEST(Wal, CorruptionRecovery)
{
  const std::string dir = temp_dir("corrupt");
  const int N = 1000;
  {
    WalWriter w(dir, 1 << 20);
    for (int i = 0; i < N; ++i) {
      char b[16];
      const int len = std::snprintf(b, sizeof(b), "r%d", i);
      w.append(1000ull * i, SensorType::kGps, i, reinterpret_cast<uint8_t *>(b), len);
    }
  }
  // Corrupt a chunk in the middle of the (single) segment file.
  std::string seg;
  for (const auto & e : fs::directory_iterator(dir)) {seg = e.path().string();}
  const auto sz = fs::file_size(seg);
  {
    std::FILE * f = std::fopen(seg.c_str(), "r+b");
    SF_ASSERT_NE(f, nullptr);
    std::fseek(f, static_cast<long>(sz / 2), SEEK_SET);
    unsigned char junk[40];
    std::memset(junk, 0xFF, sizeof(junk));
    std::fwrite(junk, 1, sizeof(junk), f);
    std::fclose(f);
  }
  WalReader r(dir);
  SF_EXPECT_GT(r.stats().records_ok, 0u);
  SF_EXPECT_LT(r.stats().records_ok, static_cast<uint64_t>(N));   // some lost
  SF_EXPECT_GE(r.stats().records_corrupt_skipped, 1u);
  // Every surviving record must still be individually valid.
  for (const auto & rec : r.records()) {
    char expect[16];
    const int len = std::snprintf(
      expect, sizeof(expect), "r%llu",
      static_cast<unsigned long long>(rec.header.sequence));
    SF_ASSERT_EQ(rec.payload.size(), static_cast<size_t>(len));
    SF_EXPECT_EQ(std::memcmp(rec.payload.data(), expect, rec.payload.size()), 0);
  }
  fs::remove_all(dir);
}
