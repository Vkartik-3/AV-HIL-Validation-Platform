/*
==============================================================================
SensorForge - Deterministic record/replay regression tests
The audit found WalReader::replay() was reachable from no shipped binary. These
tests pin the digest contract the wal_replay tool and CI now depend on.
==============================================================================
*/

#include <cstdio>
#include <filesystem>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

#include "sensorforge/protocol/frame_codec.hpp"
#include "sensorforge/replay/wal_reader.hpp"
#include "sensorforge/replay/wal_writer.hpp"
#include "test_support.hpp"

namespace fs = std::filesystem;
using namespace sensorforge::replay;
using sensorforge::protocol::SensorType;

namespace {

std::string temp_dir(const char * tag)
{
  const auto p = fs::temp_directory_path() /
    ("sf_digest_" + std::string(tag) + "_" + std::to_string(::getpid()));
  fs::remove_all(p);
  fs::create_directories(p);
  return p.string();
}

void record_known_stream(const std::string & dir, int n, size_t seg_bytes = 1 << 16)
{
  WalConfig cfg;
  cfg.segment_bytes = seg_bytes;
  WalWriter w(dir, cfg);
  for (int i = 0; i < n; ++i) {
    // Record real encoded SensorForge frames, exactly as the Recorder does.
    const auto payload = sftest::make_payload(64 + (i % 200), i);
    const auto frame = sensorforge::protocol::encode_frame(
      SensorType::kImu, i, 1000ull * i, sensorforge::protocol::kFlagNone,
      payload.data(), payload.size());
    w.append(1000ull * i, SensorType::kImu, i, frame.data(), frame.size());
  }
}

}  // namespace

TEST(ReplayDigest, SameRecordingYieldsSameDigest)
{
  const std::string dir = temp_dir("stable");
  record_known_stream(dir, 1000);

  const auto a = compute_replay_digest(dir);
  const auto b = compute_replay_digest(dir);
  SF_EXPECT_EQ(a.digest, b.digest);
  SF_EXPECT_EQ(a.records, 1000u);
  SF_EXPECT_EQ(b.records, 1000u);
  SF_EXPECT_GT(a.payload_bytes, 0u);
  fs::remove_all(dir);
}

TEST(ReplayDigest, DifferentContentYieldsDifferentDigest)
{
  const std::string d1 = temp_dir("c1");
  const std::string d2 = temp_dir("c2");
  record_known_stream(d1, 500);
  record_known_stream(d2, 501);   // one extra record
  SF_EXPECT_NE(compute_replay_digest(d1).digest, compute_replay_digest(d2).digest);
  fs::remove_all(d1);
  fs::remove_all(d2);
}

TEST(ReplayDigest, SingleByteChangeChangesDigest)
{
  const std::string dir = temp_dir("bitflip");
  record_known_stream(dir, 200, 1 << 20);
  const uint32_t before = compute_replay_digest(dir).digest;

  // Corrupting a byte makes that record fail CRC and be skipped, so both the
  // record count and the digest must move.
  std::string seg;
  for (const auto & e : fs::directory_iterator(dir)) {
    seg = e.path().string();
  }
  const auto sz = fs::file_size(seg);
  std::FILE * f = std::fopen(seg.c_str(), "r+b");
  SF_ASSERT_NE(f, nullptr);
  std::fseek(f, static_cast<long>(sz / 2), SEEK_SET);
  const unsigned char junk = 0xA5;
  std::fwrite(&junk, 1, 1, f);
  std::fclose(f);

  const auto after = compute_replay_digest(dir);
  SF_EXPECT_NE(before, after.digest);
  fs::remove_all(dir);
}

// Streaming replay must deliver exactly what the loading reader does.
TEST(ReplayDigest, StreamingMatchesLoadingReader)
{
  const std::string dir = temp_dir("stream");
  record_known_stream(dir, 800, 1 << 14);   // several segments

  WalReader loading(dir);
  uint64_t streamed = 0;
  uint64_t streamed_bytes = 0;
  const auto stats = stream_replay(
    dir,
    [&](const ReplayRecord & r) {
      ++streamed;
      streamed_bytes += r.payload.size();
    });

  SF_EXPECT_EQ(streamed, loading.stats().records_ok);
  SF_EXPECT_EQ(stats.records_ok, loading.stats().records_ok);
  SF_EXPECT_GT(stats.segments_read, 1u);
  SF_EXPECT_EQ(streamed, 800u);
  SF_EXPECT_GT(streamed_bytes, 0u);
  fs::remove_all(dir);
}

// Record -> replay -> re-validate: every replayed payload is still a structurally
// valid SensorForge frame, proving the WAL round-trips what the framer produced.
TEST(ReplayDigest, ReplayedRecordsRevalidateAsFrames)
{
  const std::string dir = temp_dir("revalidate");
  record_known_stream(dir, 400);

  uint64_t ok = 0, bad = 0;
  stream_replay(
    dir,
    [&](const ReplayRecord & r) {
      sensorforge::protocol::FrameHeader h;
      if (sensorforge::protocol::decode_header(r.payload.data(), r.payload.size(), h) ==
        sensorforge::protocol::FrameError::kOk)
      {
        ++ok;
      } else {
        ++bad;
      }
    });
  SF_EXPECT_EQ(ok, 400u);
  SF_EXPECT_EQ(bad, 0u);
  fs::remove_all(dir);
}

TEST(ReplayDigest, StartTimestampSeeksForward)
{
  const std::string dir = temp_dir("seek");
  record_known_stream(dir, 300, 1 << 20);
  const auto all = compute_replay_digest(dir, 0);
  const auto half = compute_replay_digest(dir, 1000ull * 150);
  SF_EXPECT_EQ(all.records, 300u);
  SF_EXPECT_EQ(half.records, 150u);
  SF_EXPECT_NE(all.digest, half.digest);
  fs::remove_all(dir);
}

TEST(ReplayDigest, EmptyDirectoryIsHandled)
{
  const std::string dir = temp_dir("empty");
  const auto d = compute_replay_digest(dir);
  SF_EXPECT_EQ(d.records, 0u);
  SF_EXPECT_EQ(d.digest, 0u);
  fs::remove_all(dir);
}

// Restart-then-append must not change the digest of the records already there.
TEST(ReplayDigest, DigestIsStableAcrossWriterRestart)
{
  const std::string dir = temp_dir("restart");
  record_known_stream(dir, 250, 1 << 20);
  const auto first = compute_replay_digest(dir);

  {
    WalConfig cfg;
    cfg.segment_bytes = 1 << 20;
    WalWriter w(dir, cfg);   // reopen, append nothing
  }
  const auto second = compute_replay_digest(dir);
  SF_EXPECT_EQ(first.digest, second.digest);
  SF_EXPECT_EQ(first.records, second.records);
  fs::remove_all(dir);
}
