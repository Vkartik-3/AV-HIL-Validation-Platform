/*
==============================================================================
SensorForge - WAL restart / durability tests
The audit found the writer ALWAYS started at segment 1 with ios::trunc, so a
restart against an existing directory truncated the previous run's first
segment. These tests pin the corrected behaviour.
==============================================================================
*/

#include <cstdio>
#include <unistd.h>
#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "sensorforge/replay/wal_reader.hpp"
#include "sensorforge/replay/wal_writer.hpp"
#include "test_support.hpp"

namespace fs = std::filesystem;
using sensorforge::protocol::SensorType;
using namespace sensorforge::replay;

namespace {

std::string temp_dir(const char * tag)
{
  const auto p = fs::temp_directory_path() /
    ("sf_walrestart_" + std::string(tag) + "_" + std::to_string(::getpid()) +
    "_" + std::to_string(reinterpret_cast<uintptr_t>(&tag)));
  fs::remove_all(p);
  fs::create_directories(p);
  return p.string();
}

void write_n(const std::string & dir, int n, uint64_t base_seq, const WalConfig & cfg)
{
  WalWriter w(dir, cfg);
  for (int i = 0; i < n; ++i) {
    char buf[64];
    const int len = std::snprintf(
      buf, sizeof(buf), "rec-%llu",
      static_cast<unsigned long long>(base_seq + i));
    w.append(
      1000ull * (base_seq + i), SensorType::kImu, base_seq + i,
      reinterpret_cast<uint8_t *>(buf), static_cast<size_t>(len));
  }
}

}  // namespace

// The headline regression: restarting must never destroy prior records.
TEST(WalRestart, RestartPreservesAndAppends)
{
  const std::string dir = temp_dir("append");
  WalConfig cfg;
  cfg.segment_bytes = 1 << 20;

  write_n(dir, 500, 0, cfg);       // session 1, writer destroyed at scope exit
  write_n(dir, 500, 500, cfg);     // session 2 -- must NOT truncate session 1
  write_n(dir, 500, 1000, cfg);    // session 3

  WalReader r(dir);
  SF_EXPECT_EQ(r.stats().records_ok, 1500u);
  SF_EXPECT_EQ(r.stats().records_corrupt_skipped, 0u);
  SF_ASSERT_EQ(r.size(), 1500u);

  // Every sequence 0..1499 present exactly once.
  std::vector<int> seen(1500, 0);
  for (const auto & rec : r.records()) {
    SF_ASSERT_LT(rec.header.sequence, 1500u);
    ++seen[rec.header.sequence];
  }
  for (int i = 0; i < 1500; ++i) {
    SF_EXPECT_EQ(seen[i], 1);
  }
  fs::remove_all(dir);
}

TEST(WalRestart, ReportsRecoveryOnSecondOpen)
{
  const std::string dir = temp_dir("report");
  WalConfig cfg;
  cfg.segment_bytes = 1 << 20;
  write_n(dir, 100, 0, cfg);

  WalWriter w(dir, cfg);
  const auto & rec = w.recovery();
  SF_EXPECT_TRUE(rec.recovered);
  SF_EXPECT_EQ(rec.segments_found, 1u);
  SF_EXPECT_EQ(rec.valid_records_in_tail, 100u);
  SF_EXPECT_EQ(rec.truncated_bytes, 0u);
  fs::remove_all(dir);
}

TEST(WalRestart, FreshDirectoryIsNotReportedAsRecovered)
{
  const std::string dir = temp_dir("fresh");
  WalWriter w(dir, WalConfig{});
  SF_EXPECT_FALSE(w.recovery().recovered);
  SF_EXPECT_EQ(w.recovery().segments_found, 0u);
  SF_EXPECT_EQ(w.current_segment_id(), 1u);
  fs::remove_all(dir);
}

// A process killed mid-append leaves a partial trailing record. The next open
// must discard exactly that partial record and append on a clean boundary.
TEST(WalRestart, TruncatedFinalRecordIsRecovered)
{
  const std::string dir = temp_dir("torn");
  WalConfig cfg;
  cfg.segment_bytes = 1 << 20;
  write_n(dir, 200, 0, cfg);

  // Simulate a torn tail write: chop the file mid-record.
  std::string seg;
  for (const auto & e : fs::directory_iterator(dir)) {
    seg = e.path().string();
  }
  const auto full = fs::file_size(seg);
  fs::resize_file(seg, full - 17);

  WalWriter w(dir, cfg);
  const auto rec = w.recovery();
  SF_EXPECT_TRUE(rec.recovered);
  SF_EXPECT_GT(rec.truncated_bytes, 0u);
  SF_EXPECT_EQ(rec.valid_records_in_tail, 199u);   // the torn one is gone

  // Appending after recovery must land on a clean boundary: the reader sees
  // every record with no corruption resync at all.
  for (int i = 0; i < 50; ++i) {
    char buf[32];
    const int len = std::snprintf(buf, sizeof(buf), "after-%d", i);
    w.append(
      9000000ull + i, SensorType::kGps, 1000 + i,
      reinterpret_cast<uint8_t *>(buf), static_cast<size_t>(len));
  }
  w.close();

  WalReader r(dir);
  SF_EXPECT_EQ(r.stats().records_ok, 249u);
  SF_EXPECT_EQ(r.stats().records_corrupt_skipped, 0u);
  fs::remove_all(dir);
}

TEST(WalRestart, RollsToNextSegmentWhenTailIsFull)
{
  const std::string dir = temp_dir("roll");
  WalConfig cfg;
  cfg.segment_bytes = 4096;   // tiny: forces rollover
  write_n(dir, 400, 0, cfg);

  WalWriter w(dir, cfg);
  SF_EXPECT_TRUE(w.recovery().recovered);
  SF_EXPECT_GT(w.recovery().segments_found, 1u);
  w.append(1, SensorType::kCan, 9999, reinterpret_cast<const uint8_t *>("x"), 1);
  w.close();

  WalReader r(dir);
  SF_EXPECT_EQ(r.stats().records_ok, 401u);
  fs::remove_all(dir);
}

TEST(WalRestart, FsyncPolicyEveryRecordSyncsPerAppend)
{
  const std::string dir = temp_dir("fsync");
  WalConfig cfg;
  cfg.segment_bytes = 1 << 20;
  cfg.fsync_policy = FsyncPolicy::kEveryRecord;
  WalWriter w(dir, cfg);
  for (int i = 0; i < 50; ++i) {
    w.append(i, SensorType::kImu, i, reinterpret_cast<const uint8_t *>("abc"), 3);
  }
  SF_EXPECT_GE(w.fsync_count(), 50u);
  w.close();
  fs::remove_all(dir);
}

TEST(WalRestart, FsyncPolicyNeverDoesNotSync)
{
  const std::string dir = temp_dir("nofsync");
  WalConfig cfg;
  cfg.fsync_policy = FsyncPolicy::kNever;
  WalWriter w(dir, cfg);
  for (int i = 0; i < 50; ++i) {
    w.append(i, SensorType::kImu, i, reinterpret_cast<const uint8_t *>("abc"), 3);
  }
  SF_EXPECT_EQ(w.fsync_count(), 0u);
  w.close();
  fs::remove_all(dir);
}

TEST(WalRestart, PolicyRoundTripsThroughStrings)
{
  SF_EXPECT_EQ(fsync_policy_from_string("never"), FsyncPolicy::kNever);
  SF_EXPECT_EQ(fsync_policy_from_string("interval"), FsyncPolicy::kInterval);
  SF_EXPECT_EQ(fsync_policy_from_string("every_record"), FsyncPolicy::kEveryRecord);
  SF_EXPECT_EQ(fsync_policy_from_string("on_segment_seal"), FsyncPolicy::kOnSegmentSeal);
}

TEST(WalRestart, SegmentSealedHookFiresOnRollover)
{
  const std::string dir = temp_dir("hook");
  WalConfig cfg;
  cfg.segment_bytes = 2048;
  WalWriter w(dir, cfg);
  std::vector<uint32_t> sealed;
  w.set_segment_sealed_hook([&](const std::string &, uint32_t id) {sealed.push_back(id);});
  for (int i = 0; i < 300; ++i) {
    w.append(i, SensorType::kLidar, i, reinterpret_cast<const uint8_t *>("payload!"), 8);
  }
  w.close();
  SF_EXPECT_GT(sealed.size(), 1u);
  // Ids are handed over in ascending order and never repeat.
  for (size_t i = 1; i < sealed.size(); ++i) {
    SF_EXPECT_GT(sealed[i], sealed[i - 1]);
  }
  fs::remove_all(dir);
}
