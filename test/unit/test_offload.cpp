/*
==============================================================================
SensorForge - Sealed-segment offload tests
Covers the failure modes that matter: destination down while recording
continues, backlog drain on return, restart recovery, idempotence, and a
bounded queue that rejects rather than grows.
==============================================================================
*/

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <unistd.h>

#include <gtest/gtest.h>

#include "sensorforge/offload/uploader.hpp"
#include "sensorforge/replay/wal_writer.hpp"
#include "test_support.hpp"

namespace fs = std::filesystem;
using namespace sensorforge;
using offload::FilesystemDestination;
using offload::Uploader;
using offload::UploaderConfig;

namespace {

std::string temp_dir(const char * tag)
{
  const auto p = fs::temp_directory_path() /
    ("sf_off_" + std::string(tag) + "_" + std::to_string(::getpid()));
  fs::remove_all(p);
  fs::create_directories(p);
  return p.string();
}

size_t count_objects(const std::string & dir)
{
  size_t n = 0;
  std::error_code ec;
  if (!fs::exists(dir, ec)) {
    return 0;
  }
  for (const auto & e : fs::directory_iterator(dir, ec)) {
    const std::string fn = e.path().filename().string();
    if (fn.rfind("segment-", 0) == 0 && fn.find(".partial") == std::string::npos) {
      ++n;
    }
  }
  return n;
}

/// Write `n` records into a WAL sized so segments seal frequently.
void fill_wal(const std::string & dir, int n, Uploader * up = nullptr)
{
  replay::WalConfig cfg;
  cfg.segment_bytes = 2048;
  replay::WalWriter w(dir, cfg);
  if (up) {
    w.set_segment_sealed_hook(
      [up](const std::string & path, uint32_t id) {up->enqueue(path, id);});
  }
  for (int i = 0; i < n; ++i) {
    const uint8_t payload[64] = {};
    w.append(1000ull * i, protocol::SensorType::kImu, i, payload, sizeof(payload));
  }
  w.close();
}

}  // namespace

TEST(OffloadTest, FilesystemDestinationPublishesAtomically)
{
  const std::string src = temp_dir("fsrc");
  const std::string dst = temp_dir("fdst");
  const std::string file = src + "/thing.bin";
  {
    std::ofstream o(file, std::ios::binary);
    o << "hello-sensorforge";
  }
  FilesystemDestination d(dst);
  SF_EXPECT_TRUE(d.put(file, "segment-000001.sflog"));
  SF_EXPECT_TRUE(fs::exists(dst + "/segment-000001.sflog"));
  // No .partial left behind: the rename published the whole object.
  SF_EXPECT_FALSE(fs::exists(dst + "/segment-000001.sflog.partial"));
  fs::remove_all(src);
  fs::remove_all(dst);
}

TEST(OffloadTest, SealedSegmentsAreUploaded)
{
  const std::string wal = temp_dir("wal1");
  const std::string dst = temp_dir("dst1");
  auto dest = std::make_shared<FilesystemDestination>(dst);
  Uploader up(dest, wal);
  up.start();
  fill_wal(wal, 400, &up);
  SF_EXPECT_TRUE(up.wait_drained(5000));
  up.stop();

  const auto s = up.stats();
  SF_EXPECT_GT(s.uploaded, 0u);
  SF_EXPECT_EQ(s.backlog, 0u);
  SF_EXPECT_EQ(count_objects(dst), s.uploaded);
  fs::remove_all(wal);
  fs::remove_all(dst);
}

// The central requirement: recording must not stall when the destination is
// down, and the backlog must drain when it returns.
TEST(OffloadTest, RecordingContinuesWhileDestinationIsDownThenDrains)
{
  const std::string wal = temp_dir("wal2");
  const std::string dst = temp_dir("dst2");
  auto dest = std::make_shared<FilesystemDestination>(dst);
  dest->set_available(false);            // destination is unavailable

  UploaderConfig cfg;
  cfg.base_backoff_ms = 5;
  cfg.max_backoff_ms = 20;
  Uploader up(dest, wal, cfg);
  up.start();

  const auto t0 = std::chrono::steady_clock::now();
  fill_wal(wal, 600, &up);               // recording proceeds regardless
  const auto record_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();

  SF_EXPECT_LT(record_ms, 5000);         // never blocked on the dead destination
  SF_EXPECT_GT(up.stats().enqueued, 0u);
  SF_EXPECT_EQ(count_objects(dst), 0u);  // nothing uploaded yet
  SF_EXPECT_FALSE(up.stats().destination_healthy);

  dest->set_available(true);             // destination comes back
  SF_EXPECT_TRUE(up.wait_drained(10000));
  up.stop();

  const auto s = up.stats();
  SF_EXPECT_EQ(s.backlog, 0u);
  SF_EXPECT_GT(s.uploaded, 0u);
  SF_EXPECT_GT(s.failed_attempts, 0u);
  SF_EXPECT_EQ(count_objects(dst), s.uploaded);
  fs::remove_all(wal);
  fs::remove_all(dst);
}

// Local segments must survive an interrupted offload and be re-enqueued.
TEST(OffloadTest, RestartRecoversPendingSegments)
{
  const std::string wal = temp_dir("wal3");
  const std::string dst = temp_dir("dst3");
  auto dead = std::make_shared<FilesystemDestination>(dst);
  dead->set_available(false);

  {
    UploaderConfig cfg;
    cfg.base_backoff_ms = 5;
    cfg.max_backoff_ms = 10;
    Uploader up(dead, wal, cfg);
    up.start();
    fill_wal(wal, 600, &up);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    up.stop();                       // "process killed" mid-offload
  }
  SF_EXPECT_EQ(count_objects(dst), 0u);

  // New process, healthy destination: everything sealed must still go.
  auto alive = std::make_shared<FilesystemDestination>(dst);
  Uploader up2(alive, wal);
  up2.start();
  SF_EXPECT_GT(up2.stats().recovered_on_start, 0u);
  SF_EXPECT_TRUE(up2.wait_drained(10000));
  up2.stop();

  SF_EXPECT_GT(count_objects(dst), 0u);
  // No completed segment lost: every sealed segment on disk has an object.
  size_t sealed = 0;
  for (const auto & e : fs::directory_iterator(wal)) {
    const std::string fn = e.path().filename().string();
    if (fn.rfind("segment-", 0) == 0) {
      ++sealed;
    }
  }
  SF_EXPECT_GE(count_objects(dst), sealed - 1);   // all but the live tail
  fs::remove_all(wal);
  fs::remove_all(dst);
}

TEST(OffloadTest, AlreadyUploadedSegmentIsNotDuplicated)
{
  const std::string wal = temp_dir("wal4");
  const std::string dst = temp_dir("dst4");
  auto dest = std::make_shared<FilesystemDestination>(dst);
  Uploader up(dest, wal);
  up.start();
  fill_wal(wal, 300, &up);
  SF_EXPECT_TRUE(up.wait_drained(5000));
  const auto first = up.stats().uploaded;

  // Re-enqueue the same ids: idempotent, so nothing new is produced.
  for (uint32_t id = 1; id <= 3; ++id) {
    up.enqueue(wal + "/segment-00000" + std::to_string(id) + ".sflog", id);
  }
  SF_EXPECT_TRUE(up.wait_drained(3000));
  up.stop();

  SF_EXPECT_GT(up.stats().duplicates_skipped, 0u);
  SF_EXPECT_EQ(up.stats().uploaded, first);
  SF_EXPECT_EQ(count_objects(dst), first);
  fs::remove_all(wal);
  fs::remove_all(dst);
}

TEST(OffloadTest, QueueIsBoundedAndRejectsRatherThanGrows)
{
  const std::string wal = temp_dir("wal5");
  const std::string dst = temp_dir("dst5");
  auto dest = std::make_shared<FilesystemDestination>(dst);
  dest->set_available(false);

  UploaderConfig cfg;
  cfg.max_queue = 4;
  Uploader up(dest, wal, cfg);
  // Deliberately not started: the queue simply fills.
  for (uint32_t id = 1; id <= 50; ++id) {
    up.enqueue(wal + "/nonexistent.sflog", id);
  }
  const auto s = up.stats();
  SF_EXPECT_LE(s.backlog, cfg.max_queue);
  SF_EXPECT_GT(s.rejected_queue_full, 0u);
  fs::remove_all(wal);
  fs::remove_all(dst);
}

TEST(OffloadTest, LocalSegmentSurvivesUntilAcknowledged)
{
  const std::string wal = temp_dir("wal6");
  const std::string dst = temp_dir("dst6");
  auto dest = std::make_shared<FilesystemDestination>(dst);
  dest->set_available(false);

  UploaderConfig cfg;
  cfg.base_backoff_ms = 5;
  cfg.delete_after_upload = true;     // even so, nothing may be deleted unsent
  Uploader up(dest, wal, cfg);
  up.start();
  fill_wal(wal, 400, &up);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  size_t local = 0;
  for (const auto & e : fs::directory_iterator(wal)) {
    if (e.path().filename().string().rfind("segment-", 0) == 0) {
      ++local;
    }
  }
  SF_EXPECT_GT(local, 0u);            // still on disk: never acknowledged
  up.stop();
  fs::remove_all(wal);
  fs::remove_all(dst);
}

TEST(OffloadTest, MaxAttemptsMarksFailedAndStopsRetrying)
{
  const std::string wal = temp_dir("wal7");
  const std::string dst = temp_dir("dst7");
  auto dest = std::make_shared<FilesystemDestination>(dst);
  dest->set_available(false);

  UploaderConfig cfg;
  cfg.base_backoff_ms = 1;
  cfg.max_backoff_ms = 2;
  cfg.max_attempts = 3;
  Uploader up(dest, wal, cfg);
  up.start();
  up.enqueue(wal + "/whatever.sflog", 1);
  SF_EXPECT_TRUE(up.wait_drained(5000));
  up.stop();
  SF_EXPECT_FALSE(up.is_uploaded(1));
  SF_EXPECT_GE(up.stats().failed_attempts, 3u);
  fs::remove_all(wal);
  fs::remove_all(dst);
}
