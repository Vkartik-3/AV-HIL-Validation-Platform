/*
==============================================================================
SensorForge - Recorder integration tests
Proves that sensor data actually traverses the SensorForge core: buffer ->
frame -> CRC -> WAL -> replay, with per-stream sequence, policy and accounting.
==============================================================================
*/

#include <atomic>
#include <filesystem>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

#include "sensorforge/pipeline/recorder.hpp"
#include "sensorforge/protocol/frame_codec.hpp"
#include "sensorforge/replay/wal_reader.hpp"
#include "test_support.hpp"

namespace fs = std::filesystem;
using namespace sensorforge;
using pipeline::Recorder;
using pipeline::RecorderConfig;
using pipeline::StreamSpec;
using protocol::SensorType;

namespace {

std::string temp_dir(const char * tag)
{
  const auto p = fs::temp_directory_path() /
    ("sf_rec_" + std::string(tag) + "_" + std::to_string(::getpid()));
  fs::remove_all(p);
  fs::create_directories(p);
  return p.string();
}

StreamSpec spec_for(const char * name, SensorType t, size_t max_frames = 256)
{
  StreamSpec s;
  s.name = name;
  s.sensor_type = t;
  s.policy = core::default_policy_for(t);
  s.limits.max_frames = max_frames;
  s.limits.max_bytes = 8u * 1024u * 1024u;
  return s;
}

}  // namespace

// The core claim: every captured message becomes a CRC-validated SensorForge
// frame on disk that replays back and re-validates.
TEST(RecorderTest, MultiStreamCaptureReachesWalAsValidFrames)
{
  const std::string dir = temp_dir("multi");
  RecorderConfig cfg;
  cfg.wal_dir = dir;
  cfg.wal.segment_bytes = 1 << 20;
  Recorder rec(cfg);

  const size_t lidar = rec.add_stream(spec_for("lidar", SensorType::kLidar));
  const size_t imu = rec.add_stream(spec_for("imu", SensorType::kImu));
  const size_t gps = rec.add_stream(spec_for("gps", SensorType::kGps));

  std::atomic<uint64_t> sunk{0};
  rec.set_frame_sink(
    [&](const std::string &, SensorType, const std::vector<uint8_t> & f) {
      protocol::FrameHeader h;
      if (protocol::decode_header(f.data(), f.size(), h) == protocol::FrameError::kOk) {
        sunk.fetch_add(1, std::memory_order_relaxed);
      }
    });

  rec.start();
  const auto payload = sftest::make_payload(512, 1);
  for (int i = 0; i < 200; ++i) {
    rec.capture(lidar, payload.data(), payload.size());
    rec.capture(imu, payload.data(), 64);
    rec.capture(gps, payload.data(), 32);
  }
  rec.stop();

  const auto st = rec.stats();
  SF_EXPECT_EQ(st.total_captured, 600u);
  SF_EXPECT_EQ(st.total_dropped, 0u);
  SF_EXPECT_EQ(st.total_recorded, 600u);
  SF_EXPECT_EQ(st.wal_records, 600u);
  SF_EXPECT_EQ(sunk.load(), 600u);
  SF_ASSERT_EQ(st.streams.size(), 3u);

  // Replay and re-validate every record as a frame.
  uint64_t ok = 0, bad = 0;
  replay::stream_replay(
    dir,
    [&](const replay::ReplayRecord & r) {
      protocol::FrameHeader h;
      if (protocol::decode_header(r.payload.data(), r.payload.size(), h) ==
        protocol::FrameError::kOk)
      {
        ++ok;
      } else {
        ++bad;
      }
    });
  SF_EXPECT_EQ(ok, 600u);
  SF_EXPECT_EQ(bad, 0u);
  fs::remove_all(dir);
}

TEST(RecorderTest, EachStreamHasIndependentSequence)
{
  const std::string dir = temp_dir("seq");
  RecorderConfig cfg;
  cfg.wal_dir = dir;
  Recorder rec(cfg);
  const size_t a = rec.add_stream(spec_for("imu", SensorType::kImu));
  const size_t b = rec.add_stream(spec_for("gps", SensorType::kGps));
  rec.start();
  for (int i = 0; i < 100; ++i) {
    rec.capture(a, reinterpret_cast<const uint8_t *>("aaaa"), 4);
  }
  for (int i = 0; i < 40; ++i) {
    rec.capture(b, reinterpret_cast<const uint8_t *>("bb"), 2);
  }
  rec.stop();

  const auto st = rec.stats();
  SF_EXPECT_EQ(st.streams[0].sequence, 100u);
  SF_EXPECT_EQ(st.streams[1].sequence, 40u);

  // Per-stream sequences must each start at 0 and be contiguous.
  std::vector<uint64_t> imu_seqs, gps_seqs;
  replay::stream_replay(
    dir,
    [&](const replay::ReplayRecord & r) {
      if (r.header.sensor_type == SensorType::kImu) {
        imu_seqs.push_back(r.header.sequence);
      } else if (r.header.sensor_type == SensorType::kGps) {
        gps_seqs.push_back(r.header.sequence);
      }
    });
  SF_ASSERT_EQ(imu_seqs.size(), 100u);
  SF_ASSERT_EQ(gps_seqs.size(), 40u);
  for (size_t i = 0; i < imu_seqs.size(); ++i) {
    SF_EXPECT_EQ(imu_seqs[i], i);
  }
  for (size_t i = 0; i < gps_seqs.size(); ++i) {
    SF_EXPECT_EQ(gps_seqs[i], i);
  }
  fs::remove_all(dir);
}

TEST(RecorderTest, SensorTypeSelectsDefaultPolicy)
{
  RecorderConfig cfg;
  Recorder rec(cfg);
  rec.add_stream(spec_for("camera", SensorType::kCamera));
  rec.add_stream(spec_for("lidar", SensorType::kLidar));
  rec.add_stream(spec_for("can", SensorType::kCan));
  const auto st = rec.stats();
  SF_EXPECT_EQ(st.streams[0].policy, std::string("overwrite_oldest"));
  SF_EXPECT_EQ(st.streams[1].policy, std::string("drop_newest"));
  SF_EXPECT_EQ(st.streams[2].policy, std::string("block"));
}

TEST(RecorderTest, ByteBudgetHardBreachSheds)
{
  const std::string dir = temp_dir("budget");
  RecorderConfig cfg;
  cfg.wal_dir = dir;
  cfg.budget.hard_queue_bytes = 1;   // breach immediately on first poll
  Recorder rec(cfg);
  const size_t s = rec.add_stream(spec_for("lidar", SensorType::kLidar, 8));
  rec.start();
  const auto payload = sftest::make_payload(4096, 2);
  for (int i = 0; i < 64; ++i) {
    rec.capture(s, payload.data(), payload.size());
  }
  rec.poll_resources();
  for (int i = 0; i < 200; ++i) {
    rec.capture(s, payload.data(), payload.size());
  }
  rec.stop();
  const auto st = rec.stats();
  SF_EXPECT_GT(st.shed_events, 0u);
  SF_EXPECT_EQ(st.budget_state, core::BudgetState::kHardBreach);
  fs::remove_all(dir);
}

TEST(RecorderTest, NoWalDirStillFramesAndSinks)
{
  RecorderConfig cfg;   // wal_dir intentionally empty
  Recorder rec(cfg);
  const size_t s = rec.add_stream(spec_for("imu", SensorType::kImu));
  std::atomic<uint64_t> n{0};
  rec.set_frame_sink(
    [&](const std::string &, SensorType, const std::vector<uint8_t> &) {
      n.fetch_add(1, std::memory_order_relaxed);
    });
  rec.start();
  for (int i = 0; i < 50; ++i) {
    rec.capture(s, reinterpret_cast<const uint8_t *>("x"), 1);
  }
  rec.stop();
  SF_EXPECT_EQ(n.load(), 50u);
  SF_EXPECT_EQ(rec.stats().wal_records, 0u);
}

TEST(RecorderTest, LatencyIsMeasuredForRecordedFrames)
{
  const std::string dir = temp_dir("lat");
  RecorderConfig cfg;
  cfg.wal_dir = dir;
  Recorder rec(cfg);
  // Give the buffer room for the whole burst so this test measures latency
  // accounting, not the backpressure policy (which has its own tests).
  const size_t s = rec.add_stream(spec_for("imu", SensorType::kImu, 1024));
  rec.start();
  for (int i = 0; i < 500; ++i) {
    rec.capture(s, reinterpret_cast<const uint8_t *>("abcd"), 4);
  }
  rec.stop();
  const auto st = rec.stats();
  // The invariant that matters: exactly one latency sample per recorded frame.
  SF_EXPECT_EQ(st.latency_samples, st.total_recorded);
  SF_EXPECT_EQ(st.total_recorded, 500u);
  SF_EXPECT_GE(st.p99_capture_to_record_us, st.p50_capture_to_record_us);
  SF_EXPECT_GE(st.p999_capture_to_record_us, st.p99_capture_to_record_us);
  fs::remove_all(dir);
}

TEST(RecorderTest, ConcurrentProducersPerStream)
{
  const std::string dir = temp_dir("conc");
  RecorderConfig cfg;
  cfg.wal_dir = dir;
  cfg.wal.segment_bytes = 1 << 20;
  Recorder rec(cfg);
  std::vector<size_t> ids;
  ids.push_back(rec.add_stream(spec_for("imu", SensorType::kImu, 1024)));
  ids.push_back(rec.add_stream(spec_for("gps", SensorType::kGps, 1024)));
  ids.push_back(rec.add_stream(spec_for("lidar", SensorType::kLidar, 1024)));
  rec.start();

  std::vector<std::thread> ts;
  for (size_t i = 0; i < ids.size(); ++i) {
    ts.emplace_back([&, i]() {
        const auto p = sftest::make_payload(256, i);
        for (int k = 0; k < 300; ++k) {
          rec.capture(ids[i], p.data(), p.size());
        }
      });
  }
  for (auto & t : ts) {
    t.join();
  }
  rec.stop();

  const auto st = rec.stats();
  SF_EXPECT_EQ(st.total_captured, 900u);
  SF_EXPECT_EQ(st.total_captured, st.total_recorded + st.total_dropped);
  fs::remove_all(dir);
}

TEST(RecorderTest, RestartAppendsRatherThanTruncating)
{
  const std::string dir = temp_dir("restart");
  for (int session = 0; session < 3; ++session) {
    RecorderConfig cfg;
    cfg.wal_dir = dir;
    cfg.wal.segment_bytes = 1 << 20;
    Recorder rec(cfg);
    const size_t s = rec.add_stream(spec_for("imu", SensorType::kImu));
    rec.start();
    for (int i = 0; i < 100; ++i) {
      rec.capture(s, reinterpret_cast<const uint8_t *>("data"), 4);
    }
    rec.stop();
  }
  uint64_t total = 0;
  replay::stream_replay(dir, [&](const replay::ReplayRecord &) {++total;});
  SF_EXPECT_EQ(total, 300u);
  fs::remove_all(dir);
}
