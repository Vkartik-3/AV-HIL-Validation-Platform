/*
==============================================================================
SensorForge - Recording policy (data minimisation) tests
The contract that matters: a DENIED stream's payload bytes must never reach the
WAL by any path.
==============================================================================
*/

#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

#include "sensorforge/pipeline/recorder.hpp"
#include "sensorforge/pipeline/recording_policy.hpp"
#include "sensorforge/protocol/frame_codec.hpp"
#include "sensorforge/replay/wal_reader.hpp"
#include "test_support.hpp"

namespace fs = std::filesystem;
using namespace sensorforge;
using pipeline::PolicyDecision;
using pipeline::RecordingPolicy;
using protocol::SensorType;

namespace {
std::string temp_dir(const char * tag)
{
  const auto p = fs::temp_directory_path() /
    ("sf_pol_" + std::string(tag) + "_" + std::to_string(::getpid()));
  fs::remove_all(p);
  fs::create_directories(p);
  return p.string();
}
}  // namespace

TEST(RecordingPolicyTest, EmptyPolicyAllowsEverything)
{
  RecordingPolicy p;
  SF_EXPECT_FALSE(p.active());
  SF_EXPECT_EQ(p.decide("anything"), PolicyDecision::kRecordFull);
}

TEST(RecordingPolicyTest, DenylistBlocks)
{
  RecordingPolicy p;
  p.denylist = {"camera"};
  SF_EXPECT_EQ(p.decide("camera"), PolicyDecision::kDeny);
  SF_EXPECT_EQ(p.decide("imu"), PolicyDecision::kRecordFull);
}

TEST(RecordingPolicyTest, AllowlistExcludesEverythingElse)
{
  RecordingPolicy p;
  p.allowlist = {"imu", "gps"};
  SF_EXPECT_EQ(p.decide("imu"), PolicyDecision::kRecordFull);
  SF_EXPECT_EQ(p.decide("gps"), PolicyDecision::kRecordFull);
  SF_EXPECT_EQ(p.decide("camera"), PolicyDecision::kDeny);
}

TEST(RecordingPolicyTest, DenyBeatsAllow)
{
  RecordingPolicy p;
  p.allowlist = {"camera"};
  p.denylist = {"camera"};
  SF_EXPECT_EQ(p.decide("camera"), PolicyDecision::kDeny);
}

TEST(RecordingPolicyTest, PrefixWildcardMatches)
{
  RecordingPolicy p;
  p.denylist = {"cabin_*"};
  SF_EXPECT_EQ(p.decide("cabin_camera"), PolicyDecision::kDeny);
  SF_EXPECT_EQ(p.decide("cabin_mic"), PolicyDecision::kDeny);
  SF_EXPECT_EQ(p.decide("front_camera"), PolicyDecision::kRecordFull);
}

TEST(RecordingPolicyTest, MetadataOnlyIsDistinctFromDeny)
{
  RecordingPolicy p;
  p.metadata_only = {"camera"};
  SF_EXPECT_EQ(p.decide("camera"), PolicyDecision::kMetadataOnly);
}

// End-to-end: denied payloads must not exist on disk.
TEST(RecordingPolicyTest, DeniedStreamNeverReachesWal)
{
  const std::string dir = temp_dir("deny");
  pipeline::RecorderConfig cfg;
  cfg.wal_dir = dir;
  cfg.recording_policy.denylist = {"cabin_camera"};

  pipeline::Recorder rec(cfg);
  pipeline::StreamSpec allowed;
  allowed.name = "imu";
  allowed.sensor_type = SensorType::kImu;
  pipeline::StreamSpec denied;
  denied.name = "cabin_camera";
  denied.sensor_type = SensorType::kCamera;

  const size_t a = rec.add_stream(allowed);
  const size_t d = rec.add_stream(denied);
  rec.start();

  const std::string secret = "SENSITIVE-PAYLOAD-MARKER-0123456789";
  for (int i = 0; i < 100; ++i) {
    rec.capture(a, reinterpret_cast<const uint8_t *>("imu-ok"), 6);
    rec.capture(d, reinterpret_cast<const uint8_t *>(secret.data()), secret.size());
  }
  rec.stop();

  const auto st = rec.stats();
  SF_EXPECT_EQ(st.streams[1].denied_by_policy, 100u);
  SF_EXPECT_EQ(st.streams[1].recorded, 0u);
  SF_EXPECT_EQ(st.streams[0].recorded, 100u);

  // Scan every byte of every segment: the marker must not appear anywhere.
  bool found = false;
  uint64_t camera_records = 0;
  for (const auto & e : fs::directory_iterator(dir)) {
    std::ifstream in(e.path(), std::ios::binary);
    const std::string blob(
      (std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (blob.find(secret) != std::string::npos) {
      found = true;
    }
  }
  replay::stream_replay(
    dir,
    [&](const replay::ReplayRecord & r) {
      if (r.header.sensor_type == SensorType::kCamera) {
        ++camera_records;
      }
    });
  SF_EXPECT_FALSE(found);
  SF_EXPECT_EQ(camera_records, 0u);
  fs::remove_all(dir);
}

// Metadata-only keeps timing/rate analysis working while dropping the payload.
TEST(RecordingPolicyTest, MetadataOnlyRecordsHeaderWithoutPayload)
{
  const std::string dir = temp_dir("meta");
  pipeline::RecorderConfig cfg;
  cfg.wal_dir = dir;
  cfg.recording_policy.metadata_only = {"camera"};

  pipeline::Recorder rec(cfg);
  pipeline::StreamSpec cam;
  cam.name = "camera";
  cam.sensor_type = SensorType::kCamera;
  const size_t c = rec.add_stream(cam);
  rec.start();

  const std::string secret = "PIXELS-THAT-MUST-NOT-BE-STORED";
  for (int i = 0; i < 50; ++i) {
    rec.capture(c, reinterpret_cast<const uint8_t *>(secret.data()), secret.size());
  }
  rec.stop();

  const auto st = rec.stats();
  SF_EXPECT_EQ(st.streams[0].metadata_only, 50u);
  SF_EXPECT_EQ(st.streams[0].recorded, 50u);   // headers ARE recorded

  bool found = false;
  uint64_t n = 0;
  std::vector<uint64_t> seqs;
  for (const auto & e : fs::directory_iterator(dir)) {
    std::ifstream in(e.path(), std::ios::binary);
    const std::string blob(
      (std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (blob.find(secret) != std::string::npos) {
      found = true;
    }
  }
  replay::stream_replay(
    dir,
    [&](const replay::ReplayRecord & r) {
      ++n;
      seqs.push_back(r.header.sequence);
      // The frame is present and valid, with a zero-length payload.
      protocol::FrameHeader h;
      SF_EXPECT_EQ(
        protocol::decode_header(r.payload.data(), r.payload.size(), h),
        protocol::FrameError::kOk);
      SF_EXPECT_EQ(h.payload_size, 0u);
    });
  SF_EXPECT_FALSE(found);
  SF_EXPECT_EQ(n, 50u);
  // Rate/gap analysis still works because sequence survived.
  for (size_t i = 0; i < seqs.size(); ++i) {
    SF_EXPECT_EQ(seqs[i], i);
  }
  fs::remove_all(dir);
}
