/*
==============================================================================
SensorForge - Event-triggered selective recording tests
Pins the two properties that matter: pre-roll is BOUNDED (it counts against the
byte budget), and overlapping triggers behave DETERMINISTICALLY.
==============================================================================
*/

#include <filesystem>
#include <string>
#include <thread>
#include <chrono>
#include <unistd.h>

#include <gtest/gtest.h>

#include "sensorforge/pipeline/recorder.hpp"
#include "sensorforge/pipeline/trigger.hpp"
#include "sensorforge/replay/wal_reader.hpp"
#include "test_support.hpp"

namespace fs = std::filesystem;
using namespace sensorforge;
using pipeline::PreRollEntry;
using pipeline::TriggerConfig;
using pipeline::TriggerEngine;
using pipeline::TriggerReason;

namespace {

PreRollEntry entry(uint64_t seq, size_t bytes, uint64_t mono)
{
  PreRollEntry e;
  e.sequence = seq;
  e.capture_mono_ns = mono;
  e.payload.assign(bytes, static_cast<uint8_t>(seq));
  return e;
}

std::string temp_dir(const char * tag)
{
  const auto p = fs::temp_directory_path() /
    ("sf_trig_" + std::string(tag) + "_" + std::to_string(::getpid()));
  fs::remove_all(p);
  fs::create_directories(p);
  return p.string();
}

}  // namespace

TEST(TriggerTest, PreRollIsBoundedByBytes)
{
  TriggerConfig cfg;
  cfg.pre_roll_max_bytes = 4096;
  cfg.pre_roll_max_frames = 100000;
  TriggerEngine e(cfg);
  for (uint64_t i = 0; i < 1000; ++i) {
    e.push_preroll(entry(i, 512, i * 1000));
  }
  SF_EXPECT_LE(e.preroll_bytes(), cfg.pre_roll_max_bytes);
  SF_EXPECT_GT(e.evicted(), 0u);
}

TEST(TriggerTest, PreRollIsBoundedByFrames)
{
  TriggerConfig cfg;
  cfg.pre_roll_max_bytes = 1u << 30;
  cfg.pre_roll_max_frames = 16;
  TriggerEngine e(cfg);
  for (uint64_t i = 0; i < 500; ++i) {
    e.push_preroll(entry(i, 8, i * 1000));
  }
  SF_EXPECT_LE(e.preroll_frames(), cfg.pre_roll_max_frames);
}

TEST(TriggerTest, PreRollKeepsNewestOnEviction)
{
  TriggerConfig cfg;
  cfg.pre_roll_max_frames = 4;
  cfg.pre_roll_max_bytes = 1u << 30;
  TriggerEngine e(cfg);
  for (uint64_t i = 0; i < 20; ++i) {
    e.push_preroll(entry(i, 8, i * 1000));
  }
  const auto taken = e.take_preroll();
  SF_ASSERT_EQ(taken.size(), 4u);
  SF_EXPECT_EQ(taken.front().sequence, 16u);
  SF_EXPECT_EQ(taken.back().sequence, 19u);
}

TEST(TriggerTest, TakePreRollEmptiesTheBuffer)
{
  TriggerEngine e(TriggerConfig{});
  for (uint64_t i = 0; i < 10; ++i) {
    e.push_preroll(entry(i, 64, i));
  }
  SF_EXPECT_GT(e.preroll_bytes(), 0u);
  e.take_preroll();
  SF_EXPECT_EQ(e.preroll_bytes(), 0u);
  SF_EXPECT_EQ(e.preroll_frames(), 0u);
}

TEST(TriggerTest, WindowOpensAndCloses)
{
  TriggerConfig cfg;
  cfg.post_roll_seconds = 1.0;
  TriggerEngine e(cfg);
  const uint64_t t0 = 1000000000ull;
  SF_EXPECT_FALSE(e.window_open(t0));
  e.fire(TriggerReason::kExternal, t0);
  SF_EXPECT_TRUE(e.window_open(t0));
  SF_EXPECT_TRUE(e.window_open(t0 + 500000000ull));
  SF_EXPECT_FALSE(e.window_open(t0 + 1500000000ull));
}

// Determinism under overlap: extend, never restart, never shorten.
TEST(TriggerTest, OverlappingTriggersExtendDeterministically)
{
  TriggerConfig cfg;
  cfg.post_roll_seconds = 2.0;
  const uint64_t t0 = 1000000000ull;
  const uint64_t sec = 1000000000ull;

  TriggerEngine a(cfg);
  a.fire(TriggerReason::kExternal, t0);
  a.fire(TriggerReason::kCrcFailure, t0 + sec);        // extends to t0 + 3s

  TriggerEngine b(cfg);
  b.fire(TriggerReason::kCrcFailure, t0 + sec);        // same two, reverse order
  b.fire(TriggerReason::kExternal, t0);

  for (uint64_t dt = 0; dt <= 4 * sec; dt += sec / 4) {
    SF_EXPECT_EQ(a.window_open(t0 + dt), b.window_open(t0 + dt));
  }
  SF_EXPECT_TRUE(a.window_open(t0 + 2 * sec + sec / 2));
  SF_EXPECT_FALSE(a.window_open(t0 + 3 * sec + 1));
}

TEST(TriggerTest, LateTriggerNeverShortensAnOpenWindow)
{
  TriggerConfig cfg;
  cfg.post_roll_seconds = 5.0;
  const uint64_t sec = 1000000000ull;
  TriggerEngine e(cfg);
  e.fire(TriggerReason::kExternal, 10 * sec);          // window to 15s
  e.fire(TriggerReason::kExternal, 10 * sec - sec);    // earlier -> must not shorten
  SF_EXPECT_TRUE(e.window_open(14 * sec));
  SF_EXPECT_EQ(e.activations(), 2u);
}

TEST(TriggerTest, ExpireDropsEntriesOlderThanPreRollHorizon)
{
  TriggerConfig cfg;
  cfg.pre_roll_seconds = 1.0;
  cfg.pre_roll_max_frames = 1000;
  TriggerEngine e(cfg);
  const uint64_t sec = 1000000000ull;
  for (uint64_t i = 0; i < 10; ++i) {
    e.push_preroll(entry(i, 16, i * sec / 4));
  }
  e.expire(5 * sec);
  SF_EXPECT_EQ(e.preroll_frames(), 0u);
}

// End-to-end: decimated baseline plus a trigger recovers the pre-roll, and the
// bytes-saved figure is reported.
TEST(TriggerTest, SelectiveRecordingSavesBytesAndRecoversPreRoll)
{
  const std::string dir = temp_dir("select");
  pipeline::RecorderConfig cfg;
  cfg.wal_dir = dir;
  cfg.enable_triggers = true;
  cfg.trigger.pre_roll_seconds = 30.0;
  cfg.trigger.post_roll_seconds = 30.0;
  cfg.trigger.pre_roll_max_frames = 10000;
  cfg.trigger.pre_roll_max_bytes = 8u << 20;

  pipeline::Recorder rec(cfg);
  pipeline::StreamSpec s;
  s.name = "imu";
  s.sensor_type = protocol::SensorType::kImu;
  s.baseline_decimation = 10;      // record 1 in 10 while nothing is happening
  const size_t id = rec.add_stream(s);
  rec.start();

  const auto payload = sftest::make_payload(128, 7);
  for (int i = 0; i < 300; ++i) {
    rec.capture(id, payload.data(), payload.size());
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const auto before = rec.stats();
  SF_EXPECT_GT(before.bytes_saved_vs_always_on, 0u);

  rec.fire_trigger(TriggerReason::kExternal, "test");
  for (int i = 0; i < 100; ++i) {
    rec.capture(id, payload.data(), payload.size());
  }
  rec.stop();

  const auto st = rec.stats();
  SF_EXPECT_EQ(st.trigger_activations, 1u);
  SF_EXPECT_GT(st.streams[0].skipped_decimation, 0u);
  // After the trigger, the pre-roll is flushed, so more is recorded than pure
  // decimation would have produced, but still fewer than always-on.
  SF_EXPECT_GT(st.total_recorded, 40u);
  SF_EXPECT_LE(st.total_recorded, 400u);
  fs::remove_all(dir);
}
