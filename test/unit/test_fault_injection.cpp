/*
==============================================================================
SensorForge - Fault injection engine tests (Extension G/K)
Non-ROS: verifies each fault kind's effect on an outgoing frame stream.
==============================================================================
*/

#include <gtest/gtest.h>

#include "test_support.hpp"

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "faults/fault_engine.hpp"

using namespace sensorforge::faults;

namespace {
FaultRule rule(FaultKind k, double start = 0.0, double dur = 100.0)
{
  FaultRule r;
  r.kind = k;
  r.start_s = start;
  r.duration_s = dur;
  return r;
}
std::vector<uint8_t> frame(uint8_t v, size_t n = 16) {return std::vector<uint8_t>(n, v);}
}  // namespace

TEST(FaultInjection, NoRulesForwardsEverything)
{
  std::vector<std::vector<uint8_t>> out;
  FaultEngine eng([&](const std::vector<uint8_t> & d) {out.push_back(d);});
  for (int i = 0; i < 100; ++i) {eng.process(frame(static_cast<uint8_t>(i)), 1.0, "imu");}
  SF_EXPECT_EQ(out.size(), 100u);
  SF_EXPECT_EQ(eng.stats().forwarded, 100u);
  SF_EXPECT_EQ(eng.stats().dropped, 0u);
}

TEST(FaultInjection, BurstLossDropsWithinWindowOnly)
{
  std::vector<std::vector<uint8_t>> out;
  FaultEngine eng([&](const std::vector<uint8_t> & d) {out.push_back(d);});
  auto r = rule(FaultKind::kBurstLoss, 5.0, 2.0);   // active [5,7)
  eng.add_rule(r);
  for (int i = 0; i < 100; ++i) {eng.process(frame(1), 4.9, "imu");}   // before -> pass
  for (int i = 0; i < 100; ++i) {eng.process(frame(1), 6.0, "imu");}   // during -> drop
  for (int i = 0; i < 100; ++i) {eng.process(frame(1), 7.1, "imu");}   // after -> pass
  SF_EXPECT_EQ(out.size(), 200u);
  SF_EXPECT_EQ(eng.stats().dropped, 100u);
}

TEST(FaultInjection, DropRateApproximate)
{
  std::vector<std::vector<uint8_t>> out;
  FaultEngine eng([&](const std::vector<uint8_t> & d) {out.push_back(d);}, /*seed=*/7);
  FaultRule r = rule(FaultKind::kDrop);
  r.drop_rate = 0.5;
  eng.add_rule(r);
  const int N = 20000;
  for (int i = 0; i < N; ++i) {eng.process(frame(1), 1.0, "can");}
  const double dropped_frac = static_cast<double>(eng.stats().dropped) / N;
  SF_EXPECT_GT(dropped_frac, 0.45);
  SF_EXPECT_LT(dropped_frac, 0.55);
}

TEST(FaultInjection, DuplicateSendsTwice)
{
  std::vector<std::vector<uint8_t>> out;
  FaultEngine eng([&](const std::vector<uint8_t> & d) {out.push_back(d);});
  eng.add_rule(rule(FaultKind::kDuplicate));
  for (int i = 0; i < 50; ++i) {eng.process(frame(9), 1.0, "gps");}
  SF_EXPECT_EQ(out.size(), 100u);
  SF_EXPECT_EQ(eng.stats().duplicated, 50u);
}

TEST(FaultInjection, CorruptChangesBytesButKeepsCount)
{
  std::vector<std::vector<uint8_t>> out;
  FaultEngine eng([&](const std::vector<uint8_t> & d) {out.push_back(d);}, 3);
  FaultRule r = rule(FaultKind::kCorrupt);
  r.corrupt_bits = 4;
  eng.add_rule(r);
  const auto original = frame(0xAA, 32);
  int changed = 0;
  for (int i = 0; i < 200; ++i) {
    out.clear();
    eng.process(original, 1.0, "lidar");
    SF_ASSERT_EQ(out.size(), 1u);
    SF_EXPECT_EQ(out[0].size(), original.size());
    if (out[0] != original) {++changed;}
  }
  SF_EXPECT_GT(changed, 190);   // almost always differs
  SF_EXPECT_EQ(eng.stats().corrupted, 200u);
}

TEST(FaultInjection, ReorderSwapsAdjacent)
{
  std::vector<uint8_t> order;
  FaultEngine eng([&](const std::vector<uint8_t> & d) {order.push_back(d[0]);});
  eng.add_rule(rule(FaultKind::kReorder));
  // Feed 1,2,3,4 -> expect adjacent swaps: 2,1,4,3.
  for (uint8_t v : {1, 2, 3, 4}) {eng.process(frame(v), 1.0, "imu");}
  eng.flush();
  SF_ASSERT_EQ(order.size(), 4u);
  SF_EXPECT_EQ(order[0], 2);
  SF_EXPECT_EQ(order[1], 1);
  SF_EXPECT_EQ(order[2], 4);
  SF_EXPECT_EQ(order[3], 3);
}

TEST(FaultInjection, DelayHoldsThenReleases)
{
  std::atomic<int> received{0};
  FaultEngine eng([&](const std::vector<uint8_t> &) {received.fetch_add(1);});
  FaultRule r = rule(FaultKind::kDelay);
  r.delay_ms = 40.0;
  eng.add_rule(r);
  eng.process(frame(1), 1.0, "imu");
  // Immediately after, it should not have been delivered yet.
  SF_EXPECT_EQ(received.load(), 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  SF_EXPECT_EQ(received.load(), 1);
  SF_EXPECT_EQ(eng.stats().delayed, 1u);
}

TEST(FaultInjection, EcuDisconnectSuppresses)
{
  std::vector<std::vector<uint8_t>> out;
  FaultEngine eng([&](const std::vector<uint8_t> & d) {out.push_back(d);});
  eng.add_rule(rule(FaultKind::kEcuDisconnect, 0.0, 10.0));
  for (int i = 0; i < 100; ++i) {eng.process(frame(1), 5.0, "can");}
  SF_EXPECT_EQ(out.size(), 0u);
  SF_EXPECT_EQ(eng.stats().dropped, 100u);
}

TEST(FaultInjection, ProcessRestartFiresOnce)
{
  std::atomic<int> restarts{0};
  FaultEngine eng([](const std::vector<uint8_t> &) {});
  eng.set_restart_hook([&](const std::string &) {restarts.fetch_add(1);});
  eng.add_rule(rule(FaultKind::kProcessRestart, 2.0, 3.0));
  for (int i = 0; i < 50; ++i) {eng.process(frame(1), 3.0, "camera");}   // within window
  SF_EXPECT_EQ(restarts.load(), 1);   // edge-triggered once
}

TEST(FaultInjection, StreamFilterScopesFault)
{
  std::vector<std::vector<uint8_t>> out;
  FaultEngine eng([&](const std::vector<uint8_t> & d) {out.push_back(d);});
  FaultRule r = rule(FaultKind::kEcuDisconnect);
  r.stream = "imu";   // only imu affected
  eng.add_rule(r);
  for (int i = 0; i < 10; ++i) {eng.process(frame(1), 1.0, "imu");}   // dropped
  for (int i = 0; i < 10; ++i) {eng.process(frame(1), 1.0, "gps");}   // forwarded
  SF_EXPECT_EQ(out.size(), 10u);
}
