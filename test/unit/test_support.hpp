/*
==============================================================================
SensorForge - GoogleTest support (Extension G)
Shared helpers + parameter value sets used across the unit suite.
==============================================================================
*/

#pragma once

#include <atomic>
#include <cstdint>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "sensorforge/protocol/frame.hpp"

namespace sftest {

// Global assertion counter. GoogleTest does NOT report successful assertions to
// test listeners (OnTestPartResult fires only on failure), so to report an exact
// executed-assertion count we route every check through the SF_* wrappers below,
// which tick this counter and then delegate to the real gtest macro.
inline std::atomic<long long> g_assertions{0};

}  // namespace sftest

#define SF_TICK (::sftest::g_assertions.fetch_add(1, std::memory_order_relaxed))
#define SF_EXPECT_EQ(a, b)   do {SF_TICK; EXPECT_EQ(a, b);} while (0)
#define SF_EXPECT_NE(a, b)   do {SF_TICK; EXPECT_NE(a, b);} while (0)
#define SF_EXPECT_TRUE(a)    do {SF_TICK; EXPECT_TRUE(a);} while (0)
#define SF_EXPECT_FALSE(a)   do {SF_TICK; EXPECT_FALSE(a);} while (0)
#define SF_EXPECT_GT(a, b)   do {SF_TICK; EXPECT_GT(a, b);} while (0)
#define SF_EXPECT_GE(a, b)   do {SF_TICK; EXPECT_GE(a, b);} while (0)
#define SF_EXPECT_LT(a, b)   do {SF_TICK; EXPECT_LT(a, b);} while (0)
// ASSERT_* still abort the current (void) test on failure via `return`; wrapping
// in do/while(0) preserves that because the return exits the enclosing function.
#define SF_ASSERT_EQ(a, b)   do {SF_TICK; ASSERT_EQ(a, b);} while (0)
#define SF_ASSERT_NE(a, b)   do {SF_TICK; ASSERT_NE(a, b);} while (0)
#define SF_ASSERT_TRUE(a)    do {SF_TICK; ASSERT_TRUE(a);} while (0)

namespace sftest {

// Payload sizes parameterized across the suite.
inline const std::vector<size_t> kPayloadSizes = {64, 256, 1024, 65536, 1048576};

// Sensor types parameterized across the suite.
inline const std::vector<sensorforge::protocol::SensorType> kSensorTypes = {
  sensorforge::protocol::SensorType::kLidar,
  sensorforge::protocol::SensorType::kCamera,
  sensorforge::protocol::SensorType::kImu,
  sensorforge::protocol::SensorType::kGps,
  sensorforge::protocol::SensorType::kCan,
};

enum class Corruption {
  kBitFlip,
  kTruncate,
  kMagicCorrupt,
  kCrcCorrupt,
  kSeqRegression,
};

inline std::vector<uint8_t> make_payload(size_t n, uint64_t seed = 0)
{
  std::mt19937_64 rng(seed + n * 1099511628211ull);
  std::vector<uint8_t> v(n);
  for (auto & b : v) {
    b = static_cast<uint8_t>(rng());
  }
  return v;
}

}  // namespace sftest
