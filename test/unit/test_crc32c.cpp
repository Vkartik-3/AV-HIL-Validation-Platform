/*
==============================================================================
SensorForge - CRC32C tests (Extension G)
Known vectors, incremental consistency, and randomized round-trips.
==============================================================================
*/

#include <gtest/gtest.h>

#include "test_support.hpp"

#include <cstdint>
#include <random>
#include <vector>

#include "sensorforge/protocol/crc32c.hpp"

using sensorforge::crc::crc32c;

// RFC 3720 (iSCSI) CRC32C test vectors.
TEST(Crc32c, KnownVectors)
{
  SF_EXPECT_EQ(crc32c(reinterpret_cast<const uint8_t *>(""), 0), 0x00000000u);
  SF_EXPECT_EQ(crc32c(reinterpret_cast<const uint8_t *>("123456789"), 9), 0xE3069283u);

  std::vector<uint8_t> zeros(32, 0x00);
  SF_EXPECT_EQ(crc32c(zeros.data(), 32), 0x8A9136AAu);

  std::vector<uint8_t> ones(32, 0xFF);
  SF_EXPECT_EQ(crc32c(ones.data(), 32), 0x62A8AB43u);

  std::vector<uint8_t> inc(32);
  for (int i = 0; i < 32; ++i) {inc[i] = static_cast<uint8_t>(i);}
  SF_EXPECT_EQ(crc32c(inc.data(), 32), 0x46DD794Eu);

  std::vector<uint8_t> dec(32);
  for (int i = 0; i < 32; ++i) {dec[i] = static_cast<uint8_t>(31 - i);}
  SF_EXPECT_EQ(crc32c(dec.data(), 32), 0x113FDB5Cu);
}

// Determinism: same bytes -> same CRC, every time.
TEST(Crc32c, DeterministicRepeat)
{
  std::mt19937_64 rng(1);
  for (int t = 0; t < 500; ++t) {
    std::vector<uint8_t> v(1 + rng() % 300);
    for (auto & b : v) {b = static_cast<uint8_t>(rng());}
    const uint32_t a = crc32c(v.data(), v.size());
    SF_EXPECT_EQ(crc32c(v.data(), v.size()), a);
    SF_EXPECT_EQ(crc32c(v), a);  // container overload agrees
  }
}

// Sensitivity: flipping any single bit changes the CRC.
TEST(Crc32c, SingleBitFlipChangesCrc)
{
  std::mt19937_64 rng(2);
  for (int t = 0; t < 300; ++t) {
    std::vector<uint8_t> v(8 + rng() % 120);
    for (auto & b : v) {b = static_cast<uint8_t>(rng());}
    const uint32_t base = crc32c(v.data(), v.size());
    const size_t idx = rng() % v.size();
    const uint8_t bit = static_cast<uint8_t>(1u << (rng() % 8));
    v[idx] ^= bit;
    SF_EXPECT_NE(crc32c(v.data(), v.size()), base);
    v[idx] ^= bit;  // restore
    SF_EXPECT_EQ(crc32c(v.data(), v.size()), base);
  }
}

// Length sensitivity: appending a byte changes the CRC (almost surely).
TEST(Crc32c, LengthSensitivity)
{
  std::mt19937_64 rng(3);
  for (int t = 0; t < 400; ++t) {
    std::vector<uint8_t> v(4 + rng() % 100);
    for (auto & b : v) {b = static_cast<uint8_t>(rng());}
    const uint32_t base = crc32c(v.data(), v.size());
    v.push_back(static_cast<uint8_t>(rng()));
    SF_EXPECT_NE(crc32c(v.data(), v.size()), base);
  }
}
