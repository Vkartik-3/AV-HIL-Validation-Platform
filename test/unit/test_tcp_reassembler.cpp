/*
==============================================================================
SensorForge - TCP reassembler tests (Extension G)
Framing, arbitrary fragmentation, resync on garbage, and oversized rejection.
==============================================================================
*/

#include <gtest/gtest.h>

#include "test_support.hpp"

#include <cstdint>
#include <vector>

#include "sensorforge/transport/tcp_reassembler.hpp"

using sensorforge::transport::TcpReassembler;

namespace {
// Frame a payload the way TcpInterface::write does: 0xAB 0xCD | be32 len | data.
std::vector<uint8_t> frame_payload(const std::vector<uint8_t> & p)
{
  std::vector<uint8_t> out;
  out.push_back(0xAB);
  out.push_back(0xCD);
  const uint32_t n = static_cast<uint32_t>(p.size());
  out.push_back(static_cast<uint8_t>((n >> 24) & 0xFF));
  out.push_back(static_cast<uint8_t>((n >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((n >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>(n & 0xFF));
  out.insert(out.end(), p.begin(), p.end());
  return out;
}
}  // namespace

TEST(TcpReassembler, SingleFrame)
{
  std::vector<std::vector<uint8_t>> got;
  TcpReassembler r([&](const uint8_t * p, size_t n) {got.emplace_back(p, p + n);});
  const std::vector<uint8_t> payload = {1, 2, 3, 4, 5};
  const auto wire = frame_payload(payload);
  r.feed(wire.data(), wire.size());
  SF_ASSERT_EQ(got.size(), 1u);
  SF_EXPECT_EQ(got[0], payload);
}

TEST(TcpReassembler, ManyFramesArbitraryFragmentation)
{
  std::vector<std::vector<uint8_t>> expected;
  std::vector<uint8_t> stream;
  for (int i = 0; i < 500; ++i) {
    std::vector<uint8_t> p(1 + (i % 37), static_cast<uint8_t>(i));
    expected.push_back(p);
    auto w = frame_payload(p);
    stream.insert(stream.end(), w.begin(), w.end());
  }

  std::vector<std::vector<uint8_t>> got;
  TcpReassembler r([&](const uint8_t * p, size_t n) {got.emplace_back(p, p + n);});

  // Feed the concatenated stream in awkward 3-byte chunks.
  for (size_t off = 0; off < stream.size(); off += 3) {
    const size_t n = std::min<size_t>(3, stream.size() - off);
    r.feed(stream.data() + off, n);
  }
  SF_ASSERT_EQ(got.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    SF_EXPECT_EQ(got[i], expected[i]);
  }
}

TEST(TcpReassembler, ResyncsPastGarbage)
{
  std::vector<std::vector<uint8_t>> got;
  TcpReassembler r([&](const uint8_t * p, size_t n) {got.emplace_back(p, p + n);});
  // Leading garbage with no false 0xAB 0xCD start marker, then a real frame.
  // (This weak 2-byte framing cannot resync through garbage that itself
  // contains the start marker -- a known limitation the frame protocol fixes
  // with a 4-byte magic + CRC. This test covers the recoverable case.)
  std::vector<uint8_t> stream = {0x00, 0x11, 0x22, 0x33, 0x44};
  const std::vector<uint8_t> payload = {7, 7, 7};
  auto w = frame_payload(payload);
  stream.insert(stream.end(), w.begin(), w.end());
  r.feed(stream.data(), stream.size());
  SF_ASSERT_EQ(got.size(), 1u);
  SF_EXPECT_EQ(got[0], payload);
}

TEST(TcpReassembler, OversizedLengthRejectedNoCrash)
{
  uint64_t emitted = 0;
  TcpReassembler r(
    [&](const uint8_t *, size_t) {++emitted;}, /*max_payload=*/1024);
  // 0xAB 0xCD then a huge length (0xFFFFFFFF) then junk.
  std::vector<uint8_t> stream = {0xAB, 0xCD, 0xFF, 0xFF, 0xFF, 0xFF, 1, 2, 3, 4};
  r.feed(stream.data(), stream.size());
  SF_EXPECT_EQ(emitted, 0u);
  SF_EXPECT_GE(r.resyncs(), 1u);
}
