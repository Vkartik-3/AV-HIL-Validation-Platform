/*
==============================================================================
SensorForge - Frame encode/decode benchmark (Extension C)
Throughput and latency of encode_frame / decode_header at payload sizes
64B / 1KB / 64KB / 1MB / 4MB.
==============================================================================
*/

#include <benchmark/benchmark.h>

#include <cstdint>
#include <string>
#include <vector>

#include "bench/bench_common.hpp"
#include "sensorforge/protocol/frame_codec.hpp"

namespace {

using namespace sensorforge::protocol;

std::vector<uint8_t> make_payload(size_t n)
{
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i) {
    v[i] = static_cast<uint8_t>((i * 37) ^ 0xA5);
  }
  return v;
}

void BM_FrameEncode(benchmark::State & state)
{
  const auto payload = make_payload(static_cast<size_t>(state.range(0)));
  uint64_t seq = 0;
  for (auto _ : state) {
    auto frame = encode_frame(
      SensorType::kLidar, seq++, seq, kFlagNone, payload.data(), payload.size());
    benchmark::DoNotOptimize(frame.data());
  }
  state.SetItemsProcessed(state.iterations());
  state.SetBytesProcessed(static_cast<int64_t>(state.iterations()) * payload.size());
}
BENCHMARK(BM_FrameEncode)->Arg(64)->Arg(1024)->Arg(64 * 1024)->Arg(1024 * 1024)
->Arg(4 * 1024 * 1024);

void BM_FrameDecode(benchmark::State & state)
{
  const auto payload = make_payload(static_cast<size_t>(state.range(0)));
  const auto frame =
    encode_frame(SensorType::kLidar, 1, 1, kFlagNone, payload.data(), payload.size());
  FrameHeader hdr;
  for (auto _ : state) {
    auto err = decode_header(frame.data(), frame.size(), hdr);
    benchmark::DoNotOptimize(err);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_FrameDecode)->Arg(64)->Arg(1024)->Arg(64 * 1024)->Arg(1024 * 1024)
->Arg(4 * 1024 * 1024);

const char * size_suffix(size_t n)
{
  switch (n) {
    case 64: return "64b";
    case 1024: return "1kb";
    case 64 * 1024: return "64kb";
    case 1024 * 1024: return "1mb";
    case 4 * 1024 * 1024: return "4mb";
    default: return "?";
  }
}

}  // namespace

namespace sfbench {

void bench_frame_latency()
{
  const size_t sizes[] = {64, 1024, 64 * 1024, 1024 * 1024, 4 * 1024 * 1024};
  for (size_t n : sizes) {
    const auto payload = make_payload(n);
    const uint64_t iters = n >= 1024 * 1024 ? 5000 : 200000;

    uint64_t seq = 0;
    auto enc_stats = measure([&]() {
      auto f = encode_frame(
        SensorType::kLidar, seq++, seq, kFlagNone, payload.data(), payload.size());
      volatile auto s = f.size();
      (void)s;
    }, iters);
    std::string enc_name = std::string("frame_encode_") + size_suffix(n);
    report(enc_name.c_str(), enc_stats);

    const auto frame =
      encode_frame(SensorType::kLidar, 1, 1, kFlagNone, payload.data(), payload.size());
    FrameHeader hdr;
    auto dec_stats = measure([&]() {
      auto err = decode_header(frame.data(), frame.size(), hdr);
      volatile int e = static_cast<int>(err);
      (void)e;
    }, iters);
    std::string dec_name = std::string("frame_decode_") + size_suffix(n);
    report(dec_name.c_str(), dec_stats);
  }
}

}  // namespace sfbench
