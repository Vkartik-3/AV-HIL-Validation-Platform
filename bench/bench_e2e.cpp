/*
==============================================================================
SensorForge - End-to-end pipeline benchmark (Extension C)
Measures the full in-process path: produce -> SPSC ring -> consumer ->
frame encode -> frame decode/validate, at 1KB (and a few other sizes).
Compression is intentionally excluded here so the number isolates the
SensorForge core path; the ROS/zstd path is exercised by scenario tests.
==============================================================================
*/

#include <benchmark/benchmark.h>

#include <cstdint>
#include <string>
#include <vector>

#include "bench/bench_common.hpp"
#include "sensorforge/core/spsc_ring.hpp"
#include "sensorforge/core/sensor_frame.hpp"
#include "sensorforge/protocol/frame_codec.hpp"

namespace {

using sensorforge::core::SPSCRing;
using sensorforge::core::SensorFrame;
using namespace sensorforge::protocol;

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

// One pipeline pass on a single thread (deterministic, no cross-thread jitter).
uint64_t pipeline_pass(SPSCRing<SensorFrame, 1024> & ring, size_t payload_size, uint64_t seq)
{
  SensorFrame in;
  in.data.assign(payload_size, static_cast<uint8_t>(seq));
  in.sequence = seq;
  in.timestamp_ns = seq;
  ring.try_push(std::move(in));

  SensorFrame out;
  ring.try_pop(out);

  auto frame = encode_frame(
    SensorType::kLidar, out.sequence, out.timestamp_ns, kFlagNone,
    out.data.data(), out.data.size());

  FrameHeader hdr;
  auto err = decode_header(frame.data(), frame.size(), hdr);
  return static_cast<uint64_t>(err) + frame.size();
}

void BM_E2E(benchmark::State & state)
{
  static SPSCRing<SensorFrame, 1024> ring;
  const size_t sz = static_cast<size_t>(state.range(0));
  uint64_t seq = 1;
  for (auto _ : state) {
    uint64_t r = pipeline_pass(ring, sz, seq++);
    benchmark::DoNotOptimize(r);
  }
  state.SetItemsProcessed(state.iterations());
  state.SetBytesProcessed(static_cast<int64_t>(state.iterations()) * sz);
}
BENCHMARK(BM_E2E)->Arg(64)->Arg(1024)->Arg(64 * 1024)->Arg(1024 * 1024);

}  // namespace

namespace sfbench {

void bench_e2e_latency()
{
  const size_t sizes[] = {64, 1024, 64 * 1024, 1024 * 1024};
  for (size_t n : sizes) {
    SPSCRing<SensorFrame, 1024> ring;
    uint64_t seq = 1;
    const uint64_t iters = n >= 1024 * 1024 ? 5000 : 100000;
    auto stats = measure([&]() {
      uint64_t r = pipeline_pass(ring, n, seq++);
      volatile uint64_t sink = r;
      (void)sink;
    }, iters);
    std::string name = std::string("e2e_pipeline_") + size_suffix(n);
    report(name.c_str(), stats);
  }
}

}  // namespace sfbench
