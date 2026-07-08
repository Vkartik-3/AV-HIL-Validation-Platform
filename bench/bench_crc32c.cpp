/*
==============================================================================
SensorForge - CRC32C benchmark (Extension C)
Throughput of crc32c() at 64B / 1KB / 64KB buffers.
==============================================================================
*/

#include <benchmark/benchmark.h>

#include <cstdint>
#include <vector>

#include "bench/bench_common.hpp"
#include "sensorforge/protocol/crc32c.hpp"

namespace {

std::vector<uint8_t> make_buffer(size_t n)
{
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i) {
    v[i] = static_cast<uint8_t>(i * 131 + 7);
  }
  return v;
}

void BM_Crc32c(benchmark::State & state)
{
  const auto buf = make_buffer(static_cast<size_t>(state.range(0)));
  for (auto _ : state) {
    uint32_t c = sensorforge::crc::crc32c(buf.data(), buf.size());
    benchmark::DoNotOptimize(c);
  }
  state.SetBytesProcessed(static_cast<int64_t>(state.iterations()) * buf.size());
}
BENCHMARK(BM_Crc32c)->Arg(64)->Arg(1024)->Arg(64 * 1024);

}  // namespace

namespace sfbench {

void bench_crc_throughput()
{
  struct Case { const char * name; size_t size; };
  const Case cases[] = {
    {"crc32c_throughput_64b", 64},
    {"crc32c_throughput_1kb", 1024},
    {"crc32c_throughput_64kb", 64 * 1024},
  };
  for (const auto & c : cases) {
    const auto buf = make_buffer(c.size);
    // Measure GB/s over many passes.
    const uint64_t passes = c.size >= 64 * 1024 ? 20000 : 2000000;
    using clock = std::chrono::steady_clock;
    volatile uint32_t sink = 0;
    const auto a = clock::now();
    for (uint64_t i = 0; i < passes; ++i) {
      sink ^= sensorforge::crc::crc32c(buf.data(), buf.size());
    }
    const auto b = clock::now();
    (void)sink;
    const double secs = std::chrono::duration<double>(b - a).count();
    const double gbps = secs > 0
      ? (static_cast<double>(passes) * c.size) / secs / 1e9 : 0.0;
    report_throughput(c.name, gbps);
  }
}

}  // namespace sfbench
