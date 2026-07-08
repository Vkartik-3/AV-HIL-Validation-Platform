/*
==============================================================================
SensorForge - Benchmark entry point (Extension C)

Runs the manual latency/throughput probes (which emit the [BENCH] lines the
project records as resume numbers), then hands off to Google Benchmark for the
standard framework report. Run each pass >= 3 times and take the median; see
bench/README.md.
==============================================================================
*/

#include <benchmark/benchmark.h>
#include <cstdio>

#include "bench/bench_common.hpp"

int main(int argc, char ** argv)
{
  std::printf("=== SensorForge [BENCH] latency/throughput probes ===\n");
  sfbench::bench_crc_throughput();
  sfbench::bench_ring_latency();
  sfbench::bench_frame_latency();
  sfbench::bench_e2e_latency();
  std::printf("=== Google Benchmark framework report ===\n");

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
  return 0;
}
