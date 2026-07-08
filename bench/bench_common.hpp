/*
==============================================================================
SensorForge - Benchmark helpers (Extension C)
Part of the SensorForge AV HIL validation platform.

Two layers:
  1. Google Benchmark registrations (BENCHMARK macros) in each bench_*.cpp for
     standard framework throughput reporting.
  2. A small manual latency harness here that times individual operations,
     computes p50/p99/p999 from the sample distribution, and prints the exact
     [BENCH] lines the project expects for resume numbers. The manual layer is
     invoked from bench_main.cpp before Google Benchmark runs.

All numbers are produced at RUN TIME on the target host. This source contains
no hard-coded results.
==============================================================================
*/

#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <vector>

namespace sfbench {

struct LatencyStats
{
  double p50_ns = 0;
  double p99_ns = 0;
  double p999_ns = 0;
  double msgs_per_sec = 0;
  uint64_t iterations = 0;
};

/**
 * @brief Time @p op individually @p iters times and summarize the latency
 *        distribution. A warmup pass runs first to page in caches/branch
 *        predictors. Steady_clock call overhead (~tens of ns) is inherent to
 *        per-op timing and is noted in bench/README.md.
 */
inline LatencyStats measure(const std::function<void()> & op, uint64_t iters)
{
  using clock = std::chrono::steady_clock;

  // Warmup.
  for (uint64_t i = 0; i < iters / 10 + 1; ++i) {
    op();
  }

  std::vector<double> samples;
  samples.reserve(iters);
  const auto t_start = clock::now();
  for (uint64_t i = 0; i < iters; ++i) {
    const auto a = clock::now();
    op();
    const auto b = clock::now();
    samples.push_back(std::chrono::duration<double, std::nano>(b - a).count());
  }
  const auto t_end = clock::now();

  std::sort(samples.begin(), samples.end());
  auto pct = [&samples](double p) -> double {
    if (samples.empty()) {
      return 0.0;
    }
    size_t idx = static_cast<size_t>(p * (samples.size() - 1));
    return samples[idx];
  };

  LatencyStats s;
  s.iterations = iters;
  s.p50_ns = pct(0.50);
  s.p99_ns = pct(0.99);
  s.p999_ns = pct(0.999);
  const double total_s = std::chrono::duration<double>(t_end - t_start).count();
  s.msgs_per_sec = total_s > 0 ? static_cast<double>(iters) / total_s : 0.0;
  return s;
}

inline void report(const char * name, const LatencyStats & s)
{
  std::printf(
    "[BENCH] %-24s msgs/sec=%.0f p50=%.0fns p99=%.0fns p999=%.0fns\n",
    name, s.msgs_per_sec, s.p50_ns, s.p99_ns, s.p999_ns);
}

inline void report_throughput(const char * name, double gbps)
{
  std::printf("[BENCH] %-24s GB/s=%.3f\n", name, gbps);
}

// Latency probes implemented in the individual bench_*.cpp files and driven by
// bench_main.cpp.
void bench_frame_latency();
void bench_ring_latency();
void bench_crc_throughput();
void bench_e2e_latency();

}  // namespace sfbench
