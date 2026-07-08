/*
==============================================================================
SensorForge - SPSC ring benchmark (Extension C)
Single-thread push/pop throughput and latency, plus a threaded
producer->consumer latency probe through the ring.
==============================================================================
*/

#include <benchmark/benchmark.h>

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#include "bench/bench_common.hpp"
#include "sensorforge/core/spsc_ring.hpp"
#include "sensorforge/core/sensor_frame.hpp"

namespace {

using sensorforge::core::SPSCRing;
using sensorforge::core::SensorFrame;

// Fixed-size POD payload to isolate ring mechanics from allocation cost.
template<size_t N>
struct Blob { uint8_t bytes[N]; };

void BM_RingPushPop(benchmark::State & state)
{
  static SPSCRing<Blob<1024>, 1024> ring;
  Blob<1024> in{};
  Blob<1024> out{};
  for (auto _ : state) {
    ring.try_push(in);
    ring.try_pop(out);
    benchmark::DoNotOptimize(out);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_RingPushPop);

}  // namespace

namespace sfbench {

void bench_ring_latency()
{
  // Single-threaded push+pop pair latency.
  {
    static SPSCRing<Blob<1024>, 1024> ring;
    Blob<1024> in{};
    Blob<1024> out{};
    auto stats = measure([&]() {
      ring.try_push(in);
      ring.try_pop(out);
    }, 1000000);
    report("spsc_ring_1kb", stats);
  }

  // Threaded producer->consumer latency: producer stamps a nanosecond
  // timestamp into each frame; consumer measures now - timestamp.
  {
    SPSCRing<SensorFrame, 1024> ring;
    std::atomic<bool> stop{false};
    const uint64_t target = 200000;
    std::vector<double> latencies;
    latencies.reserve(target);

    std::thread producer([&]() {
      uint64_t seq = 0;
      while (!stop.load(std::memory_order_relaxed) && seq < target) {
        SensorFrame f;
        f.data.resize(1024);
        f.sequence = seq;
        f.timestamp_ns = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        if (ring.try_push(std::move(f))) {
          ++seq;
        }
      }
    });

    SensorFrame out;
    uint64_t got = 0;
    while (got < target) {
      if (ring.try_pop(out)) {
        const uint64_t now = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        latencies.push_back(static_cast<double>(now - out.timestamp_ns));
        ++got;
      }
    }
    stop.store(true, std::memory_order_relaxed);
    producer.join();

    std::sort(latencies.begin(), latencies.end());
    auto pct = [&](double p) {
      return latencies.empty() ? 0.0 : latencies[static_cast<size_t>(p * (latencies.size() - 1))];
    };
    LatencyStats s;
    s.iterations = got;
    s.p50_ns = pct(0.50);
    s.p99_ns = pct(0.99);
    s.p999_ns = pct(0.999);
    s.msgs_per_sec = 0;  // producer/consumer latency probe, not a throughput run
    report("spsc_prod2cons_1kb", s);
  }
}

}  // namespace sfbench
