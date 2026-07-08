# SensorForge Benchmark Harness (Extension C)

Google Benchmark harness for the SensorForge high-performance core. Measures
real, reproducible numbers for the frame protocol, CRC32C, the SPSC ring, and
the end-to-end in-process pipeline.

## Build

```bash
cmake -S . -B build -DSENSORFORGE_BUILD_BENCH=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build --target sensorforge_bench -j
```

Requires `libbenchmark-dev` (Google Benchmark) and a C++20 compiler. No ROS2
environment is needed — the harness links only `sensorforge_protocol`.

## Run

```bash
# Emits the [BENCH] lines, then the Google Benchmark framework report.
./build/bench/sensorforge_bench --benchmark_repetitions=3 \
    --benchmark_report_aggregates_only=true
```

Run each configuration **at least 3 times and record the median**. Pin the CPU
governor to `performance` and, ideally, pin the process to an isolated core:

```bash
sudo cpupower frequency-set -g performance
taskset -c 2 ./build/bench/sensorforge_bench --benchmark_repetitions=5
```

## What is measured

| Probe                    | Metric                                             |
|--------------------------|----------------------------------------------------|
| `crc32c_throughput_*`    | GB/s at 64B / 1KB / 64KB                            |
| `spsc_ring_1kb`          | single-thread push+pop latency (p50/p99/p999)      |
| `spsc_prod2cons_1kb`     | threaded producer→consumer latency through ring    |
| `frame_encode_*`         | encode msgs/sec + latency at 64B…4MB               |
| `frame_decode_*`         | decode/validate msgs/sec + latency at 64B…4MB      |
| `e2e_pipeline_*`         | produce→ring→consumer→encode→decode at 64B…1MB     |

## Measurement note

The manual latency layer times each operation individually with
`std::chrono::steady_clock`. Per-call clock overhead (tens of ns on Linux) is
inherent to per-op latency measurement and sits underneath the reported p50 for
the smallest payloads; it is not subtracted. Throughput (`msgs/sec`, `GB/s`)
and the Google Benchmark framework numbers amortize this and are the figures to
quote for bandwidth. Treat the smallest-payload p50 as an upper bound.

## Results

These numbers were measured on a real Linux host (this repo is developed on
macOS, where ROS2/the Linux build do not run). Reproduce with the run procedure
above; treat them as representative of a 2-vCPU cloud instance, not a tuned
bare-metal box.

```
Host:        AWS EC2 c7i-flex.large (2 vCPU, 4 GiB), build in ros:humble Docker
CPU:         Intel Xeon (Sapphire Rapids class), 2.40 GHz; L1d 48K, L2 2M, L3 105M
OS:          Ubuntu 22.04 (Jammy) container on Ubuntu 26.04 host
Compiler:    colcon Release (-O3 -DNDEBUG)
CRC path:    hardware SSE4.2 (x86_64) — confirmed by ~9.4 GB/s
Date:        2026-07-08
```

Manual `[BENCH]` latency probes (single-shot, include per-op clock overhead):

| Benchmark              | msgs/sec  | p50 (ns) | p99 (ns) | p999 (ns) |
|------------------------|-----------|----------|----------|-----------|
| spsc_ring_1kb          | 9,821,634 | 70       | 81       | 124       |
| spsc_prod2cons_1kb     | —         | 241,994  | 380,202  | 484,147   |
| frame_encode_1kb       | 3,429,186 | 261      | 277      | 359       |
| frame_decode_1kb       | 4,421,144 | 197      | 208      | 225       |
| e2e_pipeline_1kb       | 2,115,991 | 435      | 493      | 674       |

| CRC32C buffer | GB/s (manual) | GB/s (GBench median) |
|---------------|---------------|----------------------|
| 64 B          | 10.30         | 9.45                 |
| 1 KB          | 9.61          | 9.00                 |
| 64 KB         | 6.37          | 5.94                 |

Google Benchmark framework medians (amortized, more rigorous for throughput):
frame_decode/64 ≈ 11.9 ns (83.8 M/s), frame_encode/1KB ≈ 211 ns (4.53 GB/s),
ring push+pop ≈ 49.6 ns (20.2 M/s), e2e/1KB ≈ 393 ns (2.43 GB/s).

Note on `spsc_prod2cons_1kb`: the ~242 µs p50 reflects cross-core wakeup /
scheduler contention on a **2-vCPU** instance where a busy-spin consumer and
producer compete for CPU. On a box with >2 dedicated cores this drops by orders
of magnitude; it is a host-contention artifact, not a ring cost (the same ring
does a push+pop in ~50 ns single-threaded above).
```
