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

Results are produced **on the EC2 benchmark host** — this repository is
developed on macOS where ROS2 and the Linux build do not run, so no numbers are
checked in here (fabricated benchmark numbers would be worse than none).

Fill this table from a real run and record the host details:

```
Host:        <instance type, e.g. c7i.2xlarge>
CPU:         <model, base/turbo GHz>
OS/Kernel:   <e.g. Ubuntu 24.04, 6.8.x>
Compiler:    <e.g. clang 18 / gcc 13>, -O3 -DNDEBUG
CRC path:    <hardware SSE4.2 | ARM CRC | software fallback>
```

| Benchmark              | msgs/sec | p50 (ns) | p99 (ns) | p999 (ns) |
|------------------------|----------|----------|----------|-----------|
| spsc_ring_1kb          | TBD      | TBD      | TBD      | TBD       |
| spsc_prod2cons_1kb     | —        | TBD      | TBD      | TBD       |
| frame_encode_1kb       | TBD      | TBD      | TBD      | TBD       |
| frame_decode_1kb       | TBD      | TBD      | TBD      | TBD       |
| e2e_pipeline_1kb       | TBD      | TBD      | TBD      | TBD       |

| CRC32C buffer | GB/s |
|---------------|------|
| 64 B          | TBD  |
| 1 KB          | TBD  |
| 64 KB         | TBD  |
```
