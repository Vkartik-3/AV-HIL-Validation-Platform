# SensorForge CI Regression Gate (Extension P)

`check_regression.py` parses the `[BENCH]` lines emitted by `sensorforge_bench`
and fails the build on a performance regression.

## Thresholds

| Metric | Rule |
|---|---|
| p99 latency | **FAIL** if current > baseline × 1.10 (>10% worse) |
| drop rate | **FAIL** if current > baseline + 0.1 percentage points |
| throughput (msgs/sec) | **WARN** if current < baseline × 0.95 (>5% drop) |

## How the baseline works in CI

The `benchmark` job compares against the **previous run's** results, restored
via `actions/cache` (so the comparison is between runs on the same runner
class, not across different hardware). The first run has no cache and simply
establishes the baseline. Each run rewrites the cached baseline for the next.

```bash
python3 ci/check_regression.py \
    --current benchmark_results.txt \
    --baseline bench-baseline/baseline.json \
    --update-baseline
```

Exit code is non-zero on any FAIL, which fails the CI job. A Markdown table is
written to `$GITHUB_STEP_SUMMARY`.

## Committed baseline

`baseline/benchmark_baseline.json` holds the **real numbers measured on EC2**
(c7i-flex.large, ros:humble container). It is a reference for local use and
documentation; the CI gate uses the per-runner cached baseline described above
(absolute latencies differ across machines, so a committed cross-machine
baseline would false-fail).

## Local use

```bash
./build/bench/sensorforge_bench | tee bench.txt
python3 ci/check_regression.py --current bench.txt \
    --baseline ci/baseline/benchmark_baseline.json
```
