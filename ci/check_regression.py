#!/usr/bin/env python3
"""
SensorForge - CI benchmark regression gate (Extension P)
Part of the SensorForge AV HIL validation platform.

Parses the [BENCH] lines emitted by sensorforge_bench (and, optionally,
scenario drop-rate metrics), compares them against a committed baseline, and
fails the build on a regression:

  - p99 latency   : FAIL if current > baseline * (1 + P99_FAIL_PCT/100)   [10%]
  - drop rate     : FAIL if current > baseline + DROP_FAIL_ABS_PCT        [0.1%]
  - msgs/sec      : WARN if current < baseline * (1 - THPUT_WARN_PCT/100)  [5%]

Writes a Markdown summary (to $GITHUB_STEP_SUMMARY when set) and, with
--update-baseline, rewrites the baseline for the next run.

Usage:
  python ci/check_regression.py --current bench.txt --baseline ci/baseline/benchmark_baseline.json
  python ci/check_regression.py --current bench.txt --baseline b.json --update-baseline
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys

# Thresholds are env-overridable so shared CI runners (noisy p99) can use a
# tolerant value while dedicated/bare-metal hardware uses the strict default.
P99_FAIL_PCT = float(os.environ.get("SF_P99_FAIL_PCT", "10.0"))
DROP_FAIL_ABS_PCT = float(os.environ.get("SF_DROP_FAIL_ABS_PCT", "0.1"))
THPUT_WARN_PCT = float(os.environ.get("SF_THPUT_WARN_PCT", "5.0"))

_BENCH_RE = re.compile(r"^\[BENCH\]\s+(?P<name>\S+)\s+(?P<rest>.+)$")
_KV_RE = re.compile(r"(\w+/?\w*)=([0-9.eE+-]+)")


def parse_bench(text: str) -> dict[str, dict[str, float]]:
    """Parse [BENCH] lines into {name: {metric: value}}.

    Handles both "msgs/sec=X p50=Yns p99=Zns" and "GB/s=X" forms; strips unit
    suffixes (ns) from values.
    """
    out: dict[str, dict[str, float]] = {}
    for line in text.splitlines():
        m = _BENCH_RE.match(line.strip())
        if not m:
            continue
        metrics: dict[str, float] = {}
        for kv in _KV_RE.finditer(m.group("rest")):
            key = kv.group(1).replace("/", "_")
            metrics[key] = float(kv.group(2))
        out[m.group("name")] = metrics
    return out


def _fmt(v: float) -> str:
    return f"{v:.3g}"


def compare(current: dict, baseline: dict) -> tuple[list[str], bool]:
    """Return (summary_rows, failed)."""
    rows = ["| metric | field | baseline | current | delta | verdict |",
            "|---|---|---|---|---|---|"]
    failed = False

    for name, cur in sorted(current.items()):
        base = baseline.get(name)
        if not base:
            rows.append(f"| {name} | - | (new) | - | - | NEW |")
            continue

        # p99 latency gate
        if "p99" in cur and "p99" in base and base["p99"] > 0:
            delta = 100.0 * (cur["p99"] - base["p99"]) / base["p99"]
            bad = cur["p99"] > base["p99"] * (1 + P99_FAIL_PCT / 100.0)
            verdict = "FAIL" if bad else "ok"
            failed = failed or bad
            rows.append(f"| {name} | p99 | {_fmt(base['p99'])} | {_fmt(cur['p99'])} "
                        f"| {delta:+.1f}% | {verdict} |")

        # throughput warning
        if "msgs_sec" in cur and "msgs_sec" in base and base["msgs_sec"] > 0:
            delta = 100.0 * (cur["msgs_sec"] - base["msgs_sec"]) / base["msgs_sec"]
            warn = cur["msgs_sec"] < base["msgs_sec"] * (1 - THPUT_WARN_PCT / 100.0)
            verdict = "WARN" if warn else "ok"
            rows.append(f"| {name} | msgs/sec | {_fmt(base['msgs_sec'])} | {_fmt(cur['msgs_sec'])} "
                        f"| {delta:+.1f}% | {verdict} |")

        # drop-rate gate (absolute percentage points)
        if "drop_rate_pct" in cur and "drop_rate_pct" in base:
            delta = cur["drop_rate_pct"] - base["drop_rate_pct"]
            bad = delta > DROP_FAIL_ABS_PCT
            verdict = "FAIL" if bad else "ok"
            failed = failed or bad
            rows.append(f"| {name} | drop% | {_fmt(base['drop_rate_pct'])} "
                        f"| {_fmt(cur['drop_rate_pct'])} | {delta:+.3f}pp | {verdict} |")

    return rows, failed


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--current", required=True, help="benchmark output text file")
    ap.add_argument("--baseline", required=True, help="baseline JSON path")
    ap.add_argument("--update-baseline", action="store_true")
    args = ap.parse_args()

    with open(args.current) as f:
        current = parse_bench(f.read())
    if not current:
        print("No [BENCH] lines found in current results", file=sys.stderr)
        return 2

    baseline = {}
    if os.path.exists(args.baseline):
        with open(args.baseline) as f:
            baseline = json.load(f)

    rows, failed = compare(current, baseline)
    summary = "## SensorForge benchmark regression gate\n\n" + "\n".join(rows) + "\n"
    summary += (f"\n**Gate:** p99 fail >{P99_FAIL_PCT}%, drop fail >{DROP_FAIL_ABS_PCT}pp, "
                f"throughput warn >{THPUT_WARN_PCT}%. "
                f"Result: {'FAIL' if failed else 'PASS'}\n")
    print(summary)

    step_summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if step_summary:
        with open(step_summary, "a") as f:
            f.write(summary)

    if args.update_baseline:
        os.makedirs(os.path.dirname(args.baseline) or ".", exist_ok=True)
        with open(args.baseline, "w") as f:
            json.dump(current, f, indent=2, sort_keys=True)
        print(f"Updated baseline -> {args.baseline}")

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
