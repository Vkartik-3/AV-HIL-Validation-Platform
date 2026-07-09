"""
SensorForge - Static HTML report generator (Extension N)
Part of the SensorForge AV HIL validation platform.

Combines the JSON scenario reports written by the scenario runner into a single
self-contained HTML page (Plotly charts inlined) suitable for uploading as a CI
artifact.

Usage:
    python -m dashboard.report_gen --reports ./reports --out summary.html
"""
from __future__ import annotations

import argparse
import glob
import json
import os

from . import charts


def load_reports(reports_dir: str) -> list[dict]:
    out = []
    for path in sorted(glob.glob(os.path.join(reports_dir, "*.json"))):
        try:
            with open(path) as f:
                out.append(json.load(f))
        except (OSError, json.JSONDecodeError):
            continue
    return out


def _per_sensor_latency(reports: list[dict]) -> dict[str, dict[str, float]]:
    """Latest per-sensor p50/p99/p999 across all reports."""
    out: dict[str, dict[str, float]] = {}
    for r in reports:
        for sensor, m in r.get("metrics", {}).get("per_sensor", {}).items():
            out[sensor] = {
                "p50": m.get("latency_p50_ms", 0.0),
                "p99": m.get("latency_p99_ms", 0.0),
                "p999": m.get("latency_p999_ms", 0.0),
            }
    return out


def build_html(reports: list[dict]) -> str:
    figs = []
    if reports:
        figs.append(charts.scenario_timeline(reports))
    per_sensor = _per_sensor_latency(reports)
    if per_sensor:
        figs.append(charts.latency_histogram(per_sensor))

    passed = sum(1 for r in reports if r.get("result") == "PASS")
    failed = len(reports) - passed

    parts = [
        "<html><head><meta charset='utf-8'><title>SensorForge CI Report</title></head>",
        "<body style='background:#0f1115;color:#eee;font-family:sans-serif;padding:20px'>",
        "<h1>SensorForge Scenario Summary</h1>",
        f"<p>{len(reports)} scenarios &middot; "
        f"<span style='color:#54A24B'>{passed} passed</span> &middot; "
        f"<span style='color:#E45756'>{failed} failed</span></p>",
    ]
    for i, fig in enumerate(figs):
        parts.append(fig.to_html(full_html=False, include_plotlyjs=("cdn" if i == 0 else False)))
    parts.append("</body></html>")
    return "".join(parts)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--reports", default="./reports")
    ap.add_argument("--out", default="summary.html")
    args = ap.parse_args()
    reports = load_reports(args.reports)
    html = build_html(reports)
    with open(args.out, "w") as f:
        f.write(html)
    print(f"Wrote {args.out} from {len(reports)} report(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
