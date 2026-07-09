"""
SensorForge - FastAPI analysis dashboard (Extension N)
Part of the SensorForge AV HIL validation platform.

Scrapes the SensorForge Prometheus exporter and renders live charts, and lists
the JSON scenario reports written by the scenario runner.

Run:
    pip install -r dashboard/requirements.txt
    SENSORFORGE_EXPORTER=http://localhost:9090/metrics \
    SENSORFORGE_REPORTS=./reports \
        uvicorn dashboard.app:app --host 0.0.0.0 --port 8080
"""
from __future__ import annotations

import glob
import json
import os
import time
from collections import defaultdict

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse

from . import charts
from .prometheus_client import scrape

EXPORTER_URL = os.environ.get("SENSORFORGE_EXPORTER", "http://localhost:9090/metrics")
REPORTS_DIR = os.environ.get("SENSORFORGE_REPORTS", "./reports")

app = FastAPI(title="SensorForge Dashboard")

# In-memory time series accumulated from successive scrapes (best-effort).
_START = time.time()
_throughput: dict[str, list[tuple[float, float]]] = defaultdict(list)
_drop_rate: dict[str, list[tuple[float, float]]] = defaultdict(list)
_crc: list[tuple[float, float]] = []


def _poll_once() -> dict[str, dict[str, float]]:
    """Scrape the exporter and fold the sample into the in-memory series."""
    s = scrape(EXPORTER_URL)
    t = time.time() - _START
    for sensor, v in s.sensor_values("sensorforge_msgs_per_sec").items():
        _throughput[sensor].append((t, v))
    for sensor, v in s.sensor_values("sensorforge_drop_rate_pct").items():
        _drop_rate[sensor].append((t, v))
    _crc.append((t, s.scalar("sensorforge_crc_failures_total")))
    per_sensor: dict[str, dict[str, float]] = defaultdict(dict)
    for pct, metric in (("p50", "sensorforge_latency_p50_ms"),
                        ("p99", "sensorforge_latency_p99_ms"),
                        ("p999", "sensorforge_latency_p999_ms")):
        for sensor, v in s.sensor_values(metric).items():
            per_sensor[sensor][pct] = v
    return per_sensor


def _load_reports() -> list[dict]:
    reports = []
    for path in sorted(glob.glob(os.path.join(REPORTS_DIR, "*.json"))):
        try:
            with open(path) as f:
                reports.append(json.load(f))
        except (OSError, json.JSONDecodeError):
            continue
    return reports


@app.get("/health")
def health() -> dict:
    return {"status": "ok", "exporter": EXPORTER_URL, "reports_dir": REPORTS_DIR}


@app.get("/api/metrics")
def api_metrics() -> JSONResponse:
    try:
        per_sensor = _poll_once()
    except Exception as exc:  # exporter may be down
        return JSONResponse({"error": str(exc)}, status_code=503)
    return JSONResponse({"per_sensor": per_sensor})


@app.get("/api/reports")
def api_reports() -> JSONResponse:
    return JSONResponse({"reports": _load_reports()})


@app.get("/", response_class=HTMLResponse)
def index() -> HTMLResponse:
    try:
        per_sensor = _poll_once()
        figs = [
            charts.latency_histogram(per_sensor),
            charts.throughput_over_time(_throughput),
            charts.drop_rate_over_time(_drop_rate),
            charts.crc_failure_rate(_crc),
        ]
    except Exception as exc:
        return HTMLResponse(f"<h1>SensorForge Dashboard</h1><p>Exporter unreachable: {exc}</p>")

    reports = _load_reports()
    if reports:
        figs.insert(0, charts.scenario_timeline(reports))

    body = ["<h1>SensorForge Dashboard</h1>"]
    for i, fig in enumerate(figs):
        body.append(fig.to_html(full_html=False, include_plotlyjs=("cdn" if i == 0 else False)))
    return HTMLResponse("<html><body style='background:#0f1115;color:#eee'>"
                        + "".join(body) + "</body></html>")
