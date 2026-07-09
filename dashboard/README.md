# SensorForge Dashboard (Extension N)

Python analysis dashboard for the SensorForge HIL platform. Scrapes the C++
Prometheus exporter (Extension M) and renders live Plotly charts, and turns the
scenario runner's JSON reports (Extension L) into a static HTML summary for CI
artifacts.

## Install

```bash
pip install -r dashboard/requirements.txt
```

## Live dashboard

```bash
# with the scenario runner exporting metrics on :9090
SENSORFORGE_EXPORTER=http://localhost:9090/metrics \
SENSORFORGE_REPORTS=./reports \
    uvicorn dashboard.app:app --host 0.0.0.0 --port 8080
```

Endpoints: `/` (charts), `/api/metrics` (JSON), `/api/reports`, `/health`.

## Static CI report

```bash
python -m dashboard.report_gen --reports ./reports --out summary.html
```

## Charts

- Latency percentiles (p50/p99/p999) per sensor
- Throughput over time per sensor
- Drop rate over time
- Scenario pass/fail timeline
- CRC failure count over time

`prometheus_client.parse_exposition()` has no third-party dependency (only
`scrape()` needs `requests`), so the parser is unit-testable standalone.
