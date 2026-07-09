"""
SensorForge - Plotly chart builders (Extension N)
Part of the SensorForge AV HIL validation platform.

Each function returns a Plotly Figure. Callers embed them with
fig.to_html(full_html=False, include_plotlyjs=...) or serve fig.to_json().
"""
from __future__ import annotations

from typing import Iterable

import plotly.graph_objects as go

# Brand-neutral categorical palette (accessible in light/dark).
_PALETTE = ["#4C78A8", "#F58518", "#54A24B", "#E45756", "#72B7B2", "#B279A2"]


def latency_histogram(per_sensor: dict[str, dict[str, float]]) -> go.Figure:
    """Grouped bar of p50/p99/p999 latency (ms) per sensor.

    per_sensor: {sensor: {"p50": x, "p99": y, "p999": z}}
    """
    sensors = list(per_sensor.keys())
    fig = go.Figure()
    for i, pct in enumerate(("p50", "p99", "p999")):
        fig.add_bar(
            name=pct,
            x=sensors,
            y=[per_sensor[s].get(pct, 0.0) for s in sensors],
            marker_color=_PALETTE[i % len(_PALETTE)],
        )
    fig.update_layout(
        title="Latency percentiles per sensor",
        barmode="group",
        yaxis_title="latency (ms)",
        template="plotly_dark",
    )
    return fig


def throughput_over_time(series: dict[str, list[tuple[float, float]]]) -> go.Figure:
    """Line chart of msgs/sec over time per sensor.

    series: {sensor: [(t_seconds, msgs_per_sec), ...]}
    """
    fig = go.Figure()
    for i, (sensor, points) in enumerate(series.items()):
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        fig.add_scatter(
            name=sensor, x=xs, y=ys, mode="lines",
            line=dict(color=_PALETTE[i % len(_PALETTE)]),
        )
    fig.update_layout(
        title="Throughput over time",
        xaxis_title="time (s)",
        yaxis_title="msgs/sec",
        template="plotly_dark",
    )
    return fig


def drop_rate_over_time(series: dict[str, list[tuple[float, float]]]) -> go.Figure:
    fig = go.Figure()
    for i, (sensor, points) in enumerate(series.items()):
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        fig.add_scatter(
            name=sensor, x=xs, y=ys, mode="lines",
            line=dict(color=_PALETTE[i % len(_PALETTE)]),
        )
    fig.update_layout(
        title="Drop rate over time",
        xaxis_title="time (s)",
        yaxis_title="drop rate (%)",
        template="plotly_dark",
    )
    return fig


def scenario_timeline(results: Iterable[dict]) -> go.Figure:
    """Pass/fail timeline from a list of scenario report dicts."""
    names = [r.get("scenario", f"run{i}") for i, r in enumerate(results)]
    passed = [1 if r.get("result") == "PASS" else 0 for r in results]
    colors = ["#54A24B" if p else "#E45756" for p in passed]
    fig = go.Figure(
        go.Bar(x=names, y=[1] * len(names), marker_color=colors,
               text=["PASS" if p else "FAIL" for p in passed], textposition="inside"))
    fig.update_layout(
        title="Scenario pass/fail timeline",
        yaxis=dict(visible=False),
        template="plotly_dark",
    )
    return fig


def crc_failure_rate(points: list[tuple[float, float]]) -> go.Figure:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    fig = go.Figure(go.Scatter(x=xs, y=ys, mode="lines", line=dict(color="#E45756")))
    fig.update_layout(
        title="CRC failure count over time",
        xaxis_title="time (s)",
        yaxis_title="CRC failures",
        template="plotly_dark",
    )
    return fig
