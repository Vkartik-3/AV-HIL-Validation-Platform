"""
SensorForge - Prometheus scrape client (Extension N)
Part of the SensorForge AV HIL validation platform.

Minimal parser for the Prometheus text exposition format produced by the
SensorForge C++ exporter (metrics/prometheus_exporter). No prometheus_client
dependency -- the format is simple enough to parse directly.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

_SAMPLE_RE = re.compile(r'^(?P<name>[a-zA-Z_:][a-zA-Z0-9_:]*)(?:\{(?P<labels>[^}]*)\})?\s+(?P<value>.+)$')
_LABEL_RE = re.compile(r'(\w+)="((?:[^"\\]|\\.)*)"')


@dataclass
class Sample:
    name: str
    labels: dict[str, str]
    value: float


@dataclass
class Scrape:
    samples: list[Sample] = field(default_factory=list)

    def by_name(self, name: str) -> list[Sample]:
        return [s for s in self.samples if s.name == name]

    def sensor_values(self, name: str) -> dict[str, float]:
        """Return {sensor_label: value} for a sensor-labelled metric family."""
        out: dict[str, float] = {}
        for s in self.by_name(name):
            sensor = s.labels.get("sensor")
            if sensor is not None:
                out[sensor] = s.value
        return out

    def scalar(self, name: str, default: float = 0.0) -> float:
        vals = self.by_name(name)
        return vals[0].value if vals else default


def parse_exposition(text: str) -> Scrape:
    scrape = Scrape()
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        m = _SAMPLE_RE.match(line)
        if not m:
            continue
        labels = {}
        if m.group("labels"):
            for lm in _LABEL_RE.finditer(m.group("labels")):
                labels[lm.group(1)] = lm.group(2)
        try:
            value = float(m.group("value").split()[0])
        except ValueError:
            continue
        scrape.samples.append(Sample(m.group("name"), labels, value))
    return scrape


def scrape(url: str, timeout: float = 2.0) -> Scrape:
    """Scrape a running exporter, e.g. http://localhost:9090/metrics."""
    import requests  # imported here so parse_exposition needs no deps
    resp = requests.get(url, timeout=timeout)
    resp.raise_for_status()
    return parse_exposition(resp.text)
