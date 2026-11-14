#!/usr/bin/env python3
"""Aggregates repeated qos_lab and executor_lab runs into Markdown tables.

Each cell is reported as the median over runs with the min to max range in
brackets, so run-to-run variation stays visible (roadmap: run each benchmark
at least three times and report the variation).

Usage:
    python3 tools/analysis/summarize_labs.py benchmarks/results/qos_lab_2026-11-14_r*.json
    python3 tools/analysis/summarize_labs.py benchmarks/results/executor_lab_2026-11-14_r*.json
"""

from __future__ import annotations

import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path


def cell(values: list[float], fmt: str = "{:.2f}") -> str:
    if not values:
        return "n/a"
    med = statistics.median(values)
    if len(values) == 1 or min(values) == max(values):
        return fmt.format(med)
    return f"{fmt.format(med)} [{fmt.format(min(values))}, {fmt.format(max(values))}]"


def qos(reports: list[dict]) -> str:
    groups = defaultdict(list)
    for r in reports:
        for c in r["cells"]:
            key = (c["rmw"], c["publisher_profile"], c["subscriber_profile"],
                   c["requested_payload_size"])
            groups[key].append(c)
    lines = [f"Runs: {len(reports)}; {reports[0]['count']} messages per cell at "
             f"{reports[0]['rate_hz']:.0f} Hz; median [min, max] over runs.", "",
             "| RMW | Profile | Payload | Loss % | p50 ms | p95 ms | p99 ms |",
             "| --- | --- | --- | --- | --- | --- | --- |"]
    for (rmw, pub, sub, size), cells in groups.items():
        profile = pub if pub == sub else f"pub {pub} / sub {sub}"
        size_txt = f"{size // 1024} KiB" if size >= 1024 else f"{size} B"
        lat = [c["latency_ms"] for c in cells if "latency_ms" in c]
        lines.append(
            f"| {rmw.replace('rmw_', '').replace('_cpp', '')} | {profile} | {size_txt} | "
            f"{cell([100 * c['loss_ratio'] for c in cells], '{:.1f}')} | "
            f"{cell([x['p50'] for x in lat])} | {cell([x['p95'] for x in lat])} | "
            f"{cell([x['p99'] for x in lat])} |")
    return "\n".join(lines) + "\n"


def executor(reports: list[dict]) -> str:
    groups = defaultdict(list)
    for r in reports:
        for c in r["cells"]:
            groups[c["config"]].append(c)
    first = reports[0]["cells"][0]
    lines = [f"Runs: {len(reports)}; {first['duration_s']:.0f} s each; 50 Hz timer next to "
             f"{first['slow_work_ms']} ms of CPU work every {first['slow_period_ms']} ms; "
             "median [min, max] over runs.", "",
             "| Configuration | 50 Hz calls (of 500) | Missed | Period p50 ms | p95 ms | max ms |",
             "| --- | --- | --- | --- | --- | --- |"]
    for config, cells in groups.items():
        lines.append(
            f"| {config} | {cell([c['fast_invocations'] for c in cells], '{:.0f}')} | "
            f"{cell([c['missed_activations'] for c in cells], '{:.0f}')} | "
            f"{cell([c['period_ms']['p50'] for c in cells])} | "
            f"{cell([c['period_ms']['p95'] for c in cells])} | "
            f"{cell([c['period_ms']['max'] for c in cells])} |")
    return "\n".join(lines) + "\n"


def main() -> int:
    reports = [json.loads(Path(p).read_text()) for p in sys.argv[1:]]
    if not reports:
        print(__doc__)
        return 2
    kind = reports[0]["experiment"]
    print(qos(reports) if kind == "qos_lab" else executor(reports))
    return 0


if __name__ == "__main__":
    sys.exit(main())
