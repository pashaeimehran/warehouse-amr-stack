#!/usr/bin/env python3
"""Runs executor_probe under its four configurations and tabulates the result.

Usage (workspace sourced):
    ros2 run executor_lab run_configs.py --output-dir benchmarks/results --duration 10
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import platform
import subprocess
import sys
import tempfile
from pathlib import Path

CONFIGS = ("single_default", "single_separate", "multi_default", "multi_separate")


def run(config: str, duration: float, slow_period_ms: int, slow_work_ms: int) -> dict:
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "result.json"
        subprocess.run(
            [
                "ros2", "run", "executor_lab", "executor_probe", "--ros-args",
                "-p", f"config:={config}",
                "-p", f"duration_s:={duration}",
                "-p", f"slow_period_ms:={slow_period_ms}",
                "-p", f"slow_work_ms:={slow_work_ms}",
                "-p", f"output:={out}",
            ],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=duration + 30.0,
        )
        return json.loads(out.read_text())


def table(cells: list[dict]) -> str:
    lines = [
        "| Configuration | 50 Hz calls (expected) | Missed | Period p50 ms | p95 ms | p99 ms | max ms |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for c in cells:
        p = c["period_ms"]
        lines.append(
            f"| {c['config']} | {c['fast_invocations']} ({c['expected_fast_invocations']}) | "
            f"{c['missed_activations']} | {p['p50']:.2f} | {p['p95']:.2f} | {p['p99']:.2f} | "
            f"{p['max']:.2f} |"
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--slow-period-ms", type=int, default=500)
    parser.add_argument("--slow-work-ms", type=int, default=200)
    parser.add_argument("--label", default="")
    args = parser.parse_args()

    cells = []
    for config in CONFIGS:
        print(f"{config} ...", flush=True)
        cells.append(run(config, args.duration, args.slow_period_ms, args.slow_work_ms))
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d")
    report = {
        "experiment": "executor_lab",
        "date_utc": stamp,
        "run_label": args.label,
        "rmw": os.environ.get("RMW_IMPLEMENTATION", "default (rmw_fastrtps_cpp)"),
        "host": {"machine": platform.machine(), "kernel": platform.release(),
                 "cpu_count": os.cpu_count()},
        "cells": cells,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / f"executor_lab_{stamp}{args.label}.json").write_text(
        json.dumps(report, indent=2) + "\n")
    text = table(cells)
    (args.output_dir / f"executor_lab_{stamp}{args.label}.md").write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
