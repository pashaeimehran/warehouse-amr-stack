#!/usr/bin/env python3
"""Runs the qos_lab matrix: 3 QoS profiles x 2 payload sizes x 2 RMWs.

Each cell starts qos_subscriber and qos_publisher as separate processes (so
messages really cross the RMW and the loopback interface), waits for the
subscriber's JSON result and collects all cells into one JSON file and a
Markdown table. A final cell demonstrates silent QoS incompatibility: a
best-effort publisher and a reliable subscriber never match.

Usage (inside the CI or dev image, workspace sourced):
    ros2 run qos_lab run_matrix.py --output-dir benchmarks/results --count 1000
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
import time
from pathlib import Path

RMWS = ("rmw_fastrtps_cpp", "rmw_cyclonedds_cpp")
PROFILES = ("reliable_volatile", "best_effort_volatile", "reliable_transient_local")
PAYLOADS = (256, 1024 * 1024)


def run_cell(
    rmw: str,
    pub_profile: str,
    sub_profile: str,
    payload: int,
    count: int,
    rate_hz: float,
    domain_id: int,
) -> dict:
    env = dict(os.environ, RMW_IMPLEMENTATION=rmw, ROS_DOMAIN_ID=str(domain_id))
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "result.json"
        sub = subprocess.Popen(
            [
                "ros2", "run", "qos_lab", "qos_subscriber", "--ros-args",
                "-p", f"profile:={sub_profile}",
                "-p", f"count:={count}",
                "-p", f"timeout_s:={count / rate_hz + 20.0}",
                "-p", "idle_timeout_s:=2.0",
                "-p", f"output:={out}",
            ],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
        )
        time.sleep(1.0)  # let the subscriber's reader exist before the writer
        pub = subprocess.Popen(
            [
                "ros2", "run", "qos_lab", "qos_publisher", "--ros-args",
                "-p", f"profile:={pub_profile}",
                "-p", f"count:={count}",
                "-p", f"payload_size:={payload}",
                "-p", f"rate_hz:={rate_hz}",
                "-p", "match_timeout_s:=5.0",
            ],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
        )
        timeout = count / rate_hz + 40.0
        pub.wait(timeout=timeout)
        sub.wait(timeout=timeout)
        if not out.exists():
            raise RuntimeError(f"no result for {rmw} {pub_profile} {payload}: {sub.stderr.read()}")
        result = json.loads(out.read_text())
    result["publisher_profile"] = pub_profile
    result["subscriber_profile"] = sub_profile
    result["requested_payload_size"] = payload
    result["rate_hz"] = rate_hz
    return result


def markdown_table(cells: list[dict]) -> str:
    lines = [
        "| RMW | Profile | Payload | Received | Loss | p50 ms | p95 ms | p99 ms | max ms |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for c in cells:
        lat = c.get("latency_ms", {})

        def fmt(key: str, lat: dict = lat) -> str:
            return f"{lat[key]:.2f}" if key in lat else "n/a"

        size = c["requested_payload_size"]
        size_txt = f"{size // 1024} KiB" if size >= 1024 else f"{size} B"
        profile = c["publisher_profile"]
        if c["publisher_profile"] != c["subscriber_profile"]:
            profile = f"pub {c['publisher_profile']} / sub {c['subscriber_profile']}"
        lines.append(
            f"| {c['rmw']} | {profile} | {size_txt} | {c['received']}/{c['expected']} | "
            f"{100.0 * c['loss_ratio']:.1f} % | {fmt('p50')} | {fmt('p95')} | {fmt('p99')} | "
            f"{fmt('max')} |"
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--domain-id", type=int, default=42)
    parser.add_argument("--label", default="", help="suffix of the output files, e.g. _r2")
    args = parser.parse_args()

    cells = []
    for rmw in RMWS:
        for profile in PROFILES:
            for payload in PAYLOADS:
                print(f"{rmw} {profile} {payload} B ...", flush=True)
                cells.append(run_cell(rmw, profile, profile, payload, args.count, args.rate_hz,
                                      args.domain_id))
        print(f"{rmw} incompatible QoS ...", flush=True)
        cells.append(run_cell(rmw, "best_effort_volatile", "reliable_volatile", 256, 100,
                              args.rate_hz, args.domain_id))

    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d")
    report = {
        "experiment": "qos_lab",
        "date_utc": stamp,
        "host": {"machine": platform.machine(), "kernel": platform.release(),
                 "cpu_count": os.cpu_count()},
        "run_label": args.label,
        "count": args.count,
        "rate_hz": args.rate_hz,
        "cells": cells,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / f"qos_lab_{stamp}{args.label}.json").write_text(json.dumps(report, indent=2) + "\n")
    table = markdown_table(cells)
    (args.output_dir / f"qos_lab_{stamp}{args.label}.md").write_text(table)
    print(table)
    return 0


if __name__ == "__main__":
    sys.exit(main())
