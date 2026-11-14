#!/usr/bin/env python3
"""Drives to named stations with the Nav2 simple commander.

Station poses come from warehouse_sim/config/stations.yaml (generated with the
world, map frame). Prints one JSON line per leg with the outcome and the
navigation time in simulation seconds, and exits non-zero if any leg fails.

Usage (system running with navigation:=true):
    ros2 run warehouse_bringup navigate_stations.py station_a dock
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def load_stations() -> dict:
    path = Path(get_package_share_directory("warehouse_sim")) / "config" / "stations.yaml"
    return yaml.safe_load(path.read_text())["stations"]


def pose(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("stations", nargs="+", help="station names in visiting order")
    parser.add_argument("--timeout", type=float, default=300.0, help="per leg, sim seconds")
    args = parser.parse_args()

    stations = load_stations()
    unknown = [s for s in args.stations if s not in stations]
    if unknown:
        print(f"unknown stations {unknown}; known: {sorted(stations)}", file=sys.stderr)
        return 2

    rclpy.init()
    navigator = BasicNavigator(namespace="")
    navigator.set_parameters([rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    # AMCL starts at the dock (set_initial_pose in nav2.yaml); wait for Nav2.
    navigator.waitUntilNav2Active(localizer="amcl")

    failures = 0
    for name in args.stations:
        s = stations[name]
        start = navigator.get_clock().now()
        navigator.goToPose(pose(navigator, s["x"], s["y"], s["yaw"]))
        while not navigator.isTaskComplete():
            if (navigator.get_clock().now() - start).nanoseconds / 1e9 > args.timeout:
                navigator.cancelTask()
            rclpy.spin_once(navigator, timeout_sec=0.1)
        result = navigator.getResult()
        elapsed = (navigator.get_clock().now() - start).nanoseconds / 1e9
        print(json.dumps({"station": name, "result": result.name,
                          "sim_time_s": round(elapsed, 2)}), flush=True)
        failures += result != TaskResult.SUCCEEDED

    navigator.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
