#!/usr/bin/env python3
"""Drives the robot along a fixed route through every aisle (mapping tool).

Steers on the simulator's ground-truth pose (/ground_truth/odom), which is
acceptable for this offline tool only: the route exists to give slam_toolbox
the same coverage and loop closures on every run. Nothing on the robot uses
ground truth for control.

Usage (sim running, workspace sourced):
    python3 tools/mapping/drive_route.py [--speed 0.5]
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Aisles run along x at y = 6, 3, 0, -3, -6; the open zones are x < 4 and x > 16.
ROUTE = [
    (2.0, 0.0), (2.0, 6.0), (18.0, 6.0), (18.0, 3.0), (2.0, 3.0), (2.0, 0.0),
    (18.0, 0.0), (18.0, -3.0), (2.0, -3.0), (2.0, -6.0), (18.0, -6.0), (19.0, -6.0),
    # x = 19 passes west of the drop-off table (x 19.8 to 20.8, y 3.6 to 4.4).
    (19.0, 6.5), (2.0, 6.5), (-1.5, 6.5), (-1.5, -6.5), (2.0, -6.5), (2.0, 0.0), (0.0, 0.0),
]
STUCK_AFTER_S = 30.0   # simulation seconds without 0.2 m of progress


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class RouteDriver(Node):
    def __init__(self, speed: float, tolerance: float) -> None:
        super().__init__("drive_route", parameter_overrides=[
            rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.speed = speed
        self.tolerance = tolerance
        self.index = 0
        self.pose = None
        self.cmd = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_odom, 10)
        self.create_timer(0.05, self.step)
        self.done = False
        self.failed = False
        self.progress_pose = None
        self.progress_time = None

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def step(self) -> None:
        if self.pose is None or self.done:
            return
        x, y, yaw = self.pose
        now = self.get_clock().now()
        if self.progress_pose is None or math.hypot(x - self.progress_pose[0],
                                                    y - self.progress_pose[1]) > 0.2:
            self.progress_pose, self.progress_time = (x, y), now
        elif (now - self.progress_time).nanoseconds / 1e9 > STUCK_AFTER_S:
            self.get_logger().error(f"stuck at ({x:.2f}, {y:.2f}) on the way to waypoint "
                                    f"{self.index} {ROUTE[self.index]}")
            self.done = self.failed = True
            self.publish(0.0, 0.0)
            return
        gx, gy = ROUTE[self.index]
        distance = math.hypot(gx - x, gy - y)
        if distance < self.tolerance:
            self.get_logger().info(f"reached waypoint {self.index}: ({gx:.1f}, {gy:.1f})")
            self.index += 1
            if self.index == len(ROUTE):
                self.done = True
                self.publish(0.0, 0.0)
                return
            gx, gy = ROUTE[self.index]
            distance = math.hypot(gx - x, gy - y)
        error = wrap(math.atan2(gy - y, gx - x) - yaw)
        angular = max(-1.0, min(1.0, 2.0 * error))
        # Turn in place first; slow down near the waypoint.
        linear = 0.0 if abs(error) > 0.35 else min(self.speed, 0.8 * distance) * math.cos(error)
        self.publish(linear, angular)

    def publish(self, linear: float, angular: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd.publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--speed", type=float, default=0.5)
    parser.add_argument("--tolerance", type=float, default=0.25)
    args = parser.parse_args()
    rclpy.init()
    node = RouteDriver(args.speed, args.tolerance)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    failed = node.failed
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
