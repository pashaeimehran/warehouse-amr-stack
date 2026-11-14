"""Launch test of v0.2's done-when criteria, headless: AprilTags are detected
(and their TF poses agree with the landmark map), and the robot drives from
the dock to Station A autonomously with Nav2 and AMCL on the saved map.

Ground truth (/ground_truth/odom) is used only to judge the result.
"""

import math
import os
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import rclpy.time
import yaml
from apriltag_msgs.msg import AprilTagDetectionArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

# Isolation: each launch test has its own ROS domain and Gazebo transport
# partition, so a process left over from another test (for example a Gazebo
# server that ignored SIGINT) can never answer this test's spawn, spawners or
# subscriptions. Set at import time: the test, the launch service and every
# launched process inherit it.
os.environ["ROS_DOMAIN_ID"] = "43"
os.environ["GZ_PARTITION"] = "wmm_test_navigation"

SIM_SHARE = Path(get_package_share_directory("warehouse_sim"))
TAGS = {t["id"]: t for t in yaml.safe_load((SIM_SHARE / "config" / "tag_map.yaml")
                                           .read_text())["tags"]}
STATIONS = yaml.safe_load((SIM_SHARE / "config" / "stations.yaml").read_text())["stations"]
# Seen from the dock (0, 0) looking along +x: the west ends of the two middle rows.
TAGS_VISIBLE_FROM_DOCK = {2, 4}
# PnP depth error grows with distance: a 0.2 m tag at 4 m is about 22 px wide,
# so half a pixel of corner error is about 9 cm in range.
TAG_POSITION_TOLERANCE_M = 0.15
GOAL_TOLERANCE_M = 0.35     # xy_goal_tolerance 0.25 plus localization error


@pytest.mark.launch_test
def generate_test_description():
    bringup = get_package_share_directory("warehouse_bringup")
    system = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "system.launch.py")),
        launch_arguments={"headless": "true", "navigation": "true",
                          "perception": "true"}.items(),
    )
    return launch.LaunchDescription([system, launch_testing.actions.ReadyToTest()])


class TestNavigation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_navigation", parameter_overrides=[
            rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        cls.detected = set()
        cls.ground_truth = None
        cls.node.create_subscription(AprilTagDetectionArray, "/detections", cls._on_detections,
                                     qos_profile_sensor_data)
        cls.node.create_subscription(Odometry, "/ground_truth/odom", cls._on_ground_truth,
                                     qos_profile_sensor_data)
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_detections(cls, msg):
        cls.detected.update(d.id for d in msg.detections)

    @classmethod
    def _on_ground_truth(cls, msg):
        cls.ground_truth = msg.pose.pose.position

    def spin_until(self, predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def wait_until_active(self, node_name, timeout_s):
        client = self.node.create_client(GetState, f"/{node_name}/get_state")
        deadline = time.monotonic() + timeout_s
        try:
            while time.monotonic() < deadline:
                if client.wait_for_service(timeout_sec=1.0):
                    future = client.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
                    if (future.done() and future.result() is not None and
                            future.result().current_state.id == State.PRIMARY_STATE_ACTIVE):
                        return True
                self.spin_until(lambda: False, 1.0)
            return False
        finally:
            self.node.destroy_client(client)

    def test_1_tags_detected_at_their_map_poses(self):
        self.assertTrue(self.spin_until(lambda: TAGS_VISIBLE_FROM_DOCK <= self.detected, 120.0),
                        f"detected {sorted(self.detected)}")
        # At start the robot is at the dock and odom coincides with map.
        for tag_id in sorted(TAGS_VISIBLE_FROM_DOCK):
            frame = f"tag_{tag_id}"
            self.assertTrue(self.spin_until(
                lambda f=frame: self.tf_buffer.can_transform("odom", f, rclpy.time.Time()), 30.0))
            p = self.tf_buffer.lookup_transform("odom", frame, rclpy.time.Time()).transform
            expected = TAGS[tag_id]
            error = math.dist((p.translation.x, p.translation.y, p.translation.z),
                              (expected["x"], expected["y"], expected["z"]))
            print(f"{frame}: detected at ({p.translation.x:.3f}, {p.translation.y:.3f}, "
                  f"{p.translation.z:.3f}), map ({expected['x']:.3f}, {expected['y']:.3f}, "
                  f"{expected['z']:.3f}), error {error:.3f} m")
            self.assertLess(error, TAG_POSITION_TOLERANCE_M, frame)

    def test_2_dock_to_station_a(self):
        # Fail fast instead of blocking in waitUntilNav2Active(): a hung launch
        # test is killed by CTest, which skips launch's cleanup and orphans gz.
        self.assertTrue(self.spin_until(
            lambda: self.tf_buffer.can_transform("map", "base_footprint", rclpy.time.Time()),
            120.0), "AMCL never published map -> odom")
        navigator = BasicNavigator(node_name="test_navigator")
        navigator.set_parameters([rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL,
                                                  True)])
        self.assertTrue(navigator.nav_to_pose_client.wait_for_server(timeout_sec=120.0),
                        "navigate_to_pose action server not available")
        # The action server exists once bt_navigator is configured, but goals are
        # accepted only when it is active (the lifecycle manager activates the
        # servers in order, after the costmaps have received TF).
        self.assertTrue(self.wait_until_active("bt_navigator", 180.0),
                        "bt_navigator did not become active")
        goal = STATIONS["station_a"]
        target = PoseStamped()
        target.header.frame_id = "map"
        target.header.stamp = navigator.get_clock().now().to_msg()
        target.pose.position.x = goal["x"]
        target.pose.position.y = goal["y"]
        target.pose.orientation.z = math.sin(goal["yaw"] / 2)
        target.pose.orientation.w = math.cos(goal["yaw"] / 2)

        start = navigator.get_clock().now()
        self.assertTrue(navigator.goToPose(target))
        wall_deadline = time.monotonic() + 420.0
        while not navigator.isTaskComplete():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if time.monotonic() > wall_deadline:
                navigator.cancelTask()
        result = navigator.getResult()
        sim_s = (navigator.get_clock().now() - start).nanoseconds / 1e9
        self.spin_until(lambda: False, 1.0)
        gt = self.ground_truth
        error = math.hypot(gt.x - goal["x"], gt.y - goal["y"])
        print(f"dock -> station_a: {result.name} in {sim_s:.1f} s sim time; "
              f"ground-truth goal error {error:.3f} m")
        navigator.destroy_node()
        self.assertEqual(result, TaskResult.SUCCEEDED)
        self.assertLess(error, GOAL_TOLERANCE_M)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        for name in proc_info.process_names():
            allowed = [0, -2, -15]
            if name.startswith("apriltag_node"):
                # apriltag_ros 3.4.0 segfaults during teardown after SIGINT (it ran
                # and published correctly until then); only this process gets -11.
                allowed.append(-11)
            launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=allowed,
                                                   process=name)
