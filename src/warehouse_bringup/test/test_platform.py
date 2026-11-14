"""Launch test of the simulated platform, headless (`gz sim -s`), roadmap v0.2:
topics publish within 20 s, the TF tree is complete, and /cmd_vel moves the
wheels through ros2_control.

Startup times are measured from the moment the test starts (Gazebo, the
bridge and the controllers start in parallel) and printed, so the actual
values appear in the test log.
"""

import math
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import rclpy.time
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from tf2_ros import Buffer, TransformListener

# Isolation: each launch test has its own ROS domain and Gazebo transport
# partition, so a process left over from another test (for example a Gazebo
# server that ignored SIGINT) can never answer this test's spawn, spawners or
# subscriptions. Set at import time: the test, the launch service and every
# launched process inherit it.
os.environ["ROS_DOMAIN_ID"] = "42"
os.environ["GZ_PARTITION"] = "wmm_test_platform"

STARTUP_BUDGET_S = 20.0   # roadmap: topics within 20 s
TOPICS = {
    "/clock": Clock,
    "/scan": LaserScan,
    "/imu": Imu,
    "/camera/image_raw": Image,
    "/camera/depth/image_raw": Image,
    "/camera/camera_info": CameraInfo,
    "/odom": Odometry,
    "/joint_states": JointState,
    "/ground_truth/odom": Odometry,
}
FRAMES = ("base_link", "laser_link", "camera_link", "camera_optical_frame", "imu_link",
          "left_wheel_link", "right_wheel_link")


@pytest.mark.launch_test
def generate_test_description():
    bringup = get_package_share_directory("warehouse_bringup")
    system = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "system.launch.py")),
        launch_arguments={"headless": "true", "perception": "false"}.items(),
    )
    return launch.LaunchDescription([system, launch_testing.actions.ReadyToTest()])


class TestPlatform(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.t0 = time.monotonic()
        rclpy.init()
        cls.node = rclpy.create_node("test_platform",
                                     parameter_overrides=[rclpy.Parameter(
                                         "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        cls.first_seen = {}
        cls.latest = {}
        cls.subs = []
        for topic, msg_type in TOPICS.items():
            cls.subs.append(cls.node.create_subscription(
                msg_type, topic, cls._make_callback(topic), qos_profile_sensor_data))
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _make_callback(cls, topic):
        def callback(msg):
            cls.first_seen.setdefault(topic, time.monotonic() - cls.t0)
            cls.latest[topic] = msg
        return callback

    def spin_until(self, predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def test_1_topics_publish_within_budget(self):
        self.spin_until(lambda: len(self.first_seen) == len(TOPICS), 90.0)
        report = ", ".join(f"{t}={self.first_seen[t]:.1f}s" for t in sorted(self.first_seen))
        print(f"first message after start: {report}")
        missing = sorted(set(TOPICS) - set(self.first_seen))
        self.assertEqual(missing, [], "topics never published")
        slow = {t: s for t, s in self.first_seen.items() if s > STARTUP_BUDGET_S}
        self.assertEqual(slow, {}, f"topics slower than {STARTUP_BUDGET_S} s")

    def test_2_tf_tree_complete(self):
        def all_frames():
            return all(self.tf_buffer.can_transform("odom", f, rclpy.time.Time())
                       for f in FRAMES)
        self.assertTrue(self.spin_until(all_frames, 30.0),
                        [f for f in FRAMES
                         if not self.tf_buffer.can_transform("odom", f, rclpy.time.Time())])
        # The optical frame is the camera body rotated so that z points forward.
        t = self.tf_buffer.lookup_transform("camera_link", "camera_optical_frame",
                                            rclpy.time.Time()).transform.rotation
        self.assertAlmostEqual(abs(t.w), 0.5, places=3)
        # Images carry the optical frame id.
        self.assertTrue(self.spin_until(lambda: "/camera/image_raw" in self.latest, 30.0))
        self.assertEqual(self.latest["/camera/image_raw"].header.frame_id, "camera_optical_frame")
        self.assertEqual(self.latest["/scan"].header.frame_id, "laser_link")

    def test_3_cmd_vel_moves_the_wheels(self):
        self.assertTrue(self.spin_until(
            lambda: "/joint_states" in self.latest and "/ground_truth/odom" in self.latest, 60.0))
        # Controllers become active after the spawners finish.
        self.spin_until(lambda: False, 5.0)

        def wheel_positions():
            js = self.latest["/joint_states"]
            return {n: p for n, p in zip(js.name, js.position) if n.endswith("wheel_joint")}

        before = wheel_positions()
        start = self.latest["/ground_truth/odom"].pose.pose.position
        publisher = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
        command = TwistStamped()
        command.twist.linear.x = 0.3
        sim_start = self.node.get_clock().now()
        # Drive for 3 s of simulation time.
        while (self.node.get_clock().now() - sim_start).nanoseconds < 3e9:
            command.header.stamp = self.node.get_clock().now().to_msg()
            publisher.publish(command)
            self.spin_until(lambda: False, 0.05)
        publisher.publish(TwistStamped())
        self.spin_until(lambda: False, 2.0)

        after = wheel_positions()
        for name in ("left_wheel_joint", "right_wheel_joint"):
            # 0.3 m/s for about 3 s on 0.08 m wheels is about 11 rad; allow ramp-up.
            self.assertGreater(after[name] - before[name], 5.0, name)
        end = self.latest["/ground_truth/odom"].pose.pose.position
        travelled = math.hypot(end.x - start.x, end.y - start.y)
        print(f"ground truth travel: {travelled:.3f} m")
        self.assertGreater(travelled, 0.5)
        self.assertLess(abs(end.y - start.y), 0.05, "robot should drive straight")
        odom = self.latest["/odom"].pose.pose.position
        self.assertAlmostEqual(odom.x, end.x, delta=0.05)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -6, -15])
