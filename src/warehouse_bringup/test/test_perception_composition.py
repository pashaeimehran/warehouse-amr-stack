"""Launch test: both perception components share one container and follow
manual lifecycle transitions (roadmap v0.2: `ros2 lifecycle set` configure and
activate work; two components share one container)."""

import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from composition_interfaces.srv import ListNodes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState

# Isolation: each launch test has its own ROS domain and Gazebo transport
# partition, so a process left over from another test (for example a Gazebo
# server that ignored SIGINT) can never answer this test's spawn, spawners or
# subscriptions. Set at import time: the test, the launch service and every
# launched process inherit it.
os.environ["ROS_DOMAIN_ID"] = "41"
os.environ["GZ_PARTITION"] = "wmm_test_perception_composition"

NODES = ("perception_3d", "tracking")


@pytest.mark.launch_test
def generate_test_description():
    bringup = get_package_share_directory("warehouse_bringup")
    perception = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "perception.launch.py")),
        launch_arguments={"autostart": "false"}.items(),
    )
    return launch.LaunchDescription([perception, launch_testing.actions.ReadyToTest()])


class TestPerceptionComposition(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_perception_composition")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, srv_type, name, request, timeout=20.0):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=timeout), f"{name} not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), f"{name} did not answer")
        self.node.destroy_client(client)
        return future.result()

    def state_of(self, node_name):
        return self.call(GetState, f"/{node_name}/get_state", GetState.Request()).current_state.id

    def transition(self, node_name, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        self.assertTrue(self.call(ChangeState, f"/{node_name}/change_state", request).success)

    def test_1_both_components_in_one_container(self):
        # The container's service comes up before launch_ros has loaded the
        # components, so poll until both are listed or the deadline passes.
        deadline = time.monotonic() + 30.0
        names = []
        while time.monotonic() < deadline:
            response = self.call(ListNodes, "/perception_container/_container/list_nodes",
                                 ListNodes.Request())
            names = sorted(name.lstrip("/") for name in response.full_node_names)
            if names == sorted(NODES):
                break
            time.sleep(0.2)
        self.assertEqual(names, sorted(NODES))

    def test_2_manual_lifecycle_transitions(self):
        for name in NODES:
            self.assertEqual(self.state_of(name), State.PRIMARY_STATE_UNCONFIGURED)
            self.transition(name, Transition.TRANSITION_CONFIGURE)
            self.assertEqual(self.state_of(name), State.PRIMARY_STATE_INACTIVE)
            self.transition(name, Transition.TRANSITION_ACTIVATE)
            self.assertEqual(self.state_of(name), State.PRIMARY_STATE_ACTIVE)
            self.transition(name, Transition.TRANSITION_DEACTIVATE)
            self.assertEqual(self.state_of(name), State.PRIMARY_STATE_INACTIVE)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # The container may be stopped by SIGINT at shutdown (-2).
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
