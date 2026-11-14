"""Perception layer: perception_3d and tracking composed into one container.

Both are lifecycle components; nav2_lifecycle_manager configures and activates
them in order when autostart is true. With autostart:=false they stay
unconfigured, and `ros2 lifecycle set /perception_3d configure` (then
`activate`) drives them by hand.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time = ParameterValue(use_sim_time, value_type=bool)
    intra_process = [{"use_intra_process_comms": True}]

    container = ComposableNodeContainer(
        name="perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="perception_3d",
                plugin="perception_3d::Perception3DNode",
                name="perception_3d",
                parameters=[{"use_sim_time": sim_time}],
                extra_arguments=intra_process,
            ),
            ComposableNode(
                package="tracking",
                plugin="tracking::TrackingNode",
                name="tracking",
                parameters=[{"use_sim_time": sim_time}],
                extra_arguments=intra_process,
            ),
        ],
        output="screen",
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_perception",
        output="screen",
        parameters=[{
            "use_sim_time": sim_time,
            "autostart": ParameterValue(autostart, value_type=bool),
            "node_names": ["perception_3d", "tracking"],
            # These components do not implement Nav2's bond heartbeat.
            "bond_timeout": 0.0,
        }],
    )

    # AprilTag landmarks. Gazebo images have no distortion, so image_raw
    # serves as the rectified image.
    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        parameters=[os.path.join(get_package_share_directory("warehouse_bringup"), "config",
                                 "apriltag.yaml"),
                    {"use_sim_time": sim_time}],
        remappings=[("image_rect", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info")],
        condition=IfCondition(LaunchConfiguration("apriltag")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("apriltag", default_value="false"),
        container,
        lifecycle_manager,
        apriltag,
    ])
