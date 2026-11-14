"""Mapping: sim and robot layers plus slam_toolbox (online async).

    ros2 launch warehouse_bringup mapping.launch.py
    python3 tools/mapping/drive_route.py          # drives the mapping route
    ros2 run nav2_map_server map_saver_cli -f src/warehouse_bringup/maps/warehouse
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("warehouse_bringup")
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "system.launch.py")),
        # The static world: no walking person, so no traces in the map.
        launch_arguments={"headless": LaunchConfiguration("headless"),
                          "perception": "false", "world": "warehouse_static.sdf"}.items(),
    )
    # slam_toolbox is a lifecycle node in Jazzy: it does nothing until it is
    # configured and activated, here by the same lifecycle manager the other
    # layers use (use_lifecycle_manager enables its bond with the manager).
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[os.path.join(share, "config", "slam_toolbox.yaml"),
                    {"use_lifecycle_manager": True}],
        output="screen",
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapping",
        parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["slam_toolbox"]}],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        system,
        slam,
        lifecycle_manager,
    ])
