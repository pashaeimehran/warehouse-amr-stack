"""Full system: sim, robot, navigation and perception layers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    launch_dir = os.path.join(get_package_share_directory("warehouse_bringup"), "launch")
    sim_worlds = os.path.join(get_package_share_directory("warehouse_sim"), "worlds")

    def layer(name, condition=None, **args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, f"{name}.launch.py")),
            launch_arguments=args.items(),
            condition=condition,
        )

    perception = layer("perception", IfCondition(LaunchConfiguration("perception")),
                       use_sim_time="true", apriltag="true")
    navigation = layer("navigation", IfCondition(LaunchConfiguration("navigation")))

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("world", default_value="warehouse.sdf",
                              description="world file in warehouse_sim/worlds"),
        DeclareLaunchArgument("perception", default_value="true"),
        DeclareLaunchArgument("navigation", default_value="false"),
        layer("sim", headless=LaunchConfiguration("headless"),
              world=PathJoinSubstitution([sim_worlds, LaunchConfiguration("world")])),
        layer("robot"),
        navigation,
        perception,
    ])
