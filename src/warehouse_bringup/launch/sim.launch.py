"""Simulation layer: Gazebo Harmonic with the warehouse world and the ROS bridge.

headless:=true runs only the server (`gz sim -s`) with headless EGL rendering,
which camera and LiDAR sensors need; this is how tests and CI run it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_share = get_package_share_directory("warehouse_sim")
    bringup_share = get_package_share_directory("warehouse_bringup")
    gz_launch = os.path.join(get_package_share_directory("ros_gz_sim"), "launch",
                             "gz_sim.launch.py")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    def gz(args, condition):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": args, "on_exit_shutdown": "true"}.items(),
            condition=condition,
        )

    headless_gz = gz(["-r -s --headless-rendering ", world], IfCondition(headless))
    gui_gz = gz(["-r ", world], UnlessCondition(headless))

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": os.path.join(bringup_share, "config", "bridge.yaml"),
                     "use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value=PathJoinSubstitution([sim_share, "worlds", "warehouse.sdf"])),
        DeclareLaunchArgument("headless", default_value="true"),
        # model:// URIs of the world (AprilTag models). Set here rather than
        # relying on the package's environment hook, so the launch works the
        # same from a sourced shell, colcon test and launch_testing.
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(sim_share, "models")),
        # Third-party assets baked into the images (docker/fetch_sim_assets.sh).
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", "/opt/wmm_assets/models"),
        # Software rendering when no GPU is present (CI, containers).
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE",
                               os.environ.get("LIBGL_ALWAYS_SOFTWARE", "1")),
        headless_gz,
        gui_gz,
        bridge,
    ])
