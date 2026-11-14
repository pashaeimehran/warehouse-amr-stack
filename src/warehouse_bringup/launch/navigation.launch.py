"""Navigation layer: map_server, AMCL and the Nav2 servers used by this robot.

The project's own launch instead of nav2_bringup's navigation_launch.py, which
in Jazzy also starts route_server and docking_server (unused here).

Command chain (TwistStamped):
  controller_server, behavior_server -> /cmd_vel_controller
  velocity_smoother: /cmd_vel_controller -> /cmd_vel
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

LIFECYCLE_NODES = [
    "map_server",
    "amcl",
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "velocity_smoother",
    "bt_navigator",
    "waypoint_follower",
]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("warehouse_bringup")
    params = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    def nav_node(package, executable, name=None, remappings=(), extra_params=()):
        return Node(
            package=package,
            executable=executable,
            name=name or executable,
            parameters=[params, *extra_params],
            remappings=tf_remaps + list(remappings),
            output="screen",
        )

    nodes = GroupAction([
        SetParameter("use_sim_time", True),
        nav_node("nav2_map_server", "map_server", extra_params=[{"yaml_filename": map_yaml}]),
        nav_node("nav2_amcl", "amcl"),
        nav_node("nav2_controller", "controller_server",
                 remappings=[("cmd_vel", "cmd_vel_controller")]),
        nav_node("nav2_smoother", "smoother_server"),
        nav_node("nav2_planner", "planner_server"),
        nav_node("nav2_behaviors", "behavior_server",
                 remappings=[("cmd_vel", "cmd_vel_controller")]),
        nav_node("nav2_velocity_smoother", "velocity_smoother",
                 remappings=[("cmd_vel", "cmd_vel_controller"),
                             ("cmd_vel_smoothed", "cmd_vel")]),
        nav_node("nav2_bt_navigator", "bt_navigator"),
        nav_node("nav2_waypoint_follower", "waypoint_follower"),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[{"autostart": True, "node_names": LIFECYCLE_NODES}],
            output="screen",
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(share, "config", "nav2.yaml")),
        DeclareLaunchArgument("map",
                              default_value=os.path.join(share, "maps", "warehouse.yaml")),
        nodes,
    ])
