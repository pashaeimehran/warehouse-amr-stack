"""Robot layer: description, spawn into Gazebo, ros2_control controllers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory("warehouse_description")
    xacro_file = os.path.join(description_share, "urdf", "amr_base.urdf.xacro")
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "warehouse_amr", "-topic", "robot_description",
                   "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"),
                   "-z", "0.01", "-Y", LaunchConfiguration("yaw")],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
            output="screen",
        )

    joint_state_broadcaster = spawner("joint_state_broadcaster")
    diff_drive = spawner("diff_drive_controller")

    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        state_publisher,
        spawn,
        # Controllers load once the robot (and with it gz_ros2_control) exists.
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster,
                                           on_exit=[diff_drive])),
    ])
