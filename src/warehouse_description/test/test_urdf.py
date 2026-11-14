"""Static checks of the expanded robot model: frame tree, sensors and the
consistency between URDF geometry and the diff_drive_controller parameters."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

SHARE = Path(get_package_share_directory("warehouse_description"))


@pytest.fixture(scope="module")
def robot() -> ET.Element:
    doc = xacro.process_file(str(SHARE / "urdf" / "amr_base.urdf.xacro"))
    return ET.fromstring(doc.toxml())


@pytest.fixture(scope="module")
def controllers() -> dict:
    return yaml.safe_load((SHARE / "config" / "base_controllers.yaml").read_text())


def joints(robot):
    return {j.get("name"): j for j in robot.findall("joint")}


def origin_xyz_rpy(joint):
    origin = joint.find("origin")
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    return xyz, rpy


def test_tree_has_single_root_base_footprint(robot):
    links = {link.get("name") for link in robot.findall("link")}
    children = {j.find("child").get("link") for j in robot.findall("joint")}
    assert links - children == {"base_footprint"}


def test_expected_frames_exist(robot):
    links = {link.get("name") for link in robot.findall("link")}
    expected = {"base_footprint", "base_link", "left_wheel_link", "right_wheel_link",
                "laser_link", "camera_link", "camera_optical_frame", "imu_link"}
    assert expected <= links


def test_base_link_at_wheel_axis_height(robot):
    xyz, _ = origin_xyz_rpy(joints(robot)["base_footprint_joint"])
    assert xyz[2] == pytest.approx(0.08)


def test_optical_frame_follows_rep103(robot):
    # camera_link -> optical: rpy (-pi/2, 0, -pi/2) maps optical z to camera x.
    _, rpy = origin_xyz_rpy(joints(robot)["camera_optical_joint"])
    assert rpy == pytest.approx([-math.pi / 2, 0.0, -math.pi / 2])


def test_wheels_are_continuous_about_y(robot):
    for side in ("left", "right"):
        j = joints(robot)[f"{side}_wheel_joint"]
        assert j.get("type") == "continuous"
        assert j.find("axis").get("xyz") == "0 1 0"


def test_controller_geometry_matches_urdf(robot, controllers):
    params = controllers["diff_drive_controller"]["ros__parameters"]
    left, _ = origin_xyz_rpy(joints(robot)["left_wheel_joint"])
    right, _ = origin_xyz_rpy(joints(robot)["right_wheel_joint"])
    assert left[1] - right[1] == pytest.approx(params["wheel_separation"])
    radius = float(robot.find("link[@name='left_wheel_link']/collision/geometry/cylinder")
                   .get("radius"))
    assert radius == pytest.approx(params["wheel_radius"])
    assert params["base_frame_id"] == "base_footprint"


def test_ros2_control_exposes_wheel_interfaces(robot):
    system = robot.find("ros2_control")
    assert system.find("hardware/plugin").text == "gz_ros2_control/GazeboSimSystem"
    names = {j.get("name") for j in system.findall("joint")}
    assert names == {"left_wheel_joint", "right_wheel_joint"}
    for j in system.findall("joint"):
        assert [c.get("name") for c in j.findall("command_interface")] == ["velocity"]
        assert {s.get("name") for s in j.findall("state_interface")} == {"position", "velocity"}


def test_sensors_present_with_frame_ids(robot):
    sensors = {}
    for gz in robot.findall("gazebo"):
        for s in gz.findall("sensor"):
            sensors[s.get("type")] = (s.find("gz_frame_id").text, s.find("topic").text)
    assert sensors["gpu_lidar"] == ("laser_link", "scan")
    assert sensors["rgbd_camera"] == ("camera_optical_frame", "camera")
    assert sensors["imu"] == ("imu_link", "imu")


def test_no_direct_cmd_vel_plugin(robot):
    # The base is driven only through ros2_control (roadmap: no DiffDrive Gazebo plugin).
    plugins = sorted(p.get("filename") for p in robot.iter("plugin") if p.get("filename"))
    assert plugins == ["gz-sim-odometry-publisher-system", "gz_ros2_control-system"]
