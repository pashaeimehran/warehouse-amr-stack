#!/usr/bin/env bash
# Installs the ROS 2 Jazzy packages of the simulated platform (v0.2 onward),
# shared by Dockerfile.ci and Dockerfile.dev. Runs as root during `docker build`
# on a ROS 2 Jazzy image (Ubuntu 24.04). Versions are pinned by the ROS
# distribution snapshot of the base image's apt sources.
#
# Usage: install_ros_packages.sh [ci|dev]
#   ci   headless set: Gazebo Harmonic server, ros2_control, Nav2, AprilTags,
#        both RMWs, launch_testing (no GUI tools)
#   dev  the ci set plus RViz 2
#
# Gazebo Harmonic comes from the ROS vendor packages (gz_*_vendor) pulled in by
# ros-jazzy-ros-gz, which is the pairing REP 2000 defines for Jazzy.
set -euo pipefail

variant="${1:-ci}"
export DEBIAN_FRONTEND=noninteractive

packages=(
  # Simulation and actuation
  ros-jazzy-ros-gz
  ros-jazzy-gz-ros2-control
  ros-jazzy-ros2-control
  ros-jazzy-ros2-controllers
  # Robot description and TF
  ros-jazzy-xacro
  ros-jazzy-robot-state-publisher
  ros-jazzy-tf2-tools
  # Navigation, mapping, landmarks
  ros-jazzy-navigation2
  ros-jazzy-nav2-bringup
  ros-jazzy-slam-toolbox
  ros-jazzy-apriltag-ros
  # Middleware comparison (qos_lab): Fast DDS ships with ros-base
  ros-jazzy-rmw-cyclonedds-cpp
  # Tests
  ros-jazzy-launch-testing-ament-cmake
  ros-jazzy-ament-cmake-gtest
  python3-pytest
)

case "${variant}" in
  ci) ;;
  dev) packages+=(ros-jazzy-rviz2) ;;
  *)
    echo "unknown variant '${variant}' (expected ci or dev)" >&2
    exit 1
    ;;
esac

apt-get update
apt-get install -y --no-install-recommends "${packages[@]}"
apt-get clean
rm -rf /var/lib/apt/lists/*

# Record versions in the build log.
dpkg-query -W -f='${Package} ${Version}\n' \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-navigation2 ros-jazzy-rmw-cyclonedds-cpp
