#!/usr/bin/env bash
# Quality gate for the ROS 2 workspace (v0.2 onward). Like check_core.sh, it is
# the single implementation: ci.yml and the Compose services only call steps.
# Needs ROS 2 Jazzy and the packages of docker/install_ros_packages.sh, so it
# normally runs inside the CI image:
#
#   docker compose --profile ci run --rm ci tools/ci/check_ros.sh build test
#
# Usage: tools/ci/check_ros.sh [STEP...]
#
# Steps (default: build test):
#   build     colcon build of every package under src/ (Release, warnings as errors
#             for this project's packages)
#   test      colcon test (gtest, pytest, launch_testing) and a result summary;
#             fails on any failed test
#
# Environment:
#   JOBS              parallel compile jobs per package (default: 4; Gazebo and
#                     Nav2 headers need about 1 GB per job)
#   COLCON_WORKERS    packages built in parallel (default: 2)
#   PACKAGES          optional space-separated package list (--packages-up-to)
set -euo pipefail

cd "$(dirname "$0")/../.."
JOBS="${JOBS:-4}"
COLCON_WORKERS="${COLCON_WORKERS:-2}"

# colcon trees live under build/, which the Compose services overlay per
# service, so host and container trees never mix (see docker-compose.yml).
BUILD_BASE=build/colcon
INSTALL_BASE=build/install
export COLCON_LOG_PATH=build/log
export COLCON_HOME=build/colcon_home
export CMAKE_BUILD_PARALLEL_LEVEL="${JOBS}"
export MAKEFLAGS="-j${JOBS}"
# Stack middleware (docs/02); the images set it too.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# Keep test runs from talking to other ROS graphs on the same machine.
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

step() { printf '\n=== %s ===\n' "$*"; }
die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

[[ -f /opt/ros/jazzy/setup.bash ]] ||
  die "ROS 2 Jazzy not found; run inside the CI image (docker compose --profile ci run --rm ci ...)"
# setup.bash reads unset variables.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

selection=()
if [[ -n "${PACKAGES:-}" ]]; then
  read -r -a pkgs <<<"${PACKAGES}"
  selection=(--packages-up-to "${pkgs[@]}")
fi

common=(--base-paths src --build-base "${BUILD_BASE}" --install-base "${INSTALL_BASE}")

run_build() {
  step "colcon build"
  # robot_core's own unit tests run in check_core.sh under every preset; here
  # it is built only as a dependency.
  colcon build "${common[@]}" "${selection[@]}" \
    --parallel-workers "${COLCON_WORKERS}" \
    --event-handlers console_cohesion+ summary+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DWMM_BUILD_TESTS=OFF \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --no-warn-unused-cli
}

run_test() {
  step "colcon test"
  colcon test "${common[@]}" "${selection[@]}" \
    --parallel-workers 1 \
    --event-handlers console_cohesion+ summary+ \
    --return-code-on-test-failure \
    --ctest-args --output-on-failure \
    --pytest-args -p no:cacheprovider
  colcon test-result --test-result-base "${BUILD_BASE}" --all --verbose
}

steps=("$@")
if [[ ${#steps[@]} -eq 0 ]]; then
  steps=(build test)
fi
for s in "${steps[@]}"; do
  case "${s}" in
    build) run_build ;;
    test) run_test ;;
    -h | --help)
      sed -n '2,24p' "$0"
      exit 0
      ;;
    *) die "unknown step '${s}' (see --help)" ;;
  esac
done

step "passed: ${steps[*]}"
