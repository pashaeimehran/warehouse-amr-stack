# Top-level build for the ROS-independent C++ domain libraries.
#
# ROS 2 packages (added from v0.2 on) are built with colcon; this file only
# aggregates the plain-CMake domain libraries so they can be built, tested and
# analysed without a ROS installation.
cmake_minimum_required(VERSION 3.22)
project(warehouse_amr_stack VERSION 0.1.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(ProjectOptions)

enable_testing()

add_subdirectory(src/robot_core)

if(WMM_BUILD_TESTS AND NOT WMM_SANITIZER STREQUAL "")
  add_subdirectory(test/sanitizers)
endif()
