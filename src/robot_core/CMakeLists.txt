# robot_core: ROS-independent C++17 domain library.
# Must never depend on rclcpp or any ROS package (see CLAUDE.md).
cmake_minimum_required(VERSION 3.22)
project(robot_core VERSION 0.1.0 LANGUAGES CXX)

if(PROJECT_IS_TOP_LEVEL)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")
  include(ProjectOptions)
  enable_testing()
endif()

find_package(Threads REQUIRED)

add_library(robot_core
  src/action_status.cpp
  src/clock.cpp
  src/fault.cpp
  src/geometry.cpp
  src/manipulation_types.cpp
  src/mission.cpp
  src/mission_executor.cpp
  src/mission_state_machine.cpp
  src/perception_types.cpp
)
add_library(robot_core::robot_core ALIAS robot_core)
target_include_directories(robot_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_core PUBLIC cxx_std_17)
target_link_libraries(robot_core PUBLIC Threads::Threads PRIVATE wmm_warnings)
set_target_properties(robot_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(WMM_BUILD_TESTS)
  add_subdirectory(test)
endif()
