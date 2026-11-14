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
  src/sample_statistics.cpp
)
add_library(robot_core::robot_core ALIAS robot_core)
target_include_directories(robot_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_core PUBLIC cxx_std_17)
# Warning flags are a build-time concern only; keep them out of the export.
target_link_libraries(robot_core PUBLIC Threads::Threads PRIVATE $<BUILD_INTERFACE:wmm_warnings>)
set_target_properties(robot_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Install and export, so ROS 2 adapters consume the core through colcon with
# find_package(robot_core) and robot_core::robot_core (package.xml: build_type cmake).
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(TARGETS robot_core EXPORT robot_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT robot_coreTargets
  NAMESPACE robot_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_core)
configure_package_config_file(cmake/robot_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/robot_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_core)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/robot_coreConfigVersion.cmake
  COMPATIBILITY SameMinorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/robot_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/robot_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_core)
install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/robot_core)

if(WMM_BUILD_TESTS)
  add_subdirectory(test)
endif()
