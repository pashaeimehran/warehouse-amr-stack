cmake_minimum_required(VERSION 3.22)
project(warehouse_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/navigate_stations.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_perception_composition.py TIMEOUT 120)
  # Headless Gazebo with CPU rendering; startup dominates the runtime.
  add_launch_test(test/test_platform.py TIMEOUT 300)
  # Full system with Nav2 and AprilTags on the saved map; drives dock to Station A.
  add_launch_test(test/test_navigation.py TIMEOUT 600)
endif()

ament_package()
