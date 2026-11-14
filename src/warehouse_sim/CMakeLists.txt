cmake_minimum_required(VERSION 3.22)
project(warehouse_sim NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_world.py DESTINATION lib/${PROJECT_NAME})

# Gazebo resolves model:// URIs through GZ_SIM_RESOURCE_PATH.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_world test/test_world.py TIMEOUT 120)
endif()

ament_package()
