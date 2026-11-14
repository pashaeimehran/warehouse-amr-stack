cmake_minimum_required(VERSION 3.22)
project(warehouse_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py TIMEOUT 60)
endif()

ament_package()
