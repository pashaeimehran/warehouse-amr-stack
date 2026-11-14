cmake_minimum_required(VERSION 3.22)
project(qos_lab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_core REQUIRED)

# ROS-free probe logic plus the profile table (which needs only rclcpp::QoS).
add_library(qos_lab_lib src/probe.cpp src/profiles.cpp)
target_include_directories(qos_lab_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(qos_lab_lib PUBLIC rclcpp::rclcpp)

add_executable(qos_publisher src/qos_publisher.cpp)
target_link_libraries(qos_publisher qos_lab_lib ${std_msgs_TARGETS})
add_executable(qos_subscriber src/qos_subscriber.cpp)
target_link_libraries(qos_subscriber qos_lab_lib robot_core::robot_core ${std_msgs_TARGETS})

install(TARGETS qos_publisher qos_subscriber DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/run_matrix.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_probe test/test_probe.cpp)
  target_link_libraries(test_probe qos_lab_lib)
endif()

ament_package()
