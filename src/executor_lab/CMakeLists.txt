cmake_minimum_required(VERSION 3.22)
project(executor_lab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_core REQUIRED)

# ROS-free analysis, unit tested without a ROS graph.
add_library(executor_lab_analysis src/period_analysis.cpp)
target_include_directories(executor_lab_analysis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(executor_lab_analysis PUBLIC robot_core::robot_core)

add_executable(executor_probe src/executor_probe.cpp)
target_link_libraries(executor_probe executor_lab_analysis rclcpp::rclcpp)

install(TARGETS executor_probe DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/run_configs.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_period_analysis test/test_period_analysis.cpp)
  target_link_libraries(test_period_analysis executor_lab_analysis)
endif()

ament_package()
