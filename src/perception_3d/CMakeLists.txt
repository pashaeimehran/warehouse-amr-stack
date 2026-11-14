cmake_minimum_required(VERSION 3.22)
project(perception_3d LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

# Component library: loaded into a component container (composition) or run
# standalone through the generated executable.
add_library(perception_3d_component SHARED src/perception_3d_node.cpp)
target_link_libraries(perception_3d_component
  rclcpp::rclcpp rclcpp_components::component rclcpp_lifecycle::rclcpp_lifecycle
  ${lifecycle_msgs_TARGETS} ${robot_interfaces_TARGETS})
rclcpp_components_register_node(perception_3d_component
  PLUGIN "perception_3d::Perception3DNode"
  EXECUTABLE perception_3d_node)

install(TARGETS perception_3d_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
