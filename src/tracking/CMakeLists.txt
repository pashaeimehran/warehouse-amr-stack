cmake_minimum_required(VERSION 3.22)
project(tracking LANGUAGES CXX)

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
add_library(tracking_component SHARED src/tracking_node.cpp)
target_link_libraries(tracking_component
  rclcpp::rclcpp rclcpp_components::component rclcpp_lifecycle::rclcpp_lifecycle
  ${lifecycle_msgs_TARGETS} ${robot_interfaces_TARGETS})
rclcpp_components_register_node(tracking_component
  PLUGIN "tracking::TrackingNode"
  EXECUTABLE tracking_node)

install(TARGETS tracking_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
