cmake_minimum_required(VERSION 3.22)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ObjectClass.msg
  msg/DetectedObject3D.msg
  msg/DetectedObject3DArray.msg
  msg/TrackedObject.msg
  msg/TrackedObjectArray.msg
  msg/HealthStatus.msg
  msg/Waypoint.msg
  msg/MissionLeg.msg
  msg/MissionCommand.msg
  msg/SafetyState.msg
  msg/LocalizationStatus.msg
  msg/ItemPose.msg
  msg/PickCommand.msg
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  # Message constants that mirror robot_core enums are checked at compile time,
  # so a reordered enum breaks the build instead of silently mislabelling data.
  find_package(ament_cmake_gtest REQUIRED)
  find_package(robot_core REQUIRED)
  ament_add_gtest(test_interfaces test/test_interfaces.cpp)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(test_interfaces ${cpp_typesupport_target} robot_core::robot_core)
endif()

ament_package()
