#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <rclcpp/qos.hpp>

namespace qos_lab {

/// The three QoS profiles of the experiment (roadmap: 3 profiles x 2 payload
/// sizes x 2 RMWs). Each stands for a real use in the stack:
///   reliable_volatile       ROS default (depth 10): commands, events
///   best_effort_volatile    sensor data (SensorDataQoS, depth 5): camera, LiDAR
///   reliable_transient_local  latched state (depth 1): /map, /robot_description
[[nodiscard]] std::optional<rclcpp::QoS> profileByName(std::string_view name);

/// Names accepted by profileByName, in table order.
[[nodiscard]] const std::vector<std::string>& profileNames();

}  // namespace qos_lab
