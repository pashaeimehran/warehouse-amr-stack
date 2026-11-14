#include "qos_lab/profiles.hpp"

namespace qos_lab {

const std::vector<std::string>& profileNames() {
  static const std::vector<std::string> kNames{"reliable_volatile", "best_effort_volatile",
                                               "reliable_transient_local"};
  return kNames;
}

std::optional<rclcpp::QoS> profileByName(std::string_view name) {
  if (name == "reliable_volatile") {
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  }
  if (name == "best_effort_volatile") {
    return rclcpp::SensorDataQoS();
  }
  if (name == "reliable_transient_local") {
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  }
  return std::nullopt;
}

}  // namespace qos_lab
