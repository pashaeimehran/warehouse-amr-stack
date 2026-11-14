// perception_3d lifecycle component (v0.2 skeleton).
//
// Status: lifecycle and composition skeleton only. It owns its parameters and
// its output publisher and honours every lifecycle transition, but it does not
// yet compute anything: the camera geometry (camera_geometry library) and the
// detection input arrive with v0.5-world-model. Until then it publishes nothing.
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "robot_interfaces/msg/detected_object3_d_array.hpp"

namespace perception_3d {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using robot_interfaces::msg::DetectedObject3DArray;

class Perception3DNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit Perception3DNode(const rclcpp::NodeOptions& options)
      : rclcpp_lifecycle::LifecycleNode("perception_3d", options) {
    declare_parameter<std::string>("target_frame", "map");
    // Roadmap safety contract: perception data older than 300 ms is rejected.
    declare_parameter<double>("max_detection_age_s", 0.3);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    target_frame_ = get_parameter("target_frame").as_string();
    max_detection_age_s_ = get_parameter("max_detection_age_s").as_double();
    if (target_frame_.empty() || max_detection_age_s_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "invalid parameters: target_frame empty or age <= 0");
      return CallbackReturn::FAILURE;
    }
    publisher_ = create_publisher<DetectedObject3DArray>("detections_3d", rclcpp::QoS(10));
    RCLCPP_INFO(get_logger(), "configured: target_frame=%s max_age=%.3f s", target_frame_.c_str(),
                max_detection_age_s_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override {
    LifecycleNode::on_activate(state);  // activates the lifecycle publisher
    RCLCPP_INFO(get_logger(), "activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override {
    LifecycleNode::on_deactivate(state);
    RCLCPP_INFO(get_logger(), "deactivated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    publisher_.reset();
    RCLCPP_INFO(get_logger(), "cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

 private:
  std::string target_frame_;
  double max_detection_age_s_{0.3};
  rclcpp_lifecycle::LifecyclePublisher<DetectedObject3DArray>::SharedPtr publisher_;
};

}  // namespace perception_3d

RCLCPP_COMPONENTS_REGISTER_NODE(perception_3d::Perception3DNode)
