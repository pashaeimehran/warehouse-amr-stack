// tracking lifecycle component (v0.2 skeleton).
//
// Status: lifecycle and composition skeleton only. It subscribes to the 3D
// detections of perception_3d and owns the track publisher, but has no tracker
// yet: tracking_core (Kalman filter, association, track lifecycle) arrives with
// v0.5-world-model. While active it only counts received detection messages.
#include <cstdint>
#include <memory>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "robot_interfaces/msg/detected_object3_d_array.hpp"
#include "robot_interfaces/msg/tracked_object_array.hpp"

namespace tracking {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using robot_interfaces::msg::DetectedObject3DArray;
using robot_interfaces::msg::TrackedObjectArray;

class TrackingNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit TrackingNode(const rclcpp::NodeOptions& options)
      : rclcpp_lifecycle::LifecycleNode("tracking", options) {
    declare_parameter<double>("update_rate_hz", 20.0);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    if (get_parameter("update_rate_hz").as_double() <= 0.0) {
      RCLCPP_ERROR(get_logger(), "update_rate_hz must be positive");
      return CallbackReturn::FAILURE;
    }
    publisher_ = create_publisher<TrackedObjectArray>("tracks", rclcpp::QoS(10));
    // Subscriptions are not lifecycle-managed in rclcpp: the callback checks
    // the state itself, so nothing is processed unless the node is active.
    subscription_ = create_subscription<DetectedObject3DArray>(
        "detections_3d", rclcpp::QoS(10), [this](const DetectedObject3DArray&) {
          if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            ++received_;
          }
        });
    RCLCPP_INFO(get_logger(), "configured (intra-process: %s)",
                get_node_options().use_intra_process_comms() ? "on" : "off");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override {
    LifecycleNode::on_activate(state);
    RCLCPP_INFO(get_logger(), "activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override {
    LifecycleNode::on_deactivate(state);
    RCLCPP_INFO(get_logger(), "deactivated after %lu detection messages",
                static_cast<unsigned long>(received_));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    subscription_.reset();
    publisher_.reset();
    received_ = 0;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    subscription_.reset();
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

 private:
  std::uint64_t received_{0};
  rclcpp_lifecycle::LifecyclePublisher<TrackedObjectArray>::SharedPtr publisher_;
  rclcpp::Subscription<DetectedObject3DArray>::SharedPtr subscription_;
};

}  // namespace tracking

RCLCPP_COMPONENTS_REGISTER_NODE(tracking::TrackingNode)
