// Probe publisher of the QoS experiment. Publishes `count` probe messages of
// `payload_size` bytes at `rate_hz` with the QoS profile `profile`, after
// waiting for a matched subscriber, then exits.
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "qos_lab/probe.hpp"
#include "qos_lab/profiles.hpp"

namespace {

using std_msgs::msg::UInt8MultiArray;

class QosPublisher : public rclcpp::Node {
 public:
  QosPublisher() : rclcpp::Node("qos_publisher") {
    const auto profile = declare_parameter<std::string>("profile", "reliable_volatile");
    count_ = static_cast<std::uint64_t>(declare_parameter<int>("count", 1000));
    const auto payload_size = static_cast<std::size_t>(declare_parameter<int>("payload_size", 256));
    const double rate_hz = declare_parameter<double>("rate_hz", 100.0);
    match_timeout_ =
        std::chrono::duration<double>(declare_parameter<double>("match_timeout_s", 10.0));

    const auto qos = qos_lab::profileByName(profile);
    if (!qos || rate_hz <= 0.0 || payload_size < qos_lab::kProbeHeaderSize) {
      throw std::invalid_argument("invalid profile, rate_hz or payload_size");
    }
    message_.data.assign(payload_size, 0xA5);

    rclcpp::PublisherOptions options;
    options.event_callbacks.incompatible_qos_callback =
        [this](rclcpp::QOSOfferedIncompatibleQoSInfo&) { ++incompatible_events_; };
    publisher_ = create_publisher<UInt8MultiArray>("qos_probe", *qos, options);
    started_waiting_ = now();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), [this] { onTimer(); });
    RCLCPP_INFO(get_logger(), "profile=%s payload=%zu B rate=%.1f Hz count=%lu", profile.c_str(),
                payload_size, rate_hz, static_cast<unsigned long>(count_));
  }

 private:
  void onTimer() {
    if (!matched_) {
      if (publisher_->get_subscription_count() > 0) {
        matched_ = true;
        RCLCPP_INFO(get_logger(), "subscriber matched");
      } else if ((now() - started_waiting_).seconds() > match_timeout_.count()) {
        RCLCPP_WARN(get_logger(),
                    "no subscriber matched within %.1f s (incompatible QoS events: %d)",
                    match_timeout_.count(), incompatible_events_);
        rclcpp::shutdown();
      }
      return;
    }
    if (sequence_ >= count_) {
      rclcpp::shutdown();
      return;
    }
    const qos_lab::ProbeHeader header{sequence_, now().nanoseconds()};
    if (qos_lab::writeProbeHeader(header, message_.data)) {
      publisher_->publish(message_);
    }
    ++sequence_;
  }

  std::uint64_t count_{0};
  std::uint64_t sequence_{0};
  std::chrono::duration<double> match_timeout_{10.0};
  bool matched_{false};
  int incompatible_events_{0};
  rclcpp::Time started_waiting_;
  UInt8MultiArray message_;
  rclcpp::Publisher<UInt8MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QosPublisher>());
  rclcpp::shutdown();
  return 0;
}
