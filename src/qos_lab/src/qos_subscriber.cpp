// Probe subscriber of the QoS experiment. Receives probe messages with the QoS
// profile `profile`, measures one-way latency (publisher and subscriber share
// the host clock) and loss over `count` expected messages, and writes a JSON
// result to `output` when the publisher goes quiet or `timeout_s` expires.
#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "qos_lab/probe.hpp"
#include "qos_lab/profiles.hpp"
#include "robot_core/sample_statistics.hpp"

namespace {

using std_msgs::msg::UInt8MultiArray;

class QosSubscriber : public rclcpp::Node {
 public:
  QosSubscriber() : rclcpp::Node("qos_subscriber") {
    profile_ = declare_parameter<std::string>("profile", "reliable_volatile");
    const auto count = static_cast<std::uint64_t>(declare_parameter<int>("count", 1000));
    timeout_s_ = declare_parameter<double>("timeout_s", 60.0);
    idle_timeout_s_ = declare_parameter<double>("idle_timeout_s", 2.0);
    output_ = declare_parameter<std::string>("output", "");
    tracker_ = std::make_unique<qos_lab::SequenceTracker>(count);
    latencies_ms_.reserve(static_cast<std::size_t>(count));

    const auto qos = qos_lab::profileByName(profile_);
    if (!qos) {
      throw std::invalid_argument("unknown profile " + profile_);
    }
    rclcpp::SubscriptionOptions options;
    options.event_callbacks.incompatible_qos_callback =
        [this](rclcpp::QOSRequestedIncompatibleQoSInfo&) { ++incompatible_events_; };
    subscription_ = create_subscription<UInt8MultiArray>(
        "qos_probe", *qos, [this](const UInt8MultiArray& msg) { onMessage(msg); }, options);
    started_ = now();
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this] { onWatchdog(); });
  }

 private:
  void onMessage(const UInt8MultiArray& msg) {
    const rclcpp::Time received = now();
    const auto header = qos_lab::decodeProbe(msg.data);
    if (!header) {
      ++malformed_;
      return;
    }
    tracker_->record(header->sequence);
    latencies_ms_.push_back(static_cast<double>(received.nanoseconds() - header->send_time_ns) /
                            1e6);
    payload_size_ = msg.data.size();
    last_message_ = received;
  }

  void onWatchdog() {
    const rclcpp::Time t = now();
    const bool all_received = tracker_->uniqueReceived() == tracker_->expected();
    const bool idle = last_message_.has_value() && (t - *last_message_).seconds() > idle_timeout_s_;
    const bool timed_out = (t - started_).seconds() > timeout_s_;
    if (all_received || idle || timed_out) {
      writeResult();
      rclcpp::shutdown();
    }
  }

  void writeResult() const {
    std::ostringstream json;
    json << "{\n"
         << "  \"rmw\": \"" << rmw_get_implementation_identifier() << "\",\n"
         << "  \"profile\": \"" << profile_ << "\",\n"
         << "  \"payload_size\": " << payload_size_ << ",\n"
         << "  \"expected\": " << tracker_->expected() << ",\n"
         << "  \"received\": " << tracker_->uniqueReceived() << ",\n"
         << "  \"lost\": " << tracker_->lost() << ",\n"
         << "  \"loss_ratio\": " << tracker_->lossRatio() << ",\n"
         << "  \"duplicates\": " << tracker_->duplicates() << ",\n"
         << "  \"out_of_order\": " << tracker_->outOfOrder() << ",\n"
         << "  \"malformed\": " << malformed_ << ",\n"
         << "  \"incompatible_qos_events\": " << incompatible_events_;
    if (const auto s = robot_core::summarize(latencies_ms_)) {
      json << ",\n  \"latency_ms\": {\"mean\": " << s->mean << ", \"p50\": " << s->p50
           << ", \"p95\": " << s->p95 << ", \"p99\": " << s->p99 << ", \"max\": " << s->max << "}";
    }
    json << "\n}\n";
    RCLCPP_INFO(get_logger(), "%s", json.str().c_str());
    if (!output_.empty()) {
      std::ofstream(output_) << json.str();
    }
  }

  std::string profile_;
  double timeout_s_{60.0};
  double idle_timeout_s_{2.0};
  std::string output_;
  std::unique_ptr<qos_lab::SequenceTracker> tracker_;
  std::vector<double> latencies_ms_;
  std::size_t payload_size_{0};
  std::uint64_t malformed_{0};
  int incompatible_events_{0};
  rclcpp::Time started_;
  std::optional<rclcpp::Time> last_message_;
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QosSubscriber>());
  rclcpp::shutdown();
  return 0;
}
