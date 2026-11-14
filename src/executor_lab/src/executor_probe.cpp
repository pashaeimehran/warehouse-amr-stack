// Callback starvation experiment: one node with a 50 Hz "control" timer and a
// slow timer that does `slow_work_ms` of CPU work every `slow_period_ms`.
// Four configurations (parameter `config`):
//   single_default   SingleThreadedExecutor, both timers in the default group
//   single_separate  SingleThreadedExecutor, one mutually exclusive group each
//   multi_default    MultiThreadedExecutor (4 threads), both in the default group
//   multi_separate   MultiThreadedExecutor (4 threads), one group each
// After `duration_s` the node writes a JSON report of the fast timer's periods.
#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "executor_lab/period_analysis.hpp"

namespace {

using Clock = std::chrono::steady_clock;

struct Config {
  bool multi_threaded{false};
  bool separate_groups{false};
};

std::optional<Config> parseConfig(const std::string& name) {
  if (name == "single_default") {
    return Config{false, false};
  }
  if (name == "single_separate") {
    return Config{false, true};
  }
  if (name == "multi_default") {
    return Config{true, false};
  }
  if (name == "multi_separate") {
    return Config{true, true};
  }
  return std::nullopt;
}

class ExecutorProbe : public rclcpp::Node {
 public:
  explicit ExecutorProbe(const Config& config) : rclcpp::Node("executor_probe") {
    config_name_ = declare_parameter<std::string>("config", "single_default");
    declare_parameter<double>("duration_s", 10.0);  // read in main()
    fast_hz_ = declare_parameter<double>("fast_hz", 50.0);
    slow_period_ms_ = declare_parameter<std::int64_t>("slow_period_ms", 500);
    slow_work_ms_ = declare_parameter<std::int64_t>("slow_work_ms", 200);
    output_ = declare_parameter<std::string>("output", "");

    // Nodes keep only weak references to callback groups: the groups must be
    // members, or they are destroyed with this scope and their timers never run.
    if (config.separate_groups) {
      fast_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      slow_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    fast_starts_.reserve(10000);
    fast_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / fast_hz_),
        [this] { fast_starts_.push_back(Clock::now().time_since_epoch()); }, fast_group_);
    slow_timer_ = create_wall_timer(
        std::chrono::milliseconds(slow_period_ms_),
        [this] {
          // CPU-bound work standing in for e.g. point cloud processing.
          const auto until = Clock::now() + std::chrono::milliseconds(slow_work_ms_);
          while (Clock::now() < until) {
          }
          ++slow_invocations_;
        },
        slow_group_);
  }

  void writeReport(double duration_s) const {
    const auto nominal = std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / fast_hz_));
    std::vector<std::chrono::nanoseconds> starts;
    starts.reserve(fast_starts_.size());
    for (const auto& s : fast_starts_) {
      starts.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(s));
    }
    std::ostringstream json;
    json << "{\n  \"config\": \"" << config_name_ << "\",\n"
         << "  \"duration_s\": " << duration_s << ",\n"
         << "  \"fast_hz\": " << fast_hz_ << ",\n"
         << "  \"slow_period_ms\": " << slow_period_ms_ << ",\n"
         << "  \"slow_work_ms\": " << slow_work_ms_ << ",\n"
         << "  \"slow_invocations\": " << slow_invocations_ << ",\n"
         << "  \"expected_fast_invocations\": " << static_cast<int>(duration_s * fast_hz_);
    if (const auto r = executor_lab::analyzePeriods(starts, nominal)) {
      json << ",\n  \"fast_invocations\": " << r->invocations << ",\n"
           << "  \"missed_activations\": " << r->missed_activations << ",\n"
           << "  \"late_gaps\": " << r->late_gaps << ",\n"
           << "  \"period_ms\": {\"p50\": " << r->period_ms.p50 << ", \"p95\": " << r->period_ms.p95
           << ", \"p99\": " << r->period_ms.p99 << ", \"max\": " << r->period_ms.max
           << ", \"stddev\": " << r->period_ms.stddev << "}";
    }
    json << "\n}\n";
    RCLCPP_INFO(get_logger(), "%s", json.str().c_str());
    if (!output_.empty()) {
      std::ofstream(output_) << json.str();
    }
  }

 private:
  std::string config_name_;
  double fast_hz_{50.0};
  std::int64_t slow_period_ms_{500};
  std::int64_t slow_work_ms_{200};
  std::string output_;
  std::vector<Clock::duration> fast_starts_;     // written only by the fast callback
  std::uint64_t slow_invocations_{0};            // written only by the slow callback
  rclcpp::CallbackGroup::SharedPtr fast_group_;  // null: default group
  rclcpp::CallbackGroup::SharedPtr slow_group_;
  rclcpp::TimerBase::SharedPtr fast_timer_;
  rclcpp::TimerBase::SharedPtr slow_timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // The configuration decides how the node is built, so read it first.
  auto params_node = std::make_shared<rclcpp::Node>(
      "executor_probe", rclcpp::NodeOptions()
                            .allow_undeclared_parameters(true)
                            .automatically_declare_parameters_from_overrides(true));
  const std::string config_name =
      params_node->get_parameter_or<std::string>("config", "single_default");
  const double duration_s = params_node->get_parameter_or<double>("duration_s", 10.0);
  params_node.reset();
  const auto config = parseConfig(config_name);
  if (!config) {
    std::fprintf(stderr, "unknown config '%s'\n", config_name.c_str());
    return 2;
  }

  auto node = std::make_shared<ExecutorProbe>(*config);
  std::unique_ptr<rclcpp::Executor> executor;
  if (config->multi_threaded) {
    executor =
        std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
  } else {
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  }
  executor->add_node(node);
  // Stop after the configured duration; the executor's threads are joined
  // before the report reads the samples.
  auto stopper = node->create_wall_timer(std::chrono::duration<double>(duration_s),
                                         [&executor] { executor->cancel(); });
  executor->spin();
  stopper->cancel();
  node->writeReport(duration_s);
  rclcpp::shutdown();
  return 0;
}
