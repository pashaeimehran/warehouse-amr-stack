#pragma once

#include <chrono>
#include <cstdint>

namespace robot_core {

/// Tag type for the robot's monotonic time base.
///
/// A dedicated tag (instead of std::chrono::steady_clock) prevents mixing robot
/// time with wall-clock or OS steady time at compile time. The source of robot
/// time is decided by the injected IClock: the OS steady clock in production,
/// simulation time behind a ROS adapter, or a FakeClock in tests.
struct RobotTime {
  // Member names are mandated by the C++ Clock requirements.
  // NOLINTBEGIN(readability-identifier-naming)
  using duration = std::chrono::nanoseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<RobotTime, duration>;
  static constexpr bool is_steady = true;
  // NOLINTEND(readability-identifier-naming)
};

using Duration = RobotTime::duration;
using Timestamp = RobotTime::time_point;

/// Builds a timestamp from nanoseconds since the start of the time base.
constexpr Timestamp timestampFromNanoseconds(std::int64_t ns) {
  return Timestamp{Duration{ns}};
}

/// True when `stamp` lies in [now - max_age, now].
///
/// A stamp in the future is rejected: it indicates a clock mismatch between
/// producer and consumer, and data that cannot be ordered is not trusted.
/// Data exactly `max_age` old is still fresh; anything older is stale.
constexpr bool isFresh(Timestamp stamp, Timestamp now, Duration max_age) {
  return stamp <= now && (now - stamp) <= max_age;
}

}  // namespace robot_core
