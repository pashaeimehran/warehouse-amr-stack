#pragma once

#include <cstdint>
#include <string_view>

namespace robot_core {

/// Lifecycle of a long-running request (navigation goal, pick, place).
enum class ActionStatus : std::uint8_t {
  Idle,  ///< No request issued since construction or the last cancel.
  Running,
  Succeeded,
  Failed,
  Canceled,
};

[[nodiscard]] std::string_view toString(ActionStatus status);

[[nodiscard]] constexpr bool isTerminal(ActionStatus status) {
  return status == ActionStatus::Succeeded || status == ActionStatus::Failed ||
         status == ActionStatus::Canceled;
}

}  // namespace robot_core
