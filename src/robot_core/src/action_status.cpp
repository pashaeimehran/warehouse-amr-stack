#include "robot_core/action_status.hpp"

namespace robot_core {

std::string_view toString(ActionStatus status) {
  switch (status) {
    case ActionStatus::Idle:
      return "IDLE";
    case ActionStatus::Running:
      return "RUNNING";
    case ActionStatus::Succeeded:
      return "SUCCEEDED";
    case ActionStatus::Failed:
      return "FAILED";
    case ActionStatus::Canceled:
      return "CANCELED";
  }
  return "INVALID";
}

}  // namespace robot_core
