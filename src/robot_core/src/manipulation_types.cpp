#include "robot_core/manipulation_types.hpp"

#include <cmath>

namespace robot_core {

namespace {

bool isValidPose(const Pose3D& pose) {
  return isFinite(pose) && isNormalized(pose.orientation);
}

}  // namespace

bool isValid(const GraspTarget& target) {
  return !target.item_id.empty() && !target.frame_id.empty() && isValidPose(target.grasp_pose) &&
         std::isfinite(target.approach_distance) && target.approach_distance > 0.0 &&
         std::isfinite(target.gripper_opening) && target.gripper_opening > 0.0;
}

bool isValid(const PlaceTarget& target) {
  return !target.item_id.empty() && !target.frame_id.empty() && isValidPose(target.place_pose);
}

std::string_view toString(PickOutcome outcome) {
  switch (outcome) {
    case PickOutcome::Succeeded:
      return "SUCCEEDED";
    case PickOutcome::PlanningFailed:
      return "PLANNING_FAILED";
    case PickOutcome::ExecutionFailed:
      return "EXECUTION_FAILED";
    case PickOutcome::GraspFailed:
      return "GRASP_FAILED";
    case PickOutcome::Canceled:
      return "CANCELED";
  }
  return "INVALID";
}

}  // namespace robot_core
