#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "robot_core/geometry.hpp"
#include "robot_core/time.hpp"

namespace robot_core {

/// Where and how to grasp an item. Pre-grasp, grasp and lift poses are derived
/// from this by the grasp generator (week 4).
struct GraspTarget {
  std::string item_id;
  std::string frame_id{"map"};
  Pose3D grasp_pose;               ///< Gripper TCP pose at the moment of closing.
  double approach_distance{0.10};  ///< m, along the approach axis before grasp_pose.
  double gripper_opening{0.08};    ///< m, finger opening before closing.
};

[[nodiscard]] bool isValid(const GraspTarget& target);

/// Where to put a held item down.
struct PlaceTarget {
  std::string item_id;
  std::string frame_id{"map"};
  Pose3D place_pose;  ///< Gripper TCP pose at release.
};

[[nodiscard]] bool isValid(const PlaceTarget& target);

enum class ManipulationAction : std::uint8_t { Pick, Place };

enum class PickOutcome : std::uint8_t {
  Succeeded,
  PlanningFailed,
  ExecutionFailed,
  GraspFailed,  ///< Motion executed but the item is not held.
  Canceled,
};

[[nodiscard]] std::string_view toString(PickOutcome outcome);

/// Outcome of one pick or place attempt, used for success-rate statistics.
struct PickResult {
  std::string item_id;
  ManipulationAction action{ManipulationAction::Pick};
  PickOutcome outcome{PickOutcome::Canceled};
  Duration planning_time{Duration::zero()};
  Duration execution_time{Duration::zero()};
  std::string message;

  [[nodiscard]] bool succeeded() const { return outcome == PickOutcome::Succeeded; }
};

}  // namespace robot_core
