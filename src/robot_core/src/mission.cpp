#include "robot_core/mission.hpp"

#include <cmath>

namespace robot_core {

namespace {

bool isPositiveFinite(double v) {
  return std::isfinite(v) && v > 0.0;
}

std::optional<std::string> validateWaypoint(const Waypoint& wp, std::string_view role) {
  if (wp.id.empty()) {
    return std::string(role) + " waypoint has an empty id";
  }
  const std::string prefix = std::string(role) + " waypoint '" + wp.id + "': ";
  if (!isFinite(wp.pose)) {
    return prefix + "pose is not finite";
  }
  if (!isPositiveFinite(wp.position_tolerance_m)) {
    return prefix + "position tolerance must be positive";
  }
  if (!isPositiveFinite(wp.yaw_tolerance_rad)) {
    return prefix + "yaw tolerance must be positive";
  }
  return std::nullopt;
}

/// Checks one station action against the gripper occupancy rule (at most one
/// held item) and updates the simulated gripper content.
struct GripperRule {
  std::optional<std::string>* held_item;

  std::optional<std::string> operator()(const NoAction& /*action*/) const { return std::nullopt; }

  std::optional<std::string> operator()(const PickAction& action) const {
    if (!isValid(action.target)) {
      return "invalid grasp target";
    }
    if (*held_item) {
      return "pick of '" + action.target.item_id + "' while holding '" + **held_item + "'";
    }
    *held_item = action.target.item_id;
    return std::nullopt;
  }

  std::optional<std::string> operator()(const PlaceAction& action) const {
    if (!isValid(action.target)) {
      return "invalid place target";
    }
    if (!*held_item) {
      return "place of '" + action.target.item_id + "' with an empty gripper";
    }
    if (**held_item != action.target.item_id) {
      return "place of '" + action.target.item_id + "' while holding '" + **held_item + "'";
    }
    held_item->reset();
    return std::nullopt;
  }
};

}  // namespace

std::optional<std::string> validateMission(const Mission& mission) {
  if (mission.id.empty()) {
    return "mission id is empty";
  }
  if (mission.legs.empty()) {
    return "mission '" + mission.id + "' has no legs";
  }
  if (auto err = validateWaypoint(mission.home, "home")) {
    return err;
  }

  std::optional<std::string> held_item;
  const GripperRule gripper_rule{&held_item};
  for (std::size_t i = 0; i < mission.legs.size(); ++i) {
    const MissionLeg& leg = mission.legs[i];
    auto error = validateWaypoint(leg.station, "station");
    if (!error) {
      error = std::visit(gripper_rule, leg.action);
    }
    if (error) {
      return "leg " + std::to_string(i) + ": " + *error;
    }
  }
  return std::nullopt;
}

std::string_view toString(MissionOutcome outcome) {
  switch (outcome) {
    case MissionOutcome::Succeeded:
      return "SUCCEEDED";
    case MissionOutcome::Failed:
      return "FAILED";
    case MissionOutcome::Canceled:
      return "CANCELED";
  }
  return "INVALID";
}

}  // namespace robot_core
