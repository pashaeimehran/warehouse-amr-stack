#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "robot_core/fault.hpp"
#include "robot_core/geometry.hpp"
#include "robot_core/manipulation_types.hpp"
#include "robot_core/time.hpp"

namespace robot_core {

/// A named navigation goal with arrival tolerances.
struct Waypoint {
  std::string id;
  Pose2D pose;
  double position_tolerance_m{0.10};
  double yaw_tolerance_rad{0.10};
};

struct NoAction {};
struct PickAction {
  GraspTarget target;
};
struct PlaceAction {
  PlaceTarget target;
};

/// What to do on arrival at a station. Mirrors the VDA 5050 pick and drop
/// actions the fleet interface maps onto (week 4).
using StationAction = std::variant<NoAction, PickAction, PlaceAction>;

struct MissionLeg {
  Waypoint station;
  StationAction action{NoAction{}};
};

/// An ordered fetch mission: visit each leg's station, perform its action,
/// then return to `home` (usually the dock).
struct Mission {
  std::string id;
  std::vector<MissionLeg> legs;
  Waypoint home;
};

/// Checks a mission before execution. Returns an error message, or
/// std::nullopt if the mission is executable. Rules:
///  - non-empty id and at least one leg;
///  - every waypoint (including home) has an id, a finite pose and positive tolerances;
///  - pick and place targets are valid (see isValid overloads);
///  - the gripper holds at most one item: no pick while holding, and a place
///    must put down exactly the item currently held.
[[nodiscard]] std::optional<std::string> validateMission(const Mission& mission);

enum class MissionOutcome : std::uint8_t {
  Succeeded,
  Failed,    ///< Aborted because of a fault.
  Canceled,  ///< Aborted on request.
};

[[nodiscard]] std::string_view toString(MissionOutcome outcome);

struct MissionResult {
  std::string mission_id;
  MissionOutcome outcome{MissionOutcome::Failed};
  std::size_t legs_completed{0};
  std::optional<Fault> fault;  ///< Set when the outcome is Failed.
  Timestamp started_at;
  Timestamp finished_at;

  [[nodiscard]] Duration duration() const { return finished_at - started_at; }
};

}  // namespace robot_core
