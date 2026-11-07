#pragma once

#include <ostream>
#include <string>

#include "robot_core/mission.hpp"

namespace robot_core {

// Readable GoogleTest/GoogleMock failure output (found by ADL).
// NOLINTNEXTLINE(readability-identifier-naming): name required by GoogleTest
inline void PrintTo(const Waypoint& wp, std::ostream* os) {
  *os << "Waypoint{" << wp.id << " @ (" << wp.pose.x << ", " << wp.pose.y << ", " << wp.pose.yaw
      << ")}";
}

}  // namespace robot_core

namespace robot_core::test {

inline Waypoint makeWaypoint(const std::string& id, double x, double y, double yaw = 0.0) {
  return Waypoint{id, Pose2D{x, y, yaw}, 0.1, 0.1};
}

inline GraspTarget makeGrasp(const std::string& item_id) {
  GraspTarget g;
  g.item_id = item_id;
  g.grasp_pose.position = Vector3{1.0, 2.0, 0.8};
  return g;
}

inline PlaceTarget makePlace(const std::string& item_id) {
  PlaceTarget p;
  p.item_id = item_id;
  p.place_pose.position = Vector3{5.0, 2.0, 0.6};
  return p;
}

/// The canonical fetch mission: pick at Station A, place at Station B, return to dock.
inline Mission makeFetchMission(const std::string& item_id = "klt_42") {
  Mission m;
  m.id = "order-1";
  m.home = makeWaypoint("dock", 0.0, 0.0);
  m.legs.push_back(MissionLeg{makeWaypoint("station_a", 4.0, 1.0), PickAction{makeGrasp(item_id)}});
  m.legs.push_back(
      MissionLeg{makeWaypoint("station_b", 8.0, -2.0, 1.57), PlaceAction{makePlace(item_id)}});
  return m;
}

}  // namespace robot_core::test
