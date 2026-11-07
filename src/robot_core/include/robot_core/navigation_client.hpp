#pragma once

#include "robot_core/action_status.hpp"
#include "robot_core/mission.hpp"

namespace robot_core {

/// Port to the base navigation stack. Implemented by a Nav2 action-client
/// adapter in the ROS layer and by GoogleMock doubles in tests.
///
/// The interface is non-blocking: navigateTo() issues a goal, status() is
/// polled. At most one goal is active; a new goal preempts the previous one.
class INavigationClient {
 public:
  INavigationClient() = default;
  virtual ~INavigationClient() = default;
  INavigationClient(const INavigationClient&) = delete;
  INavigationClient& operator=(const INavigationClient&) = delete;
  INavigationClient(INavigationClient&&) = delete;
  INavigationClient& operator=(INavigationClient&&) = delete;

  /// Sends a goal. Returns false if it was rejected (server unavailable, goal
  /// invalid); status() is then unchanged.
  ///
  /// Contract: when this returns true, status() must already describe the new
  /// goal (Running, or Idle until the server acknowledges it). It must never
  /// report the terminal status of a previous goal, or the caller would treat
  /// the new goal as already finished.
  [[nodiscard]] virtual bool navigateTo(const Waypoint& goal) = 0;

  /// Status of the most recent goal.
  [[nodiscard]] virtual ActionStatus status() const = 0;

  /// Requests cancellation of the active goal. No effect when none is active.
  virtual void cancel() = 0;
};

}  // namespace robot_core
