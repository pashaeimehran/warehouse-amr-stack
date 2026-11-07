#pragma once

#include <optional>

#include "robot_core/action_status.hpp"
#include "robot_core/manipulation_types.hpp"

namespace robot_core {

/// Port to arm motion planning and execution. Implemented by a
/// MoveGroupInterface adapter in the ROS layer (week 4) and by GoogleMock
/// doubles in tests.
///
/// Same non-blocking contract as INavigationClient. Safety interlocks (base
/// stopped, no person within 1.5 m) are enforced by the safety supervisor,
/// not by implementations of this interface.
class IManipulationClient {
 public:
  IManipulationClient() = default;
  virtual ~IManipulationClient() = default;
  IManipulationClient(const IManipulationClient&) = delete;
  IManipulationClient& operator=(const IManipulationClient&) = delete;
  IManipulationClient(IManipulationClient&&) = delete;
  IManipulationClient& operator=(IManipulationClient&&) = delete;

  /// Starts a pick. Returns false if rejected; status() is then unchanged.
  /// Same contract as INavigationClient::navigateTo(): once a request is
  /// accepted, status() and lastResult() describe that request, never the
  /// previous one.
  [[nodiscard]] virtual bool requestPick(const GraspTarget& target) = 0;

  /// Starts a place of the held item. Returns false if rejected.
  [[nodiscard]] virtual bool requestPlace(const PlaceTarget& target) = 0;

  /// Status of the most recent request.
  [[nodiscard]] virtual ActionStatus status() const = 0;

  /// Detailed result of the most recent request once status() is terminal.
  [[nodiscard]] virtual std::optional<PickResult> lastResult() const = 0;

  /// Stops arm motion for the active request.
  virtual void cancel() = 0;
};

}  // namespace robot_core
