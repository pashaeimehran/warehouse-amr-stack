#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "robot_core/time.hpp"

namespace robot_core {

/// Fault codes reported to the mission layer. The FMEA (docs/09-fmea.md,
/// week 6) is the authority on detection budgets and responses; this list
/// grows with it.
enum class FaultCode : std::uint8_t {
  CameraStale,
  InferenceTimeout,
  LidarStale,
  LocalizationUntrusted,
  PerceptionCrashed,
  WheelJointFault,
  ArmJointFrozen,
  GripperFault,
  FleetLinkLost,
  NavigationFailed,
  NavigationTimeout,
  ManipulationFailed,
  ManipulationTimeout,
  CommandRejected,
};

[[nodiscard]] std::string_view toString(FaultCode code);

/// Recoverable faults allow the mission layer to attempt recovery; Critical
/// faults abort the mission immediately.
enum class FaultSeverity : std::uint8_t { Recoverable, Critical };

[[nodiscard]] std::string_view toString(FaultSeverity severity);

struct Fault {
  FaultCode code{FaultCode::CommandRejected};
  FaultSeverity severity{FaultSeverity::Recoverable};
  std::string source;   ///< Reporting component, e.g. "mission_executor".
  std::string message;  ///< Human-readable detail for logs.
  Timestamp stamp;      ///< When the fault was detected.
};

}  // namespace robot_core
