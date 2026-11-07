#include "robot_core/fault.hpp"

namespace robot_core {

std::string_view toString(FaultCode code) {
  switch (code) {
    case FaultCode::CameraStale:
      return "CAMERA_STALE";
    case FaultCode::InferenceTimeout:
      return "INFERENCE_TIMEOUT";
    case FaultCode::LidarStale:
      return "LIDAR_STALE";
    case FaultCode::LocalizationUntrusted:
      return "LOCALIZATION_UNTRUSTED";
    case FaultCode::PerceptionCrashed:
      return "PERCEPTION_CRASHED";
    case FaultCode::WheelJointFault:
      return "WHEEL_JOINT_FAULT";
    case FaultCode::ArmJointFrozen:
      return "ARM_JOINT_FROZEN";
    case FaultCode::GripperFault:
      return "GRIPPER_FAULT";
    case FaultCode::FleetLinkLost:
      return "FLEET_LINK_LOST";
    case FaultCode::NavigationFailed:
      return "NAVIGATION_FAILED";
    case FaultCode::NavigationTimeout:
      return "NAVIGATION_TIMEOUT";
    case FaultCode::ManipulationFailed:
      return "MANIPULATION_FAILED";
    case FaultCode::ManipulationTimeout:
      return "MANIPULATION_TIMEOUT";
    case FaultCode::CommandRejected:
      return "COMMAND_REJECTED";
  }
  return "INVALID";
}

std::string_view toString(FaultSeverity severity) {
  switch (severity) {
    case FaultSeverity::Recoverable:
      return "RECOVERABLE";
    case FaultSeverity::Critical:
      return "CRITICAL";
  }
  return "INVALID";
}

}  // namespace robot_core
