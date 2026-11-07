#include "robot_core/mission_state_machine.hpp"

#include <utility>

namespace robot_core {

namespace {

using S = MissionState;
using E = MissionEvent;

/// The transition table. RecoverySucceeded is context-dependent and resolved
/// in MissionStateMachine::target(); here it only marks legality.
std::optional<MissionState> tableLookup(MissionState s, MissionEvent e) {
  switch (s) {
    case S::Idle:
      if (e == E::Start) {
        return S::Navigating;
      }
      break;
    case S::Navigating:
      if (e == E::ArrivedAtStation) {
        return S::AtStation;
      }
      break;
    case S::AtStation:
      switch (e) {
        case E::BeginPick:
          return S::Picking;
        case E::BeginPlace:
          return S::Placing;
        case E::ProceedToNextStation:
          return S::Navigating;
        case E::ReturnHome:
          return S::Returning;
        default:
          break;
      }
      break;
    case S::Picking:
    case S::Placing:
      if (e == E::ActionCompleted) {
        return S::AtStation;
      }
      break;
    case S::Returning:
      if (e == E::ArrivedHome) {
        return S::Idle;
      }
      break;
    case S::Fault:
      if (e == E::BeginRecovery) {
        return S::Recovering;
      }
      break;
    case S::Recovering:
      if (e == E::RecoverySucceeded) {
        return S::Recovering;  // placeholder; resolved against resume_state_
      }
      if (e == E::RecoveryFailed) {
        return S::Aborted;
      }
      break;
    case S::Aborted:
      if (e == E::Reset) {
        return S::Idle;
      }
      return std::nullopt;
  }

  // Shared rules for every state of a running mission.
  const bool active = s != S::Idle && s != S::Aborted;
  if (active && e == E::Abort) {
    return S::Aborted;
  }
  if (active && s != S::Fault && e == E::FaultDetected) {
    return S::Fault;
  }
  return std::nullopt;
}

MissionState resumeTargetFor(MissionState interrupted) {
  switch (interrupted) {
    case S::Navigating:
      return S::Navigating;
    case S::Returning:
      return S::Returning;
    default:
      return S::AtStation;  // AtStation, Picking, Placing
  }
}

}  // namespace

MissionStateMachine::MissionStateMachine(const IClock& clock)
    : clock_{&clock}, entered_at_{clock.now()} {}

std::optional<MissionState> MissionStateMachine::target(MissionEvent event) const {
  auto next = tableLookup(state_, event);
  if (next && event == E::RecoverySucceeded) {
    if (!resume_state_) {
      return std::nullopt;  // unreachable by construction; defensive
    }
    next = resume_state_;
  }
  return next;
}

bool MissionStateMachine::canHandle(MissionEvent event) const {
  return target(event).has_value();
}

bool MissionStateMachine::handle(MissionEvent event) {
  const auto next = target(event);
  if (!next) {
    return false;
  }

  const MissionState from = state_;
  if (*next == S::Fault && from != S::Recovering) {
    resume_state_ = resumeTargetFor(from);
  } else if (*next != S::Fault && *next != S::Recovering) {
    resume_state_.reset();
  }

  state_ = *next;
  entered_at_ = clock_->now();
  if (listener_) {
    listener_(MissionTransition{from, state_, event, entered_at_});
  }
  return true;
}

Duration MissionStateMachine::timeInState() const {
  return clock_->now() - entered_at_;
}

bool MissionStateMachine::isMissionActive() const {
  return state_ != S::Idle && state_ != S::Aborted;
}

void MissionStateMachine::setTransitionListener(TransitionListener listener) {
  listener_ = std::move(listener);
}

std::string_view toString(MissionState state) {
  switch (state) {
    case S::Idle:
      return "IDLE";
    case S::Navigating:
      return "NAVIGATING";
    case S::AtStation:
      return "AT_STATION";
    case S::Picking:
      return "PICKING";
    case S::Placing:
      return "PLACING";
    case S::Returning:
      return "RETURNING";
    case S::Fault:
      return "FAULT";
    case S::Recovering:
      return "RECOVERING";
    case S::Aborted:
      return "ABORTED";
  }
  return "INVALID";
}

std::string_view toString(MissionEvent event) {
  switch (event) {
    case E::Start:
      return "START";
    case E::ArrivedAtStation:
      return "ARRIVED_AT_STATION";
    case E::BeginPick:
      return "BEGIN_PICK";
    case E::BeginPlace:
      return "BEGIN_PLACE";
    case E::ActionCompleted:
      return "ACTION_COMPLETED";
    case E::ProceedToNextStation:
      return "PROCEED_TO_NEXT_STATION";
    case E::ReturnHome:
      return "RETURN_HOME";
    case E::ArrivedHome:
      return "ARRIVED_HOME";
    case E::FaultDetected:
      return "FAULT_DETECTED";
    case E::BeginRecovery:
      return "BEGIN_RECOVERY";
    case E::RecoverySucceeded:
      return "RECOVERY_SUCCEEDED";
    case E::RecoveryFailed:
      return "RECOVERY_FAILED";
    case E::Abort:
      return "ABORT";
    case E::Reset:
      return "RESET";
  }
  return "INVALID";
}

}  // namespace robot_core
