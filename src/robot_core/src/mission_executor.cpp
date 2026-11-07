#include "robot_core/mission_executor.hpp"

#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

namespace robot_core {

namespace {

constexpr const char* kSource = "mission_executor";

bool hasAction(const MissionLeg& leg) {
  return !std::holds_alternative<NoAction>(leg.action);
}

const MissionExecutorConfig& validated(const MissionExecutorConfig& config) {
  if (config.navigation_timeout < Duration::zero() ||
      config.manipulation_timeout < Duration::zero()) {
    throw std::invalid_argument("MissionExecutorConfig: timeouts must not be negative");
  }
  return config;
}

}  // namespace

MissionExecutor::MissionExecutor(INavigationClient& navigation, IManipulationClient& manipulation,
                                 const IClock& clock, MissionExecutorConfig config)
    : navigation_{&navigation},
      manipulation_{&manipulation},
      clock_{&clock},
      config_{validated(config)},
      fsm_{clock} {}

std::optional<std::string> MissionExecutor::start(Mission mission) {
  if (fsm_.isMissionActive()) {
    return "mission '" + mission_.id + "' is still active";
  }
  if (auto error = validateMission(mission)) {
    return error;
  }
  if (!navigation_->navigateTo(mission.legs.front().station)) {
    return "navigation goal to '" + mission.legs.front().station.id + "' was rejected";
  }

  if (fsm_.state() == MissionState::Aborted) {
    transition(MissionEvent::Reset);
  }
  mission_ = std::move(mission);
  leg_index_ = 0;
  legs_completed_ = 0;
  station_action_done_ = false;
  recovery_attempts_ = 0;
  last_fault_.reset();
  result_.reset();
  started_at_ = clock_->now();
  transition(MissionEvent::Start);
  return std::nullopt;
}

void MissionExecutor::tick() {
  switch (fsm_.state()) {
    case MissionState::Navigating:
      pollNavigation(MissionEvent::ArrivedAtStation);
      break;
    case MissionState::Returning:
      pollNavigation(MissionEvent::ArrivedHome);
      break;
    case MissionState::AtStation:
      onAtStation();
      break;
    case MissionState::Picking:
    case MissionState::Placing:
      pollManipulation();
      break;
    case MissionState::Fault:
      onFault();
      break;
    case MissionState::Recovering:
      onRecovering();
      break;
    case MissionState::Idle:
    case MissionState::Aborted:
      break;
  }
}

bool MissionExecutor::abort() {
  if (!fsm_.isMissionActive()) {
    return false;
  }
  stopMotion();
  transition(MissionEvent::Abort);
  finish(MissionOutcome::Canceled);
  return true;
}

bool MissionExecutor::reportFault(const Fault& fault) {
  if (!fsm_.isMissionActive()) {
    return false;
  }
  applyFault(fault);
  return true;
}

void MissionExecutor::pollNavigation(MissionEvent on_success) {
  const ActionStatus status = navigation_->status();
  if (status == ActionStatus::Succeeded) {
    transition(on_success);
    if (on_success == MissionEvent::ArrivedHome) {
      finish(MissionOutcome::Succeeded);
    }
    return;
  }
  if (status == ActionStatus::Failed || status == ActionStatus::Canceled) {
    enterFault(FaultCode::NavigationFailed, FaultSeverity::Recoverable,
               std::string{"navigation goal ended with status "} + std::string{toString(status)});
    return;
  }
  if (fsm_.timeInState() > config_.navigation_timeout) {
    enterFault(FaultCode::NavigationTimeout, FaultSeverity::Recoverable,
               "navigation goal exceeded its timeout");
  }
}

void MissionExecutor::onAtStation() {
  const MissionLeg& leg = mission_.legs[leg_index_];

  if (hasAction(leg) && !station_action_done_) {
    bool accepted = false;
    MissionEvent event = MissionEvent::BeginPick;
    std::visit(
        [&](const auto& action) {
          using T = std::decay_t<decltype(action)>;
          if constexpr (std::is_same_v<T, PickAction>) {
            accepted = manipulation_->requestPick(action.target);
            event = MissionEvent::BeginPick;
          } else if constexpr (std::is_same_v<T, PlaceAction>) {
            accepted = manipulation_->requestPlace(action.target);
            event = MissionEvent::BeginPlace;
          }
        },
        leg.action);
    if (accepted) {
      transition(event);
    } else {
      enterFault(FaultCode::CommandRejected, FaultSeverity::Recoverable,
                 "manipulation request at '" + leg.station.id + "' was rejected");
    }
    return;
  }

  // Station work is done: leave for the next station or home. The leg index
  // only advances once the new goal is accepted, so a rejection resumes here.
  const bool is_last_leg = leg_index_ + 1 == mission_.legs.size();
  const Waypoint& next = is_last_leg ? mission_.home : mission_.legs[leg_index_ + 1].station;
  if (!navigation_->navigateTo(next)) {
    enterFault(FaultCode::CommandRejected, FaultSeverity::Recoverable,
               "navigation goal to '" + next.id + "' was rejected");
    return;
  }
  ++legs_completed_;
  if (is_last_leg) {
    transition(MissionEvent::ReturnHome);
  } else {
    ++leg_index_;
    station_action_done_ = false;
    transition(MissionEvent::ProceedToNextStation);
  }
}

void MissionExecutor::pollManipulation() {
  const ActionStatus status = manipulation_->status();
  if (status == ActionStatus::Succeeded) {
    station_action_done_ = true;
    transition(MissionEvent::ActionCompleted);
    return;
  }
  if (status == ActionStatus::Failed || status == ActionStatus::Canceled) {
    std::string message =
        std::string{"manipulation ended with status "} + std::string{toString(status)};
    if (const auto result = manipulation_->lastResult()) {
      message += " (" + std::string{toString(result->outcome)} + ": " + result->message + ")";
    }
    enterFault(FaultCode::ManipulationFailed, FaultSeverity::Recoverable, std::move(message));
    return;
  }
  if (fsm_.timeInState() > config_.manipulation_timeout) {
    enterFault(FaultCode::ManipulationTimeout, FaultSeverity::Recoverable,
               "manipulation exceeded its timeout");
  }
}

void MissionExecutor::onFault() {
  if (recovery_attempts_ < config_.max_recovery_attempts) {
    ++recovery_attempts_;
    transition(MissionEvent::BeginRecovery);
    return;
  }
  transition(MissionEvent::Abort);
  finish(MissionOutcome::Failed);
}

void MissionExecutor::onRecovering() {
  const auto resume = fsm_.resumeState();
  bool ok = true;  // AtStation: the station logic retries on the next tick
  if (resume == MissionState::Navigating) {
    ok = navigation_->navigateTo(mission_.legs[leg_index_].station);
  } else if (resume == MissionState::Returning) {
    ok = navigation_->navigateTo(mission_.home);
  }
  if (ok) {
    transition(MissionEvent::RecoverySucceeded);
  } else {
    transition(MissionEvent::RecoveryFailed);
    finish(MissionOutcome::Failed);
  }
}

void MissionExecutor::enterFault(FaultCode code, FaultSeverity severity, std::string message) {
  applyFault(Fault{code, severity, kSource, std::move(message), clock_->now()});
}

void MissionExecutor::applyFault(Fault fault) {
  stopMotion();
  const bool critical = fault.severity == FaultSeverity::Critical;
  last_fault_ = std::move(fault);
  if (fsm_.state() != MissionState::Fault) {
    transition(MissionEvent::FaultDetected);
  }
  if (critical) {
    transition(MissionEvent::Abort);
    finish(MissionOutcome::Failed);
  }
}

void MissionExecutor::stopMotion() {
  switch (fsm_.state()) {
    case MissionState::Navigating:
    case MissionState::Returning:
      navigation_->cancel();
      break;
    case MissionState::Picking:
    case MissionState::Placing:
      manipulation_->cancel();
      break;
    default:
      break;
  }
}

void MissionExecutor::finish(MissionOutcome outcome) {
  result_ =
      MissionResult{mission_.id,     outcome,
                    legs_completed_, outcome == MissionOutcome::Failed ? last_fault_ : std::nullopt,
                    started_at_,     clock_->now()};
}

void MissionExecutor::transition(MissionEvent event) {
  if (!fsm_.handle(event)) {
    // Reaching this is a bug in the executor, never a runtime condition.
    throw std::logic_error("MissionExecutor: illegal event " + std::string{toString(event)} +
                           " in state " + std::string{toString(fsm_.state())});
  }
}

}  // namespace robot_core
