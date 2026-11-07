#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string_view>

#include "robot_core/clock.hpp"
#include "robot_core/time.hpp"

namespace robot_core {

enum class MissionState : std::uint8_t {
  Idle,
  Navigating,
  AtStation,
  Picking,
  Placing,
  Returning,
  Fault,
  Recovering,
  Aborted,
};

inline constexpr std::array<MissionState, 9> kAllMissionStates{
    MissionState::Idle,    MissionState::Navigating, MissionState::AtStation,
    MissionState::Picking, MissionState::Placing,    MissionState::Returning,
    MissionState::Fault,   MissionState::Recovering, MissionState::Aborted,
};

enum class MissionEvent : std::uint8_t {
  Start,                 ///< A validated mission was accepted.
  ArrivedAtStation,      ///< Navigation to the current station succeeded.
  BeginPick,             ///< Pick requested at the current station.
  BeginPlace,            ///< Place requested at the current station.
  ActionCompleted,       ///< The pick or place finished successfully.
  ProceedToNextStation,  ///< Navigation to the next leg's station requested.
  ReturnHome,            ///< All legs done; navigation home requested.
  ArrivedHome,           ///< Mission complete.
  FaultDetected,
  BeginRecovery,
  RecoverySucceeded,  ///< Resumes the state that was interrupted by the fault.
  RecoveryFailed,
  Abort,
  Reset,  ///< Clears an aborted mission.
};

inline constexpr std::array<MissionEvent, 14> kAllMissionEvents{
    MissionEvent::Start,
    MissionEvent::ArrivedAtStation,
    MissionEvent::BeginPick,
    MissionEvent::BeginPlace,
    MissionEvent::ActionCompleted,
    MissionEvent::ProceedToNextStation,
    MissionEvent::ReturnHome,
    MissionEvent::ArrivedHome,
    MissionEvent::FaultDetected,
    MissionEvent::BeginRecovery,
    MissionEvent::RecoverySucceeded,
    MissionEvent::RecoveryFailed,
    MissionEvent::Abort,
    MissionEvent::Reset,
};

// The exhaustive transition-table test iterates these arrays; keep them complete.
// Each list must end with the enum's last enumerator (enums are contiguous from 0).
static_assert(kAllMissionStates.size() == static_cast<std::size_t>(MissionState::Aborted) + 1,
              "kAllMissionStates must list every MissionState");
static_assert(kAllMissionEvents.size() == static_cast<std::size_t>(MissionEvent::Reset) + 1,
              "kAllMissionEvents must list every MissionEvent");

[[nodiscard]] std::string_view toString(MissionState state);
[[nodiscard]] std::string_view toString(MissionEvent event);

struct MissionTransition {
  MissionState from{MissionState::Idle};
  MissionState to{MissionState::Idle};
  MissionEvent event{MissionEvent::Start};
  Timestamp stamp;
};

/// Explicit mission FSM. Legal transitions live in a single table (see
/// docs/01-core-design.md for the diagram); every other (state, event) pair is
/// rejected and leaves the state unchanged.
///
/// Fault handling: entering FAULT from an active state remembers where the
/// mission was interrupted. RecoverySucceeded resumes NAVIGATING or RETURNING
/// as they were, and AT_STATION for faults during AT_STATION, PICKING or
/// PLACING (the station action is re-evaluated from the station, never resumed
/// mid-motion).
///
/// Not thread-safe: owned and driven by one thread. The clock must outlive the
/// state machine.
class MissionStateMachine {
 public:
  using TransitionListener = std::function<void(const MissionTransition&)>;

  explicit MissionStateMachine(const IClock& clock);

  [[nodiscard]] MissionState state() const { return state_; }

  /// Applies `event`. Returns true if the transition was legal and taken.
  [[nodiscard]] bool handle(MissionEvent event);

  /// Whether `event` would be accepted in the current state.
  [[nodiscard]] bool canHandle(MissionEvent event) const;

  /// Time since the last state change (or construction).
  [[nodiscard]] Duration timeInState() const;

  /// State that RecoverySucceeded would return to; set only in FAULT and RECOVERING.
  [[nodiscard]] std::optional<MissionState> resumeState() const { return resume_state_; }

  /// True in every state that belongs to a running mission.
  [[nodiscard]] bool isMissionActive() const;

  /// Called synchronously after each accepted transition (e.g. for logging).
  /// The listener must not call handle() on this state machine (no re-entrancy).
  void setTransitionListener(TransitionListener listener);

 private:
  [[nodiscard]] std::optional<MissionState> target(MissionEvent event) const;

  const IClock* clock_;
  MissionState state_{MissionState::Idle};
  Timestamp entered_at_;
  std::optional<MissionState> resume_state_;
  TransitionListener listener_;
};

}  // namespace robot_core
