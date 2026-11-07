#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>

#include "robot_core/clock.hpp"
#include "robot_core/fault.hpp"
#include "robot_core/manipulation_client.hpp"
#include "robot_core/mission.hpp"
#include "robot_core/mission_state_machine.hpp"
#include "robot_core/navigation_client.hpp"

namespace robot_core {

struct MissionExecutorConfig {
  Duration navigation_timeout{std::chrono::seconds{120}};   ///< Per navigation goal.
  Duration manipulation_timeout{std::chrono::seconds{60}};  ///< Per pick or place.
  std::size_t max_recovery_attempts{1};                     ///< Per mission.
};

/// Executes a Mission by driving MissionStateMachine from the status of the
/// navigation and manipulation ports. Purely reactive and single-threaded:
/// every decision happens inside tick(), so tests step it deterministically
/// with a FakeClock and mocked ports.
///
/// Fault policy:
///  - Recoverable faults (goal failure, timeout, rejected command) enter FAULT;
///    the next tick attempts recovery (re-issue the interrupted goal or retry
///    the station action) while the recovery budget lasts, else aborts.
///  - Critical faults abort immediately.
///  - Any fault or abort first cancels the motion in progress.
///
/// Ownership: the ports and the clock are not owned and must outlive the
/// executor. The executor is neither copyable nor movable: a copy would share
/// the robot's ports while holding independent mission state.
///
/// Threading: not thread-safe. Every member function, including abort() and
/// reportFault(), must be called from the thread that calls tick(). Faults
/// detected on other threads are handed over through a TaskQueue and applied
/// on that thread.
class MissionExecutor {
 public:
  /// Throws std::invalid_argument if a timeout in `config` is negative.
  MissionExecutor(INavigationClient& navigation, IManipulationClient& manipulation,
                  const IClock& clock, MissionExecutorConfig config = {});

  MissionExecutor(const MissionExecutor&) = delete;
  MissionExecutor& operator=(const MissionExecutor&) = delete;
  MissionExecutor(MissionExecutor&&) = delete;
  MissionExecutor& operator=(MissionExecutor&&) = delete;
  ~MissionExecutor() = default;

  /// Validates `mission` and sends the first navigation goal. Returns an error
  /// message (and changes nothing) if a mission is already active, validation
  /// fails or the first goal is rejected.
  [[nodiscard]] std::optional<std::string> start(Mission mission);

  /// Advances the mission by at most one state transition.
  void tick();

  /// Operator cancel: stops motion and aborts with outcome Canceled.
  /// Returns false if no mission is active.
  bool abort();

  /// Fault from outside the mission layer (safety supervisor, health monitor).
  /// Returns false if no mission is active.
  bool reportFault(const Fault& fault);

  [[nodiscard]] MissionState state() const { return fsm_.state(); }
  [[nodiscard]] const MissionStateMachine& stateMachine() const { return fsm_; }

  /// Set once the mission has finished (succeeded, failed or canceled).
  [[nodiscard]] const std::optional<MissionResult>& result() const { return result_; }

  [[nodiscard]] std::size_t currentLegIndex() const { return leg_index_; }
  [[nodiscard]] std::size_t recoveryAttempts() const { return recovery_attempts_; }

 private:
  void pollNavigation(MissionEvent on_success);
  void onAtStation();
  void pollManipulation();
  void onFault();
  void onRecovering();

  void enterFault(FaultCode code, FaultSeverity severity, std::string message);
  void applyFault(Fault fault);
  void stopMotion();
  void finish(MissionOutcome outcome);
  void transition(MissionEvent event);

  INavigationClient* navigation_;
  IManipulationClient* manipulation_;
  const IClock* clock_;
  MissionExecutorConfig config_;
  MissionStateMachine fsm_;

  Mission mission_;
  std::size_t leg_index_{0};
  std::size_t legs_completed_{0};
  bool station_action_done_{false};
  std::size_t recovery_attempts_{0};
  std::optional<Fault> last_fault_;
  Timestamp started_at_;
  std::optional<MissionResult> result_;
};

}  // namespace robot_core
