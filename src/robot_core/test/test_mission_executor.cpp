#include <chrono>
#include <memory>
#include <stdexcept>
#include <type_traits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "fixtures.hpp"
#include "mocks.hpp"
#include "robot_core/clock.hpp"
#include "robot_core/mission_executor.hpp"

namespace robot_core {
namespace {

using std::chrono::seconds;
using test::makeFetchMission;
using test::makeWaypoint;
using test::MockManipulationClient;
using test::MockNavigationClient;
using ::testing::_;
using ::testing::Assign;
using ::testing::DoAll;
using ::testing::Field;
using ::testing::HasSubstr;
using ::testing::InSequence;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::ReturnPointee;

using S = MissionState;

auto goalTo(const char* id) {
  return Field(&Waypoint::id, id);
}

class MissionExecutorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Behave like real action servers: accepting a request makes it Running.
    ON_CALL(nav_, navigateTo(_))
        .WillByDefault(DoAll(Assign(&nav_status_, ActionStatus::Running), Return(true)));
    ON_CALL(nav_, status()).WillByDefault(ReturnPointee(&nav_status_));
    ON_CALL(manip_, requestPick(_))
        .WillByDefault(DoAll(Assign(&manip_status_, ActionStatus::Running), Return(true)));
    ON_CALL(manip_, requestPlace(_))
        .WillByDefault(DoAll(Assign(&manip_status_, ActionStatus::Running), Return(true)));
    ON_CALL(manip_, status()).WillByDefault(ReturnPointee(&manip_status_));
    ON_CALL(manip_, lastResult()).WillByDefault(Return(std::nullopt));
    exec_ = std::make_unique<MissionExecutor>(nav_, manip_, clock_, config_);
  }

  void rebuild(MissionExecutorConfig config) {
    config_ = config;
    exec_ = std::make_unique<MissionExecutor>(nav_, manip_, clock_, config_);
  }

  void startFetch() { ASSERT_EQ(exec_->start(makeFetchMission()), std::nullopt); }

  void finishNavigation(ActionStatus outcome = ActionStatus::Succeeded) {
    nav_status_ = outcome;
    exec_->tick();
  }

  void finishManipulation(ActionStatus outcome = ActionStatus::Succeeded) {
    manip_status_ = outcome;
    exec_->tick();
  }

  /// Drives the canonical fetch mission until the pick is in progress.
  void driveToPicking() {
    startFetch();
    finishNavigation();
    exec_->tick();
    ASSERT_EQ(exec_->state(), S::Picking);
  }

  FakeClock clock_;
  NiceMock<MockNavigationClient> nav_;
  NiceMock<MockManipulationClient> manip_;
  ActionStatus nav_status_{ActionStatus::Idle};
  ActionStatus manip_status_{ActionStatus::Idle};
  MissionExecutorConfig config_{seconds{120}, seconds{60}, 1};
  std::unique_ptr<MissionExecutor> exec_;
};

TEST_F(MissionExecutorTest, CompletesFetchMissionIssuingCommandsInOrder) {
  {
    const InSequence seq;
    EXPECT_CALL(nav_, navigateTo(goalTo("station_a")));
    EXPECT_CALL(manip_, requestPick(Field(&GraspTarget::item_id, "klt_42")));
    EXPECT_CALL(nav_, navigateTo(goalTo("station_b")));
    EXPECT_CALL(manip_, requestPlace(Field(&PlaceTarget::item_id, "klt_42")));
    EXPECT_CALL(nav_, navigateTo(goalTo("dock")));
  }
  EXPECT_CALL(nav_, cancel()).Times(0);
  EXPECT_CALL(manip_, cancel()).Times(0);

  clock_.advance(seconds{5});
  startFetch();
  EXPECT_EQ(exec_->state(), S::Navigating);
  exec_->tick();  // still running
  EXPECT_EQ(exec_->state(), S::Navigating);

  clock_.advance(seconds{30});
  finishNavigation();
  EXPECT_EQ(exec_->state(), S::AtStation);
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Picking);
  finishManipulation();
  EXPECT_EQ(exec_->state(), S::AtStation);
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Navigating);
  EXPECT_EQ(exec_->currentLegIndex(), 1U);

  finishNavigation();
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Placing);
  finishManipulation();
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Returning);
  EXPECT_FALSE(exec_->result().has_value());

  clock_.advance(seconds{40});
  finishNavigation();
  EXPECT_EQ(exec_->state(), S::Idle);

  const auto& result = exec_->result();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->mission_id, "order-1");
  EXPECT_EQ(result->outcome, MissionOutcome::Succeeded);
  EXPECT_EQ(result->legs_completed, 2U);
  EXPECT_FALSE(result->fault.has_value());
  EXPECT_EQ(result->duration(), seconds{70});
  EXPECT_EQ(exec_->recoveryAttempts(), 0U);
}

TEST_F(MissionExecutorTest, NavigationOnlyLegSkipsManipulation) {
  Mission m;
  m.id = "patrol";
  m.home = makeWaypoint("dock", 0.0, 0.0);
  m.legs.push_back(MissionLeg{makeWaypoint("a", 1.0, 0.0), NoAction{}});
  EXPECT_CALL(manip_, requestPick(_)).Times(0);
  EXPECT_CALL(manip_, requestPlace(_)).Times(0);

  ASSERT_EQ(exec_->start(m), std::nullopt);
  finishNavigation();
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Returning);
  finishNavigation();
  ASSERT_TRUE(exec_->result().has_value());
  EXPECT_EQ(exec_->result()->outcome, MissionOutcome::Succeeded);
}

TEST_F(MissionExecutorTest, RejectsInvalidMissionWithoutSendingGoals) {
  EXPECT_CALL(nav_, navigateTo(_)).Times(0);
  Mission m = makeFetchMission();
  m.legs.clear();
  EXPECT_THAT(exec_->start(m), ::testing::Optional(HasSubstr("no legs")));
  EXPECT_EQ(exec_->state(), S::Idle);
}

TEST_F(MissionExecutorTest, RejectsStartWhenFirstGoalIsRejected) {
  EXPECT_CALL(nav_, navigateTo(_)).WillOnce(Return(false));
  EXPECT_THAT(exec_->start(makeFetchMission()), ::testing::Optional(HasSubstr("rejected")));
  EXPECT_EQ(exec_->state(), S::Idle);
}

TEST_F(MissionExecutorTest, RejectsSecondStartWhileActive) {
  startFetch();
  EXPECT_THAT(exec_->start(makeFetchMission()), ::testing::Optional(HasSubstr("still active")));
  EXPECT_EQ(exec_->state(), S::Navigating);
}

TEST_F(MissionExecutorTest, NavigationFailureRecoversByReissuingTheSameGoal) {
  EXPECT_CALL(nav_, navigateTo(goalTo("station_a"))).Times(2);
  startFetch();
  finishNavigation(ActionStatus::Failed);
  EXPECT_EQ(exec_->state(), S::Fault);

  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Recovering);
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Navigating);
  EXPECT_EQ(exec_->recoveryAttempts(), 1U);
}

TEST_F(MissionExecutorTest, ExhaustedRecoveryBudgetAbortsWithFailedOutcomeAndFault) {
  startFetch();
  finishNavigation(ActionStatus::Failed);
  exec_->tick();  // -> Recovering
  exec_->tick();  // -> Navigating (attempt 1)
  finishNavigation(ActionStatus::Canceled);
  EXPECT_EQ(exec_->state(), S::Fault);
  exec_->tick();  // budget of 1 exhausted -> Aborted
  EXPECT_EQ(exec_->state(), S::Aborted);

  const auto& result = exec_->result();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->outcome, MissionOutcome::Failed);
  ASSERT_TRUE(result->fault.has_value());
  EXPECT_EQ(result->fault->code, FaultCode::NavigationFailed);
  EXPECT_EQ(result->fault->source, "mission_executor");
  EXPECT_EQ(result->legs_completed, 0U);
}

TEST_F(MissionExecutorTest, ZeroRecoveryBudgetAbortsOnFirstFault) {
  rebuild(MissionExecutorConfig{seconds{120}, seconds{60}, 0});
  startFetch();
  finishNavigation(ActionStatus::Failed);
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Aborted);
}

TEST_F(MissionExecutorTest, NavigationTimeoutCancelsGoalOnlyAfterTheTimeout) {
  startFetch();
  nav_status_ = ActionStatus::Running;

  clock_.advance(seconds{120});
  exec_->tick();  // exactly at the timeout: still allowed
  EXPECT_EQ(exec_->state(), S::Navigating);

  EXPECT_CALL(nav_, cancel()).Times(1);
  clock_.advance(Duration{1});
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Fault);
  exec_->tick();
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Navigating) << "timeout is recoverable";
}

TEST_F(MissionExecutorTest, TimeoutClockRestartsForTheRecoveredGoal) {
  startFetch();
  clock_.advance(seconds{121});
  exec_->tick();  // timeout -> Fault
  exec_->tick();  // -> Recovering
  exec_->tick();  // -> Navigating, new goal
  clock_.advance(seconds{100});
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Navigating) << "100 s into the new goal is within budget";
}

TEST_F(MissionExecutorTest, PickFailureRetriesPickFromStationWithoutRenavigating) {
  driveToPicking();
  PickResult failure;
  failure.outcome = PickOutcome::GraspFailed;
  failure.message = "item slipped";
  ON_CALL(manip_, lastResult()).WillByDefault(Return(failure));
  EXPECT_CALL(nav_, navigateTo(_)).Times(0);
  EXPECT_CALL(manip_, requestPick(_)).Times(1);

  finishManipulation(ActionStatus::Failed);
  EXPECT_EQ(exec_->state(), S::Fault);
  exec_->tick();
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::AtStation);
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Picking);
}

TEST_F(MissionExecutorTest, PickFailureMessageCarriesManipulationResult) {
  rebuild(MissionExecutorConfig{seconds{120}, seconds{60}, 0});
  driveToPicking();
  PickResult failure;
  failure.outcome = PickOutcome::GraspFailed;
  failure.message = "item slipped";
  ON_CALL(manip_, lastResult()).WillByDefault(Return(failure));

  finishManipulation(ActionStatus::Failed);
  exec_->tick();
  ASSERT_TRUE(exec_->result().has_value());
  ASSERT_TRUE(exec_->result()->fault.has_value());
  EXPECT_EQ(exec_->result()->fault->code, FaultCode::ManipulationFailed);
  EXPECT_THAT(exec_->result()->fault->message, HasSubstr("GRASP_FAILED: item slipped"));
}

TEST_F(MissionExecutorTest, ManipulationTimeoutStopsTheArm) {
  driveToPicking();
  EXPECT_CALL(manip_, cancel()).Times(1);
  clock_.advance(seconds{61});
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Fault);
}

TEST_F(MissionExecutorTest, CriticalArmFaultDuringPickStopsArmAndAbortsImmediately) {
  driveToPicking();
  EXPECT_CALL(manip_, cancel()).Times(1);
  EXPECT_CALL(nav_, navigateTo(_)).Times(0);  // base must not move afterwards

  const Fault fault{FaultCode::ArmJointFrozen, FaultSeverity::Critical, "health_monitor",
                    "elbow joint frozen", clock_.now()};
  EXPECT_TRUE(exec_->reportFault(fault));
  EXPECT_EQ(exec_->state(), S::Aborted);
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Aborted);

  const auto& result = exec_->result();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->outcome, MissionOutcome::Failed);
  ASSERT_TRUE(result->fault.has_value());
  EXPECT_EQ(result->fault->code, FaultCode::ArmJointFrozen);
  EXPECT_EQ(result->fault->source, "health_monitor");
}

TEST_F(MissionExecutorTest, RecoverableExternalFaultCancelsNavigation) {
  startFetch();
  EXPECT_CALL(nav_, cancel()).Times(1);
  EXPECT_TRUE(
      exec_->reportFault(Fault{FaultCode::LocalizationUntrusted, FaultSeverity::Recoverable,
                               "localization", "covariance above threshold", clock_.now()}));
  EXPECT_EQ(exec_->state(), S::Fault);
}

TEST_F(MissionExecutorTest, CriticalFaultWhileAlreadyInFaultAborts) {
  startFetch();
  finishNavigation(ActionStatus::Failed);
  ASSERT_EQ(exec_->state(), S::Fault);
  EXPECT_TRUE(exec_->reportFault(Fault{FaultCode::WheelJointFault, FaultSeverity::Critical,
                                       "health_monitor", "", clock_.now()}));
  EXPECT_EQ(exec_->state(), S::Aborted);
  ASSERT_TRUE(exec_->result().has_value());
  ASSERT_TRUE(exec_->result()->fault.has_value());
  EXPECT_EQ(exec_->result()->fault->code, FaultCode::WheelJointFault);
}

TEST_F(MissionExecutorTest, OperatorAbortCancelsMotionAndReportsCanceled) {
  startFetch();
  EXPECT_CALL(nav_, cancel()).Times(1);
  EXPECT_TRUE(exec_->abort());
  EXPECT_EQ(exec_->state(), S::Aborted);
  ASSERT_TRUE(exec_->result().has_value());
  EXPECT_EQ(exec_->result()->outcome, MissionOutcome::Canceled);
  EXPECT_FALSE(exec_->result()->fault.has_value());
}

TEST_F(MissionExecutorTest, AbortAndFaultAreIgnoredWithoutActiveMission) {
  EXPECT_CALL(nav_, cancel()).Times(0);
  EXPECT_FALSE(exec_->abort());
  EXPECT_FALSE(exec_->reportFault(Fault{}));
  EXPECT_EQ(exec_->state(), S::Idle);
}

TEST_F(MissionExecutorTest, RejectedPickRequestBecomesRecoverableFault) {
  startFetch();
  finishNavigation();
  EXPECT_CALL(manip_, requestPick(_)).WillOnce(Return(false)).WillOnce(Return(true));
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Fault);
  exec_->tick();
  exec_->tick();
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Picking);
}

TEST_F(MissionExecutorTest, RejectedNextGoalKeepsLegIndexAndDoesNotDoubleCountLegs) {
  startFetch();
  finishNavigation();
  exec_->tick();
  finishManipulation();  // pick done, AtStation leg 0

  EXPECT_CALL(nav_, navigateTo(goalTo("station_b")))
      .WillOnce(Return(false))
      .WillOnce(DoAll(Assign(&nav_status_, ActionStatus::Running), Return(true)));
  EXPECT_CALL(manip_, requestPick(_)).Times(0);  // the pick must not be repeated
  EXPECT_CALL(nav_, navigateTo(goalTo("dock"))).Times(1);

  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Fault);
  EXPECT_EQ(exec_->currentLegIndex(), 0U);
  exec_->tick();  // Recovering
  exec_->tick();  // resume AtStation
  exec_->tick();  // retry navigation to station_b
  EXPECT_EQ(exec_->state(), S::Navigating);
  EXPECT_EQ(exec_->currentLegIndex(), 1U);

  finishNavigation();
  exec_->tick();
  finishManipulation();
  exec_->tick();
  finishNavigation();
  ASSERT_TRUE(exec_->result().has_value());
  EXPECT_EQ(exec_->result()->outcome, MissionOutcome::Succeeded);
  EXPECT_EQ(exec_->result()->legs_completed, 2U);
}

TEST_F(MissionExecutorTest, FailureWhileReturningReissuesHomeGoal) {
  startFetch();
  finishNavigation();
  exec_->tick();
  finishManipulation();
  exec_->tick();
  finishNavigation();
  exec_->tick();
  finishManipulation();
  exec_->tick();
  ASSERT_EQ(exec_->state(), S::Returning);

  EXPECT_CALL(nav_, navigateTo(goalTo("dock"))).Times(1);
  finishNavigation(ActionStatus::Failed);
  exec_->tick();
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Returning);
}

TEST_F(MissionExecutorTest, RejectedRecoveryGoalAbortsMission) {
  startFetch();
  finishNavigation(ActionStatus::Failed);
  exec_->tick();  // Recovering
  EXPECT_CALL(nav_, navigateTo(_)).WillOnce(Return(false));
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Aborted);
  ASSERT_TRUE(exec_->result().has_value());
  EXPECT_EQ(exec_->result()->outcome, MissionOutcome::Failed);
}

TEST_F(MissionExecutorTest, UnacknowledgedGoalIsWaitedOnNotTreatedAsFinished) {
  // Contract of INavigationClient: after an accepted goal, status() may be Idle
  // until the server acknowledges it. That must count as "in progress".
  EXPECT_CALL(nav_, navigateTo(_))
      .WillOnce(DoAll(Assign(&nav_status_, ActionStatus::Idle), Return(true)));
  startFetch();
  exec_->tick();
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Navigating);
  clock_.advance(seconds{121});
  exec_->tick();
  EXPECT_EQ(exec_->state(), S::Fault) << "a goal that is never acknowledged still times out";
}

TEST(MissionExecutorConfig, NegativeTimeoutsAreRejected) {
  const FakeClock clock;
  NiceMock<MockNavigationClient> nav;
  NiceMock<MockManipulationClient> manip;
  EXPECT_THROW(
      MissionExecutor(nav, manip, clock, MissionExecutorConfig{seconds{-1}, seconds{60}, 1}),
      std::invalid_argument);
  EXPECT_THROW(
      MissionExecutor(nav, manip, clock, MissionExecutorConfig{seconds{120}, seconds{-1}, 1}),
      std::invalid_argument);
  EXPECT_NO_THROW(
      MissionExecutor(nav, manip, clock, MissionExecutorConfig{seconds{0}, seconds{0}, 0}));
}

// One executor per robot: copying would duplicate mission state over shared ports.
static_assert(!std::is_copy_constructible_v<MissionExecutor>);
static_assert(!std::is_copy_assignable_v<MissionExecutor>);
static_assert(!std::is_move_constructible_v<MissionExecutor>);

TEST_F(MissionExecutorTest, NewMissionCanStartAfterAbort) {
  startFetch();
  ASSERT_TRUE(exec_->abort());
  startFetch();
  EXPECT_EQ(exec_->state(), S::Navigating);
  EXPECT_FALSE(exec_->result().has_value()) << "previous result cleared on start";
  EXPECT_EQ(exec_->recoveryAttempts(), 0U);
}

}  // namespace
}  // namespace robot_core
