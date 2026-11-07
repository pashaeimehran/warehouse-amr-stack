#include <chrono>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "robot_core/clock.hpp"
#include "robot_core/mission_state_machine.hpp"

namespace robot_core {
namespace {

using E = MissionEvent;
using S = MissionState;
using std::chrono::milliseconds;

/// Event sequence that brings a fresh state machine into `s`.
std::vector<E> pathTo(S s) {
  switch (s) {
    case S::Idle:
      return {};
    case S::Navigating:
      return {E::Start};
    case S::AtStation:
      return {E::Start, E::ArrivedAtStation};
    case S::Picking:
      return {E::Start, E::ArrivedAtStation, E::BeginPick};
    case S::Placing:
      return {E::Start, E::ArrivedAtStation, E::BeginPlace};
    case S::Returning:
      return {E::Start, E::ArrivedAtStation, E::ReturnHome};
    case S::Fault:
      return {E::Start, E::FaultDetected};
    case S::Recovering:
      return {E::Start, E::FaultDetected, E::BeginRecovery};
    case S::Aborted:
      return {E::Start, E::Abort};
  }
  return {};
}

void driveTo(MissionStateMachine& fsm, S s) {
  for (const E e : pathTo(s)) {
    ASSERT_TRUE(fsm.handle(e)) << "setup failed at " << toString(e);
  }
  ASSERT_EQ(fsm.state(), s);
}

/// The specification, written independently of the implementation. Every pair
/// not listed here must be rejected. Recovering is reached from Navigating, so
/// RecoverySucceeded resumes Navigating.
std::optional<S> expectedTarget(S s, E e) {
  struct Rule {
    S from;
    E event;
    S to;
  };
  static const std::vector<Rule> legal_rules{
      {S::Idle, E::Start, S::Navigating},
      {S::Navigating, E::ArrivedAtStation, S::AtStation},
      {S::Navigating, E::FaultDetected, S::Fault},
      {S::Navigating, E::Abort, S::Aborted},
      {S::AtStation, E::BeginPick, S::Picking},
      {S::AtStation, E::BeginPlace, S::Placing},
      {S::AtStation, E::ProceedToNextStation, S::Navigating},
      {S::AtStation, E::ReturnHome, S::Returning},
      {S::AtStation, E::FaultDetected, S::Fault},
      {S::AtStation, E::Abort, S::Aborted},
      {S::Picking, E::ActionCompleted, S::AtStation},
      {S::Picking, E::FaultDetected, S::Fault},
      {S::Picking, E::Abort, S::Aborted},
      {S::Placing, E::ActionCompleted, S::AtStation},
      {S::Placing, E::FaultDetected, S::Fault},
      {S::Placing, E::Abort, S::Aborted},
      {S::Returning, E::ArrivedHome, S::Idle},
      {S::Returning, E::FaultDetected, S::Fault},
      {S::Returning, E::Abort, S::Aborted},
      {S::Fault, E::BeginRecovery, S::Recovering},
      {S::Fault, E::Abort, S::Aborted},
      {S::Recovering, E::RecoverySucceeded, S::Navigating},
      {S::Recovering, E::RecoveryFailed, S::Aborted},
      {S::Recovering, E::FaultDetected, S::Fault},
      {S::Recovering, E::Abort, S::Aborted},
      {S::Aborted, E::Reset, S::Idle},
  };
  for (const auto& rule : legal_rules) {
    if (rule.from == s && rule.event == e) {
      return rule.to;
    }
  }
  return std::nullopt;
}

class TransitionTableTest : public ::testing::TestWithParam<std::tuple<S, E>> {};

TEST_P(TransitionTableTest, MatchesSpecification) {
  const auto [state, event] = GetParam();
  const FakeClock clock;
  MissionStateMachine fsm{clock};
  driveTo(fsm, state);

  const auto expected = expectedTarget(state, event);
  EXPECT_EQ(fsm.canHandle(event), expected.has_value());
  EXPECT_EQ(fsm.state(), state) << "canHandle must not change state";

  const bool accepted = fsm.handle(event);
  EXPECT_EQ(accepted, expected.has_value());
  EXPECT_EQ(fsm.state(), expected.value_or(state))
      << "illegal events must leave the state unchanged";
}

INSTANTIATE_TEST_SUITE_P(AllStatesAndEvents, TransitionTableTest,
                         ::testing::Combine(::testing::ValuesIn(kAllMissionStates),
                                            ::testing::ValuesIn(kAllMissionEvents)),
                         [](const auto& name_info) {
                           return std::string{toString(std::get<0>(name_info.param))} + "_" +
                                  std::string{toString(std::get<1>(name_info.param))};
                         });

TEST(MissionStateMachine, StartsIdleAndInactive) {
  const FakeClock clock;
  const MissionStateMachine fsm{clock};
  EXPECT_EQ(fsm.state(), S::Idle);
  EXPECT_FALSE(fsm.isMissionActive());
  EXPECT_EQ(fsm.resumeState(), std::nullopt);
}

TEST(MissionStateMachine, FullFetchMissionPath) {
  const FakeClock clock;
  MissionStateMachine fsm{clock};
  const std::vector<std::pair<E, S>> steps{
      {E::Start, S::Navigating},
      {E::ArrivedAtStation, S::AtStation},
      {E::BeginPick, S::Picking},
      {E::ActionCompleted, S::AtStation},
      {E::ProceedToNextStation, S::Navigating},
      {E::ArrivedAtStation, S::AtStation},
      {E::BeginPlace, S::Placing},
      {E::ActionCompleted, S::AtStation},
      {E::ReturnHome, S::Returning},
      {E::ArrivedHome, S::Idle},
  };
  for (const auto& [event, expected] : steps) {
    ASSERT_TRUE(fsm.handle(event)) << toString(event);
    ASSERT_EQ(fsm.state(), expected);
  }
  EXPECT_FALSE(fsm.isMissionActive());
}

TEST(MissionStateMachine, ActiveStatesAreExactlyTheMissionStates) {
  for (const S s : kAllMissionStates) {
    const FakeClock clock;
    MissionStateMachine fsm{clock};
    driveTo(fsm, s);
    EXPECT_EQ(fsm.isMissionActive(), s != S::Idle && s != S::Aborted) << toString(s);
  }
}

struct ResumeCase {
  S interrupted;
  S resumed;
};

class ResumeTest : public ::testing::TestWithParam<ResumeCase> {};

TEST_P(ResumeTest, RecoveryResumesTheInterruptedActivity) {
  const auto [interrupted, resumed] = GetParam();
  const FakeClock clock;
  MissionStateMachine fsm{clock};
  driveTo(fsm, interrupted);

  ASSERT_TRUE(fsm.handle(E::FaultDetected));
  EXPECT_EQ(fsm.resumeState(), resumed);
  ASSERT_TRUE(fsm.handle(E::BeginRecovery));
  EXPECT_EQ(fsm.resumeState(), resumed);
  ASSERT_TRUE(fsm.handle(E::RecoverySucceeded));
  EXPECT_EQ(fsm.state(), resumed);
  EXPECT_EQ(fsm.resumeState(), std::nullopt);
}

INSTANTIATE_TEST_SUITE_P(
    InterruptedStates, ResumeTest,
    ::testing::Values(ResumeCase{S::Navigating, S::Navigating},
                      ResumeCase{S::AtStation, S::AtStation},
                      ResumeCase{S::Picking, S::AtStation},  // never resume mid-motion
                      ResumeCase{S::Placing, S::AtStation}, ResumeCase{S::Returning, S::Returning}),
    [](const auto& name_info) { return std::string{toString(name_info.param.interrupted)}; });

TEST(MissionStateMachine, FaultDuringRecoveryKeepsOriginalResumeTarget) {
  const FakeClock clock;
  MissionStateMachine fsm{clock};
  driveTo(fsm, S::Returning);
  ASSERT_TRUE(fsm.handle(E::FaultDetected));
  ASSERT_TRUE(fsm.handle(E::BeginRecovery));
  ASSERT_TRUE(fsm.handle(E::FaultDetected));  // recovery itself faulted
  EXPECT_EQ(fsm.resumeState(), S::Returning);
  ASSERT_TRUE(fsm.handle(E::BeginRecovery));
  ASSERT_TRUE(fsm.handle(E::RecoverySucceeded));
  EXPECT_EQ(fsm.state(), S::Returning);
}

TEST(MissionStateMachine, AbortFromFaultClearsResumeState) {
  const FakeClock clock;
  MissionStateMachine fsm{clock};
  driveTo(fsm, S::Fault);
  ASSERT_TRUE(fsm.handle(E::Abort));
  EXPECT_EQ(fsm.resumeState(), std::nullopt);
  ASSERT_TRUE(fsm.handle(E::Reset));
  EXPECT_EQ(fsm.state(), S::Idle);
}

TEST(MissionStateMachine, TimeInStateUsesInjectedClockAndResetsOnTransition) {
  FakeClock clock;
  MissionStateMachine fsm{clock};
  clock.advance(milliseconds{400});
  EXPECT_EQ(fsm.timeInState(), milliseconds{400});

  ASSERT_TRUE(fsm.handle(E::Start));
  EXPECT_EQ(fsm.timeInState(), Duration::zero());
  clock.advance(milliseconds{1500});
  EXPECT_EQ(fsm.timeInState(), milliseconds{1500});
}

TEST(MissionStateMachine, IllegalEventDoesNotResetTimeInState) {
  FakeClock clock;
  MissionStateMachine fsm{clock};
  ASSERT_TRUE(fsm.handle(E::Start));
  clock.advance(milliseconds{200});
  EXPECT_FALSE(fsm.handle(E::BeginPick));
  EXPECT_EQ(fsm.timeInState(), milliseconds{200});
}

TEST(MissionStateMachine, ListenerSeesEveryAcceptedTransitionWithTimestamp) {
  FakeClock clock;
  MissionStateMachine fsm{clock};
  std::vector<MissionTransition> log;
  fsm.setTransitionListener([&log](const MissionTransition& t) { log.push_back(t); });

  clock.advance(milliseconds{10});
  ASSERT_TRUE(fsm.handle(E::Start));
  EXPECT_FALSE(fsm.handle(E::ArrivedHome));  // illegal: not reported
  clock.advance(milliseconds{20});
  ASSERT_TRUE(fsm.handle(E::ArrivedAtStation));

  ASSERT_EQ(log.size(), 2U);
  EXPECT_EQ(log[0].from, S::Idle);
  EXPECT_EQ(log[0].to, S::Navigating);
  EXPECT_EQ(log[0].event, E::Start);
  EXPECT_EQ(log[0].stamp, Timestamp{milliseconds{10}});
  EXPECT_EQ(log[1].from, S::Navigating);
  EXPECT_EQ(log[1].to, S::AtStation);
  EXPECT_EQ(log[1].stamp, Timestamp{milliseconds{30}});
}

TEST(MissionStateMachine, StateNamesMatchRoadmapVocabulary) {
  EXPECT_EQ(toString(S::AtStation), "AT_STATION");
  EXPECT_EQ(toString(S::Recovering), "RECOVERING");
  EXPECT_EQ(toString(E::RecoverySucceeded), "RECOVERY_SUCCEEDED");
  for (const S s : kAllMissionStates) {
    EXPECT_NE(toString(s), "INVALID");
  }
  for (const E e : kAllMissionEvents) {
    EXPECT_NE(toString(e), "INVALID");
  }
}

}  // namespace
}  // namespace robot_core
