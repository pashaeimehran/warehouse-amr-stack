#include <chrono>
#include <limits>

#include <gtest/gtest.h>

#include "fixtures.hpp"
#include "robot_core/fault.hpp"
#include "robot_core/manipulation_types.hpp"

namespace robot_core {
namespace {

using test::makeGrasp;
using test::makePlace;

TEST(GraspTarget, FixtureIsValid) {
  EXPECT_TRUE(isValid(makeGrasp("item")));
}

TEST(GraspTarget, RejectsMissingIdsAndNonUnitOrientation) {
  GraspTarget g = makeGrasp("");
  EXPECT_FALSE(isValid(g));

  g = makeGrasp("item");
  g.frame_id.clear();
  EXPECT_FALSE(isValid(g));

  g = makeGrasp("item");
  g.grasp_pose.orientation = Quaternion{2.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(isValid(g));

  g = makeGrasp("item");
  g.grasp_pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(isValid(g));
}

TEST(GraspTarget, RejectsNonPositiveApproachOrOpening) {
  GraspTarget g = makeGrasp("item");
  g.approach_distance = 0.0;
  EXPECT_FALSE(isValid(g));
  g = makeGrasp("item");
  g.gripper_opening = -0.01;
  EXPECT_FALSE(isValid(g));
}

TEST(PlaceTarget, Validity) {
  EXPECT_TRUE(isValid(makePlace("item")));
  EXPECT_FALSE(isValid(makePlace("")));
  PlaceTarget p = makePlace("item");
  p.place_pose.orientation = Quaternion{0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(isValid(p));
}

TEST(PickResult, SucceededOnlyForSucceededOutcome) {
  PickResult r;
  r.outcome = PickOutcome::Succeeded;
  EXPECT_TRUE(r.succeeded());
  for (const auto outcome : {PickOutcome::PlanningFailed, PickOutcome::ExecutionFailed,
                             PickOutcome::GraspFailed, PickOutcome::Canceled}) {
    r.outcome = outcome;
    EXPECT_FALSE(r.succeeded()) << toString(outcome);
  }
}

TEST(PickResult, DefaultsToNotSucceeded) {
  // A default-constructed result must never read as a success.
  EXPECT_FALSE(PickResult{}.succeeded());
}

TEST(Fault, CodesAndSeveritiesHaveStableNames) {
  EXPECT_EQ(toString(FaultCode::ArmJointFrozen), "ARM_JOINT_FROZEN");
  EXPECT_EQ(toString(FaultCode::LocalizationUntrusted), "LOCALIZATION_UNTRUSTED");
  EXPECT_EQ(toString(FaultSeverity::Critical), "CRITICAL");
}

}  // namespace
}  // namespace robot_core
