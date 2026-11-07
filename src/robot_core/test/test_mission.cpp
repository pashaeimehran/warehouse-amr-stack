#include <chrono>
#include <limits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "fixtures.hpp"
#include "robot_core/mission.hpp"

namespace robot_core {
namespace {

using test::makeFetchMission;
using test::makeGrasp;
using test::makePlace;
using test::makeWaypoint;
using ::testing::HasSubstr;
using ::testing::Optional;

TEST(ValidateMission, AcceptsCanonicalFetchMission) {
  EXPECT_EQ(validateMission(makeFetchMission()), std::nullopt);
}

TEST(ValidateMission, AcceptsNavigationOnlyMission) {
  Mission m;
  m.id = "patrol";
  m.home = makeWaypoint("dock", 0.0, 0.0);
  m.legs.push_back(MissionLeg{makeWaypoint("a", 1.0, 0.0), NoAction{}});
  EXPECT_EQ(validateMission(m), std::nullopt);
}

TEST(ValidateMission, RejectsEmptyIdAndNoLegs) {
  Mission m = makeFetchMission();
  m.id.clear();
  EXPECT_THAT(validateMission(m), Optional(HasSubstr("id is empty")));

  m = makeFetchMission();
  m.legs.clear();
  EXPECT_THAT(validateMission(m), Optional(HasSubstr("no legs")));
}

TEST(ValidateMission, RejectsBadWaypoints) {
  Mission m = makeFetchMission();
  m.home.id.clear();
  EXPECT_THAT(validateMission(m), Optional(HasSubstr("home waypoint")));

  m = makeFetchMission();
  m.legs[1].station.pose.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THAT(validateMission(m), Optional(HasSubstr("leg 1")));

  m = makeFetchMission();
  m.legs[0].station.position_tolerance_m = 0.0;
  EXPECT_THAT(validateMission(m), Optional(HasSubstr("position tolerance")));

  m = makeFetchMission();
  m.legs[0].station.yaw_tolerance_rad = -0.1;
  EXPECT_THAT(validateMission(m), Optional(HasSubstr("yaw tolerance")));
}

TEST(ValidateMission, RejectsInvalidActionTargets) {
  Mission m = makeFetchMission();
  std::get<PickAction>(m.legs[0].action).target.approach_distance = 0.0;
  EXPECT_THAT(validateMission(m), Optional(HasSubstr("invalid grasp target")));

  m = makeFetchMission();
  std::get<PlaceAction>(m.legs[1].action).target.place_pose.orientation.w = 0.0;
  EXPECT_THAT(validateMission(m), Optional(HasSubstr("invalid place target")));
}

TEST(ValidateMission, RejectsSecondPickWhileHolding) {
  Mission m = makeFetchMission("a");
  m.legs.insert(m.legs.begin() + 1,
                MissionLeg{makeWaypoint("station_c", 2.0, 2.0), PickAction{makeGrasp("b")}});
  EXPECT_THAT(validateMission(m), Optional(HasSubstr("while holding 'a'")));
}

TEST(ValidateMission, RejectsPlaceWithEmptyGripperOrWrongItem) {
  Mission m;
  m.id = "bad";
  m.home = makeWaypoint("dock", 0.0, 0.0);
  m.legs.push_back(MissionLeg{makeWaypoint("b", 1.0, 0.0), PlaceAction{makePlace("x")}});
  EXPECT_THAT(validateMission(m), Optional(HasSubstr("empty gripper")));

  m = makeFetchMission("a");
  std::get<PlaceAction>(m.legs[1].action).target.item_id = "b";
  EXPECT_THAT(validateMission(m), Optional(HasSubstr("place of 'b' while holding 'a'")));
}

TEST(ValidateMission, AllowsPickPlacePickPlaceSequence) {
  Mission m = makeFetchMission("a");
  m.legs.push_back(MissionLeg{makeWaypoint("c", 1.0, 1.0), PickAction{makeGrasp("b")}});
  m.legs.push_back(MissionLeg{makeWaypoint("d", 2.0, 1.0), PlaceAction{makePlace("b")}});
  EXPECT_EQ(validateMission(m), std::nullopt);
}

TEST(MissionResult, DurationIsFinishMinusStart) {
  MissionResult r;
  r.started_at = Timestamp{std::chrono::seconds{10}};
  r.finished_at = Timestamp{std::chrono::seconds{95}};
  EXPECT_EQ(r.duration(), std::chrono::seconds{85});
  EXPECT_EQ(toString(MissionOutcome::Canceled), "CANCELED");
}

}  // namespace
}  // namespace robot_core
