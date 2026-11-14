// Consistency between robot_interfaces and robot_core: the ROS adapters convert
// enums by value, so every mirrored constant must equal the core enumerator.
#include <cstdint>

#include <gtest/gtest.h>

#include "robot_core/perception_types.hpp"
#include "robot_interfaces/msg/mission_leg.hpp"
#include "robot_interfaces/msg/object_class.hpp"
#include "robot_interfaces/msg/pick_command.hpp"
#include "robot_interfaces/msg/tracked_object.hpp"
#include "robot_interfaces/msg/tracked_object_array.hpp"

namespace {

using robot_core::ObjectClass;
using robot_core::TrackSource;
using MsgClass = robot_interfaces::msg::ObjectClass;
using MsgTrack = robot_interfaces::msg::TrackedObject;

constexpr std::uint8_t value(ObjectClass c) {
  return static_cast<std::uint8_t>(c);
}
constexpr std::uint8_t value(TrackSource s) {
  return static_cast<std::uint8_t>(s);
}

static_assert(MsgClass::UNKNOWN == value(ObjectClass::Unknown));
static_assert(MsgClass::PERSON == value(ObjectClass::Person));
static_assert(MsgClass::PALLET == value(ObjectClass::Pallet));
static_assert(MsgClass::KLT_BOX == value(ObjectClass::KltBox));
static_assert(MsgClass::AMR == value(ObjectClass::Amr));
static_assert(MsgClass::SHELF_ITEM == value(ObjectClass::ShelfItem));

static_assert(MsgTrack::SOURCE_CAMERA == value(TrackSource::Camera));
static_assert(MsgTrack::SOURCE_LIDAR == value(TrackSource::Lidar));
static_assert(MsgTrack::SOURCE_FUSED == value(TrackSource::Fused));

TEST(Interfaces, ObjectClassNamesMatchCore) {
  // The core names are the vocabulary used in logs and docs.
  EXPECT_EQ(robot_core::toString(static_cast<ObjectClass>(MsgClass::PERSON)), "person");
  EXPECT_EQ(robot_core::toString(static_cast<ObjectClass>(MsgClass::SHELF_ITEM)), "shelf_item");
}

TEST(Interfaces, TrackedObjectDefaultsAreZero) {
  const MsgTrack track;
  EXPECT_EQ(track.track_id, 0U);
  EXPECT_EQ(track.object_class.value, MsgClass::UNKNOWN);
  EXPECT_EQ(track.position_covariance.size(), 4U);
  for (const double c : track.position_covariance) {
    EXPECT_EQ(c, 0.0);
  }
}

TEST(Interfaces, ArrayAndNestedTypesCompose) {
  robot_interfaces::msg::TrackedObjectArray array;
  array.tracks.resize(2);
  array.tracks[1].track_id = 7;
  ASSERT_EQ(array.tracks.size(), 2U);
  EXPECT_EQ(array.tracks[1].track_id, 7U);

  robot_interfaces::msg::MissionLeg leg;
  leg.action = robot_interfaces::msg::MissionLeg::ACTION_PICK;
  leg.station.id = "station_a";
  EXPECT_EQ(leg.station.id, "station_a");
  EXPECT_NE(robot_interfaces::msg::PickCommand::PICK, robot_interfaces::msg::PickCommand::PLACE);
}

}  // namespace
