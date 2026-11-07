// Enum names appear in logs, diagnostics and scenario JSON results, so every
// enumerator must have a defined name and no two may collide.
#include <cstdint>
#include <set>
#include <string>
#include <string_view>

#include <gtest/gtest.h>

#include "robot_core/action_status.hpp"
#include "robot_core/fault.hpp"
#include "robot_core/manipulation_types.hpp"
#include "robot_core/mission.hpp"
#include "robot_core/perception_types.hpp"

namespace robot_core {
namespace {

/// Checks toString for enumerators 0..last (enums here are contiguous from 0).
template <typename Enum>
void expectUniqueDefinedNames(Enum last, std::string_view invalid_marker) {
  std::set<std::string> names;
  const auto count = static_cast<int>(last) + 1;
  for (int i = 0; i < count; ++i) {
    const auto value = static_cast<Enum>(i);
    const std::string name{toString(value)};
    EXPECT_NE(name, invalid_marker) << "enumerator " << i << " has no name";
    EXPECT_TRUE(names.insert(name).second) << "duplicate name '" << name << "'";
  }
}

TEST(EnumNames, ActionStatus) {
  expectUniqueDefinedNames(ActionStatus::Canceled, "INVALID");
}

TEST(EnumNames, FaultCode) {
  expectUniqueDefinedNames(FaultCode::CommandRejected, "INVALID");
}

TEST(EnumNames, FaultSeverity) {
  expectUniqueDefinedNames(FaultSeverity::Critical, "INVALID");
}

TEST(EnumNames, ObjectClass) {
  expectUniqueDefinedNames(ObjectClass::ShelfItem, "invalid");
}

TEST(EnumNames, TrackSource) {
  expectUniqueDefinedNames(TrackSource::Fused, "invalid");
}

TEST(EnumNames, PickOutcome) {
  expectUniqueDefinedNames(PickOutcome::Canceled, "INVALID");
}

TEST(EnumNames, MissionOutcome) {
  expectUniqueDefinedNames(MissionOutcome::Canceled, "INVALID");
}

TEST(ActionStatus, OnlyFinishedStatusesAreTerminal) {
  EXPECT_FALSE(isTerminal(ActionStatus::Idle));
  EXPECT_FALSE(isTerminal(ActionStatus::Running));
  EXPECT_TRUE(isTerminal(ActionStatus::Succeeded));
  EXPECT_TRUE(isTerminal(ActionStatus::Failed));
  EXPECT_TRUE(isTerminal(ActionStatus::Canceled));
}

}  // namespace
}  // namespace robot_core
