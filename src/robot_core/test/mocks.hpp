#pragma once

#include <optional>

#include <gmock/gmock.h>

#include "robot_core/manipulation_client.hpp"
#include "robot_core/navigation_client.hpp"

namespace robot_core::test {

class MockNavigationClient : public INavigationClient {
 public:
  MOCK_METHOD(bool, navigateTo, (const Waypoint& goal), (override));
  MOCK_METHOD(ActionStatus, status, (), (const, override));
  MOCK_METHOD(void, cancel, (), (override));
};

class MockManipulationClient : public IManipulationClient {
 public:
  MOCK_METHOD(bool, requestPick, (const GraspTarget& target), (override));
  MOCK_METHOD(bool, requestPlace, (const PlaceTarget& target), (override));
  MOCK_METHOD(ActionStatus, status, (), (const, override));
  MOCK_METHOD(std::optional<PickResult>, lastResult, (), (const, override));
  MOCK_METHOD(void, cancel, (), (override));
};

}  // namespace robot_core::test
