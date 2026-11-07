add_executable(robot_core_tests
  test_clock.cpp
  test_enum_names.cpp
  test_geometry.cpp
  test_perception_types.cpp
  test_manipulation_types.cpp
  test_mission.cpp
  test_mission_state_machine.cpp
  test_mission_executor.cpp
  test_task_queue.cpp
  test_latest_sample.cpp
)
target_link_libraries(robot_core_tests PRIVATE
  robot_core::robot_core GTest::gmock GTest::gtest_main wmm_warnings)

# One CTest entry per test case, so failures are reported individually.
# PRE_TEST: tests are listed when ctest runs, not at build time, so building
# never executes an instrumented binary (TSan needs a wrapper on some kernels).
# Labels: "unit" here; GPU-dependent tests (from week 2) must use the label
# "gpu" and live in their own executables; GPU-free presets exclude that label.
gtest_discover_tests(robot_core_tests
  DISCOVERY_MODE PRE_TEST
  DISCOVERY_TIMEOUT 30
  PROPERTIES TIMEOUT 30 LABELS unit)
