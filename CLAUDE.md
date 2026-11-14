# CLAUDE.md

Guidance for working in this repository (`warehouse-amr-stack`).

**The specification is `Warehouse Mobile Manipulator Roadmap.md`.** Follow its
milestone order, its done-when criteria and its repository layout. Do not
implement a later milestone's work early. If the roadmap and this file
disagree, the roadmap wins; then update this file.

## Project constraints

- Simulation only (Gazebo Harmonic), no physical robot. Never call anything
  "real time", "production ready" or "safety certified".
- No code, rosbags or data from the author's LHFT student job, ever.
- Target stack: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, C++17 (selected
  C++20 only where a ROS package needs it), Python 3.12 for tooling.
- Hardware: RTX 2060 locally (6 GB VRAM, small detectors), FAU TinyGPU for
  training. CI runners have no GPU: perception tests use the mock backend.
- Development environment: WSL 2 on Windows 11; the repo lives on `/mnt/d`
  (slow file I/O, so prefer incremental builds).
- Do not install system packages, and do not run `git commit`, `push`,
  `merge`, `rebase` or `tag`, without the user's explicit approval.

## Architecture

The central rule: **learned components answer perceptual questions; every
decision that affects safety, timing or system state is deterministic,
bounded and unit tested.** Learned output is an untrusted, timestamped,
confidence-weighted input that deterministic code validates.

Layering (see the roadmap's repository structure):

- `src/<name>_core/`, `src/camera_geometry/`: ROS-free domain libraries
  (plain CMake, C++17). **They must never include or link ROS** (`rclcpp`,
  messages, `tf2`, …). This is the strongest structural rule in the repo.
- `src/<adapter>/`: thin ROS 2 nodes (ament_cmake) that convert messages to
  core types and call core logic. No domain logic in adapters.
- `tools/`: everything that never runs on the robot (training, datasets,
  analysis, CI scripts). Ultralytics code stays in `tools/` only.
- `src/fault_injector/`: out of band; never compiled into production nodes
  or included in the runtime launch.
- Safety logic lives in `safety_supervisor` as an explicit FSM, never inside
  the behavior tree or the mission executor.

### `robot_core` (v0.1, implemented)

`src/robot_core/include/robot_core/`:

| Header | Purpose |
| --- | --- |
| `time.hpp`, `clock.hpp` | `Timestamp` on a dedicated `RobotTime` tag; `IClock`, `SystemClock`, `FakeClock`; `isFresh()` |
| `geometry.hpp` | `Pose2D`, `Pose3D`, `Quaternion`, angle wrapping to [-π, π) |
| `perception_types.hpp` | `Detection`, `BoundingBox2D`, `TrackedObject`, `ObjectClass` |
| `manipulation_types.hpp` | `GraspTarget`, `PlaceTarget`, `PickResult` |
| `fault.hpp` | `Fault`, `FaultCode`, `FaultSeverity` |
| `mission.hpp` | `Waypoint`, `Mission`, `StationAction` variant, `MissionResult`, `validateMission()` |
| `mission_state_machine.hpp` | enum and table FSM: IDLE, NAVIGATING, AT_STATION, PICKING, PLACING, RETURNING, FAULT, RECOVERING, ABORTED |
| `task_queue.hpp`, `latest_sample.hpp` | thread-safe bounded MPMC queue; newest-value-wins slot with staleness |
| `navigation_client.hpp`, `manipulation_client.hpp` | non-blocking ports (request, then poll `status()`) |
| `mission_executor.hpp` | tick-driven mission logic over the ports, with timeouts and a recovery budget |

Design rationale and the state diagram: `docs/01-core-design.md`. When you
change the FSM, update the table in `mission_state_machine.cpp`, the
independent spec table in `test_mission_state_machine.cpp` and the Mermaid
diagram, all three together.

## Coding conventions

Enforced by `.clang-format` (Google-based, 100 columns) and `.clang-tidy`
(zero warnings, warnings are errors).

- Files `snake_case.hpp/.cpp`; headers in `include/<package>/`; `#pragma once`.
- Types, enums and enumerators `CamelCase`; functions and methods `camelBack`;
  variables and parameters `snake_case`; private members `snake_case_`;
  constants `kCamelCase`; namespaces `lower_case` (`robot_core`).
- SI units everywhere (m, rad, s, m/s). Add a unit suffix (`_m`, `_rad`) only
  where a name would otherwise be ambiguous (`position_tolerance_m`).
- Value types are plain aggregates (rule of zero) checked by free `isValid()`
  functions at trust boundaries. Polymorphic interfaces are named `I…`, have a
  virtual destructor and delete copy and move.
- Ownership: `unique_ptr` or values by default; non-owning raw pointers or
  references for injected dependencies that outlive the user (documented in
  the class comment). No `shared_ptr` without a stated shared-ownership reason.
- `std::optional` instead of sentinel values; `std::variant` plus `std::visit`
  for closed sets of alternatives; `std::chrono` types for all time. Never
  pass bare `double` seconds.
- Time comes only from an injected `IClock`. No `std::chrono::*_clock::now()`
  in domain logic.
- `[[nodiscard]]` on queries and on results that must be checked.
- Every `NOLINT` names the check and gives a reason.
- Exceptions only for programming errors (`std::logic_error`) and invalid
  arguments to test utilities. Runtime failures are values (`std::optional`,
  status enums, `Fault`).
- Conventional commits (`feat(robot_core): …`, `test: …`, `ci: …`) when the
  user asks for a commit.

## Test requirements

- Every behaviour change in a core library comes with GoogleTest coverage.
  Interfaces are tested through GoogleMock doubles (`test/mocks.hpp`).
- **Deterministic tests only:** no `sleep`, no wall-clock timing assertions.
  Use `FakeClock` for time. Concurrency tests assert interleaving-independent
  invariants (every item delivered once, FIFO per producer, no torn reads).
- Guard optional or pointer access in tests with `ASSERT_*`, not `EXPECT_*`.
- State machines: test every legal transition and every illegal (state,
  event) pair against an independent spec table.
- Targets for core libraries: ≥ 85 % line coverage (gcovr gate), zero
  clang-tidy warnings, green under the ASan, UBSan and TSan presets.
- Do not report success without running the tests. Report actual numbers.

### Commands

```bash
cmake --preset debug && cmake --build --preset debug && ctest --preset debug
tools/ci/check_core.sh all                    # full gate = CI: ros-independence format shell-lint
                                              #   tidy debug release asan ubsan tsan coverage
tools/ci/check_core.sh --fix-format format    # apply clang-format first
tools/ci/check_core.sh asan coverage          # any subset of steps
docker compose --profile ci run --rm ci tools/ci/check_core.sh <steps>   # inside the CI image
docker compose --profile ci run --rm ci-tsan tools/ci/check_core.sh tsan
# ROS 2 workspace (only inside the images; the host has no ROS):
docker compose --profile ci run --rm ci tools/ci/check_ros.sh build test
docker compose --profile ci run --rm ci env PACKAGES=warehouse_bringup tools/ci/check_ros.sh build test
# TSan stress (roadmap: 1000 iterations):
setarch "$(uname -m)" -R build/tsan/src/robot_core/test/robot_core_tests \
  --gtest_filter='TaskQueue.MultiProducer*' --gtest_repeat=1000
```

`check_core.sh` (ROS-free core) and `check_ros.sh` (colcon workspace) are the
single source of truth for checks; `ci.yml` only calls their steps. Add a check there, never inline in the workflow. Coverage uses
gcovr with `gcovr.cfg` (filters and the 85 % gate).

### CI, Docker and dependency rules

- CI runs in `docker/Dockerfile.ci` (from `ros:jazzy-ros-base-noble`, no GPU).
  `Dockerfile.dev` (from `ros:jazzy`) is the GPU dev image; the GPU comes from
  the NVIDIA Container Toolkit at run time (`gpus` in `docker-compose.yml`).
  Both install their toolchain through `docker/install_toolchain.sh` and their
  ROS 2 packages through `docker/install_ros_packages.sh` (`ci` or `dev`); keep
  them in sync there, never with per-Dockerfile apt lines.
- colcon trees live under `build/` (`build/colcon`, `build/install`,
  `build/log`), inside the per-service overlay; never at the repo root.
- New executable files: `git update-index --add --chmod=+x <file>`, because
  git on `/mnt/d` records new files as 100644 (`shell-lint` checks `*.sh`).
- **GPU separation:** tests that need a GPU get the CTest label `gpu`, live in
  their own test executables, and run only through `ctest --preset gpu`.
  Every other test preset, and therefore CI, excludes that label.
- Every sanitizer preset carries a canary (`test/sanitizers/`) that must be
  detected. Never delete a canary to make a run green.
- GoogleTest comes from the system package. `-DWMM_FETCH_GTEST=ON` is the
  only download path (SHA256-pinned). Never add implicit network fetches.
- Pin GitHub Actions to full commit SHAs, with the tag in a comment.
- Compose services overlay `/ws/build` with `build/docker-ci` (ci, ci-tsan) or
  `build/docker-dev` (dev): CMake caches are path-bound, so host and container
  trees must never share a directory. Containers run as `HOST_UID:HOST_GID`
  (default 1000), never root.
- Comments must not start with the word `shellcheck` (parsed as a directive).
- TSan needs ASLR disabled on this WSL2 kernel. Never change the dev
  machine's kernel settings; the script uses `setarch -R`.
- Never claim CI results that have not run on GitHub.

### ROS 2 conventions (v0.2 onward)

- Velocity commands are `geometry_msgs/TwistStamped` end to end (Jazzy's
  diff_drive_controller accepts only stamped commands).
- The Gazebo world frame is the map frame (origin at the dock). The world,
  the tag models, `tag_map.yaml` and `stations.yaml` are generated by
  `src/warehouse_sim/scripts/generate_world.py`; edit the generator, rerun it,
  and commit its output (a test fails on drift).
- `/ground_truth/odom` is for evaluation only; never feed it to control.
- Launch layers: `sim`, `robot`, `navigation`, `perception`, composed by
  `system.launch.py`. Parameters in YAML under `config/`, never inline.
- Launch tests run headless (`gz sim -s --headless-rendering`, Mesa) and use
  simulation time; never assert wall-clock rates.
- Hold callback groups as members (nodes keep only weak references).

## Milestone status

Authoritative and current: `docs/implementation-status.md` (what is done and
verified, blockers) and `docs/implementation-plan.md` (remaining work mapped
to the roadmap). Update both after each stage.

- **v0.1-core**: done, GitHub Actions green, tagged `v0.1-core`.
- **v0.2-platform**: in progress (see the status file).
