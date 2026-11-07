# warehouse-amr-stack

A simulated warehouse AMR with a 6-DOF arm: it receives a VDA 5050 order,
navigates with Nav2, perceives with a fine-tuned detector, tracks people,
slows or stops through a deterministic safety supervisor, picks the ordered
item with MoveIt 2, and delivers it. ROS 2 Jazzy, C++17, Gazebo Harmonic.

> **Status: v0.1-core in progress.** Only the ROS-free C++ core
> (`src/robot_core`) exists so far. Headline numbers, video and architecture
> diagrams arrive with later releases.

**Simulation only.** The project validates architecture, algorithms,
integration, failure handling and testing method. It does not validate sensor
realism, timing on real-time hardware or perception on real imagery.

## Build and test the core

Requirements (Ubuntu 24.04): GCC 13, CMake ≥ 3.25, Ninja, GoogleTest/GoogleMock
1.14, clang-format and clang-tidy 18, gcovr 7.0, ShellCheck 0.9. `docker/install_toolchain.sh`
lists the exact packages.

```bash
cmake --preset debug && cmake --build --preset debug && ctest --preset debug
tools/ci/check_core.sh all     # the full CI gate, locally
```

Presets: `debug`, `release`, `asan`, `ubsan`, `tsan`, `coverage`, `tidy`, plus
the test-only `gpu` preset for GPU-dependent tests (excluded everywhere else).

## Continuous integration

`.github/workflows/ci.yml` runs static analysis (ROS independence,
clang-format, ShellCheck, clang-tidy), the build and test matrix
(debug, release, ASan, UBSan) and gcovr coverage on every push and PR, with
TSan nightly. Jobs run inside `docker/Dockerfile.ci` (ROS 2 Jazzy base, no
GPU). To reproduce a job locally:

```bash
docker compose --profile ci run --rm ci tools/ci/check_core.sh <step>
```

See [docs/01-core-design.md](docs/01-core-design.md#quality-gate-and-ci).
The workflow has not run on GitHub yet: the repository has not been pushed.

## Documentation

- [docs/01-core-design.md](docs/01-core-design.md): core library design, mission state machine, concurrency

## License

Apache-2.0. See [LICENSE](LICENSE).
