#!/usr/bin/env bash
# Quality gate for the ROS-free C++ core. The CI workflow (.github/workflows/ci.yml)
# calls exactly these steps, so running them locally reproduces CI.
#
# Usage: tools/ci/check_core.sh [--fix-format] [STEP...]
#
# Steps (default: all):
#   ros-independence  no ROS includes in src/*_core
#   format            clang-format 18, no changes allowed
#   shell-lint        ShellCheck 0.9 on every *.sh outside build/
#   tidy              clean build with clang-tidy 18, warnings are errors
#   debug release asan ubsan tsan
#                     configure + build + ctest for that preset (GPU tests excluded)
#   coverage          coverage preset + gcovr report, fails below 85 % line coverage
#   all               every step above, in that order
#
# Environment:
#   JOBS              parallel build/test jobs (default: nproc)
set -euo pipefail

cd "$(dirname "$0")/../.."
JOBS="${JOBS:-$(nproc)}"
REQUIRED_CLANG_MAJOR=18
ALL_STEPS=(ros-independence format shell-lint tidy debug release asan ubsan tsan coverage)

FIX_FORMAT=0
STEPS=()
for arg in "$@"; do
  case "${arg}" in
    --fix-format) FIX_FORMAT=1 ;;
    all) STEPS+=("${ALL_STEPS[@]}") ;;
    -h | --help)
      sed -n '2,20p' "$0"
      exit 0
      ;;
    *) STEPS+=("${arg}") ;;
  esac
done
if [[ ${#STEPS[@]} -eq 0 ]]; then
  STEPS=("${ALL_STEPS[@]}")
fi

step() { printf '\n=== %s ===\n' "$*"; }
die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

require_tool() {
  command -v "$1" >/dev/null 2>&1 || die "'$1' not found. $2"
}

# clang-format output differs between major versions; pin the major so local
# runs and CI agree (Ubuntu 24.04 ships 18).
require_clang_major() {
  local tool="$1" version major
  require_tool "${tool}" "Ubuntu 24.04: sudo apt-get install ${tool}"
  version="$("${tool}" --version)"
  major="$(grep -oE 'version [0-9]+' <<<"${version}" | grep -oE '[0-9]+' | head -n1)"
  [[ "${major}" == "${REQUIRED_CLANG_MAJOR}" ]] ||
    die "${tool} major version ${major:-unknown}, expected ${REQUIRED_CLANG_MAJOR}: ${version}"
}

# GCC 13's TSan runtime aborts ("unexpected memory mapping") on kernels with
# high mmap ASLR entropy (WSL2 6.x, GitHub's Ubuntu 24.04 runners). Disable ASLR
# for this process tree when the kernel allows it; inside a container with the
# default seccomp profile personality(ADDR_NO_RANDOMIZE) is refused, and CI
# lowers vm.mmap_rnd_bits on its throwaway runner VM instead.
TSAN_WRAPPER=()
if setarch "$(uname -m)" -R true >/dev/null 2>&1; then
  TSAN_WRAPPER=(setarch "$(uname -m)" -R)
fi

run_ros_independence() {
  step "ROS independence of domain libraries"
  local pattern='#include *[<"](rclcpp|rcl|rmw|tf2|tf2_ros|ament_index_cpp|[a-z_]+_msgs|[a-z_]+_srvs|[a-z_]+_interfaces)/'
  if grep -rEn --include='*.hpp' --include='*.cpp' --include='*.h' "${pattern}" src/*_core/; then
    die "ROS include found in a ROS-free core library"
  fi
  # Python core packages (perception_core): no rclpy, rosidl or message imports.
  local py_pattern='^[[:space:]]*(import|from)[[:space:]]+(rclpy|rosidl[a-z_]*|cv_bridge|[a-z_]+_msgs|[a-z_]+_srvs)([[:space:].]|$)'
  if grep -rEn --include='*.py' "${py_pattern}" src/*_core/; then
    die "ROS import found in a ROS-free core package"
  fi
  echo "ok"
}

run_format() {
  require_clang_major clang-format
  local files=()
  mapfile -t files < <(find src test -type f \( -name '*.hpp' -o -name '*.cpp' \) | sort)
  step "clang-format (${#files[@]} files)"
  if [[ ${FIX_FORMAT} -eq 1 ]]; then
    clang-format -i "${files[@]}"
  fi
  clang-format --dry-run --Werror "${files[@]}"
  echo "ok"
}

run_shellcheck() {
  require_tool shellcheck "Ubuntu 24.04: sudo apt-get install shellcheck"
  local files=()
  mapfile -t files < <(find . -path ./build -prune -o -type f -name '*.sh' -print | sort)
  step "shellcheck $(shellcheck --version | sed -n 2p | grep -oE '[0-9.]+') (${#files[@]} files)"
  shellcheck -x "${files[@]}"
  # The repository lives on /mnt/d, where git runs with core.filemode=false and
  # records new files as 100644. CI then fails with exit code 126 ("permission
  # denied"), so check the mode stored in the index, not the working tree.
  if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    local bad
    bad="$(git ls-files -s -- '*.sh' | awk '$1 != "100755" {print $4}')"
    if [[ -n "${bad}" ]]; then
      die "not executable in the git index (run: git update-index --chmod=+x <file>): ${bad//$'\n'/ }"
    fi
    echo "git index: every tracked *.sh is 100755"
  else
    echo "not a git work tree: index file modes not checked"
  fi
  echo "ok"
}

run_tidy() {
  require_clang_major clang-tidy
  step "clang-tidy (clean build, warnings are errors)"
  cmake --preset tidy >/dev/null
  cmake --build --preset tidy --target clean >/dev/null
  cmake --build --preset tidy -j "${JOBS}"
  echo "ok"
}

run_preset() {
  local preset="$1"
  local wrapper=()
  if [[ "${preset}" == "tsan" ]]; then
    wrapper=("${TSAN_WRAPPER[@]}")
  fi
  step "build + test: ${preset}"
  cmake --preset "${preset}" >/dev/null
  cmake --build --preset "${preset}" -j "${JOBS}"
  if ! "${wrapper[@]}" ctest --preset "${preset}" -j "${JOBS}"; then
    if [[ "${preset}" == "tsan" && ${#TSAN_WRAPPER[@]} -eq 0 ]]; then
      echo "hint: if TSan printed 'unexpected memory mapping', run where setarch -R is" \
        "permitted or lower vm.mmap_rnd_bits (see docs/01-core-design.md)" >&2
    fi
    die "tests failed for preset ${preset}"
  fi
}

run_coverage() {
  require_tool gcovr "Ubuntu 24.04: sudo apt-get install gcovr (needs approval on the dev machine)"
  step "coverage (gcovr $(gcovr --version | head -n1 | grep -oE '[0-9]+(\.[0-9]+)+'))"
  cmake --preset coverage >/dev/null
  cmake --build --preset coverage -j "${JOBS}"
  find build/coverage -name '*.gcda' -delete
  ctest --preset coverage -j "${JOBS}"
  mkdir -p build/coverage/report
  gcovr --config gcovr.cfg \
    --txt build/coverage/report/coverage.txt \
    --cobertura build/coverage/report/coverage.xml \
    --html-details build/coverage/report/index.html
  cat build/coverage/report/coverage.txt
}

for s in "${STEPS[@]}"; do
  case "${s}" in
    ros-independence) run_ros_independence ;;
    format) run_format ;;
    shell-lint) run_shellcheck ;;
    tidy) run_tidy ;;
    debug | release | asan | ubsan | tsan) run_preset "${s}" ;;
    coverage) run_coverage ;;
    *) die "unknown step '${s}' (see --help)" ;;
  esac
done

step "passed: ${STEPS[*]}"
