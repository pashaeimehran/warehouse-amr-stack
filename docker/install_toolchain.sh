#!/usr/bin/env bash
# Installs the C++ toolchain shared by Dockerfile.ci and Dockerfile.dev.
# Runs as root during `docker build` on Ubuntu 24.04 (noble) based ROS 2 Jazzy images.
# Versions are pinned by the distribution release: GCC 13, CMake 3.28,
# clang-format/clang-tidy 18, GoogleTest/GoogleMock 1.14, gcovr 7.0, ShellCheck 0.9.
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

# shellcheck disable=SC1091
. /etc/os-release
if [[ "${VERSION_CODENAME}" != "noble" ]]; then
  echo "install_toolchain.sh expects Ubuntu 24.04 (noble), found ${VERSION_CODENAME}" >&2
  exit 1
fi

apt-get update
apt-get install -y --no-install-recommends \
  build-essential \
  ca-certificates \
  clang-format-18 \
  clang-tidy-18 \
  cmake \
  gcovr \
  gdb \
  git \
  libgmock-dev \
  libgtest-dev \
  ninja-build \
  python3 \
  shellcheck \
  util-linux
# util-linux provides setarch, used for TSan (see tools/ci/check_core.sh).

# Expose the pinned clang tools under their unversioned names.
update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-18 100
update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-18 100

apt-get clean
rm -rf /var/lib/apt/lists/*

# Record versions in the build log.
g++ --version | head -n1
cmake --version | head -n1
clang-format --version
clang-tidy --version | head -n1
gcovr --version | head -n1
shellcheck --version | sed -n 2p
