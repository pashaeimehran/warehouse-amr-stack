# Shared compiler settings, sanitizers, coverage and test dependencies.
include_guard(GLOBAL)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(WMM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
option(WMM_BUILD_TESTS "Build unit tests" ON)
option(WMM_ENABLE_COVERAGE "Instrument for gcov coverage (GCC only)" OFF)
set(WMM_SANITIZER "" CACHE STRING "Sanitizer to enable: '', address, undefined, thread")
set_property(CACHE WMM_SANITIZER PROPERTY STRINGS "" address undefined thread)

# Interface target carrying warning flags; linked PRIVATE by every project target.
add_library(wmm_warnings INTERFACE)
target_compile_options(wmm_warnings INTERFACE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
  -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual -Wnull-dereference
  -Wdouble-promotion -Wimplicit-fallthrough
  $<$<BOOL:${WMM_WARNINGS_AS_ERRORS}>:-Werror>)

if(WMM_SANITIZER STREQUAL "address")
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address)
elseif(WMM_SANITIZER STREQUAL "undefined")
  add_compile_options(-fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
  add_link_options(-fsanitize=undefined -fno-sanitize-recover=all)
elseif(WMM_SANITIZER STREQUAL "thread")
  add_compile_options(-fsanitize=thread)
  add_link_options(-fsanitize=thread)
elseif(NOT WMM_SANITIZER STREQUAL "")
  message(FATAL_ERROR "Unknown WMM_SANITIZER '${WMM_SANITIZER}'")
endif()

if(WMM_ENABLE_COVERAGE)
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

option(WMM_FETCH_GTEST
  "Download GoogleTest 1.14.0 (SHA256-verified) instead of using the system package" OFF)

if(WMM_BUILD_TESTS)
  # Dependency strategy (see docs/01-core-design.md, "Dependencies"):
  #  - Default: the distribution package (libgtest-dev / libgmock-dev 1.14.0 on
  #    Ubuntu 24.04), pinned by the Docker CI image. No network access at configure time.
  #  - Opt-in: WMM_FETCH_GTEST=ON downloads the v1.14.0 release archive and
  #    verifies its SHA256 (checked against two independent downloads on 2026-11-07).
  if(WMM_FETCH_GTEST)
    include(FetchContent)
    FetchContent_Declare(googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
      URL_HASH SHA256=8ad598c73ad796e0d8280b082cebd82a630d73e73cd3c70057938a6501bba5d7
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    # googletest >= 1.11 defines the GTest::gtest, GTest::gmock and
    # GTest::gtest_main aliases itself; do not redefine them.
  else()
    find_package(GTest 1.14 CONFIG)
    if(NOT GTest_FOUND)
      message(FATAL_ERROR
        "GoogleTest/GoogleMock >= 1.14 not found. Install them "
        "(Ubuntu 24.04: sudo apt-get install libgtest-dev libgmock-dev) "
        "or configure with -DWMM_FETCH_GTEST=ON to download a SHA256-verified copy.")
    endif()
  endif()
  foreach(target GTest::gtest GTest::gmock GTest::gtest_main)
    if(NOT TARGET ${target})
      message(FATAL_ERROR "Expected GoogleTest target ${target} is missing")
    endif()
  endforeach()
  include(GoogleTest)
endif()
