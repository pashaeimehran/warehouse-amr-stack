#pragma once

// ROS-free analysis of a periodic callback: from the start times of its
// invocations, how regular was it and how many activations were missed?

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include "robot_core/sample_statistics.hpp"

namespace executor_lab {

struct PeriodReport {
  std::uint64_t invocations{0};
  /// Activations that never ran: for each gap between consecutive starts,
  /// round(gap / nominal) - 1, so one 3-period gap counts 2 misses.
  std::uint64_t missed_activations{0};
  /// Gaps longer than 1.5 nominal periods (the callback was starved at least once there).
  std::uint64_t late_gaps{0};
  /// Gaps between consecutive starts, in ms.
  robot_core::SampleSummary period_ms;
};

/// Analyses invocation start times (monotonic, ascending) of a callback with
/// the given nominal period. Returns std::nullopt for fewer than two starts,
/// a non-positive period, or unsorted input.
[[nodiscard]] std::optional<PeriodReport> analyzePeriods(
    const std::vector<std::chrono::nanoseconds>& starts, std::chrono::nanoseconds nominal_period);

}  // namespace executor_lab
