#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace robot_core {

/// Summary of a sample of measurements (latencies, periods, errors). The unit
/// is whatever the samples are in; callers name it (`latency_ms`).
///
/// Percentiles use linear interpolation between closest ranks (the definition
/// of NumPy's default `percentile`), so results agree with the Python analysis
/// tools in `tools/analysis`.
struct SampleSummary {
  std::size_t count{0};
  double min{0.0};
  double max{0.0};
  double mean{0.0};
  double stddev{0.0};  ///< Population standard deviation.
  double p50{0.0};
  double p95{0.0};
  double p99{0.0};
};

/// Percentile `q` in [0, 100] of `sorted_samples`, which must be sorted in
/// ascending order. Returns std::nullopt for an empty sample or q outside
/// [0, 100].
[[nodiscard]] std::optional<double> percentileOfSorted(const std::vector<double>& sorted_samples,
                                                       double q);

/// Summary of `samples` (any order). Returns std::nullopt for an empty sample
/// or if any sample is not finite.
[[nodiscard]] std::optional<SampleSummary> summarize(std::vector<double> samples);

}  // namespace robot_core
