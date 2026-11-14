#include "robot_core/sample_statistics.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace robot_core {

std::optional<double> percentileOfSorted(const std::vector<double>& sorted_samples, double q) {
  // NaN fails every comparison, so it is rejected explicitly.
  if (sorted_samples.empty() || std::isnan(q) || q < 0.0 || q > 100.0) {
    return std::nullopt;
  }
  // Rank in [0, n - 1]; interpolate between the two neighbouring order statistics.
  const double rank = q / 100.0 * static_cast<double>(sorted_samples.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(rank));
  const std::size_t upper = std::min(lower + 1, sorted_samples.size() - 1);
  const double fraction = rank - static_cast<double>(lower);
  return sorted_samples[lower] + fraction * (sorted_samples[upper] - sorted_samples[lower]);
}

std::optional<SampleSummary> summarize(std::vector<double> samples) {
  if (samples.empty() ||
      !std::all_of(samples.begin(), samples.end(), [](double s) { return std::isfinite(s); })) {
    return std::nullopt;
  }
  std::sort(samples.begin(), samples.end());

  SampleSummary summary;
  summary.count = samples.size();
  summary.min = samples.front();
  summary.max = samples.back();
  const auto n = static_cast<double>(samples.size());
  summary.mean = std::accumulate(samples.begin(), samples.end(), 0.0) / n;
  double squared = 0.0;
  for (const double s : samples) {
    squared += (s - summary.mean) * (s - summary.mean);
  }
  summary.stddev = std::sqrt(squared / n);
  // q is in range and the sample is non-empty, so these always have a value.
  summary.p50 = percentileOfSorted(samples, 50.0).value_or(0.0);
  summary.p95 = percentileOfSorted(samples, 95.0).value_or(0.0);
  summary.p99 = percentileOfSorted(samples, 99.0).value_or(0.0);
  return summary;
}

}  // namespace robot_core
