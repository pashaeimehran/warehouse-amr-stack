#include "executor_lab/period_analysis.hpp"

#include <algorithm>
#include <cmath>

namespace executor_lab {

std::optional<PeriodReport> analyzePeriods(const std::vector<std::chrono::nanoseconds>& starts,
                                           std::chrono::nanoseconds nominal_period) {
  if (starts.size() < 2 || nominal_period <= std::chrono::nanoseconds::zero() ||
      !std::is_sorted(starts.begin(), starts.end())) {
    return std::nullopt;
  }
  const double nominal = static_cast<double>(nominal_period.count());
  PeriodReport report;
  report.invocations = starts.size();
  std::vector<double> gaps_ms;
  gaps_ms.reserve(starts.size() - 1);
  for (std::size_t i = 1; i < starts.size(); ++i) {
    const double gap = static_cast<double>((starts[i] - starts[i - 1]).count());
    gaps_ms.push_back(gap / 1e6);
    const double periods = std::round(gap / nominal);
    if (periods > 1.0) {
      report.missed_activations += static_cast<std::uint64_t>(periods) - 1U;
    }
    if (gap > 1.5 * nominal) {
      ++report.late_gaps;
    }
  }
  const auto summary = robot_core::summarize(std::move(gaps_ms));
  if (!summary) {
    return std::nullopt;
  }
  report.period_ms = *summary;
  return report;
}

}  // namespace executor_lab
