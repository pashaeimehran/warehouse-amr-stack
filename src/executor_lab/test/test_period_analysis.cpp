#include <chrono>
#include <vector>

#include <gtest/gtest.h>

#include "executor_lab/period_analysis.hpp"

namespace executor_lab {
namespace {

using std::chrono::milliseconds;
using std::chrono::nanoseconds;

std::vector<nanoseconds> startsMs(std::initializer_list<int> ms) {
  std::vector<nanoseconds> out;
  for (const int m : ms) {
    out.emplace_back(milliseconds(m));
  }
  return out;
}

TEST(PeriodAnalysis, RegularCallbackHasNoMisses) {
  const auto report = analyzePeriods(startsMs({0, 20, 40, 60, 80}), milliseconds(20));
  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->invocations, 5U);
  EXPECT_EQ(report->missed_activations, 0U);
  EXPECT_EQ(report->late_gaps, 0U);
  EXPECT_DOUBLE_EQ(report->period_ms.p50, 20.0);
  EXPECT_DOUBLE_EQ(report->period_ms.max, 20.0);
}

TEST(PeriodAnalysis, SmallJitterIsNotAMiss) {
  const auto report = analyzePeriods(startsMs({0, 21, 39, 62, 80}), milliseconds(20));
  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->missed_activations, 0U);
  EXPECT_EQ(report->late_gaps, 0U);
  EXPECT_DOUBLE_EQ(report->period_ms.max, 23.0);
}

TEST(PeriodAnalysis, StarvationGapCountsMissedActivations) {
  // A 200 ms slow callback blocks a 20 ms timer: one gap of 200 ms.
  const auto report = analyzePeriods(startsMs({0, 20, 220, 240}), milliseconds(20));
  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->missed_activations, 9U);  // 200 / 20 - 1
  EXPECT_EQ(report->late_gaps, 1U);
  EXPECT_DOUBLE_EQ(report->period_ms.max, 200.0);
}

TEST(PeriodAnalysis, RejectsInvalidInput) {
  EXPECT_FALSE(analyzePeriods(startsMs({0}), milliseconds(20)).has_value());
  EXPECT_FALSE(analyzePeriods(startsMs({0, 20}), milliseconds(0)).has_value());
  EXPECT_FALSE(analyzePeriods(startsMs({0, 40, 20}), milliseconds(20)).has_value());
}

}  // namespace
}  // namespace executor_lab
