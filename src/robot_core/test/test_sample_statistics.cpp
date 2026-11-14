#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "robot_core/sample_statistics.hpp"

namespace robot_core {
namespace {

TEST(SampleStatistics, EmptySampleHasNoSummary) {
  EXPECT_FALSE(summarize({}).has_value());
  EXPECT_FALSE(percentileOfSorted({}, 50.0).has_value());
}

TEST(SampleStatistics, NonFiniteSampleIsRejected) {
  EXPECT_FALSE(summarize({1.0, std::numeric_limits<double>::quiet_NaN()}).has_value());
  EXPECT_FALSE(summarize({1.0, std::numeric_limits<double>::infinity()}).has_value());
}

TEST(SampleStatistics, PercentileOutsideRangeIsRejected) {
  const std::vector<double> sorted{1.0, 2.0};
  EXPECT_FALSE(percentileOfSorted(sorted, -0.1).has_value());
  EXPECT_FALSE(percentileOfSorted(sorted, 100.1).has_value());
  EXPECT_FALSE(percentileOfSorted(sorted, std::numeric_limits<double>::quiet_NaN()).has_value());
}

TEST(SampleStatistics, SingleSample) {
  const auto summary = summarize({4.2});
  ASSERT_TRUE(summary.has_value());
  EXPECT_EQ(summary->count, 1U);
  EXPECT_DOUBLE_EQ(summary->min, 4.2);
  EXPECT_DOUBLE_EQ(summary->max, 4.2);
  EXPECT_DOUBLE_EQ(summary->mean, 4.2);
  EXPECT_DOUBLE_EQ(summary->stddev, 0.0);
  EXPECT_DOUBLE_EQ(summary->p50, 4.2);
  EXPECT_DOUBLE_EQ(summary->p99, 4.2);
}

// Reference values from numpy.percentile([1, 2, 3, 4, 5, 6, 7, 8, 9, 10], q).
TEST(SampleStatistics, PercentilesMatchNumpyLinearInterpolation) {
  const std::vector<double> sorted{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  EXPECT_DOUBLE_EQ(percentileOfSorted(sorted, 0.0).value_or(-1), 1.0);
  EXPECT_DOUBLE_EQ(percentileOfSorted(sorted, 50.0).value_or(-1), 5.5);
  EXPECT_DOUBLE_EQ(percentileOfSorted(sorted, 95.0).value_or(-1), 9.55);
  EXPECT_DOUBLE_EQ(percentileOfSorted(sorted, 99.0).value_or(-1), 9.91);
  EXPECT_DOUBLE_EQ(percentileOfSorted(sorted, 100.0).value_or(-1), 10.0);
}

TEST(SampleStatistics, SummaryIsOrderIndependent) {
  const auto a = summarize({10, 1, 7, 3, 5, 9, 2, 8, 4, 6});
  const auto b = summarize({1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  EXPECT_DOUBLE_EQ(a->p95, b->p95);
  EXPECT_DOUBLE_EQ(a->mean, 5.5);
  EXPECT_DOUBLE_EQ(a->min, 1.0);
  EXPECT_DOUBLE_EQ(a->max, 10.0);
  // Population standard deviation of 1..10 is sqrt(8.25).
  EXPECT_NEAR(a->stddev, std::sqrt(8.25), 1e-12);
}

TEST(SampleStatistics, TailDominatedSampleShowsInP99NotMean) {
  // 99 fast samples and one 400 ms outlier: the case the roadmap warns about.
  std::vector<double> latency_ms(99, 40.0);
  latency_ms.push_back(400.0);
  const auto summary = summarize(latency_ms);
  ASSERT_TRUE(summary.has_value());
  EXPECT_NEAR(summary->mean, 43.6, 1e-9);
  EXPECT_DOUBLE_EQ(summary->p50, 40.0);
  EXPECT_GT(summary->p99, 40.0);
  EXPECT_DOUBLE_EQ(summary->max, 400.0);
}

}  // namespace
}  // namespace robot_core
