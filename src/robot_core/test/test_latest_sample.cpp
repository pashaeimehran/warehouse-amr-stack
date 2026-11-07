#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "robot_core/clock.hpp"
#include "robot_core/latest_sample.hpp"

namespace robot_core {
namespace {

using std::chrono::milliseconds;

TEST(LatestSample, EmptyUntilFirstPublish) {
  const LatestSample<int> s;
  EXPECT_EQ(s.latest(), std::nullopt);
  EXPECT_EQ(s.lastSequence(), 0U);
  EXPECT_EQ(s.latestNewerThan(0), std::nullopt);
}

TEST(LatestSample, NewestValueWinsAndSequenceIncrements) {
  LatestSample<std::string> s;
  EXPECT_EQ(s.publish("a", Timestamp{milliseconds{1}}), 1U);
  EXPECT_EQ(s.publish("b", Timestamp{milliseconds{2}}), 2U);
  const auto latest = s.latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->value, "b");
  EXPECT_EQ(latest->stamp, Timestamp{milliseconds{2}});
  EXPECT_EQ(latest->sequence, 2U);
}

TEST(LatestSample, LatestNewerThanReturnsEachSampleOnceAndCountsSkips) {
  LatestSample<int> s;
  std::uint64_t seen = 0;
  s.publish(10, Timestamp{});
  auto first = s.latestNewerThan(seen);
  ASSERT_TRUE(first.has_value());
  seen = first->sequence;
  EXPECT_EQ(s.latestNewerThan(seen), std::nullopt) << "same sample must not be processed twice";

  // Three frames arrive while the consumer is busy: only the newest is processed.
  s.publish(11, Timestamp{});
  s.publish(12, Timestamp{});
  s.publish(13, Timestamp{});
  auto next = s.latestNewerThan(seen);
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(next->value, 13);
  EXPECT_EQ(next->sequence - seen - 1, 2U) << "two frames dropped";
}

TEST(LatestSample, StaleDataIsNeverReturned) {
  FakeClock clock;
  LatestSample<int> s;
  const Duration max_age = milliseconds{300};

  s.publish(1, clock.now());
  clock.advance(milliseconds{300});
  EXPECT_TRUE(s.latestIfFresh(clock.now(), max_age).has_value());
  clock.advance(Duration{1});
  EXPECT_EQ(s.latestIfFresh(clock.now(), max_age), std::nullopt);
  EXPECT_TRUE(s.latest().has_value()) << "stale value is still stored, just not served as fresh";
}

TEST(LatestSample, FreshnessUsesSourceStampNotPublishTime) {
  // A frame captured 900 ms ago (slow inference) is stale even if published now.
  const FakeClock clock{Timestamp{milliseconds{1000}}};
  LatestSample<int> s;
  s.publish(1, clock.now() - milliseconds{900});
  EXPECT_EQ(s.latestIfFresh(clock.now(), milliseconds{300}), std::nullopt);
}

TEST(LatestSample, ClearDropsValueButSequenceKeepsCounting) {
  LatestSample<int> s;
  s.publish(1, Timestamp{});
  s.clear();
  EXPECT_EQ(s.latest(), std::nullopt);
  EXPECT_EQ(s.lastSequence(), 1U);
  EXPECT_EQ(s.publish(2, Timestamp{}), 2U);
}

struct Pair {
  std::uint64_t a;
  std::uint64_t b;  // invariant: b == 2 * a; a torn read would break it
};

TEST(LatestSample, ConcurrentReadersSeeConsistentMonotonicSamples) {
  constexpr std::uint64_t kPublishes = 5000;
  LatestSample<Pair> s;
  std::atomic<bool> done{false};
  std::atomic<bool> violation{false};

  std::atomic<std::uint64_t> checked{0};
  auto reader = [&] {
    std::uint64_t last_seq = 0;
    bool finished = false;
    // Read at least once after the producer is done, so every reader checks
    // the invariant even if it starts after all publishes.
    while (!finished) {
      finished = done.load(std::memory_order_acquire);
      if (auto sample = s.latest()) {
        if (sample->value.b != 2 * sample->value.a || sample->sequence < last_seq ||
            sample->value.a != sample->sequence) {
          violation = true;
        }
        last_seq = sample->sequence;
        checked.fetch_add(1, std::memory_order_relaxed);
      }
    }
  };
  std::thread r1{reader};
  std::thread r2{reader};
  for (std::uint64_t i = 1; i <= kPublishes; ++i) {
    s.publish(Pair{i, 2 * i}, Timestamp{});
  }
  done.store(true, std::memory_order_release);
  r1.join();
  r2.join();

  EXPECT_FALSE(violation.load());
  EXPECT_GE(checked.load(), 2U) << "each reader must have checked at least one sample";
  EXPECT_EQ(s.lastSequence(), kPublishes);
}

}  // namespace
}  // namespace robot_core
