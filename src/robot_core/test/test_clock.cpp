#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "robot_core/clock.hpp"
#include "robot_core/time.hpp"

namespace robot_core {
namespace {

using std::chrono::milliseconds;

TEST(FakeClock, StartsAtGivenTime) {
  const FakeClock clock{timestampFromNanoseconds(1'000)};
  EXPECT_EQ(clock.now().time_since_epoch().count(), 1'000);
}

TEST(FakeClock, DefaultStartsAtZero) {
  const FakeClock clock;
  EXPECT_EQ(clock.now(), Timestamp{});
}

TEST(FakeClock, AdvanceMovesTimeForwardExactly) {
  FakeClock clock;
  clock.advance(milliseconds{250});
  clock.advance(milliseconds{50});
  EXPECT_EQ(clock.now() - Timestamp{}, milliseconds{300});
}

TEST(FakeClock, TimeDoesNotPassWithoutAdvance) {
  const FakeClock clock;
  const Timestamp a = clock.now();
  const Timestamp b = clock.now();
  EXPECT_EQ(a, b);
}

TEST(FakeClock, RejectsNegativeAdvance) {
  FakeClock clock;
  clock.advance(milliseconds{10});
  EXPECT_THROW(clock.advance(milliseconds{-1}), std::invalid_argument);
  EXPECT_EQ(clock.now() - Timestamp{}, milliseconds{10});
}

TEST(FakeClock, SetTimeForwardAndRejectsGoingBackwards) {
  FakeClock clock;
  clock.setTime(Timestamp{milliseconds{500}});
  EXPECT_EQ(clock.now(), Timestamp{milliseconds{500}});
  clock.setTime(Timestamp{milliseconds{500}});  // same time is allowed
  EXPECT_THROW(clock.setTime(Timestamp{milliseconds{499}}), std::invalid_argument);
  EXPECT_EQ(clock.now(), Timestamp{milliseconds{500}});
}

TEST(FakeClock, ConcurrentAdvancesAreNotLost) {
  FakeClock clock;
  constexpr int kThreads = 4;
  constexpr int kStepsPerThread = 1000;
  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&clock] {
      for (int i = 0; i < kStepsPerThread; ++i) {
        clock.advance(Duration{1});
      }
    });
  }
  for (auto& th : threads) {
    th.join();
  }
  EXPECT_EQ(clock.now().time_since_epoch().count(), kThreads * kStepsPerThread);
}

TEST(FakeClock, UsableThroughInterface) {
  FakeClock fake;
  const IClock& clock = fake;
  fake.advance(milliseconds{7});
  EXPECT_EQ(clock.now() - Timestamp{}, milliseconds{7});
}

TEST(SystemClock, IsMonotonic) {
  const SystemClock clock;
  Timestamp previous = clock.now();
  for (int i = 0; i < 1000; ++i) {
    const Timestamp current = clock.now();
    ASSERT_GE(current, previous);
    previous = current;
  }
}

TEST(IsFresh, AcceptsUpToAndIncludingMaxAge) {
  const Timestamp now{milliseconds{1000}};
  EXPECT_TRUE(isFresh(now, now, milliseconds{300}));
  EXPECT_TRUE(isFresh(now - milliseconds{300}, now, milliseconds{300}));
}

TEST(IsFresh, RejectsDataOlderThanMaxAge) {
  // Safety contract: perception data older than 300 ms is rejected.
  const Timestamp now{milliseconds{1000}};
  EXPECT_FALSE(isFresh(now - milliseconds{300} - Duration{1}, now, milliseconds{300}));
  EXPECT_FALSE(isFresh(now - milliseconds{900}, now, milliseconds{300}));
}

TEST(IsFresh, RejectsStampsFromTheFuture) {
  const Timestamp now{milliseconds{1000}};
  EXPECT_FALSE(isFresh(now + Duration{1}, now, milliseconds{300}));
}

}  // namespace
}  // namespace robot_core
