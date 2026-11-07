#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "robot_core/task_queue.hpp"

namespace robot_core {
namespace {

TEST(TaskQueue, PopsInFifoOrder) {
  TaskQueue<int> q;
  for (int i = 0; i < 5; ++i) {
    ASSERT_EQ(q.push(i), PushResult::Accepted);
  }
  EXPECT_EQ(q.size(), 5U);
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(q.tryPop(), i);
  }
  EXPECT_TRUE(q.empty());
}

TEST(TaskQueue, TryPopOnEmptyReturnsNullopt) {
  TaskQueue<int> q;
  EXPECT_EQ(q.tryPop(), std::nullopt);
}

TEST(TaskQueue, BoundedQueueRejectsWhenFullWithoutDroppingQueuedItems) {
  TaskQueue<int> q{2};
  EXPECT_EQ(q.capacity(), 2U);
  EXPECT_EQ(q.push(1), PushResult::Accepted);
  EXPECT_EQ(q.push(2), PushResult::Accepted);
  EXPECT_EQ(q.push(3), PushResult::Full);
  EXPECT_EQ(q.size(), 2U);
  EXPECT_EQ(q.tryPop(), 1);
  EXPECT_EQ(q.push(3), PushResult::Accepted);
  EXPECT_EQ(q.tryPop(), 2);
  EXPECT_EQ(q.tryPop(), 3);
}

TEST(TaskQueue, CloseRejectsPushesButAllowsDraining) {
  TaskQueue<int> q;
  ASSERT_EQ(q.push(1), PushResult::Accepted);
  q.close();
  q.close();  // idempotent
  EXPECT_TRUE(q.isClosed());
  EXPECT_EQ(q.push(2), PushResult::Closed);
  EXPECT_EQ(q.waitPop(), 1);
  EXPECT_EQ(q.waitPop(), std::nullopt);  // closed and drained: does not block
}

TEST(TaskQueue, CloseWakesBlockedConsumer) {
  TaskQueue<int> q;
  std::optional<int> result{42};
  std::thread consumer{[&] { result = q.waitPop(); }};
  q.close();
  consumer.join();  // would hang forever if close() did not wake the consumer
  EXPECT_EQ(result, std::nullopt);
}

TEST(TaskQueue, BlockedConsumerReceivesItemPushedLater) {
  TaskQueue<int> q;
  std::optional<int> result;
  std::thread consumer{[&] { result = q.waitPop(); }};
  ASSERT_EQ(q.push(7), PushResult::Accepted);
  consumer.join();
  EXPECT_EQ(result, 7);
}

TEST(TaskQueue, WaitPopForWithZeroTimeoutDoesNotBlock) {
  TaskQueue<int> q;
  EXPECT_EQ(q.waitPopFor(std::chrono::milliseconds{0}), std::nullopt);
  ASSERT_EQ(q.push(5), PushResult::Accepted);
  EXPECT_EQ(q.waitPopFor(std::chrono::milliseconds{0}), 5);
}

TEST(TaskQueue, SupportsMoveOnlyTypes) {
  TaskQueue<std::unique_ptr<int>> q;
  ASSERT_EQ(q.push(std::make_unique<int>(3)), PushResult::Accepted);
  auto item = q.tryPop();
  ASSERT_TRUE(item.has_value());
  ASSERT_NE(*item, nullptr);
  EXPECT_EQ(**item, 3);
}

TEST(TaskQueue, RejectedMoveOnlyItemIsNotConsumed) {
  TaskQueue<std::unique_ptr<int>> q{1};
  ASSERT_EQ(q.push(std::make_unique<int>(1)), PushResult::Accepted);

  // Reading `item` after std::move is the point of this test: push() must not
  // consume it when the item is rejected.
  // NOLINTBEGIN(bugprone-use-after-move)
  auto item = std::make_unique<int>(2);
  EXPECT_EQ(q.push(std::move(item)), PushResult::Full);
  ASSERT_NE(item, nullptr) << "rejected item must stay with the caller";
  EXPECT_EQ(*item, 2);

  q.close();
  EXPECT_EQ(q.push(std::move(item)), PushResult::Closed);
  ASSERT_NE(item, nullptr);
  // NOLINTEND(bugprone-use-after-move)

  (void)q.tryPop();
  EXPECT_TRUE(q.empty());
}

TEST(TaskQueue, LvaluePushCopies) {
  TaskQueue<std::string> q;
  const std::string task = "pick klt_42";
  ASSERT_EQ(q.push(task), PushResult::Accepted);
  EXPECT_EQ(task, "pick klt_42");
  EXPECT_EQ(q.tryPop(), "pick klt_42");
}

struct Tagged {
  std::size_t producer;
  int index;
};

// Every pushed item is popped exactly once and each producer's items arrive in
// the order they were pushed, under any interleaving. Run repeatedly under TSan
// (see CLAUDE.md) to check for data races.
TEST(TaskQueue, MultiProducerMultiConsumerDeliversEachItemExactlyOnceInOrder) {
  constexpr std::size_t kProducers = 4;
  constexpr std::size_t kConsumers = 3;
  constexpr int kItemsPerProducer = 1000;

  TaskQueue<Tagged> q{64};  // small bound so producers hit Full and retry
  std::vector<std::vector<Tagged>> received(kConsumers);

  std::vector<std::thread> consumers;
  consumers.reserve(kConsumers);
  for (std::size_t c = 0; c < kConsumers; ++c) {
    consumers.emplace_back([&q, &out = received[c]] {
      while (auto item = q.waitPop()) {
        out.push_back(*item);
      }
    });
  }

  std::vector<std::thread> producers;
  producers.reserve(kProducers);
  for (std::size_t p = 0; p < kProducers; ++p) {
    producers.emplace_back([&q, p] {
      for (int i = 0; i < kItemsPerProducer; ++i) {
        while (q.push(Tagged{p, i}) == PushResult::Full) {
          std::this_thread::yield();
        }
      }
    });
  }
  for (auto& t : producers) {
    t.join();
  }
  q.close();
  for (auto& t : consumers) {
    t.join();
  }

  constexpr auto kPerProducer = static_cast<std::size_t>(kItemsPerProducer);
  std::vector<int> count(kProducers * kPerProducer, 0);
  for (const auto& per_consumer : received) {
    std::vector<int> last_index(kProducers, -1);
    for (const Tagged& item : per_consumer) {
      ASSERT_GT(item.index, last_index[item.producer]) << "per-producer FIFO order violated";
      last_index[item.producer] = item.index;
      ++count[(item.producer * kPerProducer) + static_cast<std::size_t>(item.index)];
    }
  }
  EXPECT_TRUE(std::all_of(count.begin(), count.end(), [](int n) { return n == 1; }))
      << "an item was lost or duplicated";
}

}  // namespace
}  // namespace robot_core
