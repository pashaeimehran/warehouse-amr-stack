#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace robot_core {

enum class PushResult : std::uint8_t { Accepted, Full, Closed };

/// Multi-producer, multi-consumer FIFO queue with an optional capacity bound.
///
/// - push() never blocks: a full queue rejects new items instead of growing
///   without bound (a growing queue is a latency bug in a robot).
/// - close() wakes every waiting consumer; remaining items can still be
///   drained, after which pops return std::nullopt.
/// - Consumers block on a condition variable, never by polling.
template <typename T>
class TaskQueue {
 public:
  /// `capacity == 0` means unbounded.
  explicit TaskQueue(std::size_t capacity = 0) : capacity_{capacity} {}

  TaskQueue(const TaskQueue&) = delete;
  TaskQueue& operator=(const TaskQueue&) = delete;
  TaskQueue(TaskQueue&&) = delete;
  TaskQueue& operator=(TaskQueue&&) = delete;
  ~TaskQueue() = default;

  /// Copies `item` into the queue if there is room.
  [[nodiscard]] PushResult push(const T& item) { return emplaceIfRoom(item); }

  /// Moves from `item` only when the result is Accepted. On Full or Closed the
  /// argument is left untouched, so a move-only work item is never lost and the
  /// caller can retry or dispose of it.
  [[nodiscard]] PushResult push(T&& item) { return emplaceIfRoom(std::move(item)); }

  /// Non-blocking pop.
  [[nodiscard]] std::optional<T> tryPop() {
    const std::lock_guard<std::mutex> lock{mutex_};
    return popLocked();
  }

  /// Blocks until an item is available or the queue is closed and drained.
  [[nodiscard]] std::optional<T> waitPop() {
    std::unique_lock<std::mutex> lock{mutex_};
    not_empty_.wait(lock, [this] { return !items_.empty() || closed_; });
    return popLocked();
  }

  /// Like waitPop() but gives up after `timeout` (real time).
  template <typename Rep, typename Period>
  [[nodiscard]] std::optional<T> waitPopFor(std::chrono::duration<Rep, Period> timeout) {
    std::unique_lock<std::mutex> lock{mutex_};
    not_empty_.wait_for(lock, timeout, [this] { return !items_.empty() || closed_; });
    return popLocked();
  }

  /// Rejects further pushes and wakes all waiting consumers. Idempotent.
  void close() {
    {
      const std::lock_guard<std::mutex> lock{mutex_};
      closed_ = true;
    }
    not_empty_.notify_all();
  }

  [[nodiscard]] bool isClosed() const {
    const std::lock_guard<std::mutex> lock{mutex_};
    return closed_;
  }

  [[nodiscard]] std::size_t size() const {
    const std::lock_guard<std::mutex> lock{mutex_};
    return items_.size();
  }

  [[nodiscard]] bool empty() const { return size() == 0; }

  [[nodiscard]] std::size_t capacity() const { return capacity_; }

 private:
  // `U` is `const T&` or `T&&`; the item is only consumed after both checks pass.
  template <typename U>
  PushResult emplaceIfRoom(U&& item) {
    {
      const std::lock_guard<std::mutex> lock{mutex_};
      if (closed_) {
        return PushResult::Closed;
      }
      if (capacity_ != 0 && items_.size() >= capacity_) {
        return PushResult::Full;
      }
      items_.push_back(std::forward<U>(item));
    }
    not_empty_.notify_one();  // notify outside the lock to avoid a wasted wake-up
    return PushResult::Accepted;
  }

  std::optional<T> popLocked() {
    if (items_.empty()) {
      return std::nullopt;
    }
    std::optional<T> item{std::move(items_.front())};
    items_.pop_front();
    return item;
  }

  const std::size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_;
  std::deque<T> items_;
  bool closed_{false};
};

}  // namespace robot_core
