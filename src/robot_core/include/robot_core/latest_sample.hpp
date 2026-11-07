#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

#include "robot_core/time.hpp"

namespace robot_core {

/// A value with its source timestamp and a monotonically increasing sequence
/// number (1 for the first publish).
template <typename T>
struct Sample {
  T value;
  Timestamp stamp;
  std::uint64_t sequence{0};
};

/// Single-slot, thread-safe "newest value wins" buffer.
///
/// Producers overwrite; consumers read a copy. This implements the
/// process-only-the-newest-frame policy: an old value is never queued behind a
/// newer one. The sequence number lets a consumer detect how many samples it
/// skipped (dropped-frame counting) and avoid processing the same sample twice.
template <typename T>
class LatestSample {
 public:
  LatestSample() = default;
  LatestSample(const LatestSample&) = delete;
  LatestSample& operator=(const LatestSample&) = delete;
  LatestSample(LatestSample&&) = delete;
  LatestSample& operator=(LatestSample&&) = delete;
  ~LatestSample() = default;

  /// Stores `value`, replacing any previous one. Returns its sequence number.
  std::uint64_t publish(T value, Timestamp stamp) {
    const std::lock_guard<std::mutex> lock{mutex_};
    ++last_sequence_;
    sample_ = Sample<T>{std::move(value), stamp, last_sequence_};
    return last_sequence_;
  }

  [[nodiscard]] std::optional<Sample<T>> latest() const {
    const std::lock_guard<std::mutex> lock{mutex_};
    return sample_;
  }

  /// The newest sample only if it is newer than `last_seen_sequence`.
  [[nodiscard]] std::optional<Sample<T>> latestNewerThan(std::uint64_t last_seen_sequence) const {
    const std::lock_guard<std::mutex> lock{mutex_};
    if (sample_ && sample_->sequence > last_seen_sequence) {
      return sample_;
    }
    return std::nullopt;
  }

  /// The newest sample only if its stamp passes isFresh(stamp, now, max_age).
  /// Stale data is never returned: the caller treats it as missing.
  [[nodiscard]] std::optional<Sample<T>> latestIfFresh(Timestamp now, Duration max_age) const {
    const std::lock_guard<std::mutex> lock{mutex_};
    if (sample_ && isFresh(sample_->stamp, now, max_age)) {
      return sample_;
    }
    return std::nullopt;
  }

  /// Sequence number of the newest publish (0 if none).
  [[nodiscard]] std::uint64_t lastSequence() const {
    const std::lock_guard<std::mutex> lock{mutex_};
    return last_sequence_;
  }

  /// Drops the stored value. The sequence counter keeps counting so consumers
  /// never mistake a later sample for one they have already seen.
  void clear() {
    const std::lock_guard<std::mutex> lock{mutex_};
    sample_.reset();
  }

 private:
  mutable std::mutex mutex_;
  std::optional<Sample<T>> sample_;
  std::uint64_t last_sequence_{0};
};

}  // namespace robot_core
