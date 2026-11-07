#pragma once

#include <atomic>
#include <cstdint>

#include "robot_core/time.hpp"

namespace robot_core {

/// Source of monotonic robot time. Injected into every component whose logic
/// depends on time, so that logic is testable without sleeping.
class IClock {
 public:
  IClock() = default;
  virtual ~IClock() = default;
  IClock(const IClock&) = delete;
  IClock& operator=(const IClock&) = delete;
  IClock(IClock&&) = delete;
  IClock& operator=(IClock&&) = delete;

  /// Current time. Never decreases between calls. Thread-safe.
  [[nodiscard]] virtual Timestamp now() const = 0;
};

/// Production clock backed by std::chrono::steady_clock (monotonic, unaffected
/// by wall-clock adjustments).
class SystemClock final : public IClock {
 public:
  [[nodiscard]] Timestamp now() const override;
};

/// Manually advanced clock for deterministic tests. Thread-safe: one thread may
/// advance time while others read it.
class FakeClock final : public IClock {
 public:
  explicit FakeClock(Timestamp start = Timestamp{}) noexcept;

  [[nodiscard]] Timestamp now() const override;

  /// Moves time forward. Throws std::invalid_argument for a negative step.
  void advance(Duration step);

  /// Jumps to an absolute time. Throws std::invalid_argument if `t` is before
  /// the current time (the clock is monotonic).
  void setTime(Timestamp t);

 private:
  std::atomic<std::int64_t> now_ns_;
};

}  // namespace robot_core
