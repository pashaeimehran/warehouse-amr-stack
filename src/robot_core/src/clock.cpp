#include "robot_core/clock.hpp"

#include <chrono>
#include <stdexcept>

namespace robot_core {

Timestamp SystemClock::now() const {
  const auto since_epoch = std::chrono::steady_clock::now().time_since_epoch();
  return Timestamp{std::chrono::duration_cast<Duration>(since_epoch)};
}

FakeClock::FakeClock(Timestamp start) noexcept : now_ns_{start.time_since_epoch().count()} {}

Timestamp FakeClock::now() const {
  return timestampFromNanoseconds(now_ns_.load(std::memory_order_acquire));
}

void FakeClock::advance(Duration step) {
  if (step < Duration::zero()) {
    throw std::invalid_argument("FakeClock::advance: negative step");
  }
  now_ns_.fetch_add(step.count(), std::memory_order_acq_rel);
}

void FakeClock::setTime(Timestamp t) {
  const std::int64_t target = t.time_since_epoch().count();
  std::int64_t current = now_ns_.load(std::memory_order_acquire);
  // CAS loop so a concurrent advance() can never be lost or reversed. On
  // failure compare_exchange_weak reloads `current`, and the check repeats.
  for (;;) {
    if (target < current) {
      throw std::invalid_argument("FakeClock::setTime: time must not go backwards");
    }
    if (now_ns_.compare_exchange_weak(current, target, std::memory_order_acq_rel,
                                      std::memory_order_acquire)) {
      return;
    }
  }
}

}  // namespace robot_core
