#include "robot_core/geometry.hpp"

#include <cmath>

namespace robot_core {

double normalizeAngle(double angle_rad) {
  constexpr double kTwoPi = 2.0 * kPi;
  double wrapped = std::remainder(angle_rad, kTwoPi);  // [-pi, pi]
  if (wrapped >= kPi) {
    wrapped -= kTwoPi;
  }
  return wrapped;
}

double angleDifference(double a_rad, double b_rad) {
  return normalizeAngle(a_rad - b_rad);
}

double Vector2D::norm() const {
  return std::hypot(x, y);
}

double distance(const Point2D& a, const Point2D& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double distance(const Pose2D& a, const Pose2D& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool isFinite(const Pose2D& pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

double Quaternion::norm() const {
  return std::sqrt((w * w) + (x * x) + (y * y) + (z * z));
}

bool isNormalized(const Quaternion& q, double tolerance) {
  const double n = q.norm();
  return std::isfinite(n) && std::abs(n - 1.0) <= tolerance;
}

bool isFinite(const Pose3D& pose) {
  const auto& p = pose.position;
  const auto& q = pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(q.w) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

}  // namespace robot_core
