#pragma once

namespace robot_core {

// All quantities are SI (m, rad, s). Planar types are expressed in the map
// frame unless a frame_id says otherwise (REP 103 / REP 105 conventions).

inline constexpr double kPi = 3.14159265358979323846;

/// Wraps an angle to [-pi, pi).
[[nodiscard]] double normalizeAngle(double angle_rad);

/// Smallest signed difference a - b, wrapped to [-pi, pi).
[[nodiscard]] double angleDifference(double a_rad, double b_rad);

struct Point2D {
  double x{0.0};
  double y{0.0};
  friend bool operator==(const Point2D& a, const Point2D& b) { return a.x == b.x && a.y == b.y; }
  friend bool operator!=(const Point2D& a, const Point2D& b) { return !(a == b); }
};

struct Vector2D {
  double x{0.0};
  double y{0.0};
  [[nodiscard]] double norm() const;
  friend bool operator==(const Vector2D& a, const Vector2D& b) { return a.x == b.x && a.y == b.y; }
  friend bool operator!=(const Vector2D& a, const Vector2D& b) { return !(a == b); }
};

/// Planar pose of the mobile base.
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  friend bool operator==(const Pose2D& a, const Pose2D& b) {
    return a.x == b.x && a.y == b.y && a.yaw == b.yaw;
  }
  friend bool operator!=(const Pose2D& a, const Pose2D& b) { return !(a == b); }
};

[[nodiscard]] double distance(const Point2D& a, const Point2D& b);
[[nodiscard]] double distance(const Pose2D& a, const Pose2D& b);
[[nodiscard]] bool isFinite(const Pose2D& pose);

struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// Unit quaternion (Hamilton convention, w first).
struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  [[nodiscard]] double norm() const;
};

/// True if |q| is within `tolerance` of 1.
[[nodiscard]] bool isNormalized(const Quaternion& q, double tolerance = 1e-6);

/// 6-DOF pose, used for grasp and place targets.
struct Pose3D {
  Vector3 position;
  Quaternion orientation;
};

[[nodiscard]] bool isFinite(const Pose3D& pose);

}  // namespace robot_core
