#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "robot_core/geometry.hpp"

namespace robot_core {
namespace {

constexpr double kTol = 1e-12;

TEST(NormalizeAngle, LeavesInRangeAnglesUnchanged) {
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, kTol);
  EXPECT_NEAR(normalizeAngle(1.0), 1.0, kTol);
  EXPECT_NEAR(normalizeAngle(-1.0), -1.0, kTol);
}

TEST(NormalizeAngle, WrapsIntoHalfOpenRange) {
  EXPECT_NEAR(normalizeAngle(kPi), -kPi, kTol);  // +pi maps to -pi: range is [-pi, pi)
  EXPECT_NEAR(normalizeAngle(-kPi), -kPi, kTol);
  EXPECT_NEAR(normalizeAngle(3.0 * kPi), -kPi, 1e-9);
  EXPECT_NEAR(normalizeAngle(2.0 * kPi + 0.5), 0.5, 1e-9);
  EXPECT_NEAR(normalizeAngle(-2.0 * kPi - 0.5), -0.5, 1e-9);
}

TEST(NormalizeAngle, ResultAlwaysInRange) {
  for (int i = -135; i <= 135; ++i) {
    const double a = 0.37 * static_cast<double>(i);  // covers [-50, 50] rad
    const double n = normalizeAngle(a);
    ASSERT_GE(n, -kPi);
    ASSERT_LT(n, kPi);
    ASSERT_NEAR(std::sin(n), std::sin(a), 1e-9);
    ASSERT_NEAR(std::cos(n), std::cos(a), 1e-9);
  }
}

TEST(AngleDifference, TakesTheShortWayAround) {
  // 170 deg vs -170 deg differ by 20 deg, not 340 deg.
  const double a = 170.0 * kPi / 180.0;
  const double b = -170.0 * kPi / 180.0;
  EXPECT_NEAR(angleDifference(a, b), -20.0 * kPi / 180.0, 1e-9);
  EXPECT_NEAR(angleDifference(b, a), 20.0 * kPi / 180.0, 1e-9);
}

TEST(Distance, EuclideanIgnoresYaw) {
  EXPECT_NEAR(distance(Pose2D{0.0, 0.0, 0.0}, Pose2D{3.0, 4.0, 2.0}), 5.0, kTol);
  EXPECT_NEAR(distance(Point2D{1.0, 1.0}, Point2D{1.0, -1.0}), 2.0, kTol);
}

TEST(Pose2D, EqualityComparesAllFields) {
  EXPECT_EQ((Pose2D{1.0, 2.0, 0.5}), (Pose2D{1.0, 2.0, 0.5}));
  EXPECT_NE((Pose2D{1.0, 2.0, 0.5}), (Pose2D{1.0, 2.0, 0.6}));
}

TEST(Pose2D, FinitenessCheck) {
  EXPECT_TRUE(isFinite(Pose2D{1.0, 2.0, 3.0}));
  EXPECT_FALSE(isFinite(Pose2D{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}));
  EXPECT_FALSE(isFinite(Pose2D{0.0, 0.0, std::numeric_limits<double>::infinity()}));
}

TEST(Quaternion, DefaultIsIdentityAndNormalized) {
  const Quaternion q;
  EXPECT_DOUBLE_EQ(q.w, 1.0);
  EXPECT_TRUE(isNormalized(q));
}

TEST(Quaternion, DetectsNonUnitQuaternion) {
  EXPECT_FALSE(isNormalized(Quaternion{1.0, 1.0, 0.0, 0.0}));
  EXPECT_FALSE(isNormalized(Quaternion{0.0, 0.0, 0.0, 0.0}));
  const double h = std::sqrt(0.5);
  EXPECT_TRUE(isNormalized(Quaternion{h, 0.0, 0.0, h}));
}

TEST(Pose3D, FinitenessCheck) {
  Pose3D p;
  EXPECT_TRUE(isFinite(p));
  p.orientation.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isFinite(p));
}

TEST(Vector2D, Norm) {
  EXPECT_NEAR((Vector2D{-3.0, 4.0}).norm(), 5.0, kTol);
}

}  // namespace
}  // namespace robot_core
