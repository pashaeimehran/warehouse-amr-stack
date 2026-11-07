#include <limits>

#include <gtest/gtest.h>

#include "robot_core/perception_types.hpp"

namespace robot_core {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

TEST(BoundingBox2D, GeometryOfValidBox) {
  const BoundingBox2D box{10.0, 20.0, 50.0, 100.0};
  EXPECT_TRUE(box.isValid());
  EXPECT_DOUBLE_EQ(box.width(), 40.0);
  EXPECT_DOUBLE_EQ(box.height(), 80.0);
  EXPECT_DOUBLE_EQ(box.area(), 3200.0);
}

TEST(BoundingBox2D, DegenerateAndInvertedBoxesAreInvalidWithZeroArea) {
  const BoundingBox2D zero_width{10.0, 20.0, 10.0, 100.0};
  const BoundingBox2D inverted{50.0, 20.0, 10.0, 100.0};
  const BoundingBox2D nan_box{kNaN, 0.0, 1.0, 1.0};
  for (const auto& box : {zero_width, inverted, nan_box}) {
    EXPECT_FALSE(box.isValid());
    EXPECT_DOUBLE_EQ(box.area(), 0.0);
  }
}

TEST(Detection, ValidityRequiresConfidenceInUnitIntervalAndValidBox) {
  Detection d{ObjectClass::Person, 0.9, BoundingBox2D{0.0, 0.0, 10.0, 10.0}, Timestamp{}};
  EXPECT_TRUE(isValid(d));
  d.confidence = 1.0;
  EXPECT_TRUE(isValid(d));
  d.confidence = 1.01;
  EXPECT_FALSE(isValid(d));
  d.confidence = -0.01;
  EXPECT_FALSE(isValid(d));
  d.confidence = kNaN;
  EXPECT_FALSE(isValid(d));
  d.confidence = 0.5;
  d.box = BoundingBox2D{0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(isValid(d));
}

TEST(ObjectClass, StringNamesMatchDatasetTaxonomy) {
  EXPECT_EQ(toString(ObjectClass::Person), "person");
  EXPECT_EQ(toString(ObjectClass::KltBox), "klt_box");
  EXPECT_EQ(toString(ObjectClass::ShelfItem), "shelf_item");
  EXPECT_EQ(toString(TrackSource::Fused), "fused");
}

TrackedObject makeTrack(std::array<double, 4> cov) {
  TrackedObject t;
  t.track_id = 7;
  t.object_class = ObjectClass::Person;
  t.velocity = Vector2D{1.2, 0.0};
  t.position_covariance = cov;
  return t;
}

TEST(TrackedObject, SpeedIsVelocityNorm) {
  TrackedObject t = makeTrack({0.1, 0.0, 0.0, 0.1});
  t.velocity = Vector2D{0.6, 0.8};
  EXPECT_DOUBLE_EQ(t.speed(), 1.0);
}

TEST(TrackedObject, AcceptsSymmetricPositiveSemidefiniteCovariance) {
  EXPECT_TRUE(hasValidCovariance(makeTrack({0.04, 0.01, 0.01, 0.09})));
  EXPECT_TRUE(hasValidCovariance(makeTrack({0.0, 0.0, 0.0, 0.0})));
}

TEST(TrackedObject, RejectsAsymmetricNegativeOrIndefiniteCovariance) {
  EXPECT_FALSE(hasValidCovariance(makeTrack({0.04, 0.01, 0.02, 0.09})));  // asymmetric
  EXPECT_FALSE(hasValidCovariance(makeTrack({-0.01, 0.0, 0.0, 0.09})));   // negative variance
  EXPECT_FALSE(hasValidCovariance(makeTrack({0.01, 0.5, 0.5, 0.01})));    // det < 0
  EXPECT_FALSE(hasValidCovariance(makeTrack({kNaN, 0.0, 0.0, 0.01})));
}

}  // namespace
}  // namespace robot_core
