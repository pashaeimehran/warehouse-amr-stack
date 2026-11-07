#include "robot_core/perception_types.hpp"

#include <cmath>

namespace robot_core {

std::string_view toString(ObjectClass c) {
  switch (c) {
    case ObjectClass::Unknown:
      return "unknown";
    case ObjectClass::Person:
      return "person";
    case ObjectClass::Pallet:
      return "pallet";
    case ObjectClass::KltBox:
      return "klt_box";
    case ObjectClass::Amr:
      return "amr";
    case ObjectClass::ShelfItem:
      return "shelf_item";
  }
  return "invalid";
}

std::string_view toString(TrackSource s) {
  switch (s) {
    case TrackSource::Camera:
      return "camera";
    case TrackSource::Lidar:
      return "lidar";
    case TrackSource::Fused:
      return "fused";
  }
  return "invalid";
}

double BoundingBox2D::area() const {
  return isValid() ? width() * height() : 0.0;
}

bool BoundingBox2D::isValid() const {
  return std::isfinite(x_min) && std::isfinite(y_min) && std::isfinite(x_max) &&
         std::isfinite(y_max) && x_max > x_min && y_max > y_min;
}

bool isValid(const Detection& detection) {
  return std::isfinite(detection.confidence) && detection.confidence >= 0.0 &&
         detection.confidence <= 1.0 && detection.box.isValid();
}

bool hasValidCovariance(const TrackedObject& track, double symmetry_tolerance) {
  const auto& c = track.position_covariance;
  for (const double v : c) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  const double xx = c[0];
  const double xy = c[1];
  const double yx = c[2];
  const double yy = c[3];
  if (std::abs(xy - yx) > symmetry_tolerance) {
    return false;
  }
  // A symmetric 2x2 matrix is PSD iff both diagonal entries and the determinant are >= 0.
  return xx >= 0.0 && yy >= 0.0 && ((xx * yy) - (xy * yx)) >= 0.0;
}

}  // namespace robot_core
