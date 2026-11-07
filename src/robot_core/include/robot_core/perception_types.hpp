#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "robot_core/geometry.hpp"
#include "robot_core/time.hpp"

namespace robot_core {

/// Object classes the detector is trained on (roadmap week 5 taxonomy).
enum class ObjectClass : std::uint8_t { Unknown, Person, Pallet, KltBox, Amr, ShelfItem };

[[nodiscard]] std::string_view toString(ObjectClass c);

/// Axis-aligned image-space box in pixels; (x_min, y_min) is the top-left corner.
struct BoundingBox2D {
  double x_min{0.0};
  double y_min{0.0};
  double x_max{0.0};
  double y_max{0.0};

  [[nodiscard]] double width() const { return x_max - x_min; }
  [[nodiscard]] double height() const { return y_max - y_min; }
  [[nodiscard]] double area() const;
  /// Finite coordinates and strictly positive width and height.
  [[nodiscard]] bool isValid() const;
};

/// One 2D detection from the learned detector. Untrusted input: consumers must
/// check isValid() and staleness against source_stamp before using it.
struct Detection {
  ObjectClass object_class{ObjectClass::Unknown};
  double confidence{0.0};  ///< [0, 1]
  BoundingBox2D box;
  Timestamp source_stamp;  ///< Capture time of the source image, not processing time.
};

[[nodiscard]] bool isValid(const Detection& detection);

/// Which sensors contributed to a track (track-level fusion, roadmap week 3).
enum class TrackSource : std::uint8_t { Camera, Lidar, Fused };

[[nodiscard]] std::string_view toString(TrackSource s);

/// A tracked object in the map frame, as produced by the tracker.
struct TrackedObject {
  std::uint64_t track_id{0};
  ObjectClass object_class{ObjectClass::Unknown};
  Point2D position;
  Vector2D velocity;
  /// Row-major 2x2 position covariance [xx, xy, yx, yy] in m^2.
  std::array<double, 4> position_covariance{0.0, 0.0, 0.0, 0.0};
  TrackSource source{TrackSource::Camera};
  Timestamp stamp;

  [[nodiscard]] double speed() const { return velocity.norm(); }
};

/// Covariance is finite, symmetric (within tolerance) and positive semidefinite.
[[nodiscard]] bool hasValidCovariance(const TrackedObject& track, double symmetry_tolerance = 1e-9);

}  // namespace robot_core
