"""Value types of the perception core. Plain, immutable dataclasses; no ROS."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Box:
    """Axis-aligned box in pixel coordinates, (x1, y1) top left, (x2, y2) bottom right."""

    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def area(self) -> float:
        return max(self.width, 0.0) * max(self.height, 0.0)

    def is_valid(self) -> bool:
        values = (self.x1, self.y1, self.x2, self.y2)
        return all(math.isfinite(v) for v in values) and self.x2 > self.x1 and self.y2 > self.y1


@dataclass(frozen=True)
class Detection:
    """One detection in original image coordinates.

    class_id indexes the backend's class list; label is its name. confidence
    is in [0, 1]. Detections are untrusted input to the deterministic layers,
    which validate and timestamp-check them (the source stamp travels with the
    image, outside this type).
    """

    class_id: int
    label: str
    confidence: float
    box: Box

    def is_valid(self) -> bool:
        return (self.class_id >= 0 and 0.0 <= self.confidence <= 1.0 and self.box.is_valid())
