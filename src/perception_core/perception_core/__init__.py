"""ROS-independent perception core: preprocessing, detector backends, NMS.

Never imports ROS (rclpy, messages); the `perception` package adapts it.
"""

from perception_core.backend import DetectorBackend, MockBackend
from perception_core.geometry import iou, iou_matrix
from perception_core.nms import nms
from perception_core.preprocess import LetterboxParams, letterbox, to_original
from perception_core.types import Box, Detection

__all__ = [
    "Box", "Detection", "DetectorBackend", "LetterboxParams", "MockBackend", "iou",
    "iou_matrix", "letterbox", "nms", "to_original",
]
