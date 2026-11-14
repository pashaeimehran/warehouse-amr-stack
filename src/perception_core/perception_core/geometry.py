"""Box geometry: intersection over union."""

from __future__ import annotations

import numpy as np

from perception_core.types import Box


def iou(a: Box, b: Box) -> float:
    """Intersection over union of two boxes; 0 if they do not overlap or both are empty."""
    ix = min(a.x2, b.x2) - max(a.x1, b.x1)
    iy = min(a.y2, b.y2) - max(a.y1, b.y1)
    if ix <= 0.0 or iy <= 0.0:
        return 0.0
    inter = ix * iy
    union = a.area + b.area - inter
    return inter / union if union > 0.0 else 0.0


def iou_matrix(boxes_a: np.ndarray, boxes_b: np.ndarray) -> np.ndarray:
    """Pairwise IoU of (N, 4) and (M, 4) arrays in xyxy format; returns (N, M)."""
    a = np.asarray(boxes_a, dtype=np.float64).reshape(-1, 4)
    b = np.asarray(boxes_b, dtype=np.float64).reshape(-1, 4)
    ix = np.clip(np.minimum(a[:, None, 2], b[None, :, 2]) - np.maximum(a[:, None, 0], b[None, :, 0]),
                 0.0, None)
    iy = np.clip(np.minimum(a[:, None, 3], b[None, :, 3]) - np.maximum(a[:, None, 1], b[None, :, 1]),
                 0.0, None)
    inter = ix * iy
    area_a = np.clip(a[:, 2] - a[:, 0], 0.0, None) * np.clip(a[:, 3] - a[:, 1], 0.0, None)
    area_b = np.clip(b[:, 2] - b[:, 0], 0.0, None) * np.clip(b[:, 3] - b[:, 1], 0.0, None)
    union = area_a[:, None] + area_b[None, :] - inter
    with np.errstate(divide="ignore", invalid="ignore"):
        result = np.where(union > 0.0, inter / union, 0.0)
    return result
