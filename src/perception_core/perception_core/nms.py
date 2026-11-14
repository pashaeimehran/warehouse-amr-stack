"""Own non-maximum suppression (greedy, class-aware, deterministic)."""

from __future__ import annotations

import numpy as np

from perception_core.geometry import iou_matrix


def nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float,
        class_ids: np.ndarray | None = None, max_detections: int | None = None) -> list[int]:
    """Indices of the boxes kept by greedy NMS, highest score first.

    A box is suppressed when its IoU with an already kept box of the same class
    is strictly greater than iou_threshold. Without class_ids all boxes are one
    class. Equal scores are broken by the lower index, so the result does not
    depend on the sort implementation.
    """
    boxes = np.asarray(boxes, dtype=np.float64).reshape(-1, 4)
    scores = np.asarray(scores, dtype=np.float64).reshape(-1)
    if boxes.shape[0] != scores.shape[0]:
        raise ValueError("boxes and scores differ in length")
    if not 0.0 <= iou_threshold <= 1.0:
        raise ValueError("iou_threshold must be in [0, 1]")
    n = boxes.shape[0]
    if n == 0:
        return []
    classes = (np.zeros(n, dtype=np.int64) if class_ids is None
               else np.asarray(class_ids, dtype=np.int64).reshape(-1))
    if classes.shape[0] != n:
        raise ValueError("class_ids and scores differ in length")

    # lexsort sorts by the last key first: score descending, then index ascending.
    order = np.lexsort((np.arange(n), -scores))
    overlaps = iou_matrix(boxes, boxes)
    suppressed = np.zeros(n, dtype=bool)
    keep: list[int] = []
    for i in order:
        if suppressed[i]:
            continue
        keep.append(int(i))
        if max_detections is not None and len(keep) >= max_detections:
            break
        same_class = classes == classes[i]
        suppressed |= same_class & (overlaps[i] > iou_threshold)
    return keep
