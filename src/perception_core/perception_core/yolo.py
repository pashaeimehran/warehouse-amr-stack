"""Post-processing of anchor-free YOLO heads (YOLOv8 and YOLO11 output layout).

The exported model returns one tensor of shape (1, 4 + C, N): for each of N
candidate locations the box centre, width and height in model input pixels,
followed by C class scores (already sigmoid-activated, no objectness term).
Shared by every backend so that the decoding is tested once, without a model.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from perception_core.nms import nms
from perception_core.preprocess import LetterboxParams, to_original
from perception_core.types import Box, Detection


def decode(raw: np.ndarray, params: LetterboxParams, class_names: Sequence[str],
           confidence_threshold: float = 0.25, iou_threshold: float = 0.45,
           max_detections: int = 100) -> list[Detection]:
    """(1, 4 + C, N) head output -> detections in original image pixels."""
    out = np.asarray(raw, dtype=np.float32)
    if out.ndim == 3:
        if out.shape[0] != 1:
            raise ValueError("batch size must be 1")
        out = out[0]
    if out.ndim != 2 or out.shape[0] != 4 + len(class_names):
        raise ValueError(f"expected (4 + {len(class_names)}, N) output, got {out.shape}")
    candidates = out.T                                # (N, 4 + C)
    scores_all = candidates[:, 4:]
    class_ids = np.argmax(scores_all, axis=1)
    scores = scores_all[np.arange(len(class_ids)), class_ids]
    mask = scores >= confidence_threshold
    if not np.any(mask):
        return []
    cx, cy, w, h = (candidates[mask, k] for k in range(4))
    boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1)
    kept = nms(boxes, scores[mask], iou_threshold, class_ids[mask], max_detections)
    detections = []
    for i in kept:
        box = to_original(Box(*map(float, boxes[i])), params)
        if not box.is_valid():
            continue      # degenerate after clipping (entirely in the padding)
        cid = int(class_ids[mask][i])
        detections.append(Detection(cid, class_names[cid], float(scores[mask][i]), box))
    return detections
