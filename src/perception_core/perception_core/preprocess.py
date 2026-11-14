"""Deterministic image preprocessing for single-stage detectors (letterbox).

The model sees a square input of `size` x `size`: the image is scaled to fit
without changing its aspect ratio, centred, and padded with a constant grey.
`LetterboxParams` keeps the transform so detections map back to the original
image exactly.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from perception_core.types import Box

PAD_VALUE = 114   # grey used by the YOLO family during training


@dataclass(frozen=True)
class LetterboxParams:
    scale: float          # model pixels per original pixel
    pad_x: float          # left padding in model pixels
    pad_y: float          # top padding in model pixels
    size: int             # model input edge
    original_width: int
    original_height: int


def letterbox(image_bgr: np.ndarray, size: int = 640,
              pad_value: int = PAD_VALUE) -> tuple[np.ndarray, LetterboxParams]:
    """BGR uint8 HxWx3 image -> (1x3xSxS float32 RGB tensor in [0, 1], params)."""
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or image_bgr.dtype != np.uint8:
        raise ValueError("expected an HxWx3 uint8 BGR image")
    if size <= 0:
        raise ValueError("size must be positive")
    height, width = image_bgr.shape[:2]
    scale = min(size / width, size / height)
    new_w, new_h = int(round(width * scale)), int(round(height * scale))
    resized = cv2.resize(image_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    pad_x = (size - new_w) // 2
    pad_y = (size - new_h) // 2
    canvas = np.full((size, size, 3), pad_value, dtype=np.uint8)
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    rgb = canvas[:, :, ::-1]
    tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1), dtype=np.float32)[None] / 255.0
    params = LetterboxParams(scale=scale, pad_x=float(pad_x), pad_y=float(pad_y), size=size,
                             original_width=width, original_height=height)
    return tensor.astype(np.float32), params


def to_original(box: Box, params: LetterboxParams) -> Box:
    """Maps a box from model input coordinates back to the original image, clipped to it."""
    def x(v: float) -> float:
        return float(np.clip((v - params.pad_x) / params.scale, 0.0, params.original_width))

    def y(v: float) -> float:
        return float(np.clip((v - params.pad_y) / params.scale, 0.0, params.original_height))

    return Box(x(box.x1), y(box.y1), x(box.x2), y(box.y2))
