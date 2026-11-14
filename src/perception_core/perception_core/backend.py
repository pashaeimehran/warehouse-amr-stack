"""Detector backend abstraction.

Every inference path (mock, PyTorch, ONNX Runtime, TensorRT) implements
DetectorBackend, so the ROS node and the evaluation tools never depend on a
specific framework, and CI runs everything GPU-free with MockBackend.
"""

from __future__ import annotations

import abc
from collections.abc import Callable, Sequence

import numpy as np

from perception_core.types import Detection


class DetectorBackend(abc.ABC):
    """Detects objects in one BGR uint8 image; returns detections in image pixels."""

    @property
    @abc.abstractmethod
    def name(self) -> str:
        """Short identifier used in diagnostics and benchmark tables."""

    @property
    @abc.abstractmethod
    def class_names(self) -> Sequence[str]:
        """Labels indexed by Detection.class_id."""

    @abc.abstractmethod
    def detect(self, image_bgr: np.ndarray) -> list[Detection]:
        """Runs inference. Must not keep a reference to image_bgr."""


class MockBackend(DetectorBackend):
    """Deterministic backend for tests: returns scripted detections.

    `script` is either a sequence of per-call detection lists (repeated
    cyclically) or a callable from the image to detections. It never inspects
    pixel content unless the callable does, so tests need no GPU and no model.
    """

    def __init__(self, class_names: Sequence[str],
                 script: Sequence[list[Detection]] | Callable[[np.ndarray], list[Detection]]):
        self._class_names = list(class_names)
        self._script = script
        self.calls = 0

    @property
    def name(self) -> str:
        return "mock"

    @property
    def class_names(self) -> Sequence[str]:
        return self._class_names

    def detect(self, image_bgr: np.ndarray) -> list[Detection]:
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("expected an HxWx3 image")
        index = self.calls
        self.calls += 1
        if callable(self._script):
            return list(self._script(image_bgr))
        if not self._script:
            return []
        return list(self._script[index % len(self._script)])
