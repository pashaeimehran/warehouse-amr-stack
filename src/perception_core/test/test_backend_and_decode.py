import ast
from pathlib import Path

import numpy as np
import pytest

from perception_core.backend import DetectorBackend, MockBackend
from perception_core.preprocess import letterbox
from perception_core.types import Box, Detection
from perception_core.yolo import decode

CLASSES = ["person", "pallet", "klt_box"]


def head(candidates, n_classes=3):
    """Builds a (1, 4 + C, N) head output from (cx, cy, w, h, class, score) tuples."""
    out = np.zeros((4 + n_classes, len(candidates)), dtype=np.float32)
    for k, (cx, cy, w, h, cls, score) in enumerate(candidates):
        out[:4, k] = (cx, cy, w, h)
        out[4 + cls, k] = score
    return out[None]


def test_decode_maps_to_original_and_applies_nms():
    _, params = letterbox(np.zeros((480, 640, 3), np.uint8), 640)   # pad_y 80
    raw = head([
        (320, 320, 100, 200, 0, 0.9),   # person
        (322, 318, 100, 200, 0, 0.6),   # duplicate person, suppressed
        (100, 400, 50, 50, 2, 0.8),     # klt box
        (500, 300, 40, 40, 1, 0.1),     # below the confidence threshold
    ])
    dets = decode(raw, params, CLASSES, confidence_threshold=0.25, iou_threshold=0.45)
    assert [d.label for d in dets] == ["person", "klt_box"]
    person = dets[0].box
    assert (person.x1, person.y1, person.x2, person.y2) == pytest.approx((270, 140, 370, 340))
    assert dets[0].confidence == pytest.approx(0.9)
    assert all(d.is_valid() for d in dets)


def test_decode_drops_boxes_entirely_in_padding():
    _, params = letterbox(np.zeros((480, 640, 3), np.uint8), 640)
    raw = head([(320, 20, 40, 20, 0, 0.9)])   # inside the top padding band
    assert decode(raw, params, CLASSES) == []


def test_decode_rejects_wrong_layout():
    _, params = letterbox(np.zeros((64, 64, 3), np.uint8), 64)
    with pytest.raises(ValueError):
        decode(np.zeros((1, 6, 10)), params, CLASSES)


def test_mock_backend_is_scripted_and_cyclic():
    person = Detection(0, "person", 0.9, Box(0, 0, 10, 20))
    backend = MockBackend(CLASSES, [[person], []])
    image = np.zeros((10, 10, 3), np.uint8)
    assert isinstance(backend, DetectorBackend)
    assert backend.name == "mock"
    assert [backend.detect(image) for _ in range(3)] == [[person], [], [person]]
    assert backend.calls == 3
    with pytest.raises(ValueError):
        backend.detect(np.zeros((10, 10), np.uint8))


def test_mock_backend_callable_script():
    backend = MockBackend(CLASSES, lambda img: [Detection(1, "pallet", 0.5,
                                                          Box(0, 0, img.shape[1], img.shape[0]))])
    det = backend.detect(np.zeros((20, 30, 3), np.uint8))[0]
    assert (det.box.x2, det.box.y2) == (30, 20)


def test_detection_validation():
    assert not Detection(0, "person", 1.5, Box(0, 0, 1, 1)).is_valid()
    assert not Detection(0, "person", 0.5, Box(1, 1, 1, 2)).is_valid()
    assert not Detection(-1, "x", 0.5, Box(0, 0, 1, 1)).is_valid()


def test_package_never_imports_ros():
    forbidden = ("rclpy", "rosidl", "sensor_msgs", "vision_msgs", "cv_bridge", "std_msgs")
    package = Path(__file__).resolve().parents[1] / "perception_core"
    for source in package.glob("*.py"):
        tree = ast.parse(source.read_text())
        for node in ast.walk(tree):
            names = ([a.name for a in node.names] if isinstance(node, ast.Import) else
                     [node.module or ""] if isinstance(node, ast.ImportFrom) else [])
            for name in names:
                assert not name.split(".")[0] in forbidden, f"{source.name} imports {name}"
