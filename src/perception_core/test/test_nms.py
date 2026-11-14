import numpy as np
import pytest

from perception_core.geometry import iou, iou_matrix
from perception_core.nms import nms
from perception_core.types import Box


def test_iou_known_values():
    a = Box(0, 0, 10, 10)
    assert iou(a, a) == pytest.approx(1.0)
    assert iou(a, Box(5, 0, 15, 10)) == pytest.approx(50 / 150)
    assert iou(a, Box(10, 0, 20, 10)) == 0.0          # touching edges
    assert iou(a, Box(20, 20, 30, 30)) == 0.0
    assert iou(Box(0, 0, 0, 0), Box(0, 0, 0, 0)) == 0.0


def test_iou_matrix_matches_scalar():
    boxes = [Box(0, 0, 10, 10), Box(5, 5, 15, 15), Box(100, 100, 110, 120)]
    arr = np.array([[b.x1, b.y1, b.x2, b.y2] for b in boxes])
    m = iou_matrix(arr, arr)
    for i, a in enumerate(boxes):
        for j, b in enumerate(boxes):
            assert m[i, j] == pytest.approx(iou(a, b))


def test_overlapping_same_class_is_suppressed():
    boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11], [50, 50, 60, 60]], float)
    scores = np.array([0.9, 0.8, 0.7])
    assert nms(boxes, scores, 0.5) == [0, 2]


def test_different_classes_are_not_suppressed():
    boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11]], float)
    assert nms(boxes, np.array([0.9, 0.8]), 0.5, class_ids=np.array([0, 1])) == [0, 1]


def test_threshold_is_strict():
    boxes = np.array([[0, 0, 10, 10], [5, 0, 15, 10]], float)   # IoU exactly 1/3
    assert nms(boxes, np.array([0.9, 0.8]), 1 / 3) == [0, 1]
    assert nms(boxes, np.array([0.9, 0.8]), 0.33) == [0]


def test_ties_break_by_index_and_order_is_by_score():
    boxes = np.array([[0, 0, 10, 10], [100, 0, 110, 10], [0, 0, 10, 10]], float)
    scores = np.array([0.5, 0.9, 0.5])
    assert nms(boxes, scores, 0.5) == [1, 0]   # 2 duplicates 0 with an equal score


def test_max_detections_and_empty_input():
    boxes = np.array([[i * 20, 0, i * 20 + 10, 10] for i in range(5)], float)
    assert len(nms(boxes, np.linspace(0.9, 0.5, 5), 0.5, max_detections=3)) == 3
    assert nms(np.zeros((0, 4)), np.zeros(0), 0.5) == []


def test_invalid_arguments():
    with pytest.raises(ValueError):
        nms(np.zeros((2, 4)), np.zeros(3), 0.5)
    with pytest.raises(ValueError):
        nms(np.zeros((1, 4)), np.zeros(1), 1.5)
