import numpy as np
import pytest

from perception_core.preprocess import PAD_VALUE, letterbox, to_original
from perception_core.types import Box


def image(h, w, value=(10, 20, 30)):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :] = value
    return img


@pytest.mark.parametrize("h,w", [(480, 640), (640, 480), (640, 640), (100, 1000), (37, 53)])
def test_shape_dtype_and_range(h, w):
    tensor, params = letterbox(image(h, w), 640)
    assert tensor.shape == (1, 3, 640, 640)
    assert tensor.dtype == np.float32
    assert tensor.min() >= 0.0 and tensor.max() <= 1.0
    # The longer side fills the input exactly.
    assert max(round(w * params.scale), round(h * params.scale)) == 640


def test_landscape_padding_is_symmetric_grey():
    tensor, params = letterbox(image(480, 640), 640)
    assert params.scale == pytest.approx(1.0)
    assert (params.pad_x, params.pad_y) == (0.0, 80.0)
    grey = PAD_VALUE / 255.0
    assert np.allclose(tensor[0, :, :80, :], grey)          # top band
    assert np.allclose(tensor[0, :, 560:, :], grey)         # bottom band
    assert not np.allclose(tensor[0, :, 80:560, :], grey)   # image area


def test_bgr_is_converted_to_rgb_channel_first():
    tensor, _ = letterbox(image(64, 64, value=(255, 0, 0)), 64)   # pure blue in BGR
    assert np.allclose(tensor[0, 2], 1.0)   # blue lands in channel 2 (RGB order)
    assert np.allclose(tensor[0, 0], 0.0)


def test_deterministic():
    rng = np.random.default_rng(42)
    img = rng.integers(0, 256, size=(300, 500, 3), dtype=np.uint8)
    a, pa = letterbox(img, 320)
    b, pb = letterbox(img.copy(), 320)
    assert np.array_equal(a, b) and pa == pb


def test_box_round_trip_and_clipping():
    _, params = letterbox(image(480, 640), 320)   # scale 0.5, pad_y 40
    model_box = Box(50.0, 40.0 + 25.0, 150.0, 40.0 + 125.0)
    original = to_original(model_box, params)
    assert (original.x1, original.y1, original.x2, original.y2) == pytest.approx(
        (100.0, 50.0, 300.0, 250.0))
    clipped = to_original(Box(-10.0, 0.0, 400.0, 400.0), params)
    assert clipped.x1 == 0.0 and clipped.y1 == 0.0
    assert clipped.x2 == 640.0 and clipped.y2 == 480.0


@pytest.mark.parametrize("bad", [np.zeros((10, 10), np.uint8), np.zeros((10, 10, 3), np.float32),
                                 np.zeros((10, 10, 4), np.uint8)])
def test_rejects_bad_input(bad):
    with pytest.raises(ValueError):
        letterbox(bad, 64)
