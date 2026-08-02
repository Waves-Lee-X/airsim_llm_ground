# -*- coding: utf-8 -*-
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from aeromind_apm_lite.ground.browser.vision_evidence import (  # noqa: E402
    ColorDetector,
    utc_now,
)


def _frame_bytes(image_bgr):
    ok, encoded = cv2.imencode(".jpg", image_bgr)
    assert ok
    return encoded.tobytes()


def test_detector_measures_target_region_instead_of_whole_frame():
    detector = ColorDetector()
    frame = np.full((360, 640, 3), (128, 128, 128), dtype=np.uint8)
    # Small orange blob (right of center) so the whole frame stays gray.
    cv2.circle(frame, (400, 180), 25, (0, 120, 255), -1)  # hue ~28deg -> OpenCV 14, palette orange
    data = _frame_bytes(frame)

    whole = detector.detect(
        data,
        sequence=1,
        captured_at_utc=utc_now(),
    )
    assert whole is None or whole.color == "gray"

    region = detector.detect(
        data,
        sequence=2,
        captured_at_utc=utc_now(),
        region_center=(400 / 640, 180 / 360),
    )
    assert region is not None
    assert region.color == "orange"


def test_detector_region_ignores_invalid_center():
    detector = ColorDetector()
    frame = np.full((360, 640, 3), (128, 128, 128), dtype=np.uint8)
    data = _frame_bytes(frame)
    result = detector.detect(
        data,
        sequence=3,
        captured_at_utc=utc_now(),
        region_center=(-0.5, 0.5),
    )
    assert result is None or result.color == "gray"
