from datetime import datetime, timezone

import pytest

from aeromind_apm_lite.common.sensors import DepthFrame, ImageEncoding, RgbFrame


def test_rgb_and_depth_frames_validate_exact_buffer_size():
    now = datetime.now(timezone.utc)
    rgb = RgbFrame(
        observed_at_utc=now,
        width=2,
        height=2,
        encoding=ImageEncoding.RGB8,
        data=bytes(12),
        source="airsim/front_center",
    )
    depth = DepthFrame(
        observed_at_utc=now,
        width=2,
        height=2,
        encoding=ImageEncoding.Z16_MM,
        data=bytes(8),
        depth_scale_m=0.001,
        source="realsense/depth",
    )

    assert len(rgb.data) == 12
    assert len(depth.data) == 8


def test_frame_rejects_mismatched_buffer():
    with pytest.raises(ValueError):
        RgbFrame(
            observed_at_utc=datetime.now(timezone.utc),
            width=2,
            height=2,
            encoding=ImageEncoding.RGB8,
            data=bytes(3),
            source="bad",
        )
