"""Unit tests for the flat-ground vision target estimator."""

import math

from aeromind_apm_lite.ground.browser.visual_navigation import (
    estimate_target_ned,
)


def test_center_pixel_ray_is_horizontal_and_misses_ground():
    result = estimate_target_ned(
        frame_width_px=640,
        frame_height_px=360,
        fov_degrees=90.0,
        target_center_normalized=(0.5, 0.5),
        vehicle_position_ned=(0.0, 0.0, -2.0),
        vehicle_yaw_rad=0.0,
    )
    assert result is None  # horizontal ray never meets the ground plane


def test_below_center_target_lands_north_of_aircraft():
    result = estimate_target_ned(
        frame_width_px=640,
        frame_height_px=360,
        fov_degrees=90.0,
        target_center_normalized=(0.5, 0.75),
        vehicle_position_ned=(0.0, 0.0, -2.0),
        vehicle_yaw_rad=0.0,
    )
    assert result is not None
    north, east, down = result
    assert down == 0.0
    assert abs(north - 2.0 * math.cos(math.radians(22.5)) / math.sin(math.radians(22.5))) < 1e-3
    assert abs(east) < 1e-9


def test_yaw_rotates_target_estimation():
    result = estimate_target_ned(
        frame_width_px=640,
        frame_height_px=360,
        fov_degrees=90.0,
        target_center_normalized=(0.5, 0.75),
        vehicle_position_ned=(0.0, 0.0, -2.0),
        vehicle_yaw_rad=math.radians(90.0),
    )
    assert result is not None
    north, east, down = result
    assert abs(north) < 1e-9
    assert east > 0.0
    assert abs(east - 2.0 * math.cos(math.radians(22.5)) / math.sin(math.radians(22.5))) < 1e-3


def test_invalid_inputs_return_none():
    base = dict(
        frame_width_px=640,
        frame_height_px=360,
        fov_degrees=90.0,
        target_center_normalized=(0.5, 0.75),
        vehicle_position_ned=(0.0, 0.0, -2.0),
        vehicle_yaw_rad=0.0,
    )
    assert estimate_target_ned(**base) is not None
    assert estimate_target_ned(**{**base, "target_center_normalized": (1.5, 0.5)}) is None
    assert estimate_target_ned(**{**base, "frame_width_px": 0}) is None
    assert estimate_target_ned(**{**base, "vehicle_position_ned": (0.0, 0.0, -2.0, 1.0)}) is None
    assert estimate_target_ned(**{**base, "target_center_normalized": (0.5,)}) is None
