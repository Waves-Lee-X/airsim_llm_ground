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
    # Vertical FOV follows from the 16:9 aspect ratio.
    half_v = math.atan(math.tan(math.radians(45.0)) / (640.0 / 360.0))
    assert abs(north - 2.0 / math.tan(half_v / 2.0)) < 1e-3
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
    half_v = math.atan(math.tan(math.radians(45.0)) / (640.0 / 360.0))
    assert abs(east - 2.0 / math.tan(half_v / 2.0)) < 1e-3


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


def test_line_intersection_and_standoff_geometry():
    from aeromind_apm_lite.ground.browser.mission_planner import (
        _line_intersection,
        _standoff_point,
    )

    point = _line_intersection((0.0, 0.0), (1.0, 0.0), (5.0, 2.0), (0.0, 1.0))
    assert point is not None
    assert abs(point[0] - 5.0) < 1e-6 and abs(point[1] - 0.0) < 1e-6

    assert (
        _line_intersection((0.0, 0.0), (1.0, 0.0), (5.0, 0.0), (1.0, 0.0))
        is None
    )

    # intersection behind the observer (t < 0) is rejected
    assert (
        _line_intersection((0.0, 0.0), (1.0, 0.0), (-5.0, 2.0), (0.0, 1.0))
        is None
    )

    assert _standoff_point((10.0, 0.0), (0.0, 0.0), 4.0) == (6.0, 0.0)
    assert _standoff_point((0.0, 10.0), (0.0, 0.0), 4.0) == (0.0, 6.0)
    px, py = _standoff_point((6.0, 8.0), (0.0, 0.0), 4.0)
    assert abs(px - 3.6) < 1e-9 and abs(py - 4.8) < 1e-9


def test_estimate_object_radius_from_bbox_and_depth():
    from aeromind_apm_lite.ground.browser.visual_navigation import (
        estimate_object_radius,
    )

    # 640px wide, 95 deg FOV -> focal ~296.6 px; 100 px bbox at 10 m
    radius = estimate_object_radius(
        bbox_height_px=100.0,
        distance_m=10.0,
        frame_width_px=640,
        fov_degrees=95.0,
    )
    assert radius is not None
    expected = 50.0 * 10.0 / (320.0 / math.tan(math.radians(47.5)))
    assert abs(radius - expected) < 1e-6
    assert estimate_object_radius(
        bbox_height_px=0.0, distance_m=10.0, frame_width_px=640, fov_degrees=95.0
    ) is None
    assert estimate_object_radius(
        bbox_height_px=50.0, distance_m=0.0, frame_width_px=640, fov_degrees=95.0
    ) is None
