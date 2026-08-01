"""Vision target navigation helpers.

Estimates the local-NED position of a visual target from the VLM bounding-box
center using a flat-ground assumption (no depth sensor required): the camera
ray through the target pixel is intersected with the target plane (default:
ground level z=0 in NED). Accuracy is best when the target sits on the ground
and the aircraft is roughly level; used by the planner ``analyze`` step with
``navigate_after``.
"""

from __future__ import annotations

import math
from typing import Sequence

_MAX_RANGE_M = 300.0
_MIN_DOWNWARD_SLOPE = 0.05


def _deg2rad(value: float) -> float:
    return math.radians(float(value))


def estimate_target_ned(
    *,
    frame_width_px: int,
    frame_height_px: int,
    fov_degrees: float,
    target_center_normalized: Sequence[float],
    vehicle_position_ned: Sequence[float],
    vehicle_yaw_rad: float,
    target_plane_z_ned: float = 0.0,
) -> tuple[float, float, float] | None:
    """Estimate the target NED position from a normalized pixel center.

    ``target_center_normalized`` is ``(cx, cy)`` in [0, 1] with x increasing to
    the right of the image and y increasing downward. Returns ``(north, east,
    down)`` or ``None`` when the ray does not intersect the target plane or is
    out of range.
    """
    if frame_width_px <= 0 or frame_height_px <= 0:
        return None
    if fov_degrees <= 0.0:
        return None
    if len(target_center_normalized) != 2:
        return None
    cx = float(target_center_normalized[0])
    cy = float(target_center_normalized[1])
    if not (0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0):
        return None
    if len(vehicle_position_ned) != 3:
        return None
    px, py, pz = (float(value) for value in vehicle_position_ned)

    horizontal = (cx - 0.5) * 2.0  # -1 .. 1, right positive
    vertical = (0.5 - cy) * 2.0    # -1 .. 1, up positive
    half = _deg2rad(fov_degrees) / 2.0
    h_angle = horizontal * half
    v_angle = vertical * half

    # Camera ray in the body frame (x forward, y right, z down).
    dir_x_b = math.cos(v_angle) * math.cos(h_angle)
    dir_y_b = math.cos(v_angle) * math.sin(h_angle)
    dir_z_b = -math.sin(v_angle)

    yaw = float(vehicle_yaw_rad)
    dir_x = math.cos(yaw) * dir_x_b - math.sin(yaw) * dir_y_b
    dir_y = math.sin(yaw) * dir_x_b + math.cos(yaw) * dir_y_b
    dir_z = dir_z_b

    if dir_z < _MIN_DOWNWARD_SLOPE:
        return None  # ray points up or level; flat-ground assumption fails

    distance = (float(target_plane_z_ned) - pz) / dir_z
    if distance <= 0.0 or distance > _MAX_RANGE_M:
        return None
    return (
        round(px + distance * dir_x, 3),
        round(py + distance * dir_y, 3),
        round(float(target_plane_z_ned), 3),
    )


def detect_salient_target_center(
    image_bytes: bytes,
    *,
    frame_width_px: int,
    frame_height_px: int,
) -> tuple[float, float] | None:
    """Return the normalized center of the largest non-background color blob.

    Background-ish colors (gray/white/black) are excluded so the ground plane
    does not win; useful as a deterministic fallback when the VLM does not
    report pixel coordinates. Returns ``(center_x, center_y)`` in [0, 1] or
    ``None``.
    """
    try:
        import cv2
        import numpy as np
    except ImportError:  # pragma: no cover - ground extra dependency
        return None
    if not image_bytes or frame_width_px <= 0 or frame_height_px <= 0:
        return None
    encoded = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        return None
    height, width = image.shape[:2]
    if width <= 0 or height <= 0:
        return None
    scale_x = frame_width_px / width
    scale_y = frame_height_px / height
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Saturation/Value mask keeps vivid objects and drops gray/white/black.
    mask = cv2.inRange(hsv, (0, 80, 60), (179, 255, 255))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 200.0:
        return None
    moments = cv2.moments(largest)
    if moments.get("m00", 0.0) <= 0.0:
        return None
    cx = moments["m10"] / moments["m00"] * scale_x / frame_width_px
    cy = moments["m01"] / moments["m00"] * scale_y / frame_height_px
    if not (0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0):
        return None
    return (round(cx, 4), round(cy, 4))
