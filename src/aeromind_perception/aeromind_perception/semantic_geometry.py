"""Pure geometry helpers for the semantic world-model fusion node."""

from __future__ import annotations

import math
from typing import Iterable


def project_pixel(u: float, v: float, depth: float, camera_matrix: Iterable[float]):
    matrix = list(camera_matrix)
    fx, fy = float(matrix[0]), float(matrix[4])
    cx, cy = float(matrix[2]), float(matrix[5])
    if depth <= 0.0 or fx <= 0.0 or fy <= 0.0:
        raise ValueError("invalid depth or camera intrinsics")
    return (
        (float(u) - cx) * depth / fx,
        (float(v) - cy) * depth / fy,
        depth,
    )


def rotate_vector(vector, quaternion):
    """Rotate a vector by a normalized (x, y, z, w) quaternion."""
    x, y, z = (float(value) for value in vector)
    qx, qy, qz, qw = (float(value) for value in quaternion)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        raise ValueError("invalid quaternion")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def transform_point(point, translation, quaternion):
    rotated = rotate_vector(point, quaternion)
    return tuple(rotated[index] + float(translation[index]) for index in range(3))


def associate_track(class_name, position, tracks, maximum_distance: float):
    """Return the nearest same-class track id within the association gate."""
    best_id = None
    best_distance = float(maximum_distance)
    for track_id, track in tracks.items():
        if track.get("class_name") != class_name or not track.get("position_valid"):
            continue
        distance = math.dist(position, track["position"])
        if distance <= best_distance:
            best_id = track_id
            best_distance = distance
    return best_id

