"""Dependency-free body/map frame transforms used by the local mapper."""

from __future__ import annotations


Point3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def rotate_vector(vector: Point3, quaternion: Quaternion) -> Point3:
    """Rotate a vector by quaternion (x, y, z, w)."""
    vx, vy, vz = vector
    qx, qy, qz, qw = quaternion
    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if norm <= 1e-9:
        return vector
    qx, qy, qz, qw = (value / norm for value in quaternion)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def body_to_world(
    point: Point3,
    position: Point3,
    orientation: Quaternion,
) -> Point3:
    rotated = rotate_vector(point, orientation)
    return tuple(position[index] + rotated[index] for index in range(3))
