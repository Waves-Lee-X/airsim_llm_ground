"""Pose composition helpers for converting odometry into a SLAM map frame."""

from __future__ import annotations

import math


def normalize_quaternion(quaternion):
    values = tuple(float(value) for value in quaternion)
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise ValueError("invalid quaternion")
    return tuple(value / norm for value in values)


def multiply_quaternions(left, right):
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def rotate_vector(vector, quaternion):
    x, y, z = (float(value) for value in vector)
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def compose_pose(transform_translation, transform_rotation, position, orientation):
    rotated = rotate_vector(position, transform_rotation)
    translated = tuple(
        rotated[index] + float(transform_translation[index]) for index in range(3)
    )
    return translated, multiply_quaternions(transform_rotation, orientation)

