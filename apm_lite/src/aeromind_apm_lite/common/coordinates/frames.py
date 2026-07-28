"""Venue-map, ENU and NED transforms with an explicit calibration identity."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class VenueCalibration:
    calibration_id: str
    map_origin_enu_m: Vec3
    map_x_yaw_from_east_rad: float

    def __post_init__(self) -> None:
        if not self.calibration_id.strip():
            raise ValueError("calibration_id must not be empty")
        if not math.isfinite(self.map_x_yaw_from_east_rad):
            raise ValueError("map_x_yaw_from_east_rad must be finite")


def map_to_enu(point_map: Vec3, calibration: VenueCalibration) -> Vec3:
    """Rotate venue map X/Y into ENU, then apply the surveyed ENU origin."""

    yaw = calibration.map_x_yaw_from_east_rad
    east = math.cos(yaw) * point_map.x - math.sin(yaw) * point_map.y
    north = math.sin(yaw) * point_map.x + math.cos(yaw) * point_map.y
    return Vec3(
        calibration.map_origin_enu_m.x + east,
        calibration.map_origin_enu_m.y + north,
        calibration.map_origin_enu_m.z + point_map.z,
    )


def enu_to_map(point_enu: Vec3, calibration: VenueCalibration) -> Vec3:
    """Inverse of :func:`map_to_enu`."""

    east = point_enu.x - calibration.map_origin_enu_m.x
    north = point_enu.y - calibration.map_origin_enu_m.y
    up = point_enu.z - calibration.map_origin_enu_m.z
    yaw = calibration.map_x_yaw_from_east_rad
    return Vec3(
        math.cos(yaw) * east + math.sin(yaw) * north,
        -math.sin(yaw) * east + math.cos(yaw) * north,
        up,
    )


def enu_to_ned(point_enu: Vec3) -> Vec3:
    return Vec3(point_enu.y, point_enu.x, -point_enu.z)


def ned_to_enu(point_ned: Vec3) -> Vec3:
    return Vec3(point_ned.y, point_ned.x, -point_ned.z)


def map_to_ned(point_map: Vec3, calibration: VenueCalibration) -> Vec3:
    return enu_to_ned(map_to_enu(point_map, calibration))
