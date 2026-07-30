"""WGS84, venue-map, ENU and NED transforms for local flight operations."""

from __future__ import annotations

import math
from dataclasses import dataclass


WGS84_SEMI_MAJOR_AXIS_M = 6_378_137.0
WGS84_INVERSE_FLATTENING = 298.257_223_563
WGS84_FLATTENING = 1.0 / WGS84_INVERSE_FLATTENING
WGS84_FIRST_ECCENTRICITY_SQUARED = WGS84_FLATTENING * (
    2.0 - WGS84_FLATTENING
)
WGS84_SEMI_MINOR_AXIS_M = WGS84_SEMI_MAJOR_AXIS_M * (1.0 - WGS84_FLATTENING)
WGS84_SECOND_ECCENTRICITY_SQUARED = (
    WGS84_SEMI_MAJOR_AXIS_M**2 - WGS84_SEMI_MINOR_AXIS_M**2
) / WGS84_SEMI_MINOR_AXIS_M**2


def _finite(value: float, *, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        for field_name in ("x", "y", "z"):
            object.__setattr__(
                self,
                field_name,
                _finite(getattr(self, field_name), label=field_name),
            )


@dataclass(frozen=True)
class GeodeticPosition:
    """WGS84 position whose height uses one consistent altitude datum."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float

    def __post_init__(self) -> None:
        latitude = _finite(self.latitude_deg, label="latitude_deg")
        longitude = _finite(self.longitude_deg, label="longitude_deg")
        altitude = _finite(self.altitude_m, label="altitude_m")
        if not -90.0 <= latitude <= 90.0:
            raise ValueError("latitude_deg must be in [-90, 90]")
        if not -180.0 <= longitude <= 180.0:
            raise ValueError("longitude_deg must be in [-180, 180]")
        object.__setattr__(self, "latitude_deg", latitude)
        object.__setattr__(self, "longitude_deg", longitude)
        object.__setattr__(self, "altitude_m", altitude)


@dataclass(frozen=True)
class VenueCalibration:
    calibration_id: str
    map_origin_enu_m: Vec3
    map_x_yaw_from_east_rad: float

    def __post_init__(self) -> None:
        if not self.calibration_id.strip():
            raise ValueError("calibration_id must not be empty")
        object.__setattr__(
            self,
            "map_x_yaw_from_east_rad",
            _finite(
                self.map_x_yaw_from_east_rad,
                label="map_x_yaw_from_east_rad",
            ),
        )


def geodetic_to_ecef(position: GeodeticPosition) -> Vec3:
    """Convert WGS84 to Earth-centered Earth-fixed metres."""

    latitude = math.radians(position.latitude_deg)
    longitude = math.radians(position.longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    radius = WGS84_SEMI_MAJOR_AXIS_M / math.sqrt(
        1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_latitude**2
    )
    radial = radius + position.altitude_m
    return Vec3(
        radial * cos_latitude * math.cos(longitude),
        radial * cos_latitude * math.sin(longitude),
        (
            radius * (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED)
            + position.altitude_m
        )
        * sin_latitude,
    )


def ecef_to_geodetic(point_ecef: Vec3) -> GeodeticPosition:
    """Convert ECEF metres to WGS84 using Bowring's method."""

    horizontal = math.hypot(point_ecef.x, point_ecef.y)
    if horizontal < 1e-9:
        if abs(point_ecef.z) < 1e-9:
            raise ValueError("ECEF origin has no geodetic position")
        latitude_deg = 90.0 if point_ecef.z > 0.0 else -90.0
        return GeodeticPosition(
            latitude_deg=latitude_deg,
            longitude_deg=0.0,
            altitude_m=abs(point_ecef.z) - WGS84_SEMI_MINOR_AXIS_M,
        )

    longitude = math.atan2(point_ecef.y, point_ecef.x)
    theta = math.atan2(
        point_ecef.z * WGS84_SEMI_MAJOR_AXIS_M,
        horizontal * WGS84_SEMI_MINOR_AXIS_M,
    )
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    latitude = math.atan2(
        point_ecef.z
        + WGS84_SECOND_ECCENTRICITY_SQUARED
        * WGS84_SEMI_MINOR_AXIS_M
        * sin_theta**3,
        horizontal
        - WGS84_FIRST_ECCENTRICITY_SQUARED
        * WGS84_SEMI_MAJOR_AXIS_M
        * cos_theta**3,
    )
    sin_latitude = math.sin(latitude)
    radius = WGS84_SEMI_MAJOR_AXIS_M / math.sqrt(
        1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_latitude**2
    )
    altitude = horizontal / math.cos(latitude) - radius
    longitude_deg = math.degrees(longitude)
    if longitude_deg > 180.0:
        longitude_deg -= 360.0
    elif longitude_deg < -180.0:
        longitude_deg += 360.0
    return GeodeticPosition(
        latitude_deg=math.degrees(latitude),
        longitude_deg=longitude_deg,
        altitude_m=altitude,
    )


def geodetic_to_enu(
    position: GeodeticPosition,
    origin: GeodeticPosition,
) -> Vec3:
    """Project WGS84 into the local tangent ENU frame at ``origin``."""

    point_ecef = geodetic_to_ecef(position)
    origin_ecef = geodetic_to_ecef(origin)
    delta_x = point_ecef.x - origin_ecef.x
    delta_y = point_ecef.y - origin_ecef.y
    delta_z = point_ecef.z - origin_ecef.z
    latitude = math.radians(origin.latitude_deg)
    longitude = math.radians(origin.longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)
    return Vec3(
        -sin_longitude * delta_x + cos_longitude * delta_y,
        -sin_latitude * cos_longitude * delta_x
        - sin_latitude * sin_longitude * delta_y
        + cos_latitude * delta_z,
        cos_latitude * cos_longitude * delta_x
        + cos_latitude * sin_longitude * delta_y
        + sin_latitude * delta_z,
    )


def enu_to_geodetic(
    point_enu: Vec3,
    origin: GeodeticPosition,
) -> GeodeticPosition:
    """Inverse local tangent-plane projection for :func:`geodetic_to_enu`."""

    origin_ecef = geodetic_to_ecef(origin)
    latitude = math.radians(origin.latitude_deg)
    longitude = math.radians(origin.longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)
    delta_x = (
        -sin_longitude * point_enu.x
        - sin_latitude * cos_longitude * point_enu.y
        + cos_latitude * cos_longitude * point_enu.z
    )
    delta_y = (
        cos_longitude * point_enu.x
        - sin_latitude * sin_longitude * point_enu.y
        + cos_latitude * sin_longitude * point_enu.z
    )
    delta_z = cos_latitude * point_enu.y + sin_latitude * point_enu.z
    return ecef_to_geodetic(
        Vec3(
            origin_ecef.x + delta_x,
            origin_ecef.y + delta_y,
            origin_ecef.z + delta_z,
        )
    )


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


def enu_to_local_ned(point_enu: Vec3, origin_enu_m: Vec3) -> Vec3:
    """Convert shared ENU coordinates to NED relative to one vehicle home."""

    return enu_to_ned(
        Vec3(
            point_enu.x - origin_enu_m.x,
            point_enu.y - origin_enu_m.y,
            point_enu.z - origin_enu_m.z,
        )
    )


def local_ned_to_enu(point_ned: Vec3, origin_enu_m: Vec3) -> Vec3:
    """Convert vehicle-local NED coordinates into the shared ENU frame."""

    offset_enu = ned_to_enu(point_ned)
    return Vec3(
        origin_enu_m.x + offset_enu.x,
        origin_enu_m.y + offset_enu.y,
        origin_enu_m.z + offset_enu.z,
    )


def geodetic_to_local_ned(
    position: GeodeticPosition,
    origin: GeodeticPosition,
) -> Vec3:
    """Convert WGS84 into ArduPilot/AirSim NED relative to ``origin``."""

    return enu_to_ned(geodetic_to_enu(position, origin))


def local_ned_to_geodetic(
    point_ned: Vec3,
    origin: GeodeticPosition,
) -> GeodeticPosition:
    return enu_to_geodetic(ned_to_enu(point_ned), origin)


def map_to_ned(point_map: Vec3, calibration: VenueCalibration) -> Vec3:
    return enu_to_ned(map_to_enu(point_map, calibration))


def ned_to_map(point_ned: Vec3, calibration: VenueCalibration) -> Vec3:
    return enu_to_map(ned_to_enu(point_ned), calibration)


def map_to_local_ned(
    point_map: Vec3,
    calibration: VenueCalibration,
    home_enu_m: Vec3,
) -> Vec3:
    return enu_to_local_ned(map_to_enu(point_map, calibration), home_enu_m)


def local_ned_to_map(
    point_ned: Vec3,
    calibration: VenueCalibration,
    home_enu_m: Vec3,
) -> Vec3:
    return enu_to_map(local_ned_to_enu(point_ned, home_enu_m), calibration)


def map_to_geodetic(
    point_map: Vec3,
    calibration: VenueCalibration,
    geodetic_origin: GeodeticPosition,
) -> GeodeticPosition:
    return enu_to_geodetic(map_to_enu(point_map, calibration), geodetic_origin)


def geodetic_to_map(
    position: GeodeticPosition,
    calibration: VenueCalibration,
    geodetic_origin: GeodeticPosition,
) -> Vec3:
    return enu_to_map(geodetic_to_enu(position, geodetic_origin), calibration)


def map_to_airsim_ned(point_map: Vec3, calibration: VenueCalibration) -> Vec3:
    """Map into AirSim NED when ENU zero is AirSim's OriginGeopoint."""

    return map_to_ned(point_map, calibration)


def airsim_ned_to_map(point_ned: Vec3, calibration: VenueCalibration) -> Vec3:
    return ned_to_map(point_ned, calibration)
