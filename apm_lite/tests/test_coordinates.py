import math

import pytest

from aeromind_apm_lite.common.coordinates import (
    Vec3,
    VenueCalibration,
    enu_to_map,
    enu_to_ned,
    map_to_enu,
    ned_to_enu,
)


def assert_vec_close(actual: Vec3, expected: Vec3):
    assert actual.x == pytest.approx(expected.x)
    assert actual.y == pytest.approx(expected.y)
    assert actual.z == pytest.approx(expected.z)


def test_enu_ned_round_trip():
    point = Vec3(3.0, -4.0, 2.0)
    assert_vec_close(ned_to_enu(enu_to_ned(point)), point)


def test_venue_map_rotation_and_round_trip():
    calibration = VenueCalibration(
        calibration_id="venue-v1",
        map_origin_enu_m=Vec3(100.0, 200.0, 5.0),
        map_x_yaw_from_east_rad=math.pi / 2,
    )
    point_map = Vec3(10.0, 2.0, 3.0)

    point_enu = map_to_enu(point_map, calibration)

    assert_vec_close(point_enu, Vec3(98.0, 210.0, 8.0))
    assert_vec_close(enu_to_map(point_enu, calibration), point_map)
