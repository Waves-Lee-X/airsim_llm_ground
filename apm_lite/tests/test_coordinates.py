import math

import pytest

from aeromind_apm_lite.common.coordinates import (
    GeodeticPosition,
    Vec3,
    VenueCalibration,
    airsim_ned_to_map,
    ecef_to_geodetic,
    enu_to_geodetic,
    enu_to_local_ned,
    enu_to_map,
    enu_to_ned,
    geodetic_to_ecef,
    geodetic_to_enu,
    geodetic_to_local_ned,
    geodetic_to_map,
    local_ned_to_enu,
    local_ned_to_geodetic,
    local_ned_to_map,
    map_to_airsim_ned,
    map_to_enu,
    map_to_geodetic,
    map_to_local_ned,
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


def test_coordinate_models_reject_invalid_values():
    with pytest.raises(ValueError, match="x must be finite"):
        Vec3(float("nan"), 0.0, 0.0)
    with pytest.raises(ValueError, match="latitude_deg"):
        GeodeticPosition(91.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="longitude_deg"):
        GeodeticPosition(0.0, -181.0, 0.0)
    with pytest.raises(ValueError, match="altitude_m"):
        GeodeticPosition(0.0, 0.0, float("inf"))


@pytest.mark.parametrize(
    "position",
    [
        GeodeticPosition(0.0, 0.0, 0.0),
        GeodeticPosition(34.7466, 113.6254, 112.3),
        GeodeticPosition(-33.8568, 151.2153, 18.0),
        GeodeticPosition(89.999, -45.0, 1000.0),
    ],
)
def test_geodetic_ecef_round_trip(position):
    decoded = ecef_to_geodetic(geodetic_to_ecef(position))

    assert decoded.latitude_deg == pytest.approx(
        position.latitude_deg, abs=1e-9
    )
    assert decoded.longitude_deg == pytest.approx(
        position.longitude_deg, abs=1e-9
    )
    assert decoded.altitude_m == pytest.approx(position.altitude_m, abs=1e-5)


def test_known_wgs84_axes():
    equator = geodetic_to_ecef(GeodeticPosition(0.0, 0.0, 0.0))
    east_quadrant = geodetic_to_ecef(GeodeticPosition(0.0, 90.0, 0.0))

    assert_vec_close(equator, Vec3(6_378_137.0, 0.0, 0.0))
    assert east_quadrant.x == pytest.approx(0.0, abs=1e-8)
    assert east_quadrant.y == pytest.approx(6_378_137.0)
    assert east_quadrant.z == pytest.approx(0.0)


def test_enu_geodetic_round_trip_at_venue_scale():
    origin = GeodeticPosition(34.7466, 113.6254, 112.3)
    expected_enu = Vec3(125.0, -80.0, 16.0)

    position = enu_to_geodetic(expected_enu, origin)
    actual_enu = geodetic_to_enu(position, origin)

    assert_vec_close(actual_enu, expected_enu)
    assert_vec_close(geodetic_to_enu(origin, origin), Vec3(0.0, 0.0, 0.0))


def test_enu_round_trip_across_longitude_wrap():
    origin = GeodeticPosition(0.0, 179.999, 0.0)
    position = GeodeticPosition(0.0, -179.999, 25.0)

    enu = geodetic_to_enu(position, origin)
    decoded = enu_to_geodetic(enu, origin)

    assert enu.x == pytest.approx(222.64, abs=0.1)
    assert decoded.latitude_deg == pytest.approx(
        position.latitude_deg, abs=1e-9
    )
    assert decoded.longitude_deg == pytest.approx(
        position.longitude_deg, abs=1e-9
    )
    assert decoded.altitude_m == pytest.approx(position.altitude_m, abs=1e-5)


def test_vehicle_home_local_ned_round_trip():
    home_enu = Vec3(20.0, 50.0, 3.0)
    point_enu = Vec3(26.0, 65.0, 8.0)

    point_ned = enu_to_local_ned(point_enu, home_enu)

    assert_vec_close(point_ned, Vec3(15.0, 6.0, -5.0))
    assert_vec_close(local_ned_to_enu(point_ned, home_enu), point_enu)


def test_geodetic_local_ned_round_trip():
    origin = GeodeticPosition(34.7466, 113.6254, 112.3)
    expected_ned = Vec3(40.0, -25.0, -12.0)

    position = local_ned_to_geodetic(expected_ned, origin)

    assert_vec_close(geodetic_to_local_ned(position, origin), expected_ned)


def test_full_map_wgs84_airsim_and_vehicle_home_chain():
    origin = GeodeticPosition(34.7466, 113.6254, 112.3)
    calibration = VenueCalibration(
        calibration_id="zhengzhou-demo-v1",
        map_origin_enu_m=Vec3(100.0, 200.0, 5.0),
        map_x_yaw_from_east_rad=math.pi / 2,
    )
    point_map = Vec3(12.0, -4.0, 3.0)
    home_enu = Vec3(90.0, 190.0, 4.0)

    position = map_to_geodetic(point_map, calibration, origin)
    airsim_ned = map_to_airsim_ned(point_map, calibration)
    local_ned = map_to_local_ned(point_map, calibration, home_enu)

    assert_vec_close(geodetic_to_map(position, calibration, origin), point_map)
    assert_vec_close(
        airsim_ned_to_map(airsim_ned, calibration), point_map
    )
    assert_vec_close(
        local_ned_to_map(local_ned, calibration, home_enu), point_map
    )
