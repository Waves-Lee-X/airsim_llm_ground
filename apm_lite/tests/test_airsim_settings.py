import json

from aeromind_apm_lite.common.coordinates import GeodeticPosition, geodetic_to_local_ned
from aeromind_apm_lite.ground.browser.airsim_settings import AirSimSpawnMap


def _settings_payload():
    return {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "OriginGeopoint": {
            "Latitude": 47.641468,
            "Longitude": -122.140165,
            "Altitude": 0.0,
        },
        "Vehicles": {
            "Drone1": {"X": 0.0, "Y": -4.5, "Z": -0.5},
            "Drone2": {"X": 0.0, "Y": -1.5, "Z": -0.5},
            "Drone3": {"X": 0.0, "Y": 1.5, "Z": -0.5},
            "Drone4": {"X": 0.0, "Y": 4.5, "Z": -0.5},
        },
    }


def test_airsim_spawn_map_derives_stable_per_vehicle_homes(tmp_path):
    settings = tmp_path / "settings.json"
    settings.write_text(
        json.dumps(_settings_payload()),
        encoding="utf-8",
    )
    spawn_map = AirSimSpawnMap(settings)
    assert spawn_map.known_ids() == (1, 2, 3, 4)

    origin = GeodeticPosition(47.641468, -122.140165, 0.0)
    # Spawn Y offsets must match the formation line slots: -4.5/-1.5/1.5/4.5
    expected_east = {-4.5: 1, -1.5: 2, 1.5: 3, 4.5: 4}
    for east, vehicle_id in expected_east.items():
        home = spawn_map.home_for(vehicle_id)
        assert home is not None
        ned = geodetic_to_local_ned(home, origin)
        assert abs(ned.x) < 0.05
        assert abs(ned.y - east) < 0.15

    # The spawn home is the vehicle's local-NED origin: converting a map
    # point at its own spawn must yield ~(0, 0).
    home_1 = spawn_map.home_for(1)
    assert home_1 is not None
    local = geodetic_to_local_ned(home_1, home_1)
    assert abs(local.x) < 0.01 and abs(local.y) < 0.01


def test_airsim_spawn_map_missing_file_is_empty(tmp_path):
    spawn_map = AirSimSpawnMap(tmp_path / "missing.json")
    assert spawn_map.known_ids() == ()
    assert spawn_map.home_for(1) is None