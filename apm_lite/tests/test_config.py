from pathlib import Path

import pytest

from aeromind_apm_lite.common.config import ConfigError, load_fleet_config
from aeromind_apm_lite.onboard import ApmLink
from aeromind_apm_lite.onboard.mavlink import FakeMavlinkTransport

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "relative_path,expected_mode",
    [
        ("configs/sim/fleet.yaml", "sim"),
        ("configs/real/fleet.example.yaml", "real"),
    ],
)
def test_four_vehicle_examples_are_valid(relative_path, expected_mode):
    config = load_fleet_config(ROOT / relative_path)

    assert config.mode.value == expected_mode
    assert [vehicle.vehicle_id for vehicle in config.vehicles] == [1, 2, 3, 4]
    assert len({(vehicle.runtime_host, vehicle.fcu.endpoint) for vehicle in config.vehicles}) == 4
    assert all(vehicle.vehicle_id == vehicle.fcu.target_system for vehicle in config.vehicles)


def test_trajectory_timeout_is_wired_into_apm_link():
    config = load_fleet_config(ROOT / "configs/sim/fleet.yaml")

    link = ApmLink.from_vehicle_config(FakeMavlinkTransport(), config.vehicles[0])

    assert link.trajectory_timeout_s == pytest.approx(1.0)


def test_duplicate_vehicle_id_is_rejected(tmp_path):
    source = (ROOT / "configs/sim/fleet.yaml").read_text(encoding="utf-8")
    invalid = source.replace("vehicle_id: 2", "vehicle_id: 1", 1)
    path = tmp_path / "invalid.yaml"
    path.write_text(invalid, encoding="utf-8")

    with pytest.raises(ConfigError):
        load_fleet_config(path)


def test_duplicate_endpoint_on_same_runtime_host_is_rejected(tmp_path):
    source = (ROOT / "configs/sim/fleet.yaml").read_text(encoding="utf-8")
    invalid = source.replace("udpin:0.0.0.0:14560", "udpin:0.0.0.0:14550", 1)
    path = tmp_path / "invalid.yaml"
    path.write_text(invalid, encoding="utf-8")

    with pytest.raises(ConfigError):
        load_fleet_config(path)
