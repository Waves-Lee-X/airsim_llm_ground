from pathlib import Path

import pytest

from aeromind_apm_lite.onboard.probe import select_vehicle

ROOT = Path(__file__).resolve().parents[1]


def test_probe_selects_one_vehicle_from_strict_fleet_config():
    vehicle = select_vehicle(ROOT / "configs/sim/fleet.yaml", 2)

    assert vehicle.vehicle_id == 2
    assert vehicle.runtime_host == "ground-sim"
    assert vehicle.fcu.target_system == 2


def test_probe_rejects_unknown_vehicle():
    with pytest.raises(ValueError, match="vehicle_id 99"):
        select_vehicle(ROOT / "configs/sim/fleet.yaml", 99)
