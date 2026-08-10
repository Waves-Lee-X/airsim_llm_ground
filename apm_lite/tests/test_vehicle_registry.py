import pytest

from aeromind_apm_lite.common.contracts import Vector3
from aeromind_apm_lite.common.contracts.twin import VehicleProfile
from aeromind_apm_lite.common.identity import RegistryError, VehicleRegistry
from test_twin_contracts import twin_state


def profile(vehicle_id, capabilities):
    return VehicleProfile(
        vehicle_id=vehicle_id,
        display_name=f"UAV{vehicle_id}",
        platform_type="multirotor",
        sensors=("rgb",),
        max_endurance_min=20.0,
        max_speed_m_s=10.0,
        capabilities=tuple(capabilities),
        whitelist_level=1,
    )


def populated_registry():
    registry = VehicleRegistry()
    for vehicle_id, capabilities, energy, communication in (
        (1, ("survey", "capture"), 0.9, 0.8),
        (2, ("survey", "relay"), 0.7, 0.95),
        (3, ("survey", "capture", "relay"), 0.4, 0.9),
    ):
        state = twin_state(vehicle_id=vehicle_id).model_copy(
            update={
                "position_m": Vector3(x=float(vehicle_id * 10), y=0.0, z=3.0),
                "energy_remaining": energy,
                "communication_quality": communication,
            }
        )
        registry.register(profile(vehicle_id, capabilities), state)
    return registry


def test_registry_queries_capability_energy_position_and_communication():
    registry = populated_registry()

    matches = registry.query(
        required_capabilities={"survey", "capture"},
        minimum_energy=0.5,
        minimum_communication_quality=0.75,
        origin_m=Vector3(x=0.0, y=0.0, z=3.0),
        maximum_distance_m=25.0,
    )

    assert [match.profile.vehicle_id for match in matches] == [1]
    assert matches[0].distance_m == pytest.approx(10.0)
    assert "survey" in matches[0].matched_capabilities


def test_registry_persistence_is_deterministic_and_round_trips(tmp_path):
    registry = populated_registry()
    first = tmp_path / "registry.json"
    second = tmp_path / "registry-copy.json"

    registry.save(first)
    loaded = VehicleRegistry.load(first)
    loaded.save(second)

    assert first.read_bytes() == second.read_bytes()
    assert loaded.get_profile(2).display_name == "UAV2"
    assert loaded.get_state(2).energy_remaining == pytest.approx(0.7)


def test_registry_rejects_identity_mismatch_and_unknown_state():
    registry = VehicleRegistry()
    with pytest.raises(RegistryError, match="registered"):
        registry.update_state(twin_state(vehicle_id=1))
    with pytest.raises(RegistryError, match="vehicle_id"):
        registry.register(profile(1, ("survey",)), twin_state(vehicle_id=2))
