"""Fleet orchestration service and browser API tests."""

import pytest
from fastapi.testclient import TestClient

from aeromind_apm_lite.common.formation import FleetVehicleState, FormationType
from aeromind_apm_lite.ground.browser.app import create_app
from aeromind_apm_lite.ground.browser.fleet_service import (
    FleetConfigError,
    FleetService,
)
from aeromind_apm_lite.ground.browser.runtime import (
    ManualRuntime,
    ManualRuntimeConfig,
    ManualRuntimeMode,
)


def ready_state(vehicle_id, x, y):
    return FleetVehicleState(
        vehicle_id=vehicle_id,
        ready=True,
        link_ok=True,
        last_seen_age_s=0.1,
        position_map_m=(x, y, -2.0),
    )


def test_fleet_service_defaults_and_validation():
    service = FleetService(min_spacing_m=3.0)
    assert service.formation == FormationType.LINE
    assert service.vehicle_ids == (1, 2, 3, 4)
    with pytest.raises(FleetConfigError):
        service.set_formation(FormationType.V, spacing_m=0.1)
    with pytest.raises(FleetConfigError):
        service.set_formation(FormationType.V, transition_progress=1.5)
    with pytest.raises(FleetConfigError):
        service.set_formation(FormationType.V, vehicle_ids=[1, 1])
    with pytest.raises(FleetConfigError):
        service.set_formation(FormationType.V, leader_target_map_m=[0.0, 0.0])


def test_fleet_service_status_builds_slot_directives():
    service = FleetService(min_spacing_m=3.0)
    states = {
        1: ready_state(1, 0.0, -4.6),
        2: ready_state(2, 0.0, -1.4),
        3: ready_state(3, 0.0, 1.6),
        4: ready_state(4, 0.0, 4.4),
    }
    payload = service.status_payload(states)
    assert payload["formation"] == "line"
    assert payload["phase"] == "formation_hold"
    assert payload["preview_only"] is True
    assert len(payload["directives"]) == 4
    assert {item["slot"] for item in payload["directives"]} == {0, 1, 2, 3}
    assert all(item["action"] == "goto" for item in payload["directives"])


def test_fleet_service_phase_reflects_not_ready_vehicles():
    service = FleetService(min_spacing_m=3.0)
    states = {
        1: ready_state(1, 0.0, -1.5),
        2: FleetVehicleState(
            vehicle_id=2,
            ready=False,
            link_ok=True,
            last_seen_age_s=0.1,
            position_map_m=(0.0, 1.5, -2.0),
        ),
    }
    payload = service.status_payload(states)
    assert payload["phase"] == "syncing"
    assert all(item["action"] == "hold" for item in payload["directives"])


def test_browser_fleet_status_lists_all_requested_vehicles():
    runtime = ManualRuntime(
        ManualRuntimeConfig(mode=ManualRuntimeMode.DEMO, startup_timeout_s=2.0)
    )
    app = create_app(runtime)
    with TestClient(app) as client:
        status = client.get("/api/fleet/status")
        assert status.status_code == 200
        body = status.json()
        assert body["formation"] == "line"
        assert body["vehicle_ids"] == [1, 2, 3, 4]
        assert body["preview_only"] is True
        assert len(body["directives"]) == 4


def test_browser_fleet_formation_switch_and_validation():
    runtime = ManualRuntime(
        ManualRuntimeConfig(mode=ManualRuntimeMode.DEMO, startup_timeout_s=2.0)
    )
    app = create_app(runtime)
    with TestClient(app) as client:
        switched = client.post(
            "/api/fleet/formation",
            json={
                "formation": "diamond",
                "spacing_m": 4.0,
                "leader_target_map_m": [10.0, 2.0, -3.0],
            },
        )
        assert switched.status_code == 200
        body = switched.json()
        assert body["formation"] == "diamond"
        assert body["spacing_m"] == 4.0
        assert body["leader_target_map_m"] == [10.0, 2.0, -3.0]

        invalid = client.post(
            "/api/fleet/formation",
            json={"formation": "circle"},
        )
        assert invalid.status_code == 422

        bad_spacing = client.post(
            "/api/fleet/formation",
            json={"formation": "v", "spacing_m": 0.1},
        )
        assert bad_spacing.status_code == 422


def test_fleet_status_is_included_in_station_status():
    runtime = ManualRuntime(
        ManualRuntimeConfig(mode=ManualRuntimeMode.DEMO, startup_timeout_s=2.0)
    )
    app = create_app(runtime)
    with TestClient(app) as client:
        body = client.get("/api/status").json()
        assert body["fleet"]["formation"] == "line"
        assert body["fleet"]["preview_only"] is True
