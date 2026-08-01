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
    DemoApmLink,
    FleetRuntime,
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


def test_browser_fleet_map_aggregates_all_known_vehicles():
    runtime = ManualRuntime(
        ManualRuntimeConfig(mode=ManualRuntimeMode.DEMO, startup_timeout_s=2.0)
    )
    app = create_app(runtime)
    with TestClient(app) as client:
        body = client.get("/api/fleet/map")
        assert body.status_code == 200
        payload = body.json()
        assert payload["coordinate_frame"] == "shared_ned"
        assert payload["origin_deg_m"] is None
        assert payload["formation"]["formation"] == "line"
        assert payload["formation"]["leader_target_map_m"] == [0.0, 0.0, -3.0]
        ids = {item["vehicle_id"] for item in payload["vehicles"]}
        assert ids == {1, 2, 3, 4}
        for vehicle in payload["vehicles"]:
            if vehicle["available"]:
                assert set(vehicle) >= {
                    "vehicle_id",
                    "available",
                    "fcu_link_ok",
                    "mode",
                    "armed",
                    "position_m",
                }
            else:
                assert set(vehicle) == {
                    "vehicle_id",
                    "available",
                    "vehicle_name",
                }
        switched = client.post(
            "/api/fleet/formation",
            json={
                "formation": "v",
                "leader_target_map_m": [8.0, 0.0, -2.0],
            },
        )
        assert switched.status_code == 200
        refreshed = client.get("/api/fleet/map").json()
        assert refreshed["formation"]["formation"] == "v"
        assert refreshed["formation"]["leader_target_map_m"] == [8.0, 0.0, -2.0]


def test_fleet_execute_api_requires_sim_vehicles_and_reports_status():
    runtime = ManualRuntime(
        ManualRuntimeConfig(mode=ManualRuntimeMode.DEMO, startup_timeout_s=2.0)
    )
    app = create_app(runtime)
    with TestClient(app) as client:
        idle = client.get("/api/fleet/execute")
        assert idle.status_code == 200
        assert idle.json()["phase"] == "idle"

        rejected = client.post(
            "/api/fleet/execute",
            json={
                "formation": "v",
                "leader_target_map_m": [8.0, 0.0, -2.0],
                "spacing_m": 3.0,
                "altitude_m": 2.0,
                "hold_s": 1.0,
                "vehicle_ids": [1, 2, 3, 4],
            },
        )
        assert rejected.status_code == 409
        assert "仿真" in rejected.json()["detail"]

        status = client.get("/api/fleet/status").json()
        assert status["execution"]["phase"] == "idle"


def test_fleet_mission_sequence_and_cancel_round_trip():
    import time as _time

    runtimes = [
        ManualRuntime(
            ManualRuntimeConfig(
                mode=ManualRuntimeMode.SITL,
                vehicle_id=vehicle_id,
                vehicle_name=f"SITL UAV {vehicle_id}",
                startup_timeout_s=2.0,
            ),
            link_factory=lambda _config: DemoApmLink(),
        )
        for vehicle_id in (1, 2, 3, 4)
    ]
    app = create_app(FleetRuntime(runtimes))
    with TestClient(app) as client:
        invalid = client.post(
            "/api/fleet/execute",
            json={
                "formation": "line",
                "formations": ["line", "circle"],
                "leader_target_map_m": [8.0, 0.0, -2.0],
                "vehicle_ids": [1, 2, 3, 4],
            },
        )
        assert invalid.status_code == 422

        started = client.post(
            "/api/fleet/execute",
            json={
                "formation": "line",
                "formations": ["line", "v", "diamond"],
                "leader_target_map_m": [8.0, 0.0, -2.0],
                "spacing_m": 3.0,
                "altitude_m": 2.0,
                "hold_s": 0.2,
                "vehicle_ids": [1, 2, 3, 4],
            },
        )
        assert started.status_code == 200
        assert started.json()["config"]["formations"] == ["line", "v", "diamond"]

        deadline = _time.time() + 30.0
        phase = None
        while _time.time() < deadline:
            phase = client.get("/api/fleet/execute").json()["phase"]
            if phase in {"done", "failed", "cancelled"}:
                break
            _time.sleep(0.2)
        assert phase == "done", client.get("/api/fleet/execute").json()

        # cancel path: start a mission, cancel it, and expect cancelled + land
        started = client.post(
            "/api/fleet/execute",
            json={
                "formation": "v",
                "leader_target_map_m": [8.0, 0.0, -2.0],
                "hold_s": 30.0,
                "vehicle_ids": [1, 2, 3, 4],
            },
        )
        assert started.status_code == 200
        cancelled = client.post("/api/fleet/execute/cancel")
        assert cancelled.status_code == 200
        assert cancelled.json()["phase"] == "cancelling"

        deadline = _time.time() + 30.0
        phase = None
        while _time.time() < deadline:
            phase = client.get("/api/fleet/execute").json()["phase"]
            if phase in {"done", "failed", "cancelled"}:
                break
            _time.sleep(0.2)
        assert phase == "cancelled", client.get("/api/fleet/execute").json()


def test_fleet_mission_runner_completes_demo_sitl_round_trip():
    import time as _time

    runtimes = [
        ManualRuntime(
            ManualRuntimeConfig(
                mode=ManualRuntimeMode.SITL,
                vehicle_id=vehicle_id,
                vehicle_name=f"SITL UAV {vehicle_id}",
                startup_timeout_s=2.0,
            ),
            link_factory=lambda _config: DemoApmLink(),
        )
        for vehicle_id in (1, 2, 3, 4)
    ]
    app = create_app(FleetRuntime(runtimes))
    with TestClient(app) as client:
        started = client.post(
            "/api/fleet/execute",
            json={
                "formation": "v",
                "leader_target_map_m": [8.0, 0.0, -2.0],
                "spacing_m": 3.0,
                "altitude_m": 2.0,
                "hold_s": 0.5,
                "vehicle_ids": [1, 2, 3, 4],
            },
        )
        assert started.status_code == 200
        assert started.json()["phase"] == "starting"

        deadline = _time.time() + 30.0
        phase = None
        while _time.time() < deadline:
            phase = client.get("/api/fleet/execute").json()["phase"]
            if phase in {"done", "failed", "cancelled"}:
                break
            _time.sleep(0.2)
        assert phase == "done", client.get("/api/fleet/execute").json()


def test_fleet_status_is_included_in_station_status():
    runtime = ManualRuntime(
        ManualRuntimeConfig(mode=ManualRuntimeMode.DEMO, startup_timeout_s=2.0)
    )
    app = create_app(runtime)
    with TestClient(app) as client:
        body = client.get("/api/status").json()
        assert body["fleet"]["formation"] == "line"
        assert body["fleet"]["preview_only"] is True
