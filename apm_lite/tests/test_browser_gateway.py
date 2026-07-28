import asyncio
import time

from fastapi.testclient import TestClient

from aeromind_apm_lite.ground.browser.app import create_app
from aeromind_apm_lite.ground.browser.runtime import (
    DemoApmLink,
    ManualRuntime,
    ManualRuntimeConfig,
    ManualRuntimeMode,
)


def demo_app(*, static_dir=None):
    runtime = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode.DEMO,
            startup_timeout_s=2.0,
        )
    )
    return create_app(runtime, static_dir=static_dir)


def wait_for_telemetry(client: TestClient, timeout_s=2.0, predicate=None):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        response = client.get("/api/vehicle/telemetry")
        payload = response.json()
        if payload["available"] and (
            predicate is None or predicate(payload["telemetry"])
        ):
            return payload
        time.sleep(0.02)
    raise TimeoutError("browser telemetry did not become available")


def test_demo_health_config_status_and_telemetry_are_browser_safe():
    app = demo_app()
    with TestClient(app) as client:
        health = client.get("/api/health")
        assert health.status_code == 200
        assert health.json() == {
            "status": "ok",
            "ready": True,
            "runtime_mode": "demo",
            "vehicle_id": 1,
            "fcu_link_ok": False,
        }

        config = client.get("/api/config").json()
        assert config["fcu_endpoint"] == "udpin:0.0.0.0:14550"
        assert not config["flight_output_enabled"]
        assert config["commands_are_simulated"]
        assert not config["external_processes_managed"]
        assert config["camera"]["stream_available"] is False
        assert "secret" not in str(config).lower()
        assert "token" not in str(config).lower()

        status = client.get("/api/status").json()
        assert status["vehicle_connected"]
        assert status["onboard_agent_connected"]
        assert not status["fcu_ready"]
        assert status["commands_are_simulated"]

        telemetry = wait_for_telemetry(client)
        assert telemetry["coordinate_frame"] == "local_ned"
        assert telemetry["telemetry"]["frame"] == "local_ned"
        assert telemetry["telemetry"]["armed"] is False
        assert telemetry["telemetry"]["health"]["fcu_link_ok"] is False


def test_demo_command_reports_all_three_evidence_layers():
    app = demo_app()
    with TestClient(app) as client:
        response = client.post(
            "/api/vehicles/1/commands/takeoff",
            json={"altitude_m": 2.5, "completion_timeout_s": 2.0},
        )
        assert response.status_code == 200
        outcome = response.json()
        assert outcome["command"] == "takeoff"
        assert outcome["simulated"]
        assert outcome["application"]["accepted"]
        assert outcome["application"]["status"] == "accepted"
        assert not outcome["fcu_ack"]["applicable"]
        assert not outcome["fcu_ack"]["received"]
        assert outcome["fcu_ack"]["result"] is None
        assert outcome["physical_completion"]["confirmed"]
        assert outcome["terminal"]["status"] == "completed"
        assert outcome["successful"]

        telemetry = wait_for_telemetry(
            client,
            predicate=lambda value: value["armed"],
        )
        assert telemetry["telemetry"]["armed"]
        assert telemetry["telemetry"]["position_m"]["z"] == -2.5


def test_websocket_streams_snapshot_progress_and_result():
    app = demo_app()
    with TestClient(app) as client:
        with client.websocket_connect("/ws") as websocket:
            snapshot = websocket.receive_json()
            assert snapshot["event_type"] == "snapshot"
            assert snapshot["payload"]["runtime_started"]
            assert snapshot["payload"]["telemetry"]["mode"] == "STANDBY"

            response = client.post(
                "/api/control/arm",
                json={"completion_timeout_s": 2.0},
            )
            assert response.status_code == 200
            command_id = response.json()["command_id"]

            command_events = []
            while not any(
                event["event_type"] == "command_result"
                for event in command_events
            ):
                event = websocket.receive_json()
                payload = event["payload"]
                if payload.get("command_id") == command_id:
                    command_events.append(event)

            stages = [
                event["payload"].get("stage")
                for event in command_events
                if event["event_type"] == "command_progress"
            ]
            assert stages[0] == "sent"
            assert "accepted" in stages
            assert "running" in stages
            assert "completed" in stages
            assert command_events[-1]["payload"]["successful"]


def test_command_validation_and_unknown_vehicle_fail_closed():
    app = demo_app()
    with TestClient(app) as client:
        missing_altitude = client.post(
            "/api/control/takeoff",
            json={"completion_timeout_s": 2.0},
        )
        assert missing_altitude.status_code == 422
        assert (
            missing_altitude.json()["detail"]
            == "takeoff requires altitude_m"
        )

        stray_altitude = client.post(
            "/api/control/land",
            json={"altitude": 2.0},
        )
        assert stray_altitude.status_code == 422
        assert "only valid for takeoff" in stray_altitude.json()["detail"]

        unknown_vehicle = client.post(
            "/api/vehicles/2/commands/arm",
            json={"completion_timeout_s": 2.0},
        )
        assert unknown_vehicle.status_code == 404


def test_static_web_directory_is_mounted_without_shadowing_api(tmp_path):
    index = tmp_path / "index.html"
    index.write_text("<html><body>lite-ground-station</body></html>")
    app = demo_app(static_dir=tmp_path)
    with TestClient(app) as client:
        root = client.get("/")
        assert root.status_code == 200
        assert "lite-ground-station" in root.text
        assert client.get("/api/status").status_code == 200


def test_manual_sitl_config_keeps_external_lifecycle_outside_gateway():
    config = ManualRuntimeConfig(mode=ManualRuntimeMode.SITL)
    public = config.public_payload()
    assert public["runtime_mode"] == "sitl"
    assert public["flight_output_enabled"]
    assert not public["commands_are_simulated"]
    assert public["external_processes_managed"] is False
    assert config.fcu_connection().endpoint == "udpin:0.0.0.0:14550"


def test_manual_runtime_wires_server_agent_and_injected_link():
    async def scenario():
        link = DemoApmLink()
        runtime = ManualRuntime(
            ManualRuntimeConfig(
                mode=ManualRuntimeMode.SITL,
                startup_timeout_s=2.0,
            ),
            link_factory=lambda _config: link,
        )
        await runtime.start()
        try:
            assert runtime.started
            assert runtime.agent_connected
            assert link.ready
            assert runtime.server.session_info(1) is not None
        finally:
            await runtime.stop()
        assert not runtime.started
        assert not link.ready

    asyncio.run(scenario())
