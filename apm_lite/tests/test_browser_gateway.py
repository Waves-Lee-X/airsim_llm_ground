import asyncio
import time
from datetime import datetime, timezone

from fastapi.testclient import TestClient

from aeromind_apm_lite.ground.browser.app import create_app
from aeromind_apm_lite.ground.browser.camera import CameraFrame
from aeromind_apm_lite.ground.browser.runtime import (
    DemoApmLink,
    HybridRuntime,
    ManualRuntime,
    ManualRuntimeConfig,
    ManualRuntimeMode,
)
from aeromind_apm_lite.common.communication import SerialChannelClosed
from aeromind_apm_lite.common.contracts import VehicleCommandType
from aeromind_apm_lite.common.coordinates import GeoReferenceStore


REAL_SECRET = b"browser-real-serial-test-secret-32-bytes"


class IdleSerialStream:
    def __init__(self):
        self.incoming = asyncio.Queue()
        self.closed = False

    async def read(self, _max_bytes):
        item = await self.incoming.get()
        if item is None:
            raise SerialChannelClosed("test serial stream closed")
        return item

    async def write(self, _data):
        if self.closed:
            raise SerialChannelClosed("test serial stream closed")

    async def close(self):
        if not self.closed:
            self.closed = True
            await self.incoming.put(None)


class StaticCameraBridge:
    def __init__(self, name, data):
        self.name = name
        self.data = data
        self.running = False

    async def start(self):
        self.running = True

    async def stop(self):
        self.running = False

    def status_payload(self):
        return {
            "name": self.name,
            "kind": "test_rgb",
            "state": "online",
            "stream_available": self.running,
            "stream_url": "/api/camera/frame",
        }

    async def get_frame(self):
        return CameraFrame(
            data=self.data,
            media_type="image/jpeg",
            sequence=1,
            captured_at_utc=datetime.now(timezone.utc),
            captured_monotonic_s=time.monotonic(),
        )


def demo_app(*, static_dir=None, georeference=None):
    runtime = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode.DEMO,
            startup_timeout_s=2.0,
        )
    )
    return create_app(
        runtime,
        static_dir=static_dir,
        georeference=georeference,
    )


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


def test_real_serial_api_rejects_commands_outside_onboard_allowlist():
    async def open_serial():
        return IdleSerialStream()

    runtime = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode.REAL_SERIAL,
            vehicle_id=3,
            ground_serial_port="COM_TEST",
            startup_timeout_s=1.0,
        ),
        shared_secret=REAL_SECRET,
        serial_stream_factory=open_serial,
    )
    app = create_app(runtime)
    with TestClient(app) as client:
        heartbeat_type = type("HeartbeatStub", (), {})
        heartbeat = heartbeat_type()
        heartbeat.command_output_enabled = True
        heartbeat.allowed_commands = (
            VehicleCommandType.ARM,
            VehicleCommandType.DISARM,
        )
        runtime.server.last_heartbeat = lambda _vehicle_id: heartbeat
        runtime.server.session_info = lambda _vehicle_id: object()

        response = client.post(
            "/api/vehicles/3/commands/takeoff",
            json={"altitude_m": 2.0},
        )
        assert response.status_code == 403
        assert "onboard command allowlist" in response.json()["detail"]


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


def test_real_serial_settings_can_be_changed_from_the_browser_api():
    opened_streams = []

    async def open_serial():
        stream = IdleSerialStream()
        opened_streams.append(stream)
        return stream

    runtime = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode.REAL_SERIAL,
            vehicle_id=3,
            frame_calibration_id="venue-v1",
            ground_serial_port="COM3",
            ground_serial_baudrate=57_600,
        ),
        shared_secret=REAL_SECRET,
        serial_stream_factory=open_serial,
    )
    app = create_app(runtime)
    with TestClient(app) as client:
        config = client.get("/api/config").json()
        assert config["serial_runtime_configurable"] is True
        assert config["ground_serial_port"] == "COM3"

        response = client.put(
            "/api/serial/config",
            json={"port": "COM7", "baudrate": 115_200},
        )
        assert response.status_code == 200
        assert response.json()["port"] == "COM7"
        assert response.json()["baudrate"] == 115_200
        assert len(opened_streams) == 2
        assert opened_streams[0].closed

        status = client.get("/api/status").json()
        assert status["ground_link"]["port"] == "COM7"
        assert status["ground_link"]["baudrate"] == 115_200
        assert status["runtime_error"] is None


def test_real_serial_open_failure_keeps_web_settings_available():
    async def fail_to_open():
        raise OSError("test port is unavailable")

    runtime = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode.REAL_SERIAL,
            vehicle_id=3,
            frame_calibration_id="venue-v1",
            ground_serial_port="COM3",
        ),
        shared_secret=REAL_SECRET,
        serial_stream_factory=fail_to_open,
    )
    app = create_app(runtime)
    with TestClient(app) as client:
        assert client.get("/").status_code == 200
        status = client.get("/api/status").json()
        assert status["runtime_started"] is False
        assert "test port is unavailable" in status["runtime_error"]
        assert client.get("/api/serial/ports").status_code == 200


def test_hybrid_mode_routes_simulation_and_real_vehicle_independently():
    opened_streams = []

    async def open_serial():
        stream = IdleSerialStream()
        opened_streams.append(stream)
        return stream

    simulation_link = DemoApmLink()
    simulation = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode.SITL,
            vehicle_id=1,
            vehicle_name="SITL UAV 1",
            startup_timeout_s=2.0,
        ),
        link_factory=lambda _config: simulation_link,
    )
    real = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode.REAL_SERIAL,
            vehicle_id=3,
            vehicle_name="实机 UAV 3",
            frame_calibration_id="venue-v1",
            ground_serial_port="COM3",
        ),
        shared_secret=REAL_SECRET,
        serial_stream_factory=open_serial,
    )
    runtime = HybridRuntime(simulation, real)
    app = create_app(
        runtime,
        cameras={
            1: StaticCameraBridge("AirSim RGB", b"simulation-frame"),
            3: StaticCameraBridge("D435i RGB", b"real-frame"),
        },
    )

    with TestClient(app) as client:
        config = client.get("/api/config").json()
        assert config["runtime_mode"] == "hybrid"
        assert config["deployment_mode"] == "hybrid"
        assert [vehicle["vehicle_id"] for vehicle in config["vehicles"]] == [1, 3]
        assert [
            vehicle["deployment_mode"] for vehicle in config["vehicles"]
        ] == ["sim", "real"]

        simulation_status = client.get("/api/status?vehicle_id=1").json()
        real_status = client.get("/api/status?vehicle_id=3").json()
        assert simulation_status["vehicle_connected"]
        assert simulation_status["ground_link"]["transport"] == "loopback_websocket"
        assert real_status["runtime_started"]
        assert real_status["ground_link"]["transport"] == "serial"
        assert client.get("/api/status?vehicle_id=2").status_code == 404

        telemetry = client.get("/api/vehicles/1/telemetry").json()
        assert telemetry["available"]
        assert telemetry["vehicle_id"] == 1
        assert client.get("/api/vehicles/2/telemetry").status_code == 404

        assert client.get("/api/camera/status?vehicle_id=1").json()["name"] == "AirSim RGB"
        assert client.get("/api/camera/status?vehicle_id=3").json()["name"] == "D435i RGB"
        assert client.get("/api/camera/frame?vehicle_id=1").content == b"simulation-frame"
        assert client.get("/api/camera/frame?vehicle_id=3").content == b"real-frame"

        command = client.post(
            "/api/vehicles/1/commands/arm",
            json={"completion_timeout_s": 2.0},
        )
        assert command.status_code == 200
        assert command.json()["vehicle_id"] == 1
        assert simulation_link.telemetry_snapshot().armed is True

        heartbeat_type = type("HeartbeatStub", (), {})
        heartbeat = heartbeat_type()
        heartbeat.command_output_enabled = True
        heartbeat.allowed_commands = (
            VehicleCommandType.ARM,
            VehicleCommandType.DISARM,
        )
        real.server.last_heartbeat = lambda _vehicle_id: heartbeat
        blocked = client.post(
            "/api/vehicles/3/commands/takeoff",
            json={"altitude_m": 2.0},
        )
        assert blocked.status_code == 403

        simulation_session = simulation.server.session_info(1).session_id
        reconfigured = client.put(
            "/api/serial/config",
            json={"port": "COM7", "baudrate": 115_200},
        )
        assert reconfigured.status_code == 200
        assert reconfigured.json()["vehicle_id"] == 3
        assert simulation.server.session_info(1).session_id == simulation_session
        assert simulation_link.ready
        assert len(opened_streams) == 2
        assert opened_streams[0].closed


def test_hybrid_mode_rejects_vehicle_id_collision():
    simulation = ManualRuntime(
        ManualRuntimeConfig(mode=ManualRuntimeMode.SITL, vehicle_id=3)
    )
    real = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode.REAL_SERIAL,
            vehicle_id=3,
            frame_calibration_id="venue-v1",
            ground_serial_port="COM3",
        ),
        shared_secret=REAL_SECRET,
    )
    try:
        HybridRuntime(simulation, real)
    except ValueError as exc:
        assert "vehicle IDs must be different" in str(exc)
    else:
        raise AssertionError("hybrid runtime accepted duplicate vehicle IDs")


def test_serial_reconfiguration_is_rejected_outside_real_mode():
    app = demo_app()
    with TestClient(app) as client:
        response = client.put(
            "/api/serial/config",
            json={"port": "COM7", "baudrate": 57_600},
        )
        assert response.status_code == 409


def test_georeference_api_saves_version_hash_and_round_trip_report(tmp_path):
    config_path = tmp_path / "venue.yaml"
    app = demo_app(georeference=GeoReferenceStore(config_path))
    with TestClient(app) as client:
        initial = client.get("/api/georeference")
        assert initial.status_code == 200
        assert initial.json()["configuration"]["status"] == "draft"
        assert not initial.json()["round_trip_report"]["ready"]

        payload = initial.json()["configuration"]
        payload.update(
            {
                "calibration_id": "test-venue-v1",
                "status": "surveyed",
                "map_origin_wgs84": {
                    "latitude_deg": 34.7472,
                    "longitude_deg": 113.6253,
                    "altitude_m": 112.4,
                },
                "map_x_heading_from_true_north_deg": 28.0,
                "airsim_origin_wgs84": {
                    "latitude_deg": 34.74718,
                    "longitude_deg": 113.62525,
                    "altitude_m": 111.9,
                },
            }
        )
        response = client.put("/api/georeference", json=payload)

        assert response.status_code == 200
        result = response.json()
        assert result["immutable"]
        assert len(result["config_hash"]) == 64
        assert result["round_trip_report"]["passed"]
        assert config_path.is_file()
        assert GeoReferenceStore(config_path).value.calibration_id == "test-venue-v1"

        public = client.get("/api/config").json()["georeference"]
        assert public["config_hash"] == result["config_hash"]
        assert public["complete"]


def test_georeference_api_rejects_mutating_a_surveyed_id(tmp_path):
    store = GeoReferenceStore(tmp_path / "venue.yaml")
    app = demo_app(georeference=store)
    with TestClient(app) as client:
        payload = client.get("/api/georeference").json()["configuration"]
        payload.update(
            {
                "calibration_id": "immutable-v1",
                "status": "surveyed",
                "map_origin_wgs84": {
                    "latitude_deg": 34.7,
                    "longitude_deg": 113.6,
                    "altitude_m": 100.0,
                },
                "map_x_heading_from_true_north_deg": 0.0,
                "airsim_origin_wgs84": {
                    "latitude_deg": 34.7,
                    "longitude_deg": 113.6,
                    "altitude_m": 100.0,
                },
            }
        )
        assert client.put("/api/georeference", json=payload).status_code == 200

        payload["notes"] = "attempted mutation"
        rejected = client.put("/api/georeference", json=payload)
        assert rejected.status_code == 409
        assert "new calibration_id" in rejected.json()["detail"]
