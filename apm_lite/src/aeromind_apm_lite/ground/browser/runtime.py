"""In-process wiring for a user-managed ArduPilot/AirSim session.

This module deliberately has no dependency on the simulation lifecycle code.
It opens the configured MAVLink endpoint, but never launches or terminates
ArduPilot SITL, Unreal Engine, or AirSim.
"""

from __future__ import annotations

import asyncio
import hashlib
import json
import secrets
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable
from uuid import uuid4

from aeromind_apm_lite.common.communication import ByteStreamFactory
from aeromind_apm_lite.common.config import FcuConnection, FcuTransport
from aeromind_apm_lite.ground.server import GroundServer
from aeromind_apm_lite.onboard.agent import OnboardAgent
from aeromind_apm_lite.onboard.apm_link import ApmLink, CommandHandle
from aeromind_apm_lite.onboard.mavlink.models import (
    ApplicationAcceptance,
    CommandResult,
    CommandStatus,
    FcuAction,
    FcuCommand,
    MavlinkAckEvidence,
    PhysicalCompletionEvidence,
    TelemetrySnapshot,
)


class ManualRuntimeMode(str, Enum):
    SITL = "sitl"
    DEMO = "demo"
    REAL_SERIAL = "real_serial"
    HYBRID = "hybrid"


@dataclass(frozen=True)
class ManualRuntimeConfig:
    """Configuration kept intentionally separate from M1 acceptance config."""

    mode: ManualRuntimeMode = ManualRuntimeMode.SITL
    vehicle_id: int = 1
    vehicle_name: str = "sitl-uav1"
    fcu_endpoint: str = "udpin:0.0.0.0:14550"
    mavlink_target_system: int | None = None
    source_system: int = 201
    source_component: int = 191
    target_component: int = 1
    frame_calibration_id: str = "manual-sim-local-ned-v1"
    startup_timeout_s: float = 15.0
    heartbeat_timeout_s: float = 3.0
    ack_timeout_s: float = 3.0
    physical_timeout_s: float = 60.0
    telemetry_freshness_s: float = 1.0
    completion_hold_s: float = 0.5
    command_ttl_ms: int = 5_000
    ground_serial_port: str | None = None
    ground_serial_baudrate: int = 57_600

    def __post_init__(self) -> None:
        if not 1 <= self.vehicle_id <= 255:
            raise ValueError("vehicle_id must be in [1, 255]")
        if self.mavlink_target_system is not None and not 1 <= self.mavlink_target_system <= 255:
            raise ValueError("mavlink_target_system must be in [1, 255]")
        if not self.vehicle_name:
            raise ValueError("vehicle_name must not be empty")
        if not self.frame_calibration_id:
            raise ValueError("frame_calibration_id must not be empty")
        if self.mode == ManualRuntimeMode.REAL_SERIAL and not self.ground_serial_port:
            raise ValueError("real_serial mode requires ground_serial_port")
        if self.mode == ManualRuntimeMode.HYBRID:
            raise ValueError("hybrid mode must be constructed from separate runtimes")
        if self.ground_serial_baudrate <= 0:
            raise ValueError("ground_serial_baudrate must be positive")
        if not 100 <= self.command_ttl_ms <= 300_000:
            raise ValueError("command_ttl_ms must be in [100, 300000]")
        for label, value in (
            ("startup_timeout_s", self.startup_timeout_s),
            ("heartbeat_timeout_s", self.heartbeat_timeout_s),
            ("ack_timeout_s", self.ack_timeout_s),
            ("physical_timeout_s", self.physical_timeout_s),
            ("telemetry_freshness_s", self.telemetry_freshness_s),
            ("completion_hold_s", self.completion_hold_s),
        ):
            if value <= 0.0:
                raise ValueError(f"{label} must be positive")
        if self.mode != ManualRuntimeMode.REAL_SERIAL:
            self.fcu_connection()

    def fcu_connection(self) -> FcuConnection:
        if self.mode == ManualRuntimeMode.REAL_SERIAL:
            raise ValueError("real_serial FCU connection is owned by the onboard Pi")
        return FcuConnection(
            transport=FcuTransport.UDP,
            endpoint=self.fcu_endpoint,
            source_system=self.source_system,
            source_component=self.source_component,
            target_system=(
                self.mavlink_target_system
                if self.mavlink_target_system is not None
                else self.vehicle_id
            ),
            target_component=self.target_component,
        )

    def public_payload(self) -> dict[str, Any]:
        real_serial = self.mode == ManualRuntimeMode.REAL_SERIAL
        return {
            "runtime_mode": self.mode.value,
            "deployment_mode": "real" if real_serial else "sim",
            "vehicle_id": self.vehicle_id,
            "vehicle_name": self.vehicle_name,
            "fcu_endpoint": "onboard-pi" if real_serial else self.fcu_endpoint,
            "ground_transport": "serial" if real_serial else "loopback_websocket",
            "ground_serial_port": self.ground_serial_port if real_serial else None,
            "ground_serial_baudrate": (
                self.ground_serial_baudrate if real_serial else None
            ),
            "coordinate_frame": "local_ned",
            "frame_calibration_id": self.frame_calibration_id,
            "flight_output_enabled": self.mode != ManualRuntimeMode.DEMO,
            "commands_are_simulated": self.mode == ManualRuntimeMode.DEMO,
            "external_processes_managed": False,
            "camera": {
                "name": "front_center",
                "kind": "onboard_rgb" if real_serial else "scene_rgb",
                "width": 1280,
                "height": 720,
                "fov_degrees": 90.0,
                "stream_available": False,
                "detail": (
                    "real camera transport is not connected to this gateway"
                    if real_serial
                    else "camera streaming is not connected to this gateway"
                ),
            },
        }

    def configuration_hash(self) -> str:
        encoded = json.dumps(
            self.public_payload(), sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()


class DemoApmLink:
    """No-output link used to exercise the UI without SITL or an FCU."""

    guided_mode = "GUIDED"

    def __init__(self) -> None:
        self.ready = False
        self._armed = False
        self._mode = "STANDBY"
        self._position = (0.0, 0.0, 0.0)
        self._velocity = (0.0, 0.0, 0.0)
        self._started_at = time.monotonic()

    async def start(self) -> None:
        self.ready = True

    async def stop(self) -> None:
        self.ready = False

    def telemetry_snapshot(self) -> TelemetrySnapshot:
        now = time.monotonic()
        return TelemetrySnapshot(
            observed_monotonic_s=now,
            fcu_link_ok=False,
            armed=self._armed,
            mode=self._mode,
            relative_altitude_m=max(0.0, -self._position[2]),
            local_position_ned_m=self._position,
            velocity_ned_m_s=self._velocity,
            global_position_deg_m=None,
            attitude_rpy_rad=(0.0, 0.0, 0.0),
            battery_remaining=0.82,
            battery_voltage_v=15.6,
            gps_fix_type=0,
            satellites_visible=0,
            gps_hdop=None,
            gps_healthy=False,
            prearm_ok=True,
            ekf_flags=None,
            landed_state=1 if self._position[2] == 0.0 else 2,
            home_position_deg_m=None,
            home_position_ned_m=(0.0, 0.0, 0.0),
            last_status_text="DEMO: no MAVLink output",
            last_heartbeat_monotonic_s=self._started_at,
            field_ages_s={
                "heartbeat": 0.0,
                "local_position": 0.0,
                "velocity": 0.0,
            },
        )

    def submit(
        self,
        command: FcuCommand,
        *,
        expires_monotonic_s: float | None = None,
    ) -> CommandHandle:
        now = time.monotonic()
        request_id = uuid4()
        accepted = self.ready and (
            expires_monotonic_s is None or expires_monotonic_s > now
        )
        application = ApplicationAcceptance(
            accepted=accepted,
            observed_monotonic_s=now,
            detail=(
                "demo command accepted; no MAVLink output"
                if accepted
                else "demo link is stopped or command expired"
            ),
        )
        if accepted:
            self._apply(command)
        mavlink_ack = MavlinkAckEvidence(
            applicable=False,
            received=False,
            command_id=None,
            result=None,
            observed_monotonic_s=None,
            detail="demo mode has no FCU ACK",
        )
        physical = PhysicalCompletionEvidence(
            confirmed=accepted,
            observed_monotonic_s=now if accepted else None,
            detail=(
                "synthetic demo telemetry reached the requested state"
                if accepted
                else "demo command was not applied"
            ),
        )
        status = (
            CommandStatus.COMPLETED
            if accepted
            else CommandStatus.APPLICATION_REJECTED
        )
        result = CommandResult(
            request_id=request_id,
            action=command.action,
            status=status,
            application=application,
            sent_monotonic_s=None,
            mavlink_ack=mavlink_ack,
            physical_completion=physical,
            detail=(
                "demo state transition completed"
                if accepted
                else application.detail
            ),
        )
        loop = asyncio.get_running_loop()
        ack_future = loop.create_future()
        ack_future.set_result(mavlink_ack)
        physical_future = loop.create_future()
        physical_future.set_result(physical)
        result_future = loop.create_future()
        result_future.set_result(result)
        return CommandHandle(
            request_id=request_id,
            action=command.action,
            application=application,
            _mavlink_ack=ack_future,
            _physical_completion=physical_future,
            _result=result_future,
        )

    def _apply(self, command: FcuCommand) -> None:
        if command.action == FcuAction.ARM:
            self._armed = True
            self._mode = self.guided_mode
        elif command.action == FcuAction.DISARM:
            self._armed = False
        elif command.action == FcuAction.SET_MODE:
            self._mode = command.mode or self._mode
        elif command.action == FcuAction.TAKEOFF:
            self._armed = True
            self._mode = self.guided_mode
            self._position = (
                self._position[0],
                self._position[1],
                -float(command.target_altitude_m or 0.0),
            )
        elif command.action == FcuAction.HOLD:
            self._mode = "LOITER"
        elif command.action in {FcuAction.LAND, FcuAction.RTL}:
            self._position = (0.0, 0.0, 0.0)
            self._armed = False
            self._mode = command.action.value.upper()


LinkFactory = Callable[[ManualRuntimeConfig], Any]


class ManualRuntime:
    """Own the internal bridge while external apps remain user-managed."""

    def __init__(
        self,
        config: ManualRuntimeConfig | None = None,
        *,
        link_factory: LinkFactory | None = None,
        shared_secret: bytes | None = None,
        serial_stream_factory: ByteStreamFactory | None = None,
    ) -> None:
        self.config = config or ManualRuntimeConfig()
        self._link_factory = link_factory
        if self.config.mode == ManualRuntimeMode.REAL_SERIAL:
            if shared_secret is None or len(shared_secret) < 32:
                raise ValueError("real_serial mode requires a 32-byte shared secret")
            self._secret = shared_secret
        else:
            self._secret = shared_secret or secrets.token_bytes(32)
        self._serial_stream_factory = serial_stream_factory
        self._server: GroundServer | None = None
        self._agent: OnboardAgent | None = None
        self._link: Any | None = None
        self._started = False

    @property
    def started(self) -> bool:
        return self._started

    @property
    def vehicle_ids(self) -> tuple[int, ...]:
        return (self.config.vehicle_id,)

    @property
    def default_vehicle_id(self) -> int:
        return self.config.vehicle_id

    @property
    def server(self) -> GroundServer:
        if self._server is None:
            raise RuntimeError("manual runtime is not started")
        return self._server

    @property
    def link(self) -> Any | None:
        return self._link

    @property
    def agent_connected(self) -> bool:
        if self.config.mode == ManualRuntimeMode.REAL_SERIAL:
            return bool(
                self._server is not None
                and self._server.session_info(self.config.vehicle_id) is not None
            )
        return bool(self._agent is not None and self._agent.connected)

    async def start(self) -> None:
        if self._started or self._server is not None:
            raise RuntimeError("manual runtime can only be started once")
        server_options = {
            "vehicle_secrets": {self.config.vehicle_id: self._secret},
            "calibration_ids": {
                self.config.vehicle_id: self.config.frame_calibration_id
            },
        }
        if self.config.mode == ManualRuntimeMode.REAL_SERIAL:
            from aeromind_apm_lite.ground.serial_server import SerialGroundServer

            self._server = SerialGroundServer(
                serial_port=self.config.ground_serial_port or "",
                serial_baudrate=self.config.ground_serial_baudrate,
                stream_factory=self._serial_stream_factory,
                **server_options,
            )
        else:
            self._server = GroundServer(host="127.0.0.1", port=0, **server_options)
        try:
            await self._server.start()
            if self.config.mode == ManualRuntimeMode.REAL_SERIAL:
                self._started = True
                return
            self._link = self._create_link()
            await self._link.start()
            self._agent = OnboardAgent(
                ground_uri=self._server.uri,
                vehicle_id=self.config.vehicle_id,
                shared_secret=self._secret,
                frame_calibration_id=self.config.frame_calibration_id,
                apm_link=self._link,
                configuration_hash=self.config.configuration_hash(),
                heartbeat_interval_s=1.0,
                telemetry_interval_s=0.2,
            )
            await self._agent.start(timeout_s=self.config.startup_timeout_s)
            await self._server.wait_for_vehicle(
                self.config.vehicle_id,
                timeout_s=self.config.startup_timeout_s,
            )
        except BaseException:
            await self._cleanup()
            raise
        self._started = True

    async def stop(self) -> None:
        await self._cleanup()

    def public_config(self) -> dict[str, Any]:
        payload = self.config.public_payload()
        payload["vehicles"] = [dict(payload)]
        return payload

    def config_for(self, vehicle_id: int) -> ManualRuntimeConfig:
        if vehicle_id != self.config.vehicle_id:
            raise KeyError(f"vehicle {vehicle_id} is not registered")
        return self.config

    def runtime_for(self, vehicle_id: int) -> ManualRuntime:
        self.config_for(vehicle_id)
        return self

    def server_for(self, vehicle_id: int) -> GroundServer:
        self.config_for(vehicle_id)
        return self.server

    def link_for(self, vehicle_id: int) -> Any | None:
        self.config_for(vehicle_id)
        return self.link

    def started_for(self, vehicle_id: int) -> bool:
        self.config_for(vehicle_id)
        return self.started

    def agent_connected_for(self, vehicle_id: int) -> bool:
        self.config_for(vehicle_id)
        return self.agent_connected

    def error_for(self, vehicle_id: int) -> str | None:
        self.config_for(vehicle_id)
        return None

    @property
    def serial_runtime(self) -> ManualRuntime | None:
        if self.config.mode == ManualRuntimeMode.REAL_SERIAL:
            return self
        return None

    def _create_link(self) -> Any:
        if self._link_factory is not None:
            return self._link_factory(self.config)
        if self.config.mode == ManualRuntimeMode.DEMO:
            return DemoApmLink()
        if self.config.mode == ManualRuntimeMode.REAL_SERIAL:
            raise RuntimeError("real serial mode does not own an FCU link")

        # Keep pymavlink optional for demo-only ground-station installations.
        from aeromind_apm_lite.onboard.mavlink.pymavlink_transport import (
            PymavlinkTransport,
        )

        transport = PymavlinkTransport(self.config.fcu_connection())
        return ApmLink(
            transport,
            target_system=(
                self.config.mavlink_target_system
                if self.config.mavlink_target_system is not None
                else self.config.vehicle_id
            ),
            target_component=self.config.target_component,
            source_system=self.config.source_system,
            source_component=self.config.source_component,
            startup_timeout_s=self.config.startup_timeout_s,
            heartbeat_timeout_s=self.config.heartbeat_timeout_s,
            ack_timeout_s=self.config.ack_timeout_s,
            physical_timeout_s=self.config.physical_timeout_s,
            telemetry_freshness_s=self.config.telemetry_freshness_s,
            completion_hold_s=self.config.completion_hold_s,
            trajectory_timeout_s=10.0,
            companion_heartbeat_hz=1.0,
            setpoint_rate_hz=20.0,
            guided_mode="GUIDED",
            hold_mode="LOITER",
        )

    async def _cleanup(self) -> None:
        agent, server, link = self._agent, self._server, self._link
        self._agent = None
        self._server = None
        self._link = None
        self._started = False
        if agent is not None:
            await agent.stop()
        if server is not None:
            await server.stop()
        if link is not None:
            await link.stop()


class HybridRuntime:
    """Run one local SITL bridge and one real P9 bridge behind one Web API."""

    def __init__(
        self,
        simulation: ManualRuntime,
        real: ManualRuntime,
    ) -> None:
        if simulation.config.mode != ManualRuntimeMode.SITL:
            raise ValueError("hybrid simulation runtime must use sitl mode")
        if real.config.mode != ManualRuntimeMode.REAL_SERIAL:
            raise ValueError("hybrid real runtime must use real_serial mode")
        if simulation.config.vehicle_id == real.config.vehicle_id:
            raise ValueError("hybrid simulation and real vehicle IDs must be different")
        self._runtimes = {
            simulation.config.vehicle_id: simulation,
            real.config.vehicle_id: real,
        }
        self.config = simulation.config
        self._real_vehicle_id = real.config.vehicle_id
        self._errors: dict[int, str] = {}
        self._initialized = False

    @property
    def started(self) -> bool:
        return self._initialized

    @property
    def vehicle_ids(self) -> tuple[int, ...]:
        return tuple(self._runtimes)

    @property
    def default_vehicle_id(self) -> int:
        return self.config.vehicle_id

    @property
    def agent_connected(self) -> bool:
        return self.agent_connected_for(self.default_vehicle_id)

    @property
    def link(self) -> Any | None:
        return self.link_for(self.default_vehicle_id)

    @property
    def server(self) -> GroundServer:
        return self.server_for(self.default_vehicle_id)

    @property
    def serial_runtime(self) -> ManualRuntime:
        return self._runtimes[self._real_vehicle_id]

    def config_for(self, vehicle_id: int) -> ManualRuntimeConfig:
        return self.runtime_for(vehicle_id).config

    def runtime_for(self, vehicle_id: int) -> ManualRuntime:
        try:
            return self._runtimes[vehicle_id]
        except KeyError as exc:
            raise KeyError(f"vehicle {vehicle_id} is not registered") from exc

    def server_for(self, vehicle_id: int) -> GroundServer:
        return self.runtime_for(vehicle_id).server

    def link_for(self, vehicle_id: int) -> Any | None:
        return self.runtime_for(vehicle_id).link

    def started_for(self, vehicle_id: int) -> bool:
        return self.runtime_for(vehicle_id).started

    def agent_connected_for(self, vehicle_id: int) -> bool:
        return self.runtime_for(vehicle_id).agent_connected

    def error_for(self, vehicle_id: int) -> str | None:
        self.runtime_for(vehicle_id)
        return self._errors.get(vehicle_id)

    def clear_error(self, vehicle_id: int) -> None:
        self.runtime_for(vehicle_id)
        self._errors.pop(vehicle_id, None)

    def set_error(self, vehicle_id: int, error: str) -> None:
        self.runtime_for(vehicle_id)
        self._errors[vehicle_id] = error

    def public_config(self) -> dict[str, Any]:
        simulation = self.config.public_payload()
        real = self.serial_runtime.config.public_payload()
        vehicles = [dict(simulation), dict(real)]
        return {
            **simulation,
            "runtime_mode": ManualRuntimeMode.HYBRID.value,
            "deployment_mode": "hybrid",
            "ground_transport": "mixed",
            "serial_runtime_configurable": True,
            "ground_serial_port": real["ground_serial_port"],
            "ground_serial_baudrate": real["ground_serial_baudrate"],
            "vehicles": vehicles,
        }

    async def start(self) -> None:
        if self._initialized:
            raise RuntimeError("hybrid runtime is already started")
        self._errors.clear()
        results = await asyncio.gather(
            *(runtime.start() for runtime in self._runtimes.values()),
            return_exceptions=True,
        )
        for vehicle_id, result in zip(self.vehicle_ids, results):
            if isinstance(result, BaseException):
                self._errors[vehicle_id] = str(result)
        self._initialized = True

    async def stop(self) -> None:
        await asyncio.gather(
            *(runtime.stop() for runtime in self._runtimes.values()),
            return_exceptions=True,
        )
        self._initialized = False


class FleetRuntime:
    """Run several SITL bridges plus an optional real P9 bridge behind one API."""

    def __init__(
        self,
        simulation: ManualRuntime | list[ManualRuntime],
        real: ManualRuntime | None = None,
        *,
        default_vehicle_id: int | None = None,
    ) -> None:
        simulations = (
            [simulation] if isinstance(simulation, ManualRuntime) else list(simulation)
        )
        if not simulations:
            raise ValueError("fleet requires at least one simulation runtime")
        for runtime in simulations:
            if runtime.config.mode != ManualRuntimeMode.SITL:
                raise ValueError("fleet simulation runtimes must use sitl mode")
        if real is not None and real.config.mode != ManualRuntimeMode.REAL_SERIAL:
            raise ValueError("fleet real runtime must use real_serial mode")
        self._sim_vehicle_ids = tuple(runtime.config.vehicle_id for runtime in simulations)
        self._runtimes: dict[int, ManualRuntime] = {
            runtime.config.vehicle_id: runtime for runtime in simulations
        }
        if real is not None:
            self._runtimes[real.config.vehicle_id] = real
        if len(self._runtimes) != len(simulations) + (1 if real is not None else 0):
            raise ValueError("fleet vehicle ids must be unique")
        self._real_vehicle_id = real.config.vehicle_id if real is not None else None
        self.config = simulations[0].config
        self._default_vehicle_id = (
            default_vehicle_id
            if default_vehicle_id is not None
            else self._sim_vehicle_ids[0]
        )
        if self._default_vehicle_id not in self._runtimes:
            raise ValueError("default fleet vehicle is not registered")
        self._errors: dict[int, str] = {}
        self._initialized = False

    @property
    def started(self) -> bool:
        return self._initialized

    @property
    def vehicle_ids(self) -> tuple[int, ...]:
        return tuple(self._runtimes)

    @property
    def sim_vehicle_ids(self) -> tuple[int, ...]:
        return self._sim_vehicle_ids

    @property
    def real_vehicle_id(self) -> int | None:
        return self._real_vehicle_id

    @property
    def default_vehicle_id(self) -> int:
        return self._default_vehicle_id

    @property
    def agent_connected(self) -> bool:
        return self.agent_connected_for(self.default_vehicle_id)

    @property
    def link(self) -> Any | None:
        return self.link_for(self.default_vehicle_id)

    @property
    def server(self) -> GroundServer:
        return self.server_for(self.default_vehicle_id)

    @property
    def serial_runtime(self) -> ManualRuntime | None:
        if self._real_vehicle_id is None:
            return None
        return self._runtimes[self._real_vehicle_id]

    def config_for(self, vehicle_id: int) -> ManualRuntimeConfig:
        return self.runtime_for(vehicle_id).config

    def runtime_for(self, vehicle_id: int) -> ManualRuntime:
        try:
            return self._runtimes[vehicle_id]
        except KeyError as exc:
            raise KeyError(f"vehicle {vehicle_id} is not registered") from exc

    def server_for(self, vehicle_id: int) -> GroundServer:
        return self.runtime_for(vehicle_id).server

    def link_for(self, vehicle_id: int) -> Any | None:
        return self.runtime_for(vehicle_id).link

    def started_for(self, vehicle_id: int) -> bool:
        return self.runtime_for(vehicle_id).started

    def agent_connected_for(self, vehicle_id: int) -> bool:
        return self.runtime_for(vehicle_id).agent_connected

    def error_for(self, vehicle_id: int) -> str | None:
        self.runtime_for(vehicle_id)
        return self._errors.get(vehicle_id)

    def clear_error(self, vehicle_id: int) -> None:
        self.runtime_for(vehicle_id)
        self._errors.pop(vehicle_id, None)

    def set_error(self, vehicle_id: int, error: str) -> None:
        self.runtime_for(vehicle_id)
        self._errors[vehicle_id] = error

    def public_config(self) -> dict[str, Any]:
        base = self.config.public_payload()
        vehicles = [
            runtime.config.public_payload()
            for runtime in self._runtimes.values()
        ]
        real = (
            self.serial_runtime.config.public_payload()
            if self._real_vehicle_id is not None
            else None
        )
        return {
            **base,
            "runtime_mode": "fleet",
            "deployment_mode": "hybrid",
            "ground_transport": "mixed",
            "serial_runtime_configurable": self._real_vehicle_id is not None,
            "ground_serial_port": real["ground_serial_port"] if real else None,
            "ground_serial_baudrate": (
                real["ground_serial_baudrate"] if real else None
            ),
            "vehicles": vehicles,
        }

    async def start(self) -> None:
        if self._initialized:
            raise RuntimeError("fleet runtime is already started")
        self._errors.clear()
        results = await asyncio.gather(
            *(runtime.start() for runtime in self._runtimes.values()),
            return_exceptions=True,
        )
        for vehicle_id, result in zip(self.vehicle_ids, results):
            if isinstance(result, BaseException):
                self._errors[vehicle_id] = str(result)
        self._initialized = True

    async def stop(self) -> None:
        await asyncio.gather(
            *(runtime.stop() for runtime in self._runtimes.values()),
            return_exceptions=True,
        )
        self._initialized = False
