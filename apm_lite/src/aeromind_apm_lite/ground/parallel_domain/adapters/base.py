"""Stack-neutral adapter mechanics and telemetry decoding."""

from __future__ import annotations

import asyncio
import math
import time
from abc import ABC, abstractmethod
from collections.abc import Awaitable, Callable
from datetime import datetime, timezone
from typing import Any

from aeromind_apm_lite.common.coordinates import (
    GeoReference,
    Vec3,
    geodetic_to_map,
    local_ned_to_map,
    map_to_geodetic,
    map_to_enu,
    map_to_local_ned,
)
from aeromind_apm_lite.onboard.mavlink.models import MavlinkEnvelope

from ..config import MavlinkEndpointConfig
from ..models import (
    AdapterKind,
    AdapterTelemetry,
    DispatchRecord,
    EndpointRole,
    MissionAction,
    MissionPlan,
    MissionStep,
)
from .base_types import MissionWireItem
from .io import MavlinkIO, PymavlinkIO


TelemetryCallback = Callable[[AdapterTelemetry], Awaitable[None]]


class AdapterError(RuntimeError):
    pass


class FlightControllerAdapter(ABC):
    """A single FCU endpoint owned by one asyncio task."""

    @property
    @abstractmethod
    def kind(self) -> AdapterKind:
        raise NotImplementedError

    @property
    @abstractmethod
    def mission_mode(self) -> str:
        raise NotImplementedError

    @abstractmethod
    async def start(self) -> None:
        raise NotImplementedError

    @abstractmethod
    async def stop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    async def execute(self, plan: MissionPlan) -> DispatchRecord:
        raise NotImplementedError

    @abstractmethod
    async def safety_hold(self) -> str:
        """Enter the stack-specific task-level hold mode and return its command label."""

        raise NotImplementedError

    @abstractmethod
    async def recovery_land(self) -> str:
        """Enter the stack-specific task-level landing mode and return its command label."""

        raise NotImplementedError

    @abstractmethod
    def latest_telemetry(self) -> AdapterTelemetry | None:
        raise NotImplementedError

    def link_status(self) -> dict[str, object]:
        """Return transport diagnostics without requiring test adapters to implement it."""

        return {
            "connected": self.latest_telemetry() is not None,
            "connection_generation": 0,
            "reconnect_count": 0,
            "last_error": None,
            "last_message_monotonic_s": None,
        }


class BaseMavlinkAdapter(FlightControllerAdapter):
    def __init__(
        self,
        *,
        endpoint: MavlinkEndpointConfig,
        georeference: GeoReference,
        line_id: str,
        vehicle_id: int,
        platform_type: str,
        role: EndpointRole,
        telemetry_callback: TelemetryCallback,
        io: MavlinkIO | None = None,
        io_poll_interval_s: float = 0.01,
        reconnect_delay_s: float = 1.0,
        silence_timeout_s: float = 5.0,
        sample_interval_s: float = 0.0,
    ) -> None:
        if not georeference.is_complete:
            raise ValueError("parallel-domain adapters require a complete GeoReference")
        if len([home for home in georeference.vehicle_homes if home.vehicle_id == vehicle_id]) != 1:
            raise ValueError(f"GeoReference requires one home for vehicle {vehicle_id}")
        self.endpoint = endpoint
        self.georeference = georeference
        self.line_id = line_id
        self.vehicle_id = vehicle_id
        self.platform_type = platform_type
        self.role = role
        self.telemetry_callback = telemetry_callback
        self.io = io or PymavlinkIO(
            _fcu_connection(endpoint),
            poll_interval_s=io_poll_interval_s,
            dialect=endpoint.dialect,
        )
        if reconnect_delay_s <= 0.0 or silence_timeout_s <= 0.0:
            raise ValueError("reconnect and silence timeouts must be positive")
        if sample_interval_s < 0.0:
            raise ValueError("sample interval must not be negative")
        self.reconnect_delay_s = reconnect_delay_s
        self.silence_timeout_s = silence_timeout_s
        self.sample_interval_s = sample_interval_s
        self._task: asyncio.Task[None] | None = None
        self._commands: asyncio.Queue[
            tuple[MissionPlan, asyncio.Future[DispatchRecord]]
        ] = asyncio.Queue()
        self._stop_event = asyncio.Event()
        self._latest: AdapterTelemetry | None = None
        self._connected = False
        self._connection_generation = 0
        self._reconnect_count = 0
        self._last_error: str | None = None
        self._last_message_monotonic_s: float | None = None
        self._source_sequence = 0
        self._last_mission_sequence: int | None = None
        self._last_heartbeat_monotonic_s: float | None = None
        self._last_gps_monotonic_s: float | None = None
        self._global_position: tuple[float, float, float] | None = None
        self._local_position: tuple[float, float, float] | None = None
        self._velocity: tuple[float, float, float] | None = None
        self._attitude: tuple[float, float, float, float] | None = None
        self._armed: bool | None = None
        self._mode = "UNKNOWN"
        self._mission_sequence: int | None = None
        self._mission_complete = False
        self._mission_phase = "idle"
        self._battery: float | None = None
        self._gps_fix: int | None = None
        self._gps_hdop: float | None = None
        self._satellites: int | None = None
        self._health_ok = True
        self._sim_time_s: float | None = None
        self._last_sample_emitted_monotonic_s: float | None = None
        self._command_acks: dict[int, int] = {}

    async def start(self) -> None:
        if self._task is not None:
            raise RuntimeError(f"{self.line_id}/{self.role.value} adapter is running")
        self._stop_event.clear()
        self._task = asyncio.create_task(
            self._run(), name=f"mavlink-{self.line_id}-{self.role.value}"
        )

    async def stop(self) -> None:
        if self._task is None:
            return
        self._stop_event.set()
        await self._task
        self._task = None

    async def execute(self, plan: MissionPlan) -> DispatchRecord:
        if self._task is None:
            raise AdapterError("adapter must be started before execute")
        if plan.vehicle_id != self.vehicle_id or plan.line_id != self.line_id:
            raise AdapterError("mission target does not match adapter")
        future: asyncio.Future[DispatchRecord] = asyncio.get_running_loop().create_future()
        await self._commands.put((plan, future))
        return await future

    def latest_telemetry(self) -> AdapterTelemetry | None:
        return self._latest

    def link_status(self) -> dict[str, object]:
        return {
            "connected": self._connected,
            "connection_generation": self._connection_generation,
            "reconnect_count": self._reconnect_count,
            "last_error": self._last_error,
            "last_message_monotonic_s": self._last_message_monotonic_s,
            "command_acks": dict(self._command_acks),
        }

    async def _run(self) -> None:
        while not self._stop_event.is_set():
            opened_at = time.monotonic()
            try:
                self._reset_connection_state()
                await self.io.open()
                await self.io.send_heartbeat()
                self._connected = True
                self._connection_generation += 1
                self._last_error = None
                while not self._stop_event.is_set():
                    envelope = await self.io.recv(0.05)
                    now = time.monotonic()
                    if envelope is not None:
                        if envelope.source_system == self.endpoint.target_system:
                            self._last_message_monotonic_s = now
                            await self._handle_envelope(envelope)
                        # A busy UDP socket can otherwise keep returning immediately.
                        # Yield even for filtered system IDs so one high-rate SITL
                        # cannot starve the other three adapter tasks.
                        await asyncio.sleep(0)
                    elif now - (
                        self._last_message_monotonic_s or opened_at
                    ) >= self.silence_timeout_s:
                        raise AdapterError(
                            f"MAVLink silent for {self.silence_timeout_s:.1f}s"
                        )
                    await self._process_one_command()
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._reconnect_count += 1
            finally:
                self._connected = False
                self._latest = None
                try:
                    await self.io.close()
                except Exception as exc:
                    if self._last_error is None:
                        self._last_error = f"{type(exc).__name__}: {exc}"
            if not self._stop_event.is_set():
                try:
                    await asyncio.wait_for(
                        self._stop_event.wait(), timeout=self.reconnect_delay_s
                    )
                except asyncio.TimeoutError:
                    pass

    async def _process_one_command(self) -> None:
        try:
            plan, future = self._commands.get_nowait()
        except asyncio.QueueEmpty:
            return
        if future.cancelled():
            return
        try:
            result = await self._dispatch(plan)
        except Exception as exc:  # command failures must reach the caller
            future.set_exception(exc)
        else:
            future.set_result(result)

    def _reset_connection_state(self) -> None:
        """A new transport generation must never inherit cached telemetry."""

        self._latest = None
        self._last_message_monotonic_s = None
        self._last_heartbeat_monotonic_s = None
        self._last_gps_monotonic_s = None
        self._global_position = None
        self._local_position = None
        self._velocity = None
        self._attitude = None
        self._armed = None
        self._mode = "UNKNOWN"
        self._mission_sequence = None
        self._mission_complete = False
        self._mission_phase = "idle"
        self._battery = None
        self._gps_fix = None
        self._gps_hdop = None
        self._satellites = None
        self._health_ok = True
        self._sim_time_s = None
        self._last_sample_emitted_monotonic_s = None
        self._command_acks = {}

    @property
    def virtual_position_messages(self) -> frozenset[str]:
        """Messages that can form AirSim predicted position samples."""

        return frozenset({"LOCAL_POSITION_NED"})

    @property
    def prefer_global_guidance_position(self) -> bool:
        return False

    @abstractmethod
    async def _dispatch(self, plan: MissionPlan) -> DispatchRecord:
        """Translate one task plan into this stack's guided execution semantics."""

        raise NotImplementedError

    async def _handle_envelope(self, envelope: MavlinkEnvelope) -> None:
        name = envelope.name.upper()
        fields = envelope.fields
        time_boot_ms = fields.get("time_boot_ms")
        if time_boot_ms is not None:
            self._sim_time_s = max(0.0, float(time_boot_ms) / 1_000.0)
        if name == "HEARTBEAT":
            self._last_heartbeat_monotonic_s = time.monotonic()
            self._armed = bool(int(fields.get("base_mode", 0)) & 128)
            mode = str(fields.get("mode_name", fields.get("custom_mode", "UNKNOWN")))
            self._mode = self.normalize_mode(mode)
            if self._mode in {"LAND", "RTL"}:
                self._mission_phase = self._mode.lower()
        elif name == "GLOBAL_POSITION_INT":
            self._global_position = (
                float(fields.get("lat", 0)) / 10_000_000.0,
                float(fields.get("lon", 0)) / 10_000_000.0,
                float(fields.get("alt", 0)) / 1_000.0,
            )
            self._velocity = (
                float(fields.get("vx", 0)) / 100.0,
                float(fields.get("vy", 0)) / 100.0,
                float(fields.get("vz", 0)) / 100.0,
            )
        elif name == "LOCAL_POSITION_NED":
            self._local_position = (
                float(fields.get("x", 0)),
                float(fields.get("y", 0)),
                float(fields.get("z", 0)),
            )
            self._velocity = (
                float(fields.get("vx", 0)),
                float(fields.get("vy", 0)),
                float(fields.get("vz", 0)),
            )
        elif name == "ATTITUDE_QUATERNION":
            self._attitude = tuple(
                float(fields.get(key, 0.0))
                for key in ("q1", "q2", "q3", "q4")
            )  # type: ignore[assignment]
        elif name == "GPS_RAW_INT":
            self._last_gps_monotonic_s = time.monotonic()
            self._gps_fix = int(fields.get("fix_type", 0))
            eph = float(fields.get("eph", 65535))
            self._gps_hdop = None if eph >= 65535 else eph / 100.0
            self._satellites = int(fields.get("satellites_visible", 0))
        elif name in {"SYS_STATUS", "BATTERY_STATUS"}:
            remaining = fields.get("battery_remaining")
            if remaining is not None and float(remaining) >= 0:
                self._battery = min(1.0, float(remaining) / 100.0)
        elif name == "MISSION_CURRENT":
            self._mission_sequence = int(fields.get("seq", 0))
            self._mission_phase = f"mission:{self._mission_sequence}"
        elif name == "MISSION_ITEM_REACHED":
            self._mission_sequence = int(fields.get("seq", 0))
            self._mission_phase = f"reached:{self._mission_sequence}"
            if (
                self._last_mission_sequence is not None
                and self._mission_sequence >= self._last_mission_sequence
            ):
                self._mission_complete = True
                self._mission_phase = "completed"
        elif name == "EXTENDED_SYS_STATE":
            landed = int(fields.get("landed_state", 0))
            if (
                landed in {1, 4}
                and self._last_mission_sequence is not None
                and self._mission_phase in {"land", "rtl"}
            ):
                self._mission_complete = True
                self._mission_phase = "completed"
        elif name == "EKF_STATUS_REPORT":
            self._health_ok = int(fields.get("flags", 0)) != 0
        elif name == "STATUSTEXT":
            self._health_ok = "prearm" not in str(fields.get("text", "")).lower()
        elif name == "COMMAND_ACK":
            command = int(fields.get("command", -1))
            result = int(fields.get("result", 255))
            if command >= 0:
                self._command_acks[command] = result
        sample_message = (
            name == "GLOBAL_POSITION_INT"
            if self.role == EndpointRole.REAL
            else name in self.virtual_position_messages
        )
        if not sample_message:
            return
        if (
            self.role == EndpointRole.VIRTUAL
            and name == "GLOBAL_POSITION_INT"
            and not self._global_position_sample_ready()
        ):
            return
        received = time.monotonic()
        if self.role == EndpointRole.REAL and self._global_position is None:
            return
        if (
            self.role == EndpointRole.VIRTUAL
            and self._local_position is None
            and self._global_position is None
        ):
            return
        if (
            self._last_sample_emitted_monotonic_s is not None
            and received - self._last_sample_emitted_monotonic_s
            < self.sample_interval_s
        ):
            return
        self._last_sample_emitted_monotonic_s = received
        heartbeat_ok = (
            self._last_heartbeat_monotonic_s is not None
            and received - self._last_heartbeat_monotonic_s <= 3.0
        )
        heartbeat_age_s = (
            received - self._last_heartbeat_monotonic_s
            if self._last_heartbeat_monotonic_s is not None
            else None
        )
        gps_age_s = (
            received - self._last_gps_monotonic_s
            if self._last_gps_monotonic_s is not None
            else None
        )
        self._source_sequence += 1
        telemetry = AdapterTelemetry(
            vehicle_id=self.vehicle_id,
            sampled_at_utc=datetime.now(timezone.utc),
            received_monotonic_s=received,
            source_sequence=self._source_sequence,
            connection_generation=self._connection_generation,
            sample_kind=(
                "global_position"
                if name == "GLOBAL_POSITION_INT"
                else "local_position"
            ),
            sim_time_s=self._sim_time_s,
            global_position=(
                _wgs84(self._global_position)
                if self._global_position is not None
                and (self.role == EndpointRole.REAL or name == "GLOBAL_POSITION_INT")
                else None
            ),
            local_position_ned_m=self._local_position,
            velocity_ned_m_s=self._velocity,
            attitude_quaternion_wxyz=self._attitude,
            armed=self._armed,
            mode=self._mode,
            mission_phase=self._mission_phase,
            mission_sequence=self._mission_sequence,
            mission_complete=self._mission_complete,
            battery_remaining=self._battery,
            gps_fix_type=self._gps_fix,
            gps_hdop=self._gps_hdop,
            satellites_visible=self._satellites,
            heartbeat_age_s=heartbeat_age_s,
            gps_age_s=gps_age_s,
            link_ok=heartbeat_ok,
            health_ok=self._health_ok,
        )
        self._latest = telemetry
        await self.telemetry_callback(telemetry)

    def _global_position_sample_ready(self) -> bool:
        """Reject MAVLink's zero-filled global position before GPS is usable."""

        if self._global_position is None or (self._gps_fix or 0) < 3:
            return False
        latitude, longitude, _altitude = self._global_position
        return not (math.isclose(latitude, 0.0) and math.isclose(longitude, 0.0))

    @abstractmethod
    def normalize_mode(self, mode: str) -> str:
        raise NotImplementedError

    @abstractmethod
    def command_for_action(self, action: MissionAction) -> int:
        raise NotImplementedError

    @abstractmethod
    def speed_command(self) -> int:
        raise NotImplementedError

    def _build_mission_items(self, plan: MissionPlan) -> list[MissionWireItem]:
        calibration = self.georeference.venue_calibration()
        assert self.georeference.map_origin_wgs84 is not None
        home = next(
            home
            for home in self.georeference.vehicle_homes
            if home.vehicle_id == self.vehicle_id
        )
        home_point = Vec3(*home.map_position_m)
        items: list[MissionWireItem] = [
            MissionWireItem(
                sequence=0,
                frame=6,
                command=self.speed_command(),
                current=1,
                autocontinue=1,
                params=(0.0, 1.0, plan.cruise_speed_m_s, 0.0),
                x=0,
                y=0,
                z=0.0,
            )
        ]
        last_point = home_point
        for step in plan.steps:
            point, relative_altitude = self._step_position(
                step,
                home_point,
                last_point,
            )
            geodetic = map_to_geodetic(
                point,
                calibration,
                self.georeference.map_origin_wgs84.coordinate(),
            )
            command = self.command_for_action(step.action)
            params = (0.0, 0.0, 0.0, 0.0)
            if step.action == MissionAction.WAYPOINT:
                params = (0.0, step.acceptance_radius_m, 0.0, 0.0)
            if step.action == MissionAction.TAKEOFF:
                params = (0.0, 0.0, 0.0, 0.0)
            items.append(
                MissionWireItem(
                    sequence=len(items),
                    frame=6,
                    command=command,
                    current=0,
                    autocontinue=1,
                    params=params,
                    x=round(geodetic.latitude_deg * 10_000_000),
                    y=round(geodetic.longitude_deg * 10_000_000),
                    z=relative_altitude,
                )
            )
            if step.action == MissionAction.WAYPOINT:
                last_point = point
        return items

    def _step_position(
        self,
        step: MissionStep,
        home_point: Vec3,
        last_point: Vec3,
    ) -> tuple[Vec3, float]:
        if step.action == MissionAction.TAKEOFF:
            assert step.altitude_m is not None
            return home_point, step.altitude_m - home_point.z
        if step.action == MissionAction.WAYPOINT:
            assert step.position_map_m is not None
            point = Vec3(*step.position_map_m)
            return point, point.z - home_point.z
        if step.action == MissionAction.LAND:
            return last_point, 0.0
        return home_point, 0.0

    def _home_point(self) -> Vec3:
        home = next(
            home
            for home in self.georeference.vehicle_homes
            if home.vehicle_id == self.vehicle_id
        )
        return Vec3(*home.map_position_m)

    def _home_enu(self) -> Vec3:
        return map_to_enu(self._home_point(), self.georeference.venue_calibration())

    def _map_to_local_target(self, point: Vec3) -> Vec3:
        return map_to_local_ned(
            point,
            self.georeference.venue_calibration(),
            self._home_enu(),
        )

    def _map_to_global_target(self, point: Vec3) -> tuple[float, float, float]:
        assert self.georeference.map_origin_wgs84 is not None
        geodetic = map_to_geodetic(
            point,
            self.georeference.venue_calibration(),
            self.georeference.map_origin_wgs84.coordinate(),
        )
        return (
            geodetic.latitude_deg,
            geodetic.longitude_deg,
            point.z - self._home_point().z,
        )

    def _current_map_point(self) -> Vec3 | None:
        calibration = self.georeference.venue_calibration()
        if self.prefer_global_guidance_position and self._global_position is not None:
            assert self.georeference.map_origin_wgs84 is not None
            return geodetic_to_map(
                _wgs84(self._global_position).coordinate(),
                calibration,
                self.georeference.map_origin_wgs84.coordinate(),
            )
        if self._local_position is not None:
            return local_ned_to_map(
                Vec3(*self._local_position),
                calibration,
                self._home_enu(),
            )
        if self._global_position is not None:
            assert self.georeference.map_origin_wgs84 is not None
            position = _wgs84(self._global_position)
            return geodetic_to_map(
                position.coordinate(),
                calibration,
                self.georeference.map_origin_wgs84.coordinate(),
            )
        return None

    def _step_target(self, step: MissionStep) -> Vec3 | None:
        home = self._home_point()
        if step.action == MissionAction.TAKEOFF:
            assert step.altitude_m is not None
            return Vec3(home.x, home.y, home.z + step.altitude_m)
        if step.action == MissionAction.WAYPOINT:
            assert step.position_map_m is not None
            return Vec3(*step.position_map_m)
        return None


def _fcu_connection(endpoint: MavlinkEndpointConfig) -> Any:
    from aeromind_apm_lite.common.config import FcuConnection, FcuTransport

    return FcuConnection(
        transport=FcuTransport.UDP,
        endpoint=endpoint.endpoint,
        source_system=endpoint.source_system,
        source_component=endpoint.source_component,
        target_system=endpoint.target_system,
        target_component=endpoint.target_component,
    )


def _wgs84(value: tuple[float, float, float] | None) -> Any:
    if value is None:
        return None
    from aeromind_apm_lite.common.coordinates import Wgs84Position

    return Wgs84Position(
        latitude_deg=value[0],
        longitude_deg=value[1],
        altitude_m=value[2],
    )


__all__ = ["AdapterError", "BaseMavlinkAdapter", "FlightControllerAdapter", "TelemetryCallback"]
