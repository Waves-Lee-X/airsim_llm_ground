"""Deterministic adapter for offline acceptance and unit tests."""

from __future__ import annotations

from collections.abc import Awaitable, Callable
from datetime import datetime, timezone

from ..models import (
    AdapterKind,
    AdapterTelemetry,
    DispatchRecord,
    EndpointRole,
    ExecutionSemantic,
    MissionPlan,
)
from .base import AdapterError, FlightControllerAdapter


class FakeFlightControllerAdapter(FlightControllerAdapter):
    def __init__(
        self,
        *,
        kind: AdapterKind,
        line_id: str,
        vehicle_id: int,
        role: EndpointRole,
        telemetry_callback: Callable[[AdapterTelemetry], Awaitable[None]],
    ) -> None:
        self._kind = kind
        self.line_id = line_id
        self.vehicle_id = vehicle_id
        self.role = role
        self._callback = telemetry_callback
        self._started = False
        self._latest: AdapterTelemetry | None = None
        self._source_sequence = 0
        self.executions: list[MissionPlan] = []
        self.safety_commands: list[str] = []

    @property
    def kind(self) -> AdapterKind:
        return self._kind

    @property
    def mission_mode(self) -> str:
        return "OFFBOARD" if self._kind == AdapterKind.PX4 else "GUIDED"

    async def start(self) -> None:
        if self._started:
            raise AdapterError("fake adapter already started")
        self._started = True

    async def stop(self) -> None:
        self._started = False

    async def execute(self, plan: MissionPlan) -> DispatchRecord:
        if not self._started:
            raise AdapterError("fake adapter is not started")
        self.executions.append(plan)
        return DispatchRecord(
            mission_id=plan.mission_id,
            line_id=self.line_id,
            role=self.role,
            adapter=self._kind,
            execution_semantic=ExecutionSemantic.FAKE_TASK,
            mission_item_count=len(plan.steps) + 1,
            detail="fake mission accepted",
        )

    def latest_telemetry(self) -> AdapterTelemetry | None:
        return self._latest

    async def safety_hold(self) -> str:
        command = "SET_MODE:AUTO.LOITER" if self._kind == AdapterKind.PX4 else "SET_MODE:LOITER"
        self.safety_commands.append(command)
        return command

    async def recovery_land(self) -> str:
        command = (
            "SET_MODE:AUTO.LAND"
            if self._kind == AdapterKind.PX4
            else "SET_MODE:LAND+MAV_CMD_NAV_LAND"
        )
        self.safety_commands.append(command)
        return command

    def link_status(self) -> dict[str, object]:
        return {
            "connected": self._started,
            "connection_generation": 1 if self._started else 0,
            "reconnect_count": 0,
            "last_error": None,
            "last_message_monotonic_s": (
                self._latest.received_monotonic_s
                if self._latest is not None
                else None
            ),
        }

    async def emit(
        self,
        *,
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float,
        local_position_ned_m: tuple[float, float, float],
        mission_phase: str,
        mission_sequence: int | None = None,
        mission_complete: bool = False,
        mode: str = "AUTO",
        gps_fix_type: int = 3,
        health_ok: bool = True,
        link_ok: bool = True,
        received_monotonic_s: float = 1.0,
        sampled_at_utc: datetime | None = None,
        sim_time_s: float | None = None,
        heartbeat_age_s: float | None = 0.0,
        gps_age_s: float | None = 0.0,
        source_sequence: int | None = None,
        connection_generation: int = 1,
    ) -> AdapterTelemetry:
        if not self._started:
            raise AdapterError("fake adapter is not started")
        if source_sequence is None:
            self._source_sequence += 1
            source_sequence = self._source_sequence
        else:
            self._source_sequence = max(self._source_sequence, source_sequence)
        telemetry = AdapterTelemetry(
            vehicle_id=self.vehicle_id,
            sampled_at_utc=sampled_at_utc or datetime.now(timezone.utc),
            received_monotonic_s=received_monotonic_s,
            source_sequence=source_sequence,
            connection_generation=connection_generation,
            sample_kind=(
                "global_position"
                if self.role == EndpointRole.REAL
                else "local_position"
            ),
            sim_time_s=sim_time_s,
            global_position=(
                {
                    "latitude_deg": latitude_deg,
                    "longitude_deg": longitude_deg,
                    "altitude_m": altitude_m,
                }
                if self.role == EndpointRole.REAL
                else None
            ),
            local_position_ned_m=local_position_ned_m,
            velocity_ned_m_s=(0.0, 0.0, 0.0),
            attitude_quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
            armed=True,
            mode=mode,
            mission_phase=mission_phase,
            mission_sequence=mission_sequence,
            mission_complete=mission_complete,
            battery_remaining=0.9,
            gps_fix_type=gps_fix_type,
            gps_hdop=0.8,
            satellites_visible=12,
            heartbeat_age_s=heartbeat_age_s,
            gps_age_s=gps_age_s,
            link_ok=link_ok,
            health_ok=health_ok,
        )
        self._latest = telemetry
        await self._callback(telemetry)
        return telemetry


__all__ = ["FakeFlightControllerAdapter"]
