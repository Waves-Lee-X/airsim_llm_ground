"""ArduPilot/ArduCopter-specific mode and MAVLink command mapping."""

from __future__ import annotations

import math
import time

from aeromind_apm_lite.common.coordinates import GeoReference
from aeromind_apm_lite.onboard.mavlink.models import MavlinkEnvelope

from ..config import MavlinkEndpointConfig
from ..models import (
    AdapterKind,
    DispatchRecord,
    EndpointRole,
    ExecutionSemantic,
    MissionAction,
    MissionPlan,
)
from .base import AdapterError, BaseMavlinkAdapter, TelemetryCallback
from .io import MavlinkIO


class ArduPilotAdapter(BaseMavlinkAdapter):
    @property
    def prefer_global_guidance_position(self) -> bool:
        return True

    @property
    def virtual_position_messages(self) -> frozenset[str]:
        # ArduCopter AirSim publishes both local and global positions, but its
        # local origin is not the configured WGS84 home.  One rehearsal must
        # use one coordinate source, so predicted evidence uses valid GPS only.
        return frozenset({"GLOBAL_POSITION_INT"})

    @property
    def kind(self) -> AdapterKind:
        return AdapterKind.ARDUPILOT

    @property
    def mission_mode(self) -> str:
        return "GUIDED"

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
        reconnect_delay_s: float = 1.0,
        silence_timeout_s: float = 5.0,
        sample_interval_s: float = 0.0,
    ) -> None:
        super().__init__(
            endpoint=endpoint,
            georeference=georeference,
            line_id=line_id,
            vehicle_id=vehicle_id,
            platform_type=platform_type,
            role=role,
            telemetry_callback=telemetry_callback,
            io=io,
            reconnect_delay_s=reconnect_delay_s,
            silence_timeout_s=silence_timeout_s,
            sample_interval_s=sample_interval_s,
        )
        self._task_plan: MissionPlan | None = None
        self._task_index = 0
        self._last_target_monotonic_s = 0.0
        self._last_guided_request_monotonic_s = 0.0
        self._terminal_issued = False

    def _reset_connection_state(self) -> None:
        super()._reset_connection_state()
        self._task_plan = None
        self._task_index = 0
        self._last_target_monotonic_s = 0.0
        self._last_guided_request_monotonic_s = 0.0
        self._terminal_issued = False

    async def _dispatch(self, plan: MissionPlan) -> DispatchRecord:
        if self._task_plan is not None and not self._mission_complete:
            raise AdapterError("ArduPilot GUIDED task is already active")
        takeoff = plan.steps[0]
        assert takeoff.altitude_m is not None
        self._task_plan = plan
        self._task_index = 0
        self._terminal_issued = False
        self._last_mission_sequence = len(plan.steps) - 1
        self._mission_sequence = 0
        self._mission_complete = False
        self._mission_phase = "guided:takeoff"
        await self.io.set_mode(self.mission_mode)
        await self.io.send_command_long(
            400,
            (
                1.0,
                21196.0 if self.endpoint.force_arm_for_sitl else 0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ),
        )
        await self.io.send_command_long(
            self.speed_command(),
            (1.0, plan.cruise_speed_m_s, -1.0, 0.0, 0.0, 0.0, 0.0),
        )
        await self.io.send_command_long(
            self.command_for_action(MissionAction.TAKEOFF),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, takeoff.altitude_m),
        )
        return DispatchRecord(
            mission_id=plan.mission_id,
            line_id=self.line_id,
            role=self.role,
            adapter=self.kind,
            execution_semantic=ExecutionSemantic.ARDUPILOT_GUIDED_TASK,
            mission_item_count=len(plan.steps),
            detail=(
                "task translated to ArduPilot GUIDED takeoff/global position targets; "
                "no MISSION upload, attitude, rate, thrust, or actuator command"
            ),
        )

    async def _handle_envelope(self, envelope: MavlinkEnvelope) -> None:
        if envelope.name == "ATTITUDE":
            roll = float(envelope.fields.get("roll", 0.0))
            pitch = float(envelope.fields.get("pitch", 0.0))
            yaw = float(envelope.fields.get("yaw", 0.0))
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            self._attitude = (
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            )
        await super()._handle_envelope(envelope)
        plan = self._task_plan
        if plan is None or self._mission_complete:
            return
        if self._terminal_issued:
            if self._armed is False:
                self._mission_complete = True
                self._mission_phase = "completed"
                self._task_plan = None
            return
        # AirSim/ArduCopter may emit one STABILIZE heartbeat while the sensor
        # bridge is settling.  Retry the task-level GUIDED/ARM/takeoff gate at
        # a bounded rate so a startup race cannot strand the rehearsal.
        now = time.monotonic()
        if (
            (self._armed is not True or self._mode != self.mission_mode)
            and now - self._last_guided_request_monotonic_s >= 1.0
        ):
            takeoff = plan.steps[0]
            assert takeoff.altitude_m is not None
            await self.io.set_mode(self.mission_mode)
            await self.io.send_command_long(
                400,
                (
                    1.0,
                    21196.0 if self.endpoint.force_arm_for_sitl else 0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ),
            )
            await self.io.send_command_long(
                self.command_for_action(MissionAction.TAKEOFF),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, takeoff.altitude_m),
            )
            self._last_guided_request_monotonic_s = now
        step = plan.steps[self._task_index]
        target = self._step_target(step)
        if target is None:
            await self._start_terminal(step.action)
            return
        if step.action == MissionAction.WAYPOINT:
            now = time.monotonic()
            if now - self._last_target_monotonic_s >= 1.0:
                await self._send_guided_target(target)
                self._last_target_monotonic_s = now
        current = self._current_map_point()
        if current is None:
            return
        distance = (
            abs(current.z - target.z)
            if step.action == MissionAction.TAKEOFF
            else math.sqrt(
                (current.x - target.x) ** 2
                + (current.y - target.y) ** 2
                + (current.z - target.z) ** 2
            )
        )
        if distance > step.acceptance_radius_m:
            return
        self._task_index += 1
        self._mission_sequence = self._task_index
        next_step = plan.steps[self._task_index]
        self._mission_phase = f"guided:{next_step.action.value}"
        next_target = self._step_target(next_step)
        if next_target is not None:
            await self._send_guided_target(next_target)
            self._last_target_monotonic_s = time.monotonic()
        else:
            await self._start_terminal(next_step.action)

    async def _send_guided_target(self, target) -> None:
        latitude, longitude, relative_altitude = self._map_to_global_target(target)
        await self.io.send_position_target_global_int(
            latitude,
            longitude,
            relative_altitude,
        )

    async def _start_terminal(self, action: MissionAction) -> None:
        if action == MissionAction.LAND:
            await self.io.set_mode("LAND")
            await self.io.send_command_long(
                self.command_for_action(MissionAction.LAND),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            )
            self._mission_phase = "land"
        elif action == MissionAction.RTL:
            await self.io.set_mode("RTL")
            await self.io.send_command_long(
                self.command_for_action(MissionAction.RTL),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            )
            self._mission_phase = "rtl"
        else:
            raise AdapterError(f"invalid ArduPilot terminal action {action.value}")
        self._terminal_issued = True

    async def safety_hold(self) -> str:
        await self.io.set_mode("LOITER")
        self._mission_phase = "safety_hold"
        return "SET_MODE:LOITER"

    async def recovery_land(self) -> str:
        await self.io.set_mode("LAND")
        await self.io.send_command_long(
            self.command_for_action(MissionAction.LAND),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        self._mission_phase = "recovery_land"
        self._terminal_issued = True
        return "SET_MODE:LAND+MAV_CMD_NAV_LAND"

    def normalize_mode(self, mode: str) -> str:
        normalized = mode.strip().upper().replace(" ", "_")
        return {
            "AUTO": "AUTO",
            "GUIDED": "GUIDED",
            "LOITER": "LOITER",
            "LAND": "LAND",
            "RTL": "RTL",
        }.get(normalized, normalized)

    def command_for_action(self, action: MissionAction) -> int:
        return {
            MissionAction.TAKEOFF: 22,
            MissionAction.WAYPOINT: 16,
            MissionAction.LAND: 21,
            MissionAction.RTL: 20,
        }[action]

    def speed_command(self) -> int:
        return 178


__all__ = ["ArduPilotAdapter"]
