"""PX4-specific mode and MAVLink command mapping."""

from __future__ import annotations

import asyncio
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


_MAV_CMD_DO_SET_MODE = 176
_PX4_MODE_PARAMS: dict[str, tuple[float, float, float]] = {
    # base_mode, PX4_CUSTOM_MAIN_MODE, PX4_CUSTOM_SUB_MODE_AUTO
    "OFFBOARD": (29.0, 6.0, 0.0),
    "LOITER": (29.0, 4.0, 3.0),
    "RTL": (29.0, 4.0, 5.0),
    "LAND": (29.0, 4.0, 6.0),
}


class Px4Adapter(BaseMavlinkAdapter):
    @property
    def kind(self) -> AdapterKind:
        return AdapterKind.PX4

    @property
    def mission_mode(self) -> str:
        return "OFFBOARD"

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
        self._last_setpoint_monotonic_s = 0.0
        self._terminal_issued = False

    def _reset_connection_state(self) -> None:
        super()._reset_connection_state()
        self._task_plan = None
        self._task_index = 0
        self._last_setpoint_monotonic_s = 0.0
        self._terminal_issued = False

    async def _dispatch(self, plan: MissionPlan) -> DispatchRecord:
        if self._task_plan is not None and not self._mission_complete:
            raise AdapterError("PX4 OFFBOARD task is already active")
        first_target = self._step_target(plan.steps[0])
        assert first_target is not None
        local_target = self._map_to_local_target(first_target)
        self._task_plan = plan
        self._task_index = 0
        self._terminal_issued = False
        self._last_mission_sequence = len(plan.steps) - 1
        self._mission_sequence = 0
        self._mission_complete = False
        self._mission_phase = "offboard:takeoff"

        # PX4 requires a live setpoint stream before OFFBOARD is accepted.
        for _ in range(10):
            await self.io.send_position_target_local_ned(
                local_target.x,
                local_target.y,
                local_target.z,
            )
            await asyncio.sleep(0.1)
        await self.io.send_command_long(
            self.speed_command(),
            (1.0, plan.cruise_speed_m_s, -1.0, 0.0, 0.0, 0.0, 0.0),
        )
        await self._set_mode_compat(
            self.mission_mode,
            wire_mode="OFFBOARD",
        )
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
        self._last_setpoint_monotonic_s = time.monotonic()
        return DispatchRecord(
            mission_id=plan.mission_id,
            line_id=self.line_id,
            role=self.role,
            adapter=self.kind,
            execution_semantic=ExecutionSemantic.PX4_OFFBOARD_TASK,
            mission_item_count=len(plan.steps),
            detail=(
                "task translated to PX4 OFFBOARD position-only setpoints; "
                "no MISSION upload, attitude, rate, thrust, or actuator command"
            ),
        )

    async def _handle_envelope(self, envelope: MavlinkEnvelope) -> None:
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
        step = plan.steps[self._task_index]
        target = self._step_target(step)
        if target is None:
            await self._start_terminal(step.action)
            return
        local_target = self._map_to_local_target(target)
        now = time.monotonic()
        if now - self._last_setpoint_monotonic_s >= 0.1:
            await self.io.send_position_target_local_ned(
                local_target.x,
                local_target.y,
                local_target.z,
            )
            self._last_setpoint_monotonic_s = now
        current = self._current_map_point()
        if current is None:
            return
        distance = math.sqrt(
            (current.x - target.x) ** 2
            + (current.y - target.y) ** 2
            + (current.z - target.z) ** 2
        )
        if distance > step.acceptance_radius_m:
            return
        self._task_index += 1
        self._mission_sequence = self._task_index
        next_step = plan.steps[self._task_index]
        self._mission_phase = f"offboard:{next_step.action.value}"
        if next_step.action in {MissionAction.LAND, MissionAction.RTL}:
            await self._start_terminal(next_step.action)

    async def _start_terminal(self, action: MissionAction) -> None:
        if action == MissionAction.LAND:
            await self._set_mode_compat(
                "AUTO.LAND", "AUTO_LAND", "LAND", wire_mode="LAND"
            )
            self._mission_phase = "land"
        elif action == MissionAction.RTL:
            await self._set_mode_compat(
                "AUTO.RTL", "AUTO_RTL", "RTL", wire_mode="RTL"
            )
            self._mission_phase = "rtl"
        else:
            raise AdapterError(f"invalid PX4 terminal action {action.value}")
        self._terminal_issued = True

    async def _set_mode_compat(
        self,
        *candidates: str,
        wire_mode: str | None = None,
    ) -> str:
        """Use the first mode name advertised by this PX4 build.

        PX4 firmware versions expose the same task-level modes as either
        ``AUTO.LOITER``/``AUTO.LAND`` or underscore/short aliases.  The
        wire-level lookup belongs here so the shared core never needs to know
        which spelling a particular FCU advertises.
        """

        last_error: ValueError | None = None
        for mode in candidates:
            try:
                await self.io.set_mode(mode)
                return mode
            except ValueError as exc:
                last_error = exc
        if wire_mode is not None:
            base_mode, main_mode, sub_mode = _PX4_MODE_PARAMS[wire_mode]
            await self.io.send_command_long(
                _MAV_CMD_DO_SET_MODE,
                (base_mode, main_mode, sub_mode, 0.0, 0.0, 0.0, 0.0),
            )
            return wire_mode
        if last_error is not None:
            raise last_error
        raise AdapterError("PX4 mode candidate list is empty")

    async def safety_hold(self) -> str:
        mode = await self._set_mode_compat(
            "AUTO.LOITER",
            "AUTO_LOITER",
            "LOITER",
            "HOLD",
            wire_mode="LOITER",
        )
        self._mission_phase = "safety_hold"
        return f"SET_MODE:{mode}"

    async def recovery_land(self) -> str:
        mode = await self._set_mode_compat(
            "AUTO.LAND", "AUTO_LAND", "LAND", wire_mode="LAND"
        )
        self._mission_phase = "recovery_land"
        self._terminal_issued = True
        return f"SET_MODE:{mode}"

    def normalize_mode(self, mode: str) -> str:
        normalized = mode.strip().upper().replace(" ", ".")
        return {
            "OFFBOARD": "OFFBOARD",
            "AUTO.MISSION": "AUTO.MISSION",
            "AUTO_MISSION": "AUTO.MISSION",
            "AUTO.LOITER": "AUTO.LOITER",
            "AUTO.LAND": "AUTO.LAND",
            "AUTO.RTL": "AUTO.RTL",
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


__all__ = ["Px4Adapter"]
