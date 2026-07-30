"""Health-gated single-vehicle navigation mission shared by SITL and real FCUs."""

from __future__ import annotations

import asyncio
import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum
from typing import Any
from uuid import UUID, uuid4

from aeromind_apm_lite.common.mission import (
    MissionState,
    MissionStateMachine,
    MissionTransition,
)

from .apm_link import ApmLink, CommandHandle
from .flight_commands import FlightCommandService
from .mavlink.models import (
    CommandResult,
    CommandStatus,
    TelemetrySnapshot,
)


EKF_REQUIRED_GPS_NAVIGATION_FLAGS = 55


class NavigationFailureCode(str, Enum):
    PREFLIGHT_TIMEOUT = "preflight_timeout"
    FCU_LINK_LOST = "fcu_link_lost"
    GROUND_LINK_LOST = "ground_link_lost"
    GPS_UNHEALTHY = "gps_unhealthy"
    GPS_FIX_LOST = "gps_fix_lost"
    HDOP_UNAVAILABLE = "hdop_unavailable"
    HDOP_EXCEEDED = "hdop_exceeded"
    EKF_UNHEALTHY = "ekf_unhealthy"
    POSITION_UNAVAILABLE = "position_unavailable"
    TELEMETRY_STALE = "telemetry_stale"
    HOME_UNAVAILABLE = "home_unavailable"
    PREARM_REJECTED = "prearm_rejected"
    MISSION_EXPIRED = "mission_expired"
    COMMAND_FAILED = "command_failed"
    RUNNER_EXCEPTION = "runner_exception"


@dataclass(frozen=True)
class NavigationSafetyLimits:
    minimum_gps_fix_type: int = 3
    maximum_hdop: float = 2.5
    required_ekf_flags: int = EKF_REQUIRED_GPS_NAVIGATION_FLAGS
    telemetry_freshness_s: float = 1.5
    home_freshness_s: float = 5.0
    check_interval_s: float = 0.1

    def __post_init__(self) -> None:
        if not 2 <= self.minimum_gps_fix_type <= 6:
            raise ValueError("minimum_gps_fix_type must be in [2, 6]")
        if not math.isfinite(self.maximum_hdop) or self.maximum_hdop <= 0.0:
            raise ValueError("maximum_hdop must be positive and finite")
        if self.required_ekf_flags < 0:
            raise ValueError("required_ekf_flags must not be negative")
        if self.telemetry_freshness_s <= 0.0:
            raise ValueError("telemetry_freshness_s must be positive")
        if self.home_freshness_s <= 0.0:
            raise ValueError("home_freshness_s must be positive")
        if not 0.01 <= self.check_interval_s <= 1.0:
            raise ValueError("check_interval_s must be in [0.01, 1]")


@dataclass(frozen=True)
class NavigationMissionPlan:
    target_position_ned_m: tuple[float, float, float]
    takeoff_altitude_m: float = 2.0
    yaw_rad: float | None = None
    mission_timeout_s: float = 90.0
    goto_timeout_s: float = 15.0
    mission_id: UUID = field(default_factory=uuid4)

    def __post_init__(self) -> None:
        if len(self.target_position_ned_m) != 3 or not all(
            math.isfinite(value) for value in self.target_position_ned_m
        ):
            raise ValueError("target_position_ned_m requires three finite values")
        if not 0.5 <= self.takeoff_altitude_m <= 20.0:
            raise ValueError("takeoff_altitude_m must be in [0.5, 20]")
        if self.target_position_ned_m[2] > -0.3:
            raise ValueError("GOTO target must remain at least 0.3 m above Home")
        if self.yaw_rad is not None and not -math.pi <= self.yaw_rad <= math.pi:
            raise ValueError("yaw_rad must be in [-pi, pi]")
        if not 5.0 <= self.mission_timeout_s <= 600.0:
            raise ValueError("mission_timeout_s must be in [5, 600]")
        if not 0.5 <= self.goto_timeout_s <= 120.0:
            raise ValueError("goto_timeout_s must be in [0.5, 120]")
        if self.goto_timeout_s >= self.mission_timeout_s:
            raise ValueError("goto_timeout_s must be shorter than mission_timeout_s")


@dataclass(frozen=True)
class NavigationObservation:
    snapshot: TelemetrySnapshot
    ground_link_ok: bool
    mission_deadline_monotonic_s: float
    observed_monotonic_s: float


ObservationFilter = Callable[[str, NavigationObservation], NavigationObservation]


@dataclass(frozen=True)
class NavigationSafetyEvent:
    phase: str
    code: NavigationFailureCode
    observed_monotonic_s: float
    detail: str


@dataclass(frozen=True)
class NavigationStepEvidence:
    name: str
    result: CommandResult


@dataclass(frozen=True)
class NavigationMissionResult:
    mission_id: UUID
    terminal_state: MissionState
    started_monotonic_s: float
    finished_monotonic_s: float
    steps: tuple[NavigationStepEvidence, ...]
    recovery_steps: tuple[NavigationStepEvidence, ...]
    safety_events: tuple[NavigationSafetyEvent, ...]
    transitions: tuple[MissionTransition, ...]
    failure_code: NavigationFailureCode | None
    detail: str

    @property
    def completed(self) -> bool:
        return self.terminal_state == MissionState.COMPLETED


class _NavigationAbort(RuntimeError):
    def __init__(self, event: NavigationSafetyEvent) -> None:
        super().__init__(event.detail)
        self.event = event


class _CommandFailure(RuntimeError):
    def __init__(self, phase: str, result: CommandResult) -> None:
        super().__init__(f"{phase} ended {result.status.value}: {result.detail}")
        self.phase = phase
        self.result = result


def navigation_safety_event(
    phase: str,
    observation: NavigationObservation,
    limits: NavigationSafetyLimits,
    *,
    require_prearm: bool,
) -> NavigationSafetyEvent | None:
    snapshot = observation.snapshot
    now = observation.observed_monotonic_s

    def event(code: NavigationFailureCode, detail: str) -> NavigationSafetyEvent:
        return NavigationSafetyEvent(phase, code, now, detail)

    if now >= observation.mission_deadline_monotonic_s:
        return event(NavigationFailureCode.MISSION_EXPIRED, "mission deadline expired")
    if not observation.ground_link_ok:
        return event(NavigationFailureCode.GROUND_LINK_LOST, "P9/ground link is not fresh")
    if not snapshot.fcu_link_ok:
        return event(NavigationFailureCode.FCU_LINK_LOST, "FCU heartbeat is not fresh")
    if snapshot.gps_healthy is not True:
        return event(NavigationFailureCode.GPS_UNHEALTHY, "GPS health bit is not ready")
    if (
        snapshot.gps_fix_type is None
        or snapshot.gps_fix_type < limits.minimum_gps_fix_type
    ):
        return event(
            NavigationFailureCode.GPS_FIX_LOST,
            f"GPS fix is below {limits.minimum_gps_fix_type}",
        )
    if snapshot.gps_hdop is None:
        return event(NavigationFailureCode.HDOP_UNAVAILABLE, "GPS HDOP is unavailable")
    if snapshot.gps_hdop > limits.maximum_hdop:
        return event(
            NavigationFailureCode.HDOP_EXCEEDED,
            f"GPS HDOP {snapshot.gps_hdop:.2f} exceeds {limits.maximum_hdop:.2f}",
        )
    if snapshot.ekf_flags is None or (
        snapshot.ekf_flags & limits.required_ekf_flags
    ) != limits.required_ekf_flags:
        return event(
            NavigationFailureCode.EKF_UNHEALTHY,
            "EKF flags do not permit GPS navigation",
        )
    if snapshot.local_position_ned_m is None:
        return event(
            NavigationFailureCode.POSITION_UNAVAILABLE,
            "LOCAL_POSITION_NED is unavailable",
        )
    if snapshot.home_position_deg_m is None:
        return event(
            NavigationFailureCode.HOME_UNAVAILABLE,
            "HOME_POSITION is unavailable",
        )
    if require_prearm and snapshot.prearm_ok is not True:
        return event(
            NavigationFailureCode.PREARM_REJECTED,
            "ArduPilot pre-arm checks are not ready",
        )

    maximum_ages = {
        "heartbeat": limits.telemetry_freshness_s,
        "gps": limits.telemetry_freshness_s,
        "ekf": limits.telemetry_freshness_s,
        "local_position": limits.telemetry_freshness_s,
        "home": limits.home_freshness_s,
    }
    if require_prearm:
        maximum_ages["prearm"] = limits.telemetry_freshness_s
    for field_name, maximum_age in maximum_ages.items():
        age = snapshot.field_ages_s.get(field_name)
        if age is None or age > maximum_age:
            return event(
                NavigationFailureCode.TELEMETRY_STALE,
                f"{field_name} telemetry age exceeds {maximum_age:g}s",
            )
    return None


class NavigationMissionRunner:
    """Execute one health-gated GUIDED navigation mission with safe recovery."""

    def __init__(
        self,
        link: ApmLink,
        *,
        safety_limits: NavigationSafetyLimits | None = None,
        ground_link_ok: Callable[[], bool] | None = None,
        observation_filter: ObservationFilter | None = None,
        preflight_timeout_s: float = 10.0,
        preflight_hold_s: float = 0.5,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not 0.1 <= preflight_timeout_s <= 120.0:
            raise ValueError("preflight_timeout_s must be in [0.1, 120]")
        if not 0.0 <= preflight_hold_s <= 10.0:
            raise ValueError("preflight_hold_s must be in [0, 10]")
        self._link = link
        self._service = FlightCommandService(link)
        self._limits = safety_limits or NavigationSafetyLimits()
        self._ground_link_ok = ground_link_ok or (lambda: True)
        self._observation_filter = observation_filter
        self._preflight_timeout_s = preflight_timeout_s
        self._preflight_hold_s = preflight_hold_s
        self._clock = clock
        self._lock = asyncio.Lock()
        self._last_result: NavigationMissionResult | None = None

    @property
    def last_result(self) -> NavigationMissionResult | None:
        return self._last_result

    async def run(self, plan: NavigationMissionPlan) -> NavigationMissionResult:
        async with self._lock:
            return await self._run_locked(plan)

    async def _run_locked(
        self,
        plan: NavigationMissionPlan,
    ) -> NavigationMissionResult:
        machine = MissionStateMachine(plan.mission_id)
        started = self._clock()
        deadline = started + plan.mission_timeout_s
        steps: list[NavigationStepEvidence] = []
        recovery: list[NavigationStepEvidence] = []
        safety_events: list[NavigationSafetyEvent] = []
        arm_attempted = False

        machine.transition(MissionState.VALIDATED, reason="navigation plan validated")
        preflight_event = await self._wait_for_preflight(deadline)
        if preflight_event is not None:
            safety_events.append(preflight_event)
            terminal = (
                MissionState.EXPIRED
                if preflight_event.code == NavigationFailureCode.MISSION_EXPIRED
                else MissionState.FAILED
            )
            machine.transition(terminal, reason=preflight_event.detail)
            return self._finish(
                machine,
                started,
                steps,
                recovery,
                safety_events,
                preflight_event.code,
                preflight_event.detail,
            )

        machine.transition(MissionState.ACCEPTED, reason="navigation health gate passed")
        machine.transition(MissionState.RUNNING, reason="starting GUIDED sequence")

        def goto_handle() -> CommandHandle:
            expires = min(deadline, self._clock() + plan.goto_timeout_s)
            return self._service.goto_local_ned(
                *plan.target_position_ned_m,
                yaw_rad=plan.yaw_rad,
                expires_monotonic_s=expires,
            )

        sequence: tuple[tuple[str, Callable[[], CommandHandle], bool], ...] = (
            (
                "guided",
                lambda: self._service.set_mode(
                    self._link.guided_mode,
                    expires_monotonic_s=deadline,
                ),
                True,
            ),
            (
                "arm",
                lambda: self._service.arm(expires_monotonic_s=deadline),
                True,
            ),
            (
                "takeoff",
                lambda: self._service.takeoff(
                    plan.takeoff_altitude_m,
                    expires_monotonic_s=deadline,
                ),
                False,
            ),
            ("goto", goto_handle, False),
            (
                "hold",
                lambda: self._service.hold(expires_monotonic_s=deadline),
                False,
            ),
            (
                "land",
                lambda: self._service.land(expires_monotonic_s=deadline),
                False,
            ),
        )

        try:
            for phase, create_handle, require_prearm in sequence:
                if phase == "arm":
                    arm_attempted = True
                if phase != "land":
                    self._ensure_safe(
                        phase,
                        deadline,
                        require_prearm=require_prearm,
                    )
                handle = create_handle()
                if phase == "land":
                    result = await handle.result
                else:
                    result = await self._await_result_with_safety(
                        phase,
                        handle,
                        deadline,
                        require_prearm=require_prearm,
                    )
                steps.append(NavigationStepEvidence(phase, result))
                if result.status != CommandStatus.COMPLETED:
                    raise _CommandFailure(phase, result)
        except _NavigationAbort as exc:
            safety_events.append(exc.event)
            recovery.extend(await self._recover(arm_attempted))
            terminal = (
                MissionState.EXPIRED
                if exc.event.code == NavigationFailureCode.MISSION_EXPIRED
                else MissionState.FAILED
            )
            machine.transition(terminal, reason=exc.event.detail)
            return self._finish(
                machine,
                started,
                steps,
                recovery,
                safety_events,
                exc.event.code,
                exc.event.detail,
            )
        except _CommandFailure as exc:
            recovery.extend(await self._recover(arm_attempted and exc.phase != "land"))
            expired = exc.result.status in {
                CommandStatus.EXPIRED_BEFORE_SEND,
                CommandStatus.EXPIRED_DURING_EXECUTION,
            }
            terminal = MissionState.EXPIRED if expired else MissionState.FAILED
            code = (
                NavigationFailureCode.MISSION_EXPIRED
                if expired
                else NavigationFailureCode.COMMAND_FAILED
            )
            machine.transition(terminal, reason=str(exc))
            return self._finish(
                machine,
                started,
                steps,
                recovery,
                safety_events,
                code,
                str(exc),
            )
        except asyncio.CancelledError as cancellation:
            recovery.extend(await self._recover(arm_attempted))
            machine.transition(MissionState.CANCELLED, reason="mission task cancelled")
            self._finish(
                machine,
                started,
                steps,
                recovery,
                safety_events,
                None,
                "mission task cancelled",
            )
            raise cancellation
        except Exception as exc:
            recovery.extend(await self._recover(arm_attempted))
            detail = f"runner exception: {type(exc).__name__}: {exc}"
            machine.transition(MissionState.FAILED, reason=detail)
            return self._finish(
                machine,
                started,
                steps,
                recovery,
                safety_events,
                NavigationFailureCode.RUNNER_EXCEPTION,
                detail,
            )

        machine.transition(
            MissionState.COMPLETED,
            reason="GUIDED navigation and physical landing completed",
        )
        return self._finish(
            machine,
            started,
            steps,
            recovery,
            safety_events,
            None,
            "GUIDED, TAKEOFF, GOTO, HOLD and LAND completed with physical evidence",
        )

    async def _wait_for_preflight(
        self,
        mission_deadline_s: float,
    ) -> NavigationSafetyEvent | None:
        loop = asyncio.get_running_loop()
        timeout_at = loop.time() + self._preflight_timeout_s
        healthy_since: float | None = None
        last_event: NavigationSafetyEvent | None = None
        while loop.time() < timeout_at:
            observation = self._observation("preflight", mission_deadline_s)
            current = navigation_safety_event(
                "preflight",
                observation,
                self._limits,
                require_prearm=True,
            )
            if current is None:
                healthy_since = healthy_since or loop.time()
                if loop.time() - healthy_since >= self._preflight_hold_s:
                    return None
            else:
                if current.code == NavigationFailureCode.MISSION_EXPIRED:
                    return current
                healthy_since = None
                last_event = current
            await asyncio.sleep(self._limits.check_interval_s)
        detail = "preflight health did not stabilize"
        if last_event is not None:
            detail = f"{detail}: {last_event.detail}"
        return NavigationSafetyEvent(
            "preflight",
            NavigationFailureCode.PREFLIGHT_TIMEOUT,
            self._clock(),
            detail,
        )

    def _observation(
        self,
        phase: str,
        mission_deadline_s: float,
    ) -> NavigationObservation:
        observation = NavigationObservation(
            snapshot=self._link.telemetry_snapshot(),
            ground_link_ok=bool(self._ground_link_ok()),
            mission_deadline_monotonic_s=mission_deadline_s,
            observed_monotonic_s=self._clock(),
        )
        if self._observation_filter is not None:
            observation = self._observation_filter(phase, observation)
            if not isinstance(observation, NavigationObservation):
                raise TypeError("observation_filter must return NavigationObservation")
        return observation

    def _ensure_safe(
        self,
        phase: str,
        deadline: float,
        *,
        require_prearm: bool,
    ) -> None:
        event = navigation_safety_event(
            phase,
            self._observation(phase, deadline),
            self._limits,
            require_prearm=require_prearm,
        )
        if event is not None:
            raise _NavigationAbort(event)

    async def _await_result_with_safety(
        self,
        phase: str,
        handle: CommandHandle,
        deadline: float,
        *,
        require_prearm: bool,
    ) -> CommandResult:
        result_task = asyncio.ensure_future(handle.result)
        try:
            while True:
                done, _ = await asyncio.wait(
                    {result_task},
                    timeout=self._limits.check_interval_s,
                )
                if done:
                    return result_task.result()
                self._ensure_safe(
                    phase,
                    deadline,
                    require_prearm=require_prearm,
                )
        except BaseException:
            result_task.cancel()
            await asyncio.gather(result_task, return_exceptions=True)
            raise

    async def _recover(
        self,
        enabled: bool,
    ) -> tuple[NavigationStepEvidence, ...]:
        if not enabled:
            return ()
        evidence: list[NavigationStepEvidence] = []
        for name, create_handle in (
            ("safety_hold", self._service.hold),
            ("recovery_land", self._service.land),
        ):
            try:
                result = await create_handle().result
            except Exception:
                continue
            evidence.append(NavigationStepEvidence(name, result))
        return tuple(evidence)

    def _finish(
        self,
        machine: MissionStateMachine,
        started: float,
        steps: list[NavigationStepEvidence],
        recovery: list[NavigationStepEvidence],
        safety_events: list[NavigationSafetyEvent],
        failure_code: NavigationFailureCode | None,
        detail: str,
    ) -> NavigationMissionResult:
        result = NavigationMissionResult(
            mission_id=machine.mission_id,
            terminal_state=machine.state,
            started_monotonic_s=started,
            finished_monotonic_s=self._clock(),
            steps=tuple(steps),
            recovery_steps=tuple(recovery),
            safety_events=tuple(safety_events),
            transitions=machine.history,
            failure_code=failure_code,
            detail=detail,
        )
        self._last_result = result
        return result


def navigation_mission_result_dict(
    result: NavigationMissionResult,
) -> dict[str, Any]:
    def step_payload(step: NavigationStepEvidence) -> dict[str, Any]:
        return {
            "name": step.name,
            "action": step.result.action.value,
            "status": step.result.status.value,
            "physical_completion_confirmed": (
                step.result.physical_completion.confirmed
            ),
            "detail": step.result.detail,
        }

    return {
        "mission_id": str(result.mission_id),
        "terminal_state": result.terminal_state.value,
        "started_monotonic_s": result.started_monotonic_s,
        "finished_monotonic_s": result.finished_monotonic_s,
        "steps": [step_payload(step) for step in result.steps],
        "recovery_steps": [step_payload(step) for step in result.recovery_steps],
        "safety_events": [
            {
                "phase": event.phase,
                "code": event.code.value,
                "observed_monotonic_s": event.observed_monotonic_s,
                "detail": event.detail,
            }
            for event in result.safety_events
        ],
        "transitions": [
            {
                "previous": transition.previous.value,
                "current": transition.current.value,
                "occurred_at_utc": transition.occurred_at_utc.isoformat(),
                "reason": transition.reason,
            }
            for transition in result.transitions
        ],
        "failure_code": (
            result.failure_code.value if result.failure_code is not None else None
        ),
        "detail": result.detail,
    }
