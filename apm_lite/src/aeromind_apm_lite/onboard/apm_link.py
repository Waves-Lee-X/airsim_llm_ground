"""Single-owner ArduPilot MAVLink link and command evidence coordinator."""

from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Sequence
from uuid import UUID, uuid4

from .discovery import MAV_TYPE_QUADROTOR, identity_from_messages, validate_heartbeat
from .mavlink.models import (
    ApplicationAcceptance,
    CommandResult,
    CommandStatus,
    FcuAction,
    FcuCommand,
    FcuIdentity,
    MavCommandId,
    MavLandedState,
    MavMessageId,
    MavResult,
    MavlinkAckEvidence,
    MavlinkEnvelope,
    PhysicalCompletionEvidence,
    TelemetrySnapshot,
)
from .mavlink.transport import MavlinkTransport, message_interval_command_params
from .telemetry import TelemetryAccumulator, horizontal_global_distance_m

if TYPE_CHECKING:
    from aeromind_apm_lite.common.config import VehicleRuntimeConfig

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
AUTOPILOT_VERSION_MESSAGE_ID = int(MavMessageId.AUTOPILOT_VERSION)

DEFAULT_TELEMETRY_INTERVALS_HZ: tuple[tuple[int, float], ...] = (
    (int(MavMessageId.LOCAL_POSITION_NED), 10.0),
    (int(MavMessageId.GLOBAL_POSITION_INT), 10.0),
    (int(MavMessageId.ATTITUDE_QUATERNION), 10.0),
    (int(MavMessageId.GPS_RAW_INT), 2.0),
    (int(MavMessageId.SYS_STATUS), 2.0),
    (int(MavMessageId.BATTERY_STATUS), 1.0),
    (int(MavMessageId.EKF_STATUS_REPORT), 2.0),
    (int(MavMessageId.EXTENDED_SYS_STATE), 2.0),
    (int(MavMessageId.HOME_POSITION), 0.5),
)

Clock = Callable[[], float]


class FcuStartupError(RuntimeError):
    pass


@dataclass
class CommandHandle:
    request_id: UUID
    action: FcuAction
    application: ApplicationAcceptance
    _mavlink_ack: asyncio.Future[MavlinkAckEvidence]
    _physical_completion: asyncio.Future[PhysicalCompletionEvidence]
    _result: asyncio.Future[CommandResult]

    @property
    def mavlink_ack(self) -> asyncio.Future[MavlinkAckEvidence]:
        return asyncio.shield(self._mavlink_ack)

    @property
    def physical_completion(self) -> asyncio.Future[PhysicalCompletionEvidence]:
        return asyncio.shield(self._physical_completion)

    @property
    def result(self) -> asyncio.Future[CommandResult]:
        return asyncio.shield(self._result)


@dataclass
class _QueuedCommand:
    handle: CommandHandle
    command: FcuCommand
    expires_monotonic_s: float | None


@dataclass
class _PendingCommand:
    queued: _QueuedCommand
    mavlink_command_id: int | None
    ack_required: bool
    command_params: tuple[float, ...] | None
    dispatch_not_before_s: float
    sent_monotonic_s: float | None = None
    ack_deadline_s: float = math.inf
    physical_deadline_s: float = math.inf
    next_setpoint_monotonic_s: float | None = None
    physical_candidate_since_s: float | None = None
    rejection_result: int | None = None
    rejection_deadline_s: float | None = None
    diagnostic_status_texts: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class SafetyLatch:
    action: FcuAction
    observed_monotonic_s: float
    detail: str


class ApmLink:
    """Own one transport and serialize all FCU receive/send operations."""

    _SAFETY_PRIORITY = {
        FcuAction.HOLD: 10,
        FcuAction.RTL: 20,
        FcuAction.LAND: 30,
        FcuAction.DISARM: 40,
    }
    _TERMINAL_SAFETY_ACTIONS = {FcuAction.DISARM, FcuAction.LAND, FcuAction.RTL}
    _LATCHING_SAFETY_ACTIONS = {FcuAction.LAND, FcuAction.RTL}

    def __init__(
        self,
        transport: MavlinkTransport,
        *,
        target_system: int,
        target_component: int = 1,
        expected_vehicle_type: int = MAV_TYPE_QUADROTOR,
        startup_timeout_s: float = 5.0,
        ack_timeout_s: float = 2.0,
        physical_timeout_s: float = 30.0,
        heartbeat_timeout_s: float = 3.0,
        telemetry_freshness_s: float = 1.0,
        completion_hold_s: float = 0.5,
        rejection_diagnostic_window_s: float = 0.2,
        trajectory_timeout_s: float = 1.0,
        companion_heartbeat_hz: float | None = 1.0,
        telemetry_intervals_hz: Sequence[
            tuple[int, float]
        ] = DEFAULT_TELEMETRY_INTERVALS_HZ,
        setpoint_rate_hz: float = 20.0,
        position_tolerance_m: float = 0.5,
        speed_tolerance_m_s: float = 0.5,
        altitude_tolerance_m: float = 0.3,
        rtl_home_tolerance_m: float = 2.0,
        guided_mode: str = "GUIDED",
        hold_mode: str = "LOITER",
        source_system: int | None = None,
        source_component: int | None = None,
        queue_size: int = 32,
        poll_interval_s: float = 0.02,
        clock: Clock = time.monotonic,
    ) -> None:
        if not 1 <= target_system <= 255 or not 1 <= target_component <= 255:
            raise ValueError("target system/component must be in [1, 255]")
        if (source_system is None) != (source_component is None):
            raise ValueError("source system/component must either both be set or both be omitted")
        if source_system is not None and not 1 <= source_system <= 255:
            raise ValueError("source system must be in [1, 255]")
        if source_component is not None and not 1 <= source_component <= 255:
            raise ValueError("source component must be in [1, 255]")
        if queue_size < 1:
            raise ValueError("queue_size must be positive")
        if not 1.0 <= setpoint_rate_hz <= 50.0:
            raise ValueError("setpoint_rate_hz must be in [1, 50]")
        if (
            companion_heartbeat_hz is not None
            and not 0.5 <= companion_heartbeat_hz <= 5.0
        ):
            raise ValueError("companion_heartbeat_hz must be in [0.5, 5.0] or None")
        for label, value in (
            ("startup_timeout_s", startup_timeout_s),
            ("ack_timeout_s", ack_timeout_s),
            ("physical_timeout_s", physical_timeout_s),
            ("heartbeat_timeout_s", heartbeat_timeout_s),
            ("telemetry_freshness_s", telemetry_freshness_s),
            ("trajectory_timeout_s", trajectory_timeout_s),
            ("poll_interval_s", poll_interval_s),
        ):
            if value <= 0.0:
                raise ValueError(f"{label} must be positive")
        if not 0.0 <= completion_hold_s <= 5.0:
            raise ValueError("completion_hold_s must be in [0, 5]")
        if not 0.0 <= rejection_diagnostic_window_s <= 1.0:
            raise ValueError("rejection_diagnostic_window_s must be in [0, 1]")

        self._transport = transport
        self._target_system = target_system
        self._target_component = target_component
        self._source_system = source_system
        self._source_component = source_component
        self._expected_vehicle_type = expected_vehicle_type
        self._startup_timeout_s = startup_timeout_s
        self._ack_timeout_s = ack_timeout_s
        self._physical_timeout_s = physical_timeout_s
        self._heartbeat_timeout_s = heartbeat_timeout_s
        self._telemetry_freshness_s = telemetry_freshness_s
        self._completion_hold_s = completion_hold_s
        self._rejection_diagnostic_window_s = rejection_diagnostic_window_s
        self._trajectory_timeout_s = trajectory_timeout_s
        self._companion_heartbeat_period_s = (
            None if companion_heartbeat_hz is None else 1.0 / companion_heartbeat_hz
        )
        self._telemetry_intervals_hz = tuple(telemetry_intervals_hz)
        for message_id, frequency_hz in self._telemetry_intervals_hz:
            message_interval_command_params(message_id, frequency_hz)
        self._setpoint_period_s = 1.0 / setpoint_rate_hz
        self._position_tolerance_m = position_tolerance_m
        self._speed_tolerance_m_s = speed_tolerance_m_s
        self._altitude_tolerance_m = altitude_tolerance_m
        self._rtl_home_tolerance_m = rtl_home_tolerance_m
        self._guided_mode = guided_mode.upper()
        self._hold_mode = hold_mode.upper()
        self._queue_size = queue_size
        self._poll_interval_s = poll_interval_s
        self._clock = clock

        self._normal_queue: asyncio.Queue[_QueuedCommand] = asyncio.Queue()
        self._safety_queue: list[_QueuedCommand] = []
        self._telemetry = TelemetryAccumulator()
        self._pending: _PendingCommand | None = None
        self._runner: asyncio.Task[None] | None = None
        self._ready_event = asyncio.Event()
        self._ready_monotonic_s: float | None = None
        self._fcu_boot_monotonic_s: float | None = None
        self._stop_event = asyncio.Event()
        self._startup_error: Exception | None = None
        self._identity: FcuIdentity | None = None
        self._last_internal_safety_handle: CommandHandle | None = None
        self._heartbeat_lost = False
        self._safety_latch: SafetyLatch | None = None
        self._safety_latch_candidate_since_s: float | None = None
        self._unacknowledged_command_finished_s: dict[int, float] = {}
        self._next_companion_heartbeat_s: float | None = None

    @classmethod
    def from_vehicle_config(
        cls,
        transport: MavlinkTransport,
        config: "VehicleRuntimeConfig",
    ) -> "ApmLink":
        return cls(
            transport,
            target_system=config.fcu.target_system,
            target_component=config.fcu.target_component,
            startup_timeout_s=config.startup_timeout_s,
            ack_timeout_s=config.ack_timeout_s,
            physical_timeout_s=config.physical_timeout_s,
            heartbeat_timeout_s=config.heartbeat_timeout_s,
            telemetry_freshness_s=config.telemetry_freshness_s,
            completion_hold_s=config.completion_hold_s,
            trajectory_timeout_s=config.trajectory_timeout_ms / 1_000.0,
            companion_heartbeat_hz=config.companion_heartbeat_hz,
            setpoint_rate_hz=config.setpoint_rate_hz,
            guided_mode=config.guided_mode,
            hold_mode=config.hold_mode,
            source_system=config.fcu.source_system,
            source_component=config.fcu.source_component,
        )

    @property
    def identity(self) -> FcuIdentity | None:
        return self._identity

    @property
    @property
    def ready_monotonic_s(self) -> float | None:
        """Monotonic time when the FCU first became ready (boot proxy)."""
        return self._ready_monotonic_s

    @property
    def fcu_boot_monotonic_s(self) -> float | None:
        """Host monotonic time when the FCU booted, from SYSTEM_TIME."""
        return self._fcu_boot_monotonic_s

    @property
    def ready(self) -> bool:
        return (
            self._identity is not None
            and not self._heartbeat_lost
            and self._runner is not None
            and not self._runner.done()
        )

    @property
    def last_internal_safety_handle(self) -> CommandHandle | None:
        return self._last_internal_safety_handle

    @property
    def guided_mode(self) -> str:
        return self._guided_mode

    @property
    def trajectory_timeout_s(self) -> float:
        return self._trajectory_timeout_s

    @property
    def safety_latch(self) -> SafetyLatch | None:
        return self._safety_latch

    def reset_safety_latch(self, *, operator_confirmed: bool = False) -> None:
        """Release a LAND/RTL timeout latch after an explicit operator decision.

        Rebuilding ``ApmLink`` also starts with no latch.  This method is only an
        escape hatch for an operator who has independently verified vehicle state.
        """
        if not operator_confirmed:
            raise ValueError("reset requires operator_confirmed=True")
        if self._pending is not None and self._is_safety(self._pending.queued.command.action):
            raise RuntimeError("cannot reset while a safety command is active")
        self._safety_latch = None
        self._safety_latch_candidate_since_s = None

    def telemetry_snapshot(self) -> TelemetrySnapshot:
        return self._telemetry.snapshot(self._clock(), self._heartbeat_timeout_s)

    async def start(self) -> FcuIdentity:
        if self._runner is not None:
            raise RuntimeError("ApmLink can only be started once")
        self._runner = asyncio.create_task(self._run(), name=f"apm-link-{self._target_system}")
        try:
            await asyncio.wait_for(
                self._ready_event.wait(),
                timeout=self._startup_timeout_s + 1.0,
            )
        except asyncio.CancelledError:
            self._stop_event.set()
            if self._runner is not None and not self._runner.done():
                self._runner.cancel()
                await asyncio.shield(
                    asyncio.gather(self._runner, return_exceptions=True)
                )
            raise
        except asyncio.TimeoutError as exc:
            await self.stop()
            raise FcuStartupError("FCU discovery did not finish") from exc
        if self._startup_error is not None:
            raise FcuStartupError(str(self._startup_error)) from self._startup_error
        if self._identity is None:
            raise FcuStartupError("FCU discovery ended without an identity")
        return self._identity

    async def stop(self) -> None:
        self._stop_event.set()
        if self._runner is not None:
            await asyncio.gather(self._runner, return_exceptions=True)

    def submit(
        self,
        command: FcuCommand,
        *,
        expires_monotonic_s: float | None = None,
    ) -> CommandHandle:
        now = self._clock()
        if command.action == FcuAction.GOTO_LOCAL_NED and expires_monotonic_s is None:
            expires_monotonic_s = now + self._trajectory_timeout_s

        if not self.ready:
            handle = self._new_handle(command.action, now)
            return self._reject_handle(
                handle,
                command,
                CommandStatus.APPLICATION_REJECTED,
                "FCU link is not ready",
                now,
            )
        if expires_monotonic_s is not None and now >= expires_monotonic_s:
            handle = self._new_handle(command.action, now)
            return self._reject_handle(
                handle,
                command,
                CommandStatus.EXPIRED_BEFORE_SEND,
                "command expired before application acceptance",
                now,
            )
        existing_safety = (
            self._unfinished_safety_handle(command.action)
            if self._is_safety(command.action)
            else None
        )
        if existing_safety is not None:
            return existing_safety

        handle = self._new_handle(command.action, now)
        is_safety = self._is_safety(command.action)
        latch = self._safety_latch
        if latch is not None and not self._allowed_during_latch(command.action, latch.action):
            return self._reject_handle(
                handle,
                command,
                CommandStatus.APPLICATION_REJECTED,
                f"{latch.action.value.upper()} timeout safety latch blocks this command; "
                "confirm vehicle state and explicitly reset or rebuild the link",
                now,
            )
        strongest = self._strongest_outstanding_safety_action()
        if strongest in self._TERMINAL_SAFETY_ACTIONS and not is_safety:
            return self._reject_handle(
                handle,
                command,
                CommandStatus.APPLICATION_REJECTED,
                f"active {strongest.value.upper()} safety action blocks normal commands",
                now,
            )
        if is_safety and strongest is not None:
            command_priority = self._safety_priority(command.action)
            strongest_priority = self._safety_priority(strongest)
            if command_priority < strongest_priority or (
                command_priority == strongest_priority and command.action != strongest
            ):
                return self._reject_handle(
                    handle,
                    command,
                    CommandStatus.APPLICATION_REJECTED,
                    f"higher-priority {strongest.value.upper()} safety action is active",
                    now,
                )
            self._discard_lower_priority_safety(command.action, now)
        if is_safety and self._queued_count() >= self._queue_size:
            self._discard_normal_queue(now)
        if self._queued_count() >= self._queue_size:
            return self._reject_handle(
                handle,
                command,
                CommandStatus.APPLICATION_REJECTED,
                "bounded FCU command queue is full",
                now,
            )

        queued = _QueuedCommand(
            handle=handle,
            command=command,
            expires_monotonic_s=expires_monotonic_s,
        )
        if is_safety:
            self._safety_queue.append(queued)
        else:
            self._normal_queue.put_nowait(queued)
        return handle

    async def execute(
        self,
        command: FcuCommand,
        *,
        expires_monotonic_s: float | None = None,
    ) -> CommandResult:
        handle = self.submit(command, expires_monotonic_s=expires_monotonic_s)
        return await handle.result

    async def _run(self) -> None:
        opened = False
        try:
            await self._transport.open()
            opened = True
            await self._discover_fcu()
            await self._configure_fcu_streams()
            await self._service_companion_heartbeat(self._clock())
            if self._ready_monotonic_s is None:
                self._ready_monotonic_s = self._clock()
            self._ready_event.set()
            while not self._stop_event.is_set():
                await self._service_companion_heartbeat(self._clock())
                await self._service_queues()
                message = await self._transport.receive(self._poll_interval_s)
                if message is not None and self._is_target_message(message):
                    now = self._clock()
                    if (
                        self._fcu_boot_monotonic_s is None
                        and message.name == "SYSTEM_TIME"
                    ):
                        boot_ms = message.fields.get("time_boot_ms")
                        if boot_ms is not None:
                            self._fcu_boot_monotonic_s = (
                                now - float(boot_ms) / 1000.0
                            )
                    self._telemetry.reduce(message, now)
                    self._capture_pending_status_text(message)
                    await self._handle_command_ack(message, now)
                now = self._clock()
                await self._check_heartbeat(now)
                await self._advance_pending(now)
                self._reconcile_safety_latch(now)
        except Exception as exc:
            if self._identity is None:
                self._startup_error = exc
                self._ready_event.set()
        finally:
            await self._fail_all_commands("FCU link stopped")
            self._identity = None
            if opened:
                await self._transport.close()

    async def _discover_fcu(self) -> None:
        heartbeat = await self._wait_for_target_message("HEARTBEAT", self._startup_timeout_s)
        try:
            validate_heartbeat(
                heartbeat,
                expected_vehicle_type=self._expected_vehicle_type,
            )
        except ValueError as exc:
            raise FcuStartupError(str(exc)) from exc
        self._telemetry.reduce(heartbeat, self._clock())

        await self._transport.send_command_long(
            MavCommandId.REQUEST_MESSAGE,
            (float(AUTOPILOT_VERSION_MESSAGE_ID), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        version = await self._wait_for_target_message(
            "AUTOPILOT_VERSION",
            self._startup_timeout_s,
        )
        self._identity = identity_from_messages(heartbeat, version)

    async def _configure_fcu_streams(self) -> None:
        for message_id, frequency_hz in self._telemetry_intervals_hz:
            await self._transport.request_message_interval(message_id, frequency_hz)

    async def _service_companion_heartbeat(self, now: float) -> None:
        period_s = self._companion_heartbeat_period_s
        if period_s is None:
            return
        if self._next_companion_heartbeat_s is not None and now < self._next_companion_heartbeat_s:
            return
        await self._transport.send_heartbeat()
        self._next_companion_heartbeat_s = now + period_s

    async def _wait_for_target_message(
        self,
        message_name: str,
        timeout_s: float,
    ) -> MavlinkEnvelope:
        deadline = self._clock() + timeout_s
        while True:
            remaining = deadline - self._clock()
            if remaining <= 0.0:
                raise FcuStartupError(f"timeout waiting for {message_name}")
            message = await self._transport.receive(min(self._poll_interval_s, remaining))
            if message is None or not self._is_target_message(message):
                continue
            self._telemetry.reduce(message, self._clock())
            if message.name == message_name:
                return message

    def _is_target_message(self, message: MavlinkEnvelope) -> bool:
        return (
            message.source_system == self._target_system
            and message.source_component == self._target_component
        )

    async def _service_queues(self) -> None:
        if self._safety_queue:
            next_safety = self._peek_next_safety()
            if self._pending is not None:
                if self._pending.rejection_result is not None:
                    await self._finish_pending_rejection(self._clock())
                elif self._safety_priority(
                    next_safety.command.action
                ) > self._safety_priority(self._pending.queued.command.action):
                    await self._finish_pending(
                        CommandStatus.PREEMPTED,
                        f"preempted by higher-priority {next_safety.command.action.value}",
                        physical_confirmed=False,
                        now=self._clock(),
                    )
            self._discard_normal_queue(self._clock())
        if self._pending is not None:
            return
        queued = self._get_next_queued()
        if queued is None:
            return
        now = self._clock()
        if queued.expires_monotonic_s is not None and now >= queued.expires_monotonic_s:
            self._finish_queued_without_send(
                queued,
                CommandStatus.EXPIRED_BEFORE_SEND,
                "command expired while waiting in the FCU queue",
                now,
            )
            return
        try:
            await self._begin_command(queued, now)
        except Exception as exc:
            detail = f"local FCU command dispatch failed: {type(exc).__name__}: {exc}"
            if self._pending is not None and self._pending.queued is queued:
                await self._finish_pending(
                    CommandStatus.DISPATCH_FAILED,
                    detail,
                    physical_confirmed=False,
                    now=self._clock(),
                )
                if queued.command.action == FcuAction.GOTO_LOCAL_NED:
                    self._queue_internal_hold(self._clock())
            else:
                self._finish_queued_without_send(
                    queued,
                    CommandStatus.DISPATCH_FAILED,
                    detail,
                    self._clock(),
                )

    def _get_next_queued(self) -> _QueuedCommand | None:
        if self._safety_queue:
            index = self._next_safety_index()
            return self._safety_queue.pop(index)
        try:
            return self._normal_queue.get_nowait()
        except asyncio.QueueEmpty:
            return None

    def _peek_next_safety(self) -> _QueuedCommand:
        return self._safety_queue[self._next_safety_index()]

    def _next_safety_index(self) -> int:
        return max(
            range(len(self._safety_queue)),
            key=lambda index: self._safety_priority(
                self._safety_queue[index].command.action
            ),
        )

    def _discard_normal_queue(self, now: float) -> None:
        while True:
            try:
                queued = self._normal_queue.get_nowait()
            except asyncio.QueueEmpty:
                return
            self._finish_queued_without_send(
                queued,
                CommandStatus.PREEMPTED,
                "discarded because a safety command took priority",
                now,
            )

    def _discard_lower_priority_safety(self, action: FcuAction, now: float) -> None:
        priority = self._safety_priority(action)
        retained: list[_QueuedCommand] = []
        for queued in self._safety_queue:
            if self._safety_priority(queued.command.action) >= priority:
                retained.append(queued)
                continue
            self._finish_queued_without_send(
                queued,
                CommandStatus.PREEMPTED,
                f"preempted by higher-priority {action.value}",
                now,
            )
        self._safety_queue = retained

    async def _begin_command(self, queued: _QueuedCommand, now: float) -> None:
        command = queued.command
        command_id: int | None
        params: tuple[float, ...] | None
        ack_required = True

        if command.action == FcuAction.ARM:
            command_id = MavCommandId.COMPONENT_ARM_DISARM
            params = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        elif command.action == FcuAction.DISARM:
            command_id = MavCommandId.COMPONENT_ARM_DISARM
            params = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        elif command.action == FcuAction.TAKEOFF:
            command_id = MavCommandId.NAV_TAKEOFF
            params = (0.0, 0.0, 0.0, math.nan, 0.0, 0.0, command.target_altitude_m or 0.0)
        elif command.action in {FcuAction.SET_MODE, FcuAction.HOLD}:
            mode = command.mode if command.action == FcuAction.SET_MODE else self._hold_mode
            mode_number = await self._transport.mode_id(mode or "")
            command_id = MavCommandId.DO_SET_MODE
            params = (
                float(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                float(mode_number),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
        elif command.action == FcuAction.LAND:
            command_id = MavCommandId.NAV_LAND
            params = (0.0, 0.0, 0.0, math.nan, 0.0, 0.0, 0.0)
        elif command.action == FcuAction.RTL:
            command_id = MavCommandId.NAV_RETURN_TO_LAUNCH
            params = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        elif command.action == FcuAction.GOTO_LOCAL_NED:
            command_id = None
            params = None
            ack_required = False
        else:  # pragma: no cover - exhaustive enum guard
            raise ValueError(f"unsupported FCU action: {command.action}")

        command_id_value = None if command_id is None else int(command_id)
        # Quarantine unresolved normal transactions. Safety actions must keep
        # their immediate-dispatch semantics; every ACK after send is honored.
        unacknowledged_finished_s = (
            self._unacknowledged_command_finished_s.get(command_id_value)
            if command_id_value is not None and not self._is_safety(command.action)
            else None
        )
        dispatch_not_before_s = (
            now
            if unacknowledged_finished_s is None
            else max(now, unacknowledged_finished_s + self._ack_timeout_s)
        )
        self._pending = _PendingCommand(
            queued=queued,
            mavlink_command_id=command_id_value,
            ack_required=ack_required,
            command_params=params,
            dispatch_not_before_s=dispatch_not_before_s,
        )
        if now >= dispatch_not_before_s:
            await self._dispatch_pending(now)

    async def _dispatch_pending(self, now: float) -> None:
        pending = self._pending
        if pending is None or pending.sent_monotonic_s is not None:
            return
        pending.sent_monotonic_s = now
        pending.ack_deadline_s = now + self._ack_timeout_s
        pending.physical_deadline_s = now + self._physical_timeout_s
        pending.diagnostic_status_texts.clear()
        if pending.ack_required:
            command_id = pending.mavlink_command_id
            if command_id is None:  # pragma: no cover - internal invariant
                raise ValueError("ACK command is missing MAV_CMD id")
            await self._transport.send_command_long(
                command_id,
                pending.command_params or (),
            )
        else:
            self._resolve_ack(
                pending.queued.handle,
                MavlinkAckEvidence(
                    applicable=False,
                    received=False,
                    command_id=None,
                    result=None,
                    observed_monotonic_s=now,
                    detail="SET_POSITION_TARGET_LOCAL_NED has no COMMAND_ACK",
                ),
            )
            await self._send_pending_setpoint(now)

    async def _advance_pending(self, now: float) -> None:
        pending = self._pending
        if pending is None:
            return
        command = pending.queued.command
        handle = pending.queued.handle

        if pending.sent_monotonic_s is None:
            expires_at = pending.queued.expires_monotonic_s
            if expires_at is not None and now >= expires_at:
                await self._finish_pending(
                    CommandStatus.EXPIRED_BEFORE_SEND,
                    "command expired during same-MAV_CMD dispatch isolation",
                    physical_confirmed=False,
                    now=now,
                )
                return
            if now < pending.dispatch_not_before_s:
                return
            try:
                await self._dispatch_pending(now)
            except Exception as exc:
                await self._finish_pending(
                    CommandStatus.DISPATCH_FAILED,
                    "local FCU command dispatch failed: "
                    f"{type(exc).__name__}: {exc}",
                    physical_confirmed=False,
                    now=now,
                )
            return

        if pending.rejection_result is not None:
            if (
                pending.rejection_deadline_s is None
                or now >= pending.rejection_deadline_s
            ):
                await self._finish_pending_rejection(now)
            return

        expires_at = pending.queued.expires_monotonic_s
        if (
            command.action == FcuAction.GOTO_LOCAL_NED
            and expires_at is not None
            and now >= expires_at
        ):
            await self._finish_pending(
                CommandStatus.EXPIRED_DURING_EXECUTION,
                "GUIDED target expired; stopped setpoint transmission",
                physical_confirmed=False,
                now=now,
            )
            self._queue_internal_hold(now)
            return

        if command.action == FcuAction.GOTO_LOCAL_NED:
            if (
                pending.next_setpoint_monotonic_s is not None
                and now >= pending.next_setpoint_monotonic_s
            ):
                try:
                    await self._send_pending_setpoint(now)
                except Exception as exc:
                    await self._finish_pending(
                        CommandStatus.DISPATCH_FAILED,
                        "local GUIDED setpoint dispatch failed: "
                        f"{type(exc).__name__}: {exc}",
                        physical_confirmed=False,
                        now=now,
                    )
                    self._queue_internal_hold(now)
                    return

        physical_predicate = self._physical_completion_reached(command, now)
        if physical_predicate:
            if pending.physical_candidate_since_s is None:
                pending.physical_candidate_since_s = now
            physically_complete = (
                now - pending.physical_candidate_since_s >= self._completion_hold_s
            )
        else:
            pending.physical_candidate_since_s = None
            physically_complete = False
        ack_done = handle._mavlink_ack.done()
        ack_received = ack_done and handle._mavlink_ack.result().received
        ack_not_applicable = ack_done and not handle._mavlink_ack.result().applicable

        if pending.ack_required and not ack_done and now >= pending.ack_deadline_s:
            self._resolve_ack(
                handle,
                MavlinkAckEvidence(
                    applicable=True,
                    received=False,
                    command_id=pending.mavlink_command_id,
                    result=None,
                    observed_monotonic_s=now,
                    detail="COMMAND_ACK timeout; continuing telemetry reconciliation",
                ),
            )
            ack_done = True

        if physically_complete and (ack_received or ack_not_applicable or ack_done):
            detail = "physical completion confirmed by fresh FCU telemetry"
            if pending.ack_required and not handle._mavlink_ack.result().received:
                detail += "; COMMAND_ACK was not received"
            await self._finish_pending(
                CommandStatus.COMPLETED,
                detail,
                physical_confirmed=True,
                now=now,
            )
            return

        if now >= pending.physical_deadline_s:
            if not handle._mavlink_ack.done():
                self._resolve_ack(
                    handle,
                    MavlinkAckEvidence(
                        applicable=pending.ack_required,
                        received=False,
                        command_id=pending.mavlink_command_id,
                        result=None,
                        observed_monotonic_s=now,
                        detail="link stopped waiting for COMMAND_ACK",
                    ),
                )
            ack = handle._mavlink_ack.result()
            status = CommandStatus.PHYSICAL_TIMEOUT if ack.received else CommandStatus.ACK_TIMEOUT
            if command.action in self._LATCHING_SAFETY_ACTIONS:
                self._safety_latch = SafetyLatch(
                    action=command.action,
                    observed_monotonic_s=now,
                    detail="physical completion was not confirmed before the deadline",
                )
                self._safety_latch_candidate_since_s = None
                self._discard_normal_queue(now)
            await self._finish_pending(
                status,
                "physical completion was not confirmed before the deadline",
                physical_confirmed=False,
                now=now,
            )

    async def _check_heartbeat(self, now: float) -> None:
        if self._heartbeat_lost or self._identity is None:
            return
        if self._telemetry.fresh("heartbeat", now, self._heartbeat_timeout_s):
            return
        self._heartbeat_lost = True
        await self._fail_all_commands(
            "FCU heartbeat timed out; recreate the link before sending more commands",
            status=CommandStatus.FCU_LINK_LOST,
        )

    async def _send_pending_setpoint(self, now: float) -> None:
        pending = self._pending
        if pending is None or pending.queued.command.action != FcuAction.GOTO_LOCAL_NED:
            return
        command = pending.queued.command
        target = command.target_position_ned_m
        if target is None:  # pragma: no cover - validated by FcuCommand
            raise ValueError("missing local NED target")
        await self._transport.send_local_position_target(*target, command.yaw_rad)
        pending.next_setpoint_monotonic_s = now + self._setpoint_period_s

    async def _handle_command_ack(self, message: MavlinkEnvelope, now: float) -> None:
        pending = self._pending
        if message.name != "COMMAND_ACK" or pending is None or not pending.ack_required:
            return
        try:
            command_id = int(message.fields.get("command", -1))
            result = int(message.fields.get("result", -1))
        except (TypeError, ValueError):
            return
        if command_id != pending.mavlink_command_id:
            return
        if not self._ack_targets_this_link(message):
            return
        if pending.sent_monotonic_s is None:
            # COMMAND_ACK has no transaction id.  A retry is therefore kept
            # unsent until the previous command has had a full quiet ACK
            # window; any matching ACK observed here can only be from the old
            # dispatch and restarts that window.
            pending.dispatch_not_before_s = max(
                pending.dispatch_not_before_s,
                now + self._ack_timeout_s,
            )
            return
        if pending.rejection_result is not None or pending.queued.handle._mavlink_ack.done():
            return
        if result == MavResult.IN_PROGRESS:
            return
        evidence = MavlinkAckEvidence(
            applicable=True,
            received=True,
            command_id=pending.mavlink_command_id,
            result=result,
            observed_monotonic_s=now,
            detail=f"COMMAND_ACK result={result}",
        )
        self._resolve_ack(pending.queued.handle, evidence)
        if result != MavResult.ACCEPTED:
            pending.rejection_result = result
            pending.rejection_deadline_s = now + self._rejection_diagnostic_window_s
            if self._rejection_diagnostic_window_s == 0.0:
                await self._finish_pending_rejection(now)

    def _capture_pending_status_text(self, message: MavlinkEnvelope) -> None:
        pending = self._pending
        if (
            pending is None
            or pending.sent_monotonic_s is None
            or message.name != "STATUSTEXT"
        ):
            return
        text = self._telemetry.last_status_text
        if not text or (
            pending.diagnostic_status_texts
            and pending.diagnostic_status_texts[-1] == text
        ):
            return
        if len(pending.diagnostic_status_texts) >= 8:
            pending.diagnostic_status_texts.pop(0)
        pending.diagnostic_status_texts.append(text)

    async def _finish_pending_rejection(self, now: float) -> None:
        pending = self._pending
        if pending is None or pending.rejection_result is None:
            return
        detail = (
            "ArduPilot rejected command with MAV_RESULT "
            f"{pending.rejection_result}"
        )
        if pending.diagnostic_status_texts:
            status_texts = " | ".join(repr(text) for text in pending.diagnostic_status_texts)
            detail += f"; STATUSTEXT after command dispatch: {status_texts}"
        else:
            detail += "; no new STATUSTEXT captured"
        await self._finish_pending(
            CommandStatus.MAVLINK_REJECTED,
            detail,
            physical_confirmed=False,
            now=now,
        )

    def _physical_completion_reached(self, command: FcuCommand, now: float) -> bool:
        telemetry = self._telemetry
        heartbeat_fresh = telemetry.fresh(
            "heartbeat",
            now,
            self._telemetry_freshness_s,
        )
        if command.action == FcuAction.ARM:
            return heartbeat_fresh and telemetry.armed is True
        if command.action == FcuAction.DISARM:
            return heartbeat_fresh and telemetry.armed is False
        if command.action == FcuAction.TAKEOFF:
            return (
                heartbeat_fresh
                and telemetry.armed is True
                and telemetry.mode == self._guided_mode
                and telemetry.fresh("global_position", now, self._telemetry_freshness_s)
                and telemetry.relative_altitude_m is not None
                and command.target_altitude_m is not None
                and telemetry.relative_altitude_m
                >= command.target_altitude_m - self._altitude_tolerance_m
                and telemetry.fresh("velocity", now, self._telemetry_freshness_s)
                and self._speed_within_tolerance(telemetry.velocity_ned_m_s)
            )
        if command.action in {FcuAction.SET_MODE, FcuAction.HOLD}:
            expected = command.mode if command.action == FcuAction.SET_MODE else self._hold_mode
            mode_matches = heartbeat_fresh and telemetry.mode == (expected or "").upper()
            if command.action == FcuAction.HOLD:
                return (
                    mode_matches
                    and telemetry.fresh("velocity", now, self._telemetry_freshness_s)
                    and self._speed_within_tolerance(telemetry.velocity_ned_m_s)
                )
            return mode_matches
        if command.action == FcuAction.GOTO_LOCAL_NED:
            return (
                heartbeat_fresh
                and telemetry.armed is True
                and telemetry.mode == self._guided_mode
                and telemetry.fresh("local_position", now, self._telemetry_freshness_s)
                and telemetry.local_position_ned_m is not None
                and command.target_position_ned_m is not None
                and self._distance_3d(
                    telemetry.local_position_ned_m,
                    command.target_position_ned_m,
                )
                <= self._position_tolerance_m
                and telemetry.fresh("velocity", now, self._telemetry_freshness_s)
                and self._speed_within_tolerance(telemetry.velocity_ned_m_s)
            )
        if command.action == FcuAction.LAND:
            return self._landed_and_disarmed(now)
        if command.action == FcuAction.RTL:
            return self._landed_and_disarmed(now) and self._at_home(now)
        return False

    def _landed_and_disarmed(self, now: float) -> bool:
        return (
            self._telemetry.fresh("heartbeat", now, self._telemetry_freshness_s)
            and self._telemetry.fresh("landed_state", now, self._telemetry_freshness_s)
            and self._telemetry.armed is False
            and self._telemetry.landed_state == MavLandedState.ON_GROUND
        )

    def _at_home(self, now: float) -> bool:
        if not all(
            self._telemetry.fresh(field, now, self._telemetry_freshness_s)
            for field in ("global_position", "home", "gps")
        ):
            return False
        if self._telemetry.gps_fix_type is None or self._telemetry.gps_fix_type < 3:
            return False
        current = self._telemetry.global_position_deg_m
        home = self._telemetry.home_position_deg_m
        if current is None or home is None:
            return False
        return horizontal_global_distance_m(current, home) <= self._rtl_home_tolerance_m

    def _speed_within_tolerance(
        self,
        velocity_ned_m_s: tuple[float, float, float] | None,
    ) -> bool:
        if velocity_ned_m_s is None:
            return False
        return self._distance_3d(velocity_ned_m_s, (0.0, 0.0, 0.0)) <= self._speed_tolerance_m_s

    async def _finish_pending(
        self,
        status: CommandStatus,
        detail: str,
        *,
        physical_confirmed: bool,
        now: float,
    ) -> None:
        pending = self._pending
        if pending is None:
            return
        if pending.sent_monotonic_s is None:
            self._pending = None
            self._finish_queued_without_send(
                pending.queued,
                status,
                detail,
                now,
            )
            return
        handle = pending.queued.handle
        if not handle._mavlink_ack.done():
            self._resolve_ack(
                handle,
                MavlinkAckEvidence(
                    applicable=pending.ack_required,
                    received=False,
                    command_id=pending.mavlink_command_id,
                    result=None,
                    observed_monotonic_s=now,
                    detail=detail,
                ),
            )
        physical = PhysicalCompletionEvidence(
            confirmed=physical_confirmed,
            observed_monotonic_s=now if physical_confirmed else None,
            detail=detail,
        )
        if not handle._physical_completion.done():
            handle._physical_completion.set_result(physical)
        if not handle._result.done():
            handle._result.set_result(
                CommandResult(
                    request_id=handle.request_id,
                    action=handle.action,
                    status=status,
                    application=handle.application,
                    sent_monotonic_s=pending.sent_monotonic_s,
                    mavlink_ack=handle._mavlink_ack.result(),
                    physical_completion=physical,
                    detail=detail,
                )
            )
        if pending.mavlink_command_id is not None:
            ack = handle._mavlink_ack.result()
            if pending.ack_required and not ack.received:
                self._unacknowledged_command_finished_s[
                    pending.mavlink_command_id
                ] = now
            else:
                self._unacknowledged_command_finished_s.pop(
                    pending.mavlink_command_id,
                    None,
                )
        self._pending = None

    def _finish_queued_without_send(
        self,
        queued: _QueuedCommand,
        status: CommandStatus,
        detail: str,
        now: float,
    ) -> None:
        handle = queued.handle
        ack = MavlinkAckEvidence(
            applicable=False,
            received=False,
            command_id=None,
            result=None,
            observed_monotonic_s=now,
            detail=detail,
        )
        physical = PhysicalCompletionEvidence(False, None, detail)
        self._resolve_ack(handle, ack)
        if not handle._physical_completion.done():
            handle._physical_completion.set_result(physical)
        if not handle._result.done():
            handle._result.set_result(
                CommandResult(
                    request_id=handle.request_id,
                    action=handle.action,
                    status=status,
                    application=handle.application,
                    sent_monotonic_s=None,
                    mavlink_ack=ack,
                    physical_completion=physical,
                    detail=detail,
                )
            )

    async def _fail_all_commands(
        self,
        detail: str,
        *,
        status: CommandStatus = CommandStatus.LINK_STOPPED,
    ) -> None:
        if self._pending is not None:
            if self._pending.rejection_result is not None:
                await self._finish_pending_rejection(self._clock())
            else:
                await self._finish_pending(
                    status,
                    detail,
                    physical_confirmed=False,
                    now=self._clock(),
                )
        while True:
            queued = self._get_next_queued()
            if queued is None:
                break
            self._finish_queued_without_send(
                queued,
                status,
                detail,
                self._clock(),
            )

    def _new_handle(
        self,
        action: FcuAction,
        now: float,
        *,
        detail: str = "accepted into bounded FCU queue",
    ) -> CommandHandle:
        loop = asyncio.get_running_loop()
        return CommandHandle(
            request_id=uuid4(),
            action=action,
            application=ApplicationAcceptance(True, now, detail),
            _mavlink_ack=loop.create_future(),
            _physical_completion=loop.create_future(),
            _result=loop.create_future(),
        )

    def _reject_handle(
        self,
        handle: CommandHandle,
        command: FcuCommand,
        status: CommandStatus,
        detail: str,
        now: float,
    ) -> CommandHandle:
        handle.application = ApplicationAcceptance(False, now, detail)
        self._finish_queued_without_send(
            _QueuedCommand(handle, command, None),
            status,
            detail,
            now,
        )
        return handle

    @staticmethod
    def _resolve_ack(handle: CommandHandle, evidence: MavlinkAckEvidence) -> None:
        if not handle._mavlink_ack.done():
            handle._mavlink_ack.set_result(evidence)

    def _queued_count(self) -> int:
        return self._normal_queue.qsize() + len(self._safety_queue)

    def _queue_internal_hold(self, now: float) -> None:
        existing = self._unfinished_safety_handle(FcuAction.HOLD)
        if existing is not None:
            self._last_internal_safety_handle = existing
            return
        strongest = self._strongest_outstanding_safety_action()
        if self._safety_latch is not None or (
            strongest is not None
            and self._safety_priority(strongest) > self._safety_priority(FcuAction.HOLD)
        ):
            return
        handle = self._new_handle(
            FcuAction.HOLD,
            now,
            detail="internal HOLD accepted after GUIDED target expiry",
        )
        self._last_internal_safety_handle = handle
        self._safety_queue.append(
            _QueuedCommand(
                handle=handle,
                command=FcuCommand(FcuAction.HOLD),
                expires_monotonic_s=None,
            )
        )

    def _unfinished_safety_handle(self, action: FcuAction) -> CommandHandle | None:
        pending = self._pending
        if pending is not None and pending.queued.command.action == action:
            if not pending.queued.handle._result.done():
                return pending.queued.handle
        for queued in self._safety_queue:
            if queued.command.action == action and not queued.handle._result.done():
                return queued.handle
        return None

    def _strongest_outstanding_safety_action(self) -> FcuAction | None:
        actions = [queued.command.action for queued in self._safety_queue]
        if self._pending is not None and self._is_safety(
            self._pending.queued.command.action
        ):
            actions.append(self._pending.queued.command.action)
        if not actions:
            return None
        return max(actions, key=self._safety_priority)

    @classmethod
    def _is_safety(cls, action: FcuAction) -> bool:
        return action in cls._SAFETY_PRIORITY

    @classmethod
    def _safety_priority(cls, action: FcuAction) -> int:
        return cls._SAFETY_PRIORITY.get(action, 0)

    @classmethod
    def _allowed_during_latch(
        cls,
        action: FcuAction,
        latched_action: FcuAction,
    ) -> bool:
        return cls._is_safety(action) and (
            action == latched_action
            or cls._safety_priority(action) > cls._safety_priority(latched_action)
        )

    def _ack_targets_this_link(self, message: MavlinkEnvelope) -> bool:
        if self._source_system is None or self._source_component is None:
            return True
        try:
            target_system = int(message.fields.get("target_system", 0))
            target_component = int(message.fields.get("target_component", 0))
        except (TypeError, ValueError):
            return False
        return target_system in {0, self._source_system} and target_component in {
            0,
            self._source_component,
        }

    def _reconcile_safety_latch(self, now: float) -> None:
        latch = self._safety_latch
        if latch is None:
            return
        if not self._physical_completion_reached(FcuCommand(latch.action), now):
            self._safety_latch_candidate_since_s = None
            return
        if self._safety_latch_candidate_since_s is None:
            self._safety_latch_candidate_since_s = now
        if now - self._safety_latch_candidate_since_s >= self._completion_hold_s:
            self._safety_latch = None
            self._safety_latch_candidate_since_s = None

    @staticmethod
    def _distance_3d(
        left: tuple[float, float, float],
        right: tuple[float, float, float],
    ) -> float:
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))
