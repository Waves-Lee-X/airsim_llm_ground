"""Lightweight ROS-free vehicle agent for the authenticated ground channel."""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from collections.abc import Iterable
from typing import Any
from uuid import UUID, uuid4

from aeromind_apm_lite.common.communication import (
    AuthenticationError,
    BoundedLatestQueue,
    ClientHello,
    ClientProof,
    ProtocolError,
    ServerChallenge,
    SerialChannelClosed,
    SessionAccepted,
    authentication_proof,
    decode_handshake_frame,
    decode_signed_message,
    encode_frame,
    encode_signed_message,
    verify_authentication_proof,
)
from aeromind_apm_lite.common.contracts import (
    AckStatus,
    CoordinateFrame,
    Heartbeat,
    IngressError,
    IngressGuard,
    MissionAck,
    ReceivedMessage,
    VehicleCommand,
    VehicleCommandType,
    VehicleTelemetry,
    new_session_challenge,
)
from aeromind_apm_lite.common.contracts.models import Vector3, VehicleHealth

from .apm_link import ApmLink, CommandHandle
from .mavlink.models import (
    CommandResult,
    CommandStatus,
    FcuAction,
    FcuCommand,
    MavlinkAckEvidence,
    MavResult,
    PhysicalCompletionEvidence,
)


class OnboardAgentError(RuntimeError):
    pass


class _WebSocketConnectionClosed(Exception):
    """Fallback used when serial-only deployments omit websockets."""


@dataclass
class _InboundCommand:
    received: ReceivedMessage[VehicleCommand]
    accepted_enqueued: asyncio.Event


class OnboardAgent:
    """Bridge signed vehicle commands to the one-owner :class:`ApmLink`.

    The agent creates a new session UUID on every connection. All outgoing
    ACK, heartbeat and telemetry messages share one strictly increasing
    sequence, preserving replay semantics across either transport.
    """

    def __init__(
        self,
        *,
        ground_uri: str | None = None,
        channel_factory: Any | None = None,
        vehicle_id: int,
        shared_secret: bytes,
        frame_calibration_id: str,
        apm_link: ApmLink,
        software_version: str = "0.1.0",
        configuration_hash: str,
        heartbeat_interval_s: float = 1.0,
        telemetry_interval_s: float = 0.2,
        reconnect_delay_s: float = 0.5,
        handshake_timeout_s: float = 3.0,
        queue_size: int = 32,
        max_frame_bytes: int = 256 * 1024,
        command_output_enabled: bool = True,
        allowed_commands: Iterable[VehicleCommandType | str] | None = None,
    ) -> None:
        if (ground_uri is None) == (channel_factory is None):
            raise ValueError("provide exactly one of ground_uri or channel_factory")
        if ground_uri is not None and not ground_uri.startswith(("ws://", "wss://")):
            raise ValueError("ground_uri must use ws:// or wss://")
        if channel_factory is not None and not callable(channel_factory):
            raise ValueError("channel_factory must be callable")
        if not 1 <= vehicle_id <= 255:
            raise ValueError("vehicle_id must be in [1, 255]")
        if not isinstance(shared_secret, bytes) or len(shared_secret) < 32:
            raise ValueError("shared_secret must contain at least 32 bytes")
        if not frame_calibration_id or len(frame_calibration_id) > 128:
            raise ValueError("frame_calibration_id must contain 1 to 128 characters")
        if not software_version or len(software_version) > 64:
            raise ValueError("software_version must contain 1 to 64 characters")
        if not 8 <= len(configuration_hash) <= 128:
            raise ValueError("configuration_hash must contain 8 to 128 characters")
        for name, value in (
            ("heartbeat_interval_s", heartbeat_interval_s),
            ("telemetry_interval_s", telemetry_interval_s),
            ("reconnect_delay_s", reconnect_delay_s),
            ("handshake_timeout_s", handshake_timeout_s),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if queue_size < 1:
            raise ValueError("queue_size must be positive")
        if max_frame_bytes < 1024:
            raise ValueError("max_frame_bytes must be at least 1024")

        self._ground_uri = ground_uri
        self._channel_factory = channel_factory
        self._vehicle_id = vehicle_id
        self._secret = shared_secret
        self._calibration_id = frame_calibration_id
        self._link = apm_link
        self._software_version = software_version
        self._configuration_hash = configuration_hash
        self._heartbeat_interval_s = heartbeat_interval_s
        self._telemetry_interval_s = telemetry_interval_s
        self._reconnect_delay_s = reconnect_delay_s
        self._handshake_timeout_s = handshake_timeout_s
        self._queue_size = queue_size
        self._max_frame_bytes = max_frame_bytes
        self._command_output_enabled = command_output_enabled
        requested_commands = (
            frozenset(VehicleCommandType)
            if allowed_commands is None
            else frozenset(VehicleCommandType(command) for command in allowed_commands)
        )
        self._allowed_commands = (
            requested_commands if command_output_enabled else frozenset()
        )

        self._runner: asyncio.Task[None] | None = None
        self._stop_event = asyncio.Event()
        self._connected_event = asyncio.Event()
        self._active_channel: Any | None = None
        self._active_session_id: UUID | None = None
        self._next_sequence = 0
        self._outbound_lock = asyncio.Lock()
        self._last_command_message_id: UUID | None = None
        self._last_error: Exception | None = None
        self._takeoff_sequence_tasks: set[asyncio.Task[None]] = set()

    @property
    def connected(self) -> bool:
        return self._connected_event.is_set()

    @property
    def session_id(self) -> UUID | None:
        return self._active_session_id

    @property
    def last_error(self) -> Exception | None:
        return self._last_error

    async def start(self, timeout_s: float = 5.0) -> UUID:
        await self.start_background()
        try:
            await asyncio.wait_for(self._connected_event.wait(), timeout_s)
        except asyncio.TimeoutError as exc:
            await self.stop()
            detail = f": {self._last_error}" if self._last_error is not None else ""
            raise OnboardAgentError(f"ground authentication timed out{detail}") from exc
        if self._active_session_id is None:
            raise OnboardAgentError("agent connected without an active session")
        return self._active_session_id

    async def start_background(self) -> None:
        if self._runner is not None:
            raise OnboardAgentError("agent can only be started once")
        self._runner = asyncio.create_task(
            self._run_forever(), name=f"onboard-agent-{self._vehicle_id}"
        )

    async def stop(self) -> None:
        self._stop_event.set()
        channel = self._active_channel
        if channel is not None:
            await channel.close(code=1001, reason="onboard agent stopping")
        if self._runner is not None:
            await asyncio.gather(self._runner, return_exceptions=True)
        self._connected_event.clear()

    async def _run_forever(self) -> None:
        while not self._stop_event.is_set():
            session_id = uuid4()
            try:
                await self._run_session(session_id)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self._last_error = exc
            finally:
                self._connected_event.clear()
                if self._active_session_id == session_id:
                    self._active_session_id = None
                self._active_channel = None
            if self._stop_event.is_set():
                break
            try:
                await asyncio.wait_for(
                    self._stop_event.wait(), timeout=self._reconnect_delay_s
                )
            except asyncio.TimeoutError:
                pass

    async def _run_session(self, session_id: UUID) -> None:
        async with self._open_channel() as channel:
            await self._authenticate(channel, session_id)
            outbox: BoundedLatestQueue[str] = BoundedLatestQueue(self._queue_size)
            commands: BoundedLatestQueue[_InboundCommand] = BoundedLatestQueue(
                self._queue_size
            )
            guard = IngressGuard(
                expected_vehicle_id=self._vehicle_id,
                expected_session_id=session_id,
                expected_calibration_id=self._calibration_id,
                shared_secret=self._secret,
            )
            self._next_sequence = 0
            self._active_session_id = session_id
            self._active_channel = channel
            self._connected_event.set()

            tasks = {
                asyncio.create_task(self._sender(channel, outbox)),
                asyncio.create_task(self._receiver(channel, guard, commands, outbox)),
                asyncio.create_task(self._command_worker(commands, outbox)),
                asyncio.create_task(self._heartbeat_loop(outbox, session_id)),
                asyncio.create_task(self._telemetry_loop(outbox, session_id)),
            }
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
            await commands.close()
            await outbox.close()
            for task in done:
                if task.cancelled():
                    continue
                exception = task.exception()
                if exception is not None:
                    raise exception

    def _open_channel(self) -> Any:
        if self._channel_factory is not None:
            return self._channel_factory()
        import websockets

        return websockets.connect(
            self._ground_uri,
            max_size=self._max_frame_bytes,
            ping_interval=10.0,
            ping_timeout=10.0,
        )

    @staticmethod
    def _websocket_closed_error() -> type[Exception]:
        try:
            from websockets.exceptions import ConnectionClosed
        except ImportError:
            return _WebSocketConnectionClosed
        return ConnectionClosed

    async def _authenticate(self, channel: Any, session_id: UUID) -> None:
        client_challenge = new_session_challenge()
        await channel.send(
            encode_frame(
                ClientHello(
                    vehicle_id=self._vehicle_id,
                    session_id=session_id,
                    client_challenge=client_challenge,
                )
            )
        )
        challenge_raw = await asyncio.wait_for(
            channel.recv(), timeout=self._handshake_timeout_s
        )
        challenge = decode_handshake_frame(challenge_raw)
        if not isinstance(challenge, ServerChallenge):
            raise ProtocolError("server_challenge must follow client_hello")
        if (
            challenge.vehicle_id != self._vehicle_id
            or challenge.session_id != session_id
        ):
            raise AuthenticationError("server challenge identity does not match")
        if not verify_authentication_proof(
            challenge.server_proof,
            role="server",
            challenge=client_challenge,
            vehicle_id=self._vehicle_id,
            session_id=session_id,
            secret=self._secret,
        ):
            raise AuthenticationError("ground server authentication failed")
        await channel.send(
            encode_frame(
                ClientProof(
                    vehicle_id=self._vehicle_id,
                    session_id=session_id,
                    client_proof=authentication_proof(
                        role="client",
                        challenge=challenge.server_challenge,
                        vehicle_id=self._vehicle_id,
                        session_id=session_id,
                        secret=self._secret,
                    ),
                )
            )
        )
        accepted_raw = await asyncio.wait_for(
            channel.recv(), timeout=self._handshake_timeout_s
        )
        accepted = decode_handshake_frame(accepted_raw)
        if not isinstance(accepted, SessionAccepted):
            raise ProtocolError("session_accepted must follow client_proof")
        if accepted.vehicle_id != self._vehicle_id or accepted.session_id != session_id:
            raise AuthenticationError("accepted session identity does not match")

    async def _sender(
        self, channel: Any, outbox: BoundedLatestQueue[str]
    ) -> None:
        while True:
            payload = await outbox.get()
            await channel.send(payload)

    async def _receiver(
        self,
        channel: Any,
        guard: IngressGuard,
        commands: BoundedLatestQueue[_InboundCommand],
        outbox: BoundedLatestQueue[str],
    ) -> None:
        try:
            async for payload in channel:
                received_at = time.monotonic()
                message, signature = decode_signed_message(payload)
                if not isinstance(message, VehicleCommand):
                    raise ProtocolError(
                        f"ground cannot send {message.message_type!r} on the M1 command channel"
                    )
                received = guard.receive(
                    message,
                    signature=signature,
                    received_monotonic_s=received_at,
                )
                received.ensure_fresh()
                self._last_command_message_id = message.message_id
                inbound = _InboundCommand(
                    received=received,
                    accepted_enqueued=asyncio.Event(),
                )
                await self._emit_ack(
                    outbox,
                    message,
                    AckStatus.ACCEPTED,
                    "authenticated command accepted by the onboard ingress gate",
                )
                inbound.accepted_enqueued.set()
                if not self._command_output_enabled:
                    await self._emit_ack(
                        outbox,
                        message,
                        AckStatus.FAILED,
                        "authenticated command rejected: onboard command output is disabled",
                    )
                    continue
                if message.command not in self._allowed_commands:
                    await self._emit_ack(
                        outbox,
                        message,
                        AckStatus.FAILED,
                        (
                            "authenticated command rejected: "
                            f"{message.command.value} is not in the onboard allowlist"
                        ),
                    )
                    continue
                await commands.put(inbound, replaceable=False)
        except (ProtocolError, IngressError) as exc:
            await channel.close(code=1008, reason=str(exc)[:120])
            raise
        except (self._websocket_closed_error(), SerialChannelClosed):
            return

    async def _command_worker(
        self,
        commands: BoundedLatestQueue[_InboundCommand],
        outbox: BoundedLatestQueue[str],
    ) -> None:
        trackers: set[asyncio.Task[None]] = set()
        try:
            while True:
                inbound = await commands.get()
                received = inbound.received
                command = received.message
                try:
                    received.ensure_fresh()
                except IngressError as exc:
                    tracker = asyncio.create_task(
                        self._emit_after_acceptance(
                            inbound.accepted_enqueued,
                            outbox,
                            command,
                            AckStatus.EXPIRED,
                            exc.detail,
                        )
                    )
                else:
                    expires_at = (
                        received.received_monotonic_s + command.ttl_ms / 1_000.0
                    )
                    try:
                        handle = self._submit_to_apm(command, expires_at)
                    except (TypeError, ValueError, RuntimeError) as exc:
                        tracker = asyncio.create_task(
                            self._emit_after_acceptance(
                                inbound.accepted_enqueued,
                                outbox,
                                command,
                                AckStatus.FAILED,
                                f"APM command mapping failed: {exc}",
                            )
                        )
                    else:
                        tracker = asyncio.create_task(
                            self._track_command(
                                inbound.accepted_enqueued,
                                outbox,
                                command,
                                handle,
                            )
                        )
                trackers.add(tracker)
                tracker.add_done_callback(trackers.discard)
                tracker.add_done_callback(self._consume_tracker_result)
        finally:
            background_tasks = (*trackers, *self._takeoff_sequence_tasks)
            for task in background_tasks:
                task.cancel()
            await asyncio.gather(*background_tasks, return_exceptions=True)
            self._takeoff_sequence_tasks.clear()

    async def _track_command(
        self,
        accepted_enqueued: asyncio.Event,
        outbox: BoundedLatestQueue[str],
        command: VehicleCommand,
        handle: CommandHandle,
    ) -> None:
        await accepted_enqueued.wait()
        if not handle.application.accepted:
            result = await handle.result
            await self._emit_terminal_ack(outbox, command, result)
            return

        mavlink_ack = await handle.mavlink_ack
        ack_rejected = (
            mavlink_ack.received
            and mavlink_ack.result
            not in {int(MavResult.ACCEPTED), int(MavResult.IN_PROGRESS)}
        )
        if not ack_rejected:
            await self._emit_ack(
                outbox,
                command,
                AckStatus.RUNNING,
                mavlink_ack.detail,
                apm_command_result=mavlink_ack.result,
            )
        result = await handle.result
        await self._emit_terminal_ack(outbox, command, result)

    async def _emit_after_acceptance(
        self,
        accepted_enqueued: asyncio.Event,
        outbox: BoundedLatestQueue[str],
        command: VehicleCommand,
        status: AckStatus,
        detail: str,
    ) -> None:
        await accepted_enqueued.wait()
        await self._emit_ack(outbox, command, status, detail)

    @staticmethod
    def _consume_tracker_result(task: asyncio.Task[None]) -> None:
        if not task.cancelled():
            task.exception()

    def _submit_to_apm(
        self, command: VehicleCommand, expires_monotonic_s: float
    ) -> CommandHandle:
        if command.command == VehicleCommandType.TAKEOFF:
            altitude_m = command.target_altitude_m
            if altitude_m is None:  # pragma: no cover - contract validation
                raise ValueError("takeoff requires target_altitude_m")
            return self._submit_takeoff_sequence(altitude_m, expires_monotonic_s)

        action = {
            VehicleCommandType.ARM: FcuAction.ARM,
            VehicleCommandType.DISARM: FcuAction.DISARM,
            VehicleCommandType.LAND: FcuAction.LAND,
            VehicleCommandType.RTL: FcuAction.RTL,
            VehicleCommandType.HOLD: FcuAction.HOLD,
            VehicleCommandType.CANCEL: FcuAction.HOLD,
        }[command.command]
        fcu_command = FcuCommand(action=action)
        return self._link.submit(
            fcu_command,
            expires_monotonic_s=expires_monotonic_s,
        )

    def _submit_takeoff_sequence(
        self,
        altitude_m: float,
        expires_monotonic_s: float,
    ) -> CommandHandle:
        step_name, step_handle = self._next_takeoff_step(
            altitude_m,
            expires_monotonic_s,
        )
        loop = asyncio.get_running_loop()
        outer = CommandHandle(
            request_id=uuid4(),
            action=FcuAction.TAKEOFF,
            application=step_handle.application,
            _mavlink_ack=loop.create_future(),
            _physical_completion=loop.create_future(),
            _result=loop.create_future(),
        )
        task = asyncio.create_task(
            self._run_takeoff_sequence(
                outer,
                altitude_m,
                expires_monotonic_s,
                step_name,
                step_handle,
            ),
            name=f"takeoff-sequence-{outer.request_id}",
        )
        self._takeoff_sequence_tasks.add(task)
        task.add_done_callback(self._takeoff_sequence_tasks.discard)
        task.add_done_callback(self._consume_tracker_result)
        return outer

    def _next_takeoff_step(
        self,
        altitude_m: float,
        expires_monotonic_s: float,
    ) -> tuple[str, CommandHandle]:
        telemetry = self._link.telemetry_snapshot()
        guided_mode = self._link.guided_mode.upper()
        if (telemetry.mode or "").upper() != guided_mode:
            return (
                "set_mode_guided",
                self._link.submit(
                    FcuCommand(FcuAction.SET_MODE, mode=guided_mode),
                    expires_monotonic_s=expires_monotonic_s,
                ),
            )
        if telemetry.armed is not True:
            return (
                "arm",
                self._link.submit(
                    FcuCommand(FcuAction.ARM),
                    expires_monotonic_s=expires_monotonic_s,
                ),
            )
        return (
            "takeoff",
            self._link.submit(
                FcuCommand(FcuAction.TAKEOFF, target_altitude_m=altitude_m),
                expires_monotonic_s=expires_monotonic_s,
            ),
        )

    async def _run_takeoff_sequence(
        self,
        outer: CommandHandle,
        altitude_m: float,
        expires_monotonic_s: float,
        step_name: str,
        step_handle: CommandHandle,
    ) -> None:
        try:
            while True:
                if step_name == "takeoff":
                    ack = await step_handle.mavlink_ack
                    self._resolve_outer_ack(
                        outer,
                        self._step_ack("takeoff", ack),
                    )

                result = await step_handle.result
                step_completed = (
                    result.status == CommandStatus.COMPLETED
                    and result.physical_completion.confirmed
                )
                if not step_completed:
                    self._finish_takeoff_outer(outer, step_name, result)
                    return
                if step_name == "takeoff":
                    self._finish_takeoff_outer(outer, step_name, result)
                    return

                step_name, step_handle = self._next_takeoff_step(
                    altitude_m,
                    expires_monotonic_s,
                )
        except asyncio.CancelledError:
            self._cancel_takeoff_outer(outer, step_name)
            raise
        except Exception as exc:
            self._fail_takeoff_outer_locally(outer, step_name, exc)

    def _finish_takeoff_outer(
        self,
        outer: CommandHandle,
        step_name: str,
        result: CommandResult,
    ) -> None:
        is_success = (
            step_name == "takeoff"
            and result.status == CommandStatus.COMPLETED
            and result.physical_completion.confirmed
        )
        ack = (
            outer._mavlink_ack.result()
            if outer._mavlink_ack.done()
            else self._step_ack(step_name, result.mavlink_ack)
        )
        physical = (
            result.physical_completion
            if is_success
            else self._step_physical(step_name, result.physical_completion)
        )
        detail = (
            result.detail
            if is_success
            else f"takeoff sequence failed at {step_name}: {result.detail}"
        )
        status = (
            CommandStatus.PHYSICAL_TIMEOUT
            if result.status == CommandStatus.COMPLETED and not physical.confirmed
            else result.status
        )
        self._resolve_outer_ack(outer, ack)
        if not outer._physical_completion.done():
            outer._physical_completion.set_result(physical)
        if not outer._result.done():
            outer._result.set_result(
                CommandResult(
                    request_id=outer.request_id,
                    action=FcuAction.TAKEOFF,
                    status=status,
                    application=outer.application,
                    sent_monotonic_s=result.sent_monotonic_s,
                    mavlink_ack=ack,
                    physical_completion=physical,
                    detail=detail,
                )
            )

    def _fail_takeoff_outer_locally(
        self,
        outer: CommandHandle,
        step_name: str,
        exc: Exception,
    ) -> None:
        detail = (
            f"takeoff sequence failed at {step_name}: "
            f"{type(exc).__name__}: {exc}"
        )
        ack = MavlinkAckEvidence(
            applicable=False,
            received=False,
            command_id=None,
            result=None,
            observed_monotonic_s=time.monotonic(),
            detail=detail,
        )
        physical = PhysicalCompletionEvidence(False, None, detail)
        self._resolve_outer_ack(outer, ack)
        if not outer._physical_completion.done():
            outer._physical_completion.set_result(physical)
        if not outer._result.done():
            outer._result.set_result(
                CommandResult(
                    request_id=outer.request_id,
                    action=FcuAction.TAKEOFF,
                    status=CommandStatus.DISPATCH_FAILED,
                    application=outer.application,
                    sent_monotonic_s=None,
                    mavlink_ack=outer._mavlink_ack.result(),
                    physical_completion=physical,
                    detail=detail,
                )
            )

    def _cancel_takeoff_outer(self, outer: CommandHandle, step_name: str) -> None:
        detail = f"takeoff sequence cancelled during {step_name}"
        ack = MavlinkAckEvidence(
            applicable=False,
            received=False,
            command_id=None,
            result=None,
            observed_monotonic_s=time.monotonic(),
            detail=detail,
        )
        physical = PhysicalCompletionEvidence(False, None, detail)
        self._resolve_outer_ack(outer, ack)
        if not outer._physical_completion.done():
            outer._physical_completion.set_result(physical)
        if not outer._result.done():
            outer._result.set_result(
                CommandResult(
                    request_id=outer.request_id,
                    action=FcuAction.TAKEOFF,
                    status=CommandStatus.PREEMPTED,
                    application=outer.application,
                    sent_monotonic_s=None,
                    mavlink_ack=outer._mavlink_ack.result(),
                    physical_completion=physical,
                    detail=detail,
                )
            )

    @staticmethod
    def _step_ack(step_name: str, ack: MavlinkAckEvidence) -> MavlinkAckEvidence:
        return MavlinkAckEvidence(
            applicable=ack.applicable,
            received=ack.received,
            command_id=ack.command_id,
            result=ack.result,
            observed_monotonic_s=ack.observed_monotonic_s,
            detail=f"{step_name}: {ack.detail}",
        )

    @staticmethod
    def _step_physical(
        step_name: str,
        physical: PhysicalCompletionEvidence,
    ) -> PhysicalCompletionEvidence:
        return PhysicalCompletionEvidence(
            confirmed=physical.confirmed,
            observed_monotonic_s=physical.observed_monotonic_s,
            detail=f"{step_name}: {physical.detail}",
        )

    @staticmethod
    def _resolve_outer_ack(
        outer: CommandHandle,
        ack: MavlinkAckEvidence,
    ) -> None:
        if not outer._mavlink_ack.done():
            outer._mavlink_ack.set_result(ack)

    async def _emit_terminal_ack(
        self,
        outbox: BoundedLatestQueue[str],
        command: VehicleCommand,
        result: CommandResult,
    ) -> None:
        physical_confirmed = result.physical_completion.confirmed
        if result.status == CommandStatus.COMPLETED and not physical_confirmed:
            status = AckStatus.FAILED
            detail = "ApmLink reported completion without physical evidence"
        elif result.status == CommandStatus.COMPLETED:
            status = (
                AckStatus.CANCELLED
                if command.command == VehicleCommandType.CANCEL
                else AckStatus.COMPLETED
            )
            detail = result.detail
        elif result.status in {
            CommandStatus.EXPIRED_BEFORE_SEND,
            CommandStatus.EXPIRED_DURING_EXECUTION,
        }:
            status = AckStatus.EXPIRED
            detail = result.detail
        elif result.status == CommandStatus.PREEMPTED:
            status = AckStatus.CANCELLED
            detail = result.detail
        else:
            status = AckStatus.FAILED
            detail = result.detail
        await self._emit_ack(
            outbox,
            command,
            status,
            detail,
            apm_command_result=result.mavlink_ack.result,
            physical_completion_confirmed=physical_confirmed,
        )

    async def _emit_ack(
        self,
        outbox: BoundedLatestQueue[str],
        command: VehicleCommand,
        status: AckStatus,
        detail: str,
        *,
        apm_command_result: int | None = None,
        physical_completion_confirmed: bool = False,
    ) -> None:
        session_id = self._active_session_id
        if session_id is None:
            raise OnboardAgentError("cannot emit ACK without an active session")
        async with self._outbound_lock:
            ack = MissionAck(
                mission_id=command.mission_id,
                vehicle_id=self._vehicle_id,
                sequence=self._take_sequence(),
                session_id=session_id,
                ttl_ms=max(command.ttl_ms, 1_000),
                acknowledged_message_id=command.message_id,
                status=status,
                detail=detail[:512],
                apm_command_result=apm_command_result,
                physical_completion_confirmed=physical_completion_confirmed,
            )
            await outbox.put(
                encode_signed_message(ack, self._secret), replaceable=False
            )

    async def _heartbeat_loop(
        self, outbox: BoundedLatestQueue[str], session_id: UUID
    ) -> None:
        while True:
            async with self._outbound_lock:
                heartbeat = Heartbeat(
                    vehicle_id=self._vehicle_id,
                    sequence=self._take_sequence(),
                    session_id=session_id,
                    ttl_ms=min(
                        300_000,
                        max(1_000, int(self._heartbeat_interval_s * 3_000)),
                    ),
                    software_version=self._software_version,
                    configuration_hash=self._configuration_hash,
                    last_command_message_id=self._last_command_message_id,
                    command_output_enabled=self._command_output_enabled,
                    allowed_commands=tuple(
                        sorted(self._allowed_commands, key=lambda command: command.value)
                    ),
                )
                await outbox.put(
                    encode_signed_message(heartbeat, self._secret),
                    replaceable=True,
                    replacement_key="heartbeat",
                )
            await asyncio.sleep(self._heartbeat_interval_s)

    async def _telemetry_loop(
        self, outbox: BoundedLatestQueue[str], session_id: UUID
    ) -> None:
        while True:
            async with self._outbound_lock:
                telemetry = self._build_telemetry(session_id)
                if telemetry is not None:
                    await outbox.put(
                        encode_signed_message(telemetry, self._secret),
                        replaceable=True,
                        replacement_key="telemetry",
                    )
            await asyncio.sleep(self._telemetry_interval_s)

    def _build_telemetry(self, session_id: UUID) -> VehicleTelemetry | None:
        snapshot = self._link.telemetry_snapshot()
        if snapshot.armed is None or snapshot.mode is None:
            return None
        position = (
            Vector3(
                x=snapshot.local_position_ned_m[0],
                y=snapshot.local_position_ned_m[1],
                z=snapshot.local_position_ned_m[2],
            )
            if snapshot.local_position_ned_m is not None
            else None
        )
        velocity = (
            Vector3(
                x=snapshot.velocity_ned_m_s[0],
                y=snapshot.velocity_ned_m_s[1],
                z=snapshot.velocity_ned_m_s[2],
            )
            if snapshot.velocity_ned_m_s is not None
            else None
        )
        has_geometry = position is not None or velocity is not None
        return VehicleTelemetry(
            vehicle_id=self._vehicle_id,
            sequence=self._take_sequence(),
            session_id=session_id,
            ttl_ms=min(
                300_000,
                max(15_000, int(self._telemetry_interval_s * 3_000)),
            ),
            frame=(CoordinateFrame.LOCAL_NED if has_geometry else CoordinateFrame.NONE),
            frame_calibration_id=self._calibration_id if has_geometry else None,
            observed_at_utc=datetime.now(timezone.utc),
            armed=snapshot.armed,
            mode=snapshot.mode,
            position_m=position,
            velocity_m_s=velocity,
            battery_remaining=snapshot.battery_remaining,
            health=VehicleHealth(
                fcu_link_ok=snapshot.fcu_link_ok,
                gps_fix_type=snapshot.gps_fix_type or 0,
                gps_healthy=snapshot.gps_healthy,
                prearm_ok=snapshot.prearm_ok,
            ),
        )

    def _take_sequence(self) -> int:
        self._next_sequence += 1
        return self._next_sequence
