"""Lightweight ROS-free vehicle agent for the authenticated ground channel."""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any
from uuid import UUID, uuid4

import websockets
from websockets.exceptions import ConnectionClosed

from aeromind_apm_lite.common.communication import (
    AuthenticationError,
    BoundedLatestQueue,
    ClientHello,
    ClientProof,
    ProtocolError,
    ServerChallenge,
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
    MavResult,
)


class OnboardAgentError(RuntimeError):
    pass


@dataclass(slots=True)
class _InboundCommand:
    received: ReceivedMessage[VehicleCommand]
    accepted_enqueued: asyncio.Event


class OnboardAgent:
    """Bridge signed vehicle commands to the one-owner :class:`ApmLink`.

    The agent creates a new session UUID on every connection. All outgoing
    ACK, heartbeat and telemetry messages share one strictly increasing
    sequence, preserving replay semantics across the WebSocket stream.
    """

    def __init__(
        self,
        *,
        ground_uri: str,
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
    ) -> None:
        if not ground_uri.startswith(("ws://", "wss://")):
            raise ValueError("ground_uri must use ws:// or wss://")
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

        self._runner: asyncio.Task[None] | None = None
        self._stop_event = asyncio.Event()
        self._connected_event = asyncio.Event()
        self._active_websocket: Any | None = None
        self._active_session_id: UUID | None = None
        self._next_sequence = 0
        self._outbound_lock = asyncio.Lock()
        self._last_command_message_id: UUID | None = None
        self._last_error: Exception | None = None

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
        if self._runner is not None:
            raise OnboardAgentError("agent can only be started once")
        self._runner = asyncio.create_task(
            self._run_forever(), name=f"onboard-agent-{self._vehicle_id}"
        )
        try:
            await asyncio.wait_for(self._connected_event.wait(), timeout_s)
        except asyncio.TimeoutError as exc:
            await self.stop()
            detail = f": {self._last_error}" if self._last_error is not None else ""
            raise OnboardAgentError(f"ground authentication timed out{detail}") from exc
        if self._active_session_id is None:
            raise OnboardAgentError("agent connected without an active session")
        return self._active_session_id

    async def stop(self) -> None:
        self._stop_event.set()
        websocket = self._active_websocket
        if websocket is not None:
            await websocket.close(code=1001, reason="onboard agent stopping")
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
                self._active_websocket = None
            if self._stop_event.is_set():
                break
            try:
                await asyncio.wait_for(
                    self._stop_event.wait(), timeout=self._reconnect_delay_s
                )
            except asyncio.TimeoutError:
                pass

    async def _run_session(self, session_id: UUID) -> None:
        async with websockets.connect(
            self._ground_uri,
            max_size=self._max_frame_bytes,
            ping_interval=10.0,
            ping_timeout=10.0,
        ) as websocket:
            await self._authenticate(websocket, session_id)
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
            self._active_websocket = websocket
            self._connected_event.set()

            tasks = {
                asyncio.create_task(self._sender(websocket, outbox)),
                asyncio.create_task(self._receiver(websocket, guard, commands, outbox)),
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

    async def _authenticate(self, websocket: Any, session_id: UUID) -> None:
        client_challenge = new_session_challenge()
        await websocket.send(
            encode_frame(
                ClientHello(
                    vehicle_id=self._vehicle_id,
                    session_id=session_id,
                    client_challenge=client_challenge,
                )
            )
        )
        challenge_raw = await asyncio.wait_for(
            websocket.recv(), timeout=self._handshake_timeout_s
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
        await websocket.send(
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
            websocket.recv(), timeout=self._handshake_timeout_s
        )
        accepted = decode_handshake_frame(accepted_raw)
        if not isinstance(accepted, SessionAccepted):
            raise ProtocolError("session_accepted must follow client_proof")
        if accepted.vehicle_id != self._vehicle_id or accepted.session_id != session_id:
            raise AuthenticationError("accepted session identity does not match")

    async def _sender(
        self, websocket: Any, outbox: BoundedLatestQueue[str]
    ) -> None:
        while True:
            payload = await outbox.get()
            await websocket.send(payload)

    async def _receiver(
        self,
        websocket: Any,
        guard: IngressGuard,
        commands: BoundedLatestQueue[_InboundCommand],
        outbox: BoundedLatestQueue[str],
    ) -> None:
        try:
            async for payload in websocket:
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
                await commands.put(inbound, replaceable=False)
                await self._emit_ack(
                    outbox,
                    message,
                    AckStatus.ACCEPTED,
                    "authenticated command accepted by the onboard ingress gate",
                )
                inbound.accepted_enqueued.set()
        except (ProtocolError, IngressError) as exc:
            await websocket.close(code=1008, reason=str(exc)[:120])
            raise
        except ConnectionClosed:
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
            for tracker in trackers:
                tracker.cancel()
            await asyncio.gather(*trackers, return_exceptions=True)

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
        action = {
            VehicleCommandType.ARM: FcuAction.ARM,
            VehicleCommandType.DISARM: FcuAction.DISARM,
            VehicleCommandType.TAKEOFF: FcuAction.TAKEOFF,
            VehicleCommandType.LAND: FcuAction.LAND,
            VehicleCommandType.RTL: FcuAction.RTL,
            VehicleCommandType.HOLD: FcuAction.HOLD,
            VehicleCommandType.CANCEL: FcuAction.HOLD,
        }[command.command]
        fcu_command = FcuCommand(
            action=action,
            target_altitude_m=(
                command.target_altitude_m
                if command.command == VehicleCommandType.TAKEOFF
                else None
            ),
        )
        return self._link.submit(
            fcu_command,
            expires_monotonic_s=expires_monotonic_s,
        )

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
                )
                await outbox.put(
                    encode_signed_message(heartbeat, self._secret), replaceable=True
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
                max(500, int(self._telemetry_interval_s * 3_000)),
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
