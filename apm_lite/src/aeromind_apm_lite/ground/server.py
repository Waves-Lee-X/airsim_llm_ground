"""Ground-side authenticated WebSocket gateway for one or more vehicles."""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field
from typing import Any
from uuid import UUID

import websockets
from websockets.exceptions import ConnectionClosed

from aeromind_apm_lite.common.communication import (
    AuthenticationError,
    BoundedLatestQueue,
    ClientHello,
    ClientProof,
    ProtocolError,
    QueueClosed,
    ServerChallenge,
    SessionAccepted,
    authentication_proof,
    decode_handshake_frame,
    decode_signed_message,
    encode_frame,
    encode_signed_message,
    verify_authentication_proof,
)
from aeromind_apm_lite.common.communication.serial_transport import (
    SerialChannelClosed,
)
from aeromind_apm_lite.common.contracts import (
    AckStatus,
    Heartbeat,
    IngressError,
    IngressGuard,
    MissionAck,
    ReceivedMessage,
    VehicleCommand,
    VehicleCommandType,
    VehicleTelemetry,
    WireMessage,
    new_session_challenge,
)


class GroundServerError(RuntimeError):
    pass


class VehicleNotConnected(GroundServerError):
    pass


@dataclass(frozen=True)
class VehicleSessionInfo:
    vehicle_id: int
    session_id: UUID
    connected_monotonic_s: float


@dataclass
class _VehicleConnection:
    websocket: Any
    vehicle_id: int
    session_id: UUID
    secret: bytes
    calibration_id: str | None
    connected_monotonic_s: float
    outbox: BoundedLatestQueue[str]
    events: BoundedLatestQueue[ReceivedMessage[WireMessage]]
    ingress: IngressGuard
    next_ground_sequence: int = 0
    last_heartbeat: ReceivedMessage[Heartbeat] | None = None
    latest_telemetry: ReceivedMessage[VehicleTelemetry] | None = None
    acknowledgements: dict[
        UUID, list[ReceivedMessage[MissionAck]]
    ] = field(default_factory=dict)
    ack_history_size: int = 2_048
    ack_condition: asyncio.Condition = field(default_factory=asyncio.Condition)
    send_lock: asyncio.Lock = field(default_factory=asyncio.Lock)

    def next_sequence(self) -> int:
        self.next_ground_sequence += 1
        return self.next_ground_sequence


class GroundServer:
    """Terminate vehicle sessions and expose typed commands and state.

    Each successful connection consumes a unique session UUID. Reconnecting
    with an old UUID is rejected even after the previous socket is gone.
    """

    def __init__(
        self,
        *,
        vehicle_secrets: dict[int, bytes],
        calibration_ids: dict[int, str | None] | None = None,
        host: str = "127.0.0.1",
        port: int = 0,
        queue_size: int = 32,
        ack_history_size: int = 2_048,
        handshake_timeout_s: float = 3.0,
        max_frame_bytes: int = 256 * 1024,
    ) -> None:
        if not vehicle_secrets:
            raise ValueError("at least one vehicle secret is required")
        if queue_size < 1:
            raise ValueError("queue_size must be positive")
        if ack_history_size < 1:
            raise ValueError("ack_history_size must be positive")
        if handshake_timeout_s <= 0.0:
            raise ValueError("handshake_timeout_s must be positive")
        if max_frame_bytes < 1024:
            raise ValueError("max_frame_bytes must be at least 1024")

        normalized: dict[int, bytes] = {}
        for vehicle_id, secret in vehicle_secrets.items():
            if not 1 <= vehicle_id <= 255:
                raise ValueError("vehicle IDs must be in [1, 255]")
            if not isinstance(secret, bytes) or len(secret) < 32:
                raise ValueError("each vehicle secret must contain at least 32 bytes")
            normalized[vehicle_id] = secret
        if len(set(normalized.values())) != len(normalized):
            raise ValueError("each vehicle must use an independent secret")

        calibrations = dict(calibration_ids or {})
        unknown_calibrations = set(calibrations) - set(normalized)
        if unknown_calibrations:
            raise ValueError("calibration IDs contain unregistered vehicles")

        self._vehicle_secrets = normalized
        self._calibration_ids = calibrations
        self._host = host
        self._port = port
        self._queue_size = queue_size
        self._ack_history_size = ack_history_size
        self._handshake_timeout_s = handshake_timeout_s
        self._max_frame_bytes = max_frame_bytes
        self._server: Any | None = None
        self._bound_port: int | None = None
        self._connections: dict[int, _VehicleConnection] = {}
        self._seen_sessions: dict[int, set[UUID]] = {
            vehicle_id: set() for vehicle_id in normalized
        }
        self._pending_sessions: set[tuple[int, UUID]] = set()
        self._session_lock = asyncio.Lock()
        self._connection_condition = asyncio.Condition()

    @property
    def bound_port(self) -> int:
        if self._bound_port is None:
            raise GroundServerError("server is not running")
        return self._bound_port

    @property
    def uri(self) -> str:
        return f"ws://{self._host}:{self.bound_port}"

    async def start(self) -> None:
        if self._server is not None:
            raise GroundServerError("server is already running")
        self._server = await websockets.serve(
            self._handle_connection,
            self._host,
            self._port,
            max_size=self._max_frame_bytes,
            ping_interval=10.0,
            ping_timeout=10.0,
        )
        sockets = self._server.sockets
        if not sockets:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
            raise GroundServerError("WebSocket server opened without a listening socket")
        self._bound_port = int(sockets[0].getsockname()[1])

    async def stop(self) -> None:
        server = self._server
        if server is None:
            return
        self._server = None
        connections = list(self._connections.values())
        await asyncio.gather(
            *(
                connection.websocket.close(code=1001, reason="ground server stopping")
                for connection in connections
            ),
            return_exceptions=True,
        )
        server.close()
        await server.wait_closed()
        self._connections.clear()
        self._bound_port = None
        async with self._connection_condition:
            self._connection_condition.notify_all()

    def session_info(self, vehicle_id: int) -> VehicleSessionInfo | None:
        connection = self._connections.get(vehicle_id)
        if connection is None:
            return None
        return VehicleSessionInfo(
            vehicle_id=vehicle_id,
            session_id=connection.session_id,
            connected_monotonic_s=connection.connected_monotonic_s,
        )

    async def wait_for_vehicle(self, vehicle_id: int, timeout_s: float = 5.0) -> UUID:
        if vehicle_id not in self._vehicle_secrets:
            raise KeyError(f"vehicle {vehicle_id} isn't registered")
        deadline = asyncio.get_running_loop().time() + timeout_s
        async with self._connection_condition:
            while vehicle_id not in self._connections:
                remaining = deadline - asyncio.get_running_loop().time()
                if remaining <= 0.0:
                    raise TimeoutError(f"vehicle {vehicle_id} did not connect")
                await asyncio.wait_for(self._connection_condition.wait(), remaining)
        return self._connections[vehicle_id].session_id

    async def send_command(
        self,
        vehicle_id: int,
        command: VehicleCommandType,
        *,
        mission_id: UUID | None = None,
        target_altitude_m: float | None = None,
        reason: str = "",
        ttl_ms: int = 5_000,
    ) -> VehicleCommand:
        connection = self._require_connection(vehicle_id)
        async with connection.send_lock:
            message = VehicleCommand(
                mission_id=mission_id,
                vehicle_id=vehicle_id,
                sequence=connection.next_sequence(),
                session_id=connection.session_id,
                ttl_ms=ttl_ms,
                command=command,
                target_altitude_m=target_altitude_m,
                reason=reason,
            )
            try:
                await connection.outbox.put(
                    encode_signed_message(message, connection.secret),
                    replaceable=False,
                )
            except QueueClosed as exc:
                raise VehicleNotConnected(f"vehicle {vehicle_id} disconnected") from exc
        return message

    async def next_message(self, vehicle_id: int, timeout_s: float = 5.0) -> WireMessage:
        connection = self._require_connection(vehicle_id)
        deadline = asyncio.get_running_loop().time() + timeout_s
        while True:
            remaining = deadline - asyncio.get_running_loop().time()
            if remaining <= 0.0:
                raise TimeoutError(f"no fresh message from vehicle {vehicle_id}")
            try:
                received = await asyncio.wait_for(connection.events.get(), remaining)
                received.ensure_fresh()
            except QueueClosed as exc:
                raise VehicleNotConnected(f"vehicle {vehicle_id} disconnected") from exc
            except ValueError:
                continue
            return received.message

    async def wait_for_ack(
        self,
        vehicle_id: int,
        command_message_id: UUID,
        status: AckStatus,
        timeout_s: float = 5.0,
    ) -> MissionAck:
        connection = self._require_connection(vehicle_id)
        deadline = asyncio.get_running_loop().time() + timeout_s
        async with connection.ack_condition:
            while True:
                for ack in connection.acknowledgements.get(command_message_id, []):
                    try:
                        ack.ensure_fresh()
                    except IngressError:
                        continue
                    if ack.message.status == status:
                        return ack.message
                remaining = deadline - asyncio.get_running_loop().time()
                if remaining <= 0.0:
                    raise TimeoutError(
                        f"no {status.value} ACK for command {command_message_id}"
                    )
                await asyncio.wait_for(connection.ack_condition.wait(), remaining)

    def latest_telemetry(self, vehicle_id: int) -> VehicleTelemetry | None:
        connection = self._require_connection(vehicle_id)
        received = connection.latest_telemetry
        if received is None:
            return None
        try:
            received.ensure_fresh()
        except IngressError:
            connection.latest_telemetry = None
            return None
        return received.message

    def last_heartbeat(self, vehicle_id: int) -> Heartbeat | None:
        connection = self._require_connection(vehicle_id)
        received = connection.last_heartbeat
        if received is None:
            return None
        try:
            received.ensure_fresh()
        except IngressError:
            connection.last_heartbeat = None
            return None
        return received.message

    def _require_connection(self, vehicle_id: int) -> _VehicleConnection:
        connection = self._connections.get(vehicle_id)
        if connection is None:
            raise VehicleNotConnected(f"vehicle {vehicle_id} is not connected")
        return connection

    async def _handle_connection(self, websocket: Any, *_: Any) -> None:
        connection: _VehicleConnection | None = None
        reservation: tuple[int, UUID] | None = None
        try:
            hello_raw = await asyncio.wait_for(
                websocket.recv(), timeout=self._handshake_timeout_s
            )
            hello = decode_handshake_frame(hello_raw)
            if not isinstance(hello, ClientHello):
                raise ProtocolError("the first frame must be client_hello")
            reservation = (hello.vehicle_id, hello.session_id)
            secret = self._vehicle_secrets.get(hello.vehicle_id)
            if secret is None:
                raise AuthenticationError("vehicle authentication failed")
            await self._reserve_session(*reservation)

            server_challenge = new_session_challenge()
            await websocket.send(
                encode_frame(
                    ServerChallenge(
                        vehicle_id=hello.vehicle_id,
                        session_id=hello.session_id,
                        server_challenge=server_challenge,
                        server_proof=authentication_proof(
                            role="server",
                            challenge=hello.client_challenge,
                            vehicle_id=hello.vehicle_id,
                            session_id=hello.session_id,
                            secret=secret,
                        ),
                    )
                )
            )
            proof_raw = await asyncio.wait_for(
                websocket.recv(), timeout=self._handshake_timeout_s
            )
            proof = decode_handshake_frame(proof_raw)
            if not isinstance(proof, ClientProof):
                raise ProtocolError("client_proof must follow server_challenge")
            if proof.vehicle_id != hello.vehicle_id or proof.session_id != hello.session_id:
                raise AuthenticationError("client proof identity changed during handshake")
            if not verify_authentication_proof(
                proof.client_proof,
                role="client",
                challenge=server_challenge,
                vehicle_id=hello.vehicle_id,
                session_id=hello.session_id,
                secret=secret,
            ):
                raise AuthenticationError("vehicle authentication failed")
            await self._activate_reserved_session(*reservation)
            reservation = None
            await websocket.send(
                encode_frame(
                    SessionAccepted(
                        vehicle_id=hello.vehicle_id,
                        session_id=hello.session_id,
                    )
                )
            )

            connection = _VehicleConnection(
                websocket=websocket,
                vehicle_id=hello.vehicle_id,
                session_id=hello.session_id,
                secret=secret,
                calibration_id=self._calibration_ids.get(hello.vehicle_id),
                connected_monotonic_s=time.monotonic(),
                outbox=BoundedLatestQueue(self._queue_size),
                events=BoundedLatestQueue(self._queue_size),
                ingress=IngressGuard(
                    expected_vehicle_id=hello.vehicle_id,
                    expected_session_id=hello.session_id,
                    expected_calibration_id=self._calibration_ids.get(hello.vehicle_id),
                    shared_secret=secret,
                ),
                ack_history_size=self._ack_history_size,
            )
            await self._register_connection(connection)
            receiver = asyncio.create_task(self._receive_messages(connection))
            sender = asyncio.create_task(self._send_messages(connection))
            done, pending = await asyncio.wait(
                {receiver, sender}, return_when=asyncio.FIRST_COMPLETED
            )
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
            for task in done:
                exception = task.exception()
                if exception is not None:
                    raise exception
        except (AuthenticationError, ProtocolError, ValueError) as exc:
            await websocket.close(code=1008, reason=str(exc)[:120])
        except (ConnectionClosed, SerialChannelClosed, asyncio.TimeoutError):
            pass
        finally:
            if reservation is not None:
                await self._release_reservation(*reservation)
            if connection is not None:
                await self._unregister_connection(connection)

    async def _receive_messages(self, connection: _VehicleConnection) -> None:
        async for payload in connection.websocket:
            received_at = time.monotonic()
            message, signature = decode_signed_message(payload)
            if not isinstance(message, (Heartbeat, MissionAck, VehicleTelemetry)):
                raise ProtocolError(
                    f"vehicle cannot send {message.message_type!r} on the M1 state channel"
                )
            received = connection.ingress.receive(
                message,
                signature=signature,
                received_monotonic_s=received_at,
            )
            received.ensure_fresh()
            if isinstance(message, Heartbeat):
                connection.last_heartbeat = received
            elif isinstance(message, VehicleTelemetry):
                connection.latest_telemetry = received
            else:
                async with connection.ack_condition:
                    if (
                        message.acknowledged_message_id
                        not in connection.acknowledgements
                        and len(connection.acknowledgements)
                        >= connection.ack_history_size
                    ):
                        oldest_message_id = next(iter(connection.acknowledgements))
                        connection.acknowledgements.pop(oldest_message_id)
                    connection.acknowledgements.setdefault(
                        message.acknowledged_message_id, []
                    ).append(received)
                    connection.ack_condition.notify_all()
            await connection.events.put(
                received,
                # Canonical state is retained above (latest state or ACK history).
                # This notification stream is best-effort and must never stall
                # the authenticated socket when its consumer is slow.
                replaceable=True,
            )

    async def _send_messages(self, connection: _VehicleConnection) -> None:
        while True:
            payload = await connection.outbox.get()
            await connection.websocket.send(payload)

    async def _reserve_session(self, vehicle_id: int, session_id: UUID) -> None:
        key = (vehicle_id, session_id)
        async with self._session_lock:
            if session_id in self._seen_sessions[vehicle_id] or key in self._pending_sessions:
                raise AuthenticationError("session is old or already in use")
            self._pending_sessions.add(key)

    async def _activate_reserved_session(self, vehicle_id: int, session_id: UUID) -> None:
        key = (vehicle_id, session_id)
        async with self._session_lock:
            if key not in self._pending_sessions:
                raise AuthenticationError("session reservation was lost")
            self._pending_sessions.remove(key)
            self._seen_sessions[vehicle_id].add(session_id)

    async def _release_reservation(self, vehicle_id: int, session_id: UUID) -> None:
        async with self._session_lock:
            self._pending_sessions.discard((vehicle_id, session_id))

    async def _register_connection(self, connection: _VehicleConnection) -> None:
        previous = self._connections.get(connection.vehicle_id)
        self._connections[connection.vehicle_id] = connection
        if previous is not None:
            await previous.websocket.close(code=1008, reason="superseded by a new session")
        async with self._connection_condition:
            self._connection_condition.notify_all()

    async def _unregister_connection(self, connection: _VehicleConnection) -> None:
        await connection.outbox.close()
        await connection.events.close()
        current = self._connections.get(connection.vehicle_id)
        if current is connection:
            self._connections.pop(connection.vehicle_id, None)
        async with connection.ack_condition:
            connection.ack_condition.notify_all()
        async with self._connection_condition:
            self._connection_condition.notify_all()
