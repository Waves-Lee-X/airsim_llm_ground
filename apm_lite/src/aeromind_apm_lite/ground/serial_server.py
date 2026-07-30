"""Ground-side Lite session multiplexer for a P9 transparent serial radio."""

from __future__ import annotations

import asyncio
import json
import logging
import time
from collections.abc import AsyncIterator
from typing import Any
from uuid import UUID

from aeromind_apm_lite.common.communication.protocol import (
    ClientHello,
    ProtocolError,
    decode_handshake_frame,
)
from aeromind_apm_lite.common.communication.serial_transport import (
    ByteStreamFactory,
    ReliableSerialLink,
    SerialChannelClosed,
    SerialDirection,
    SerialTransportError,
    payload_requires_delivery,
    pyserial_stream_factory,
)

from .server import GroundServer, GroundServerError


LOGGER = logging.getLogger("aeromind_apm_lite.ground.serial")


class _GroundSerialChannel:
    def __init__(
        self,
        link: ReliableSerialLink,
        *,
        vehicle_id: int,
        session_id: UUID,
    ) -> None:
        self._link = link
        self.vehicle_id = vehicle_id
        self.session_id = session_id
        self.last_received_monotonic_s = time.monotonic()
        self._incoming: asyncio.Queue[str | None] = asyncio.Queue(maxsize=64)
        self._closed = False

    async def feed(self, payload: str) -> None:
        if self._closed:
            return
        self.last_received_monotonic_s = time.monotonic()
        await self._incoming.put(payload)

    async def send(self, payload: str | bytes) -> None:
        if self._closed:
            raise SerialChannelClosed("vehicle serial session is closed")
        await self._link.send(
            self.vehicle_id,
            payload,
            reliable=payload_requires_delivery(payload),
        )

    async def recv(self) -> str:
        payload = await self._incoming.get()
        if payload is None:
            raise SerialChannelClosed("vehicle serial session is closed")
        return payload

    def __aiter__(self) -> AsyncIterator[str]:
        return self

    async def __anext__(self) -> str:
        try:
            return await self.recv()
        except SerialChannelClosed as exc:
            raise StopAsyncIteration from exc

    async def close(self, code: int = 1000, reason: str = "") -> None:
        del code, reason
        if self._closed:
            return
        self._closed = True
        if self._incoming.full():
            self._incoming.get_nowait()
        self._incoming.put_nowait(None)


class SerialGroundServer(GroundServer):
    """Expose the :class:`GroundServer` API over one multiplexed serial link."""

    def __init__(
        self,
        *,
        serial_port: str,
        serial_baudrate: int = 57_600,
        stream_factory: ByteStreamFactory | None = None,
        serial_ack_timeout_s: float = 0.4,
        serial_max_retries: int = 4,
        session_idle_timeout_s: float = 6.0,
        **ground_options: Any,
    ) -> None:
        if not serial_port:
            raise ValueError("serial_port must not be empty")
        if serial_baudrate <= 0:
            raise ValueError("serial_baudrate must be positive")
        if session_idle_timeout_s <= 0.0:
            raise ValueError("session_idle_timeout_s must be positive")
        super().__init__(**ground_options)
        self.serial_port = serial_port
        self.serial_baudrate = serial_baudrate
        self._stream_factory = stream_factory
        self._serial_ack_timeout_s = serial_ack_timeout_s
        self._serial_max_retries = serial_max_retries
        self._session_idle_timeout_s = session_idle_timeout_s
        self._serial_link: ReliableSerialLink | None = None
        self._router_task: asyncio.Task[None] | None = None
        self._channels: dict[tuple[int, UUID], _GroundSerialChannel] = {}
        self._channel_tasks: set[asyncio.Task[None]] = set()
        self._invalid_payloads = 0
        self._session_reset_requests = 0
        self._last_session_reset: dict[tuple[int, UUID], float] = {}

    @property
    def uri(self) -> str:
        return f"serial://{self.serial_port}?baud={self.serial_baudrate}"

    @property
    def invalid_payloads(self) -> int:
        return self._invalid_payloads

    @property
    def serial_stats(self) -> dict[str, int]:
        link = self._serial_link
        if link is None:
            return {}
        stats = link.stats
        return {
            "received_frames": stats.received_frames,
            "sent_frames": stats.sent_frames,
            "duplicate_frames": stats.duplicate_frames,
            "discarded_bytes": stats.discarded_bytes,
            "crc_errors": stats.crc_errors,
            "retries": stats.retries,
            "invalid_payloads": self._invalid_payloads,
            "session_reset_requests": self._session_reset_requests,
        }

    async def start(self) -> None:
        if self._serial_link is not None or self._router_task is not None:
            raise GroundServerError("serial ground server is already running")
        if self._stream_factory is None:
            stream = await pyserial_stream_factory(
                self.serial_port, self.serial_baudrate
            )
        else:
            stream = await self._stream_factory()
        link = ReliableSerialLink(
            stream,
            send_direction=SerialDirection.GROUND_TO_ONBOARD,
            ack_timeout_s=self._serial_ack_timeout_s,
            max_retries=self._serial_max_retries,
            accepted_vehicle_ids=set(self._vehicle_secrets),
        )
        try:
            await link.start()
        except BaseException:
            await stream.close()
            raise
        self._serial_link = link
        self._router_task = asyncio.create_task(
            self._route_frames(), name="serial-ground-router"
        )

    async def stop(self) -> None:
        router = self._router_task
        self._router_task = None
        if router is not None:
            router.cancel()
            await asyncio.gather(router, return_exceptions=True)
        channels = tuple(self._channels.values())
        await asyncio.gather(
            *(channel.close(reason="ground serial server stopping") for channel in channels),
            return_exceptions=True,
        )
        tasks = tuple(self._channel_tasks)
        await asyncio.gather(*tasks, return_exceptions=True)
        self._channel_tasks.clear()
        self._channels.clear()
        link = self._serial_link
        self._serial_link = None
        if link is not None:
            await link.close()
        self._connections.clear()
        async with self._connection_condition:
            self._connection_condition.notify_all()

    async def _route_frames(self) -> None:
        link = self._require_serial_link()
        next_idle_check = time.monotonic() + 1.0
        try:
            while True:
                remaining = max(0.01, next_idle_check - time.monotonic())
                try:
                    frame = await asyncio.wait_for(link.receive(), timeout=remaining)
                except asyncio.TimeoutError:
                    await self._expire_idle_channels()
                    next_idle_check = time.monotonic() + 1.0
                    continue
                try:
                    payload = frame.payload.decode("utf-8")
                    vehicle_id, session_id, is_hello = self._payload_identity(payload)
                except (UnicodeDecodeError, ValueError, ProtocolError):
                    self._invalid_payloads += 1
                    continue
                if frame.vehicle_id != vehicle_id:
                    self._invalid_payloads += 1
                    continue
                key = (vehicle_id, session_id)
                channel = self._channels.get(key)
                if channel is None:
                    if not is_hello:
                        self._invalid_payloads += 1
                        await self._request_session_reset(key)
                        continue
                    channel = _GroundSerialChannel(
                        link,
                        vehicle_id=vehicle_id,
                        session_id=session_id,
                    )
                    self._channels[key] = channel
                    task = asyncio.create_task(
                        self._serve_channel(key, channel),
                        name=f"serial-ground-session-{vehicle_id}-{session_id}",
                    )
                    self._channel_tasks.add(task)
                    task.add_done_callback(self._channel_task_done)
                    self._last_session_reset.pop(key, None)
                await channel.feed(payload)
                if time.monotonic() >= next_idle_check:
                    await self._expire_idle_channels()
                    next_idle_check = time.monotonic() + 1.0
        finally:
            await asyncio.gather(
                *(
                    channel.close(reason="serial router stopped")
                    for channel in tuple(self._channels.values())
                ),
                return_exceptions=True,
            )

    async def _request_session_reset(self, key: tuple[int, UUID]) -> None:
        """Make an agent with a stale session reopen and authenticate again."""
        now = time.monotonic()
        if now - self._last_session_reset.get(key, 0.0) < 2.0:
            return
        self._last_session_reset[key] = now
        vehicle_id, session_id = key
        payload = json.dumps(
            {
                "protocol_version": "1.0",
                "frame_type": "session_reset",
                "vehicle_id": vehicle_id,
                "session_id": str(session_id),
                "reason": "ground_session_missing",
            },
            separators=(",", ":"),
        )
        self._session_reset_requests += 1
        try:
            await self._require_serial_link().send(
                vehicle_id,
                payload,
                reliable=True,
            )
        except (SerialChannelClosed, SerialTransportError):
            return

    def _channel_task_done(self, task: asyncio.Task[None]) -> None:
        self._channel_tasks.discard(task)
        if task.cancelled():
            return
        error = task.exception()
        if error is not None:
            LOGGER.warning("serial vehicle session ended: %s", error)

    @staticmethod
    def _payload_identity(payload: str) -> tuple[int, UUID, bool]:
        try:
            data = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ProtocolError("serial payload is not a JSON protocol frame") from exc
        if not isinstance(data, dict):
            raise ProtocolError("serial protocol payload must be an object")
        try:
            vehicle_id = int(data["vehicle_id"])
            session_id = UUID(str(data["session_id"]))
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolError("serial protocol identity is malformed") from exc
        is_hello = data.get("frame_type") == "client_hello"
        if is_hello:
            hello = decode_handshake_frame(payload)
            if not isinstance(hello, ClientHello):  # pragma: no cover - discriminator
                raise ProtocolError("serial session must begin with client_hello")
        return vehicle_id, session_id, is_hello

    async def _serve_channel(
        self,
        key: tuple[int, UUID],
        channel: _GroundSerialChannel,
    ) -> None:
        try:
            await self._handle_connection(channel)
        finally:
            await channel.close()
            if self._channels.get(key) is channel:
                self._channels.pop(key, None)

    async def _expire_idle_channels(self) -> None:
        cutoff = time.monotonic() - self._session_idle_timeout_s
        expired = [
            channel
            for channel in self._channels.values()
            if channel.last_received_monotonic_s < cutoff
        ]
        await asyncio.gather(
            *(channel.close(reason="serial session idle timeout") for channel in expired),
            return_exceptions=True,
        )

    def _require_serial_link(self) -> ReliableSerialLink:
        if self._serial_link is None:
            raise SerialTransportError("serial ground server is not running")
        return self._serial_link
