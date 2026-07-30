"""Reliable Lite message framing over a transparent serial radio."""

from __future__ import annotations

import asyncio
import json
import secrets
import struct
import zlib
from collections import OrderedDict
from collections.abc import AsyncIterator, Awaitable, Callable
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Protocol


SERIAL_MAGIC = b"\xA9\x4C"
SERIAL_VERSION = 1
SERIAL_FLAG_RELIABLE = 0x01
DEFAULT_MAX_PAYLOAD_BYTES = 8 * 1024

_HEADER = struct.Struct("<2sBBBBBIH")
_CRC = struct.Struct("<I")


class SerialTransportError(RuntimeError):
    """Base error for the Lite serial transport."""


class SerialChannelClosed(SerialTransportError):
    """The serial byte stream or logical message channel is closed."""


class SerialDeliveryError(SerialTransportError):
    """A reliable frame wasn't acknowledged within the retry budget."""


class SerialDirection(IntEnum):
    ONBOARD_TO_GROUND = 1
    GROUND_TO_ONBOARD = 2

    @property
    def opposite(self) -> "SerialDirection":
        if self == SerialDirection.ONBOARD_TO_GROUND:
            return SerialDirection.GROUND_TO_ONBOARD
        return SerialDirection.ONBOARD_TO_GROUND


class SerialFrameKind(IntEnum):
    DATA = 1
    ACK = 2


@dataclass(frozen=True)
class SerialFrame:
    kind: SerialFrameKind
    direction: SerialDirection
    vehicle_id: int
    sequence: int
    payload: bytes = b""
    reliable: bool = False


@dataclass(frozen=True)
class SerialLinkStats:
    received_frames: int
    sent_frames: int
    duplicate_frames: int
    discarded_bytes: int
    crc_errors: int
    retries: int


class AsyncByteStream(Protocol):
    async def read(self, max_bytes: int) -> bytes: ...

    async def write(self, data: bytes) -> None: ...

    async def close(self) -> None: ...


ByteStreamFactory = Callable[[], Awaitable[AsyncByteStream]]


def encode_serial_frame(
    frame: SerialFrame,
    *,
    max_payload_bytes: int = DEFAULT_MAX_PAYLOAD_BYTES,
) -> bytes:
    if not 1 <= frame.vehicle_id <= 255:
        raise ValueError("vehicle_id must be in [1, 255]")
    if not 0 <= frame.sequence <= 0xFFFFFFFF:
        raise ValueError("serial sequence must be in [0, 2^32-1]")
    if len(frame.payload) > max_payload_bytes:
        raise ValueError(
            f"serial payload exceeds {max_payload_bytes} byte transport limit"
        )
    if frame.kind == SerialFrameKind.ACK and frame.payload:
        raise ValueError("serial ACK frames cannot contain a payload")
    flags = SERIAL_FLAG_RELIABLE if frame.reliable else 0
    header = _HEADER.pack(
        SERIAL_MAGIC,
        SERIAL_VERSION,
        int(frame.kind),
        int(frame.direction),
        frame.vehicle_id,
        flags,
        frame.sequence,
        len(frame.payload),
    )
    checksum = zlib.crc32(header[2:] + frame.payload) & 0xFFFFFFFF
    return header + frame.payload + _CRC.pack(checksum)


class SerialFrameDecoder:
    """Incrementally recover valid frames from noisy or fragmented input."""

    def __init__(self, *, max_payload_bytes: int = DEFAULT_MAX_PAYLOAD_BYTES) -> None:
        if not 256 <= max_payload_bytes <= 0xFFFF:
            raise ValueError("max_payload_bytes must be in [256, 65535]")
        self.max_payload_bytes = max_payload_bytes
        self.discarded_bytes = 0
        self.crc_errors = 0
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[SerialFrame]:
        if not isinstance(data, bytes):
            raise TypeError("serial decoder input must be bytes")
        self._buffer.extend(data)
        frames: list[SerialFrame] = []
        while True:
            marker = self._buffer.find(SERIAL_MAGIC)
            if marker < 0:
                keep = min(len(self._buffer), len(SERIAL_MAGIC) - 1)
                self.discarded_bytes += len(self._buffer) - keep
                if keep:
                    del self._buffer[:-keep]
                else:
                    self._buffer.clear()
                break
            if marker:
                self.discarded_bytes += marker
                del self._buffer[:marker]
            if len(self._buffer) < _HEADER.size:
                break
            try:
                (
                    _magic,
                    version,
                    kind_raw,
                    direction_raw,
                    vehicle_id,
                    flags,
                    sequence,
                    payload_length,
                ) = _HEADER.unpack_from(self._buffer)
                kind = SerialFrameKind(kind_raw)
                direction = SerialDirection(direction_raw)
            except (ValueError, struct.error):
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            if (
                version != SERIAL_VERSION
                or vehicle_id == 0
                or flags & ~SERIAL_FLAG_RELIABLE
                or payload_length > self.max_payload_bytes
                or (kind == SerialFrameKind.ACK and payload_length != 0)
            ):
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            frame_length = _HEADER.size + payload_length + _CRC.size
            if len(self._buffer) < frame_length:
                break
            payload_end = _HEADER.size + payload_length
            payload = bytes(self._buffer[_HEADER.size:payload_end])
            expected_crc = _CRC.unpack_from(self._buffer, payload_end)[0]
            actual_crc = zlib.crc32(bytes(self._buffer[2:payload_end])) & 0xFFFFFFFF
            if not secrets.compare_digest(
                expected_crc.to_bytes(4, "little"),
                actual_crc.to_bytes(4, "little"),
            ):
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            del self._buffer[:frame_length]
            frames.append(
                SerialFrame(
                    kind=kind,
                    direction=direction,
                    vehicle_id=vehicle_id,
                    sequence=sequence,
                    payload=payload,
                    reliable=bool(flags & SERIAL_FLAG_RELIABLE),
                )
            )
        return frames


class PySerialByteStream:
    """Async adapter around pyserial without blocking the asyncio loop."""

    def __init__(self, serial_port: Any, *, read_size: int = 1024) -> None:
        self._serial = serial_port
        self._read_size = read_size
        self._write_lock = asyncio.Lock()
        self._closed = False

    @classmethod
    async def open(
        cls,
        port: str,
        baudrate: int,
        *,
        read_timeout_s: float = 0.1,
        write_timeout_s: float = 1.0,
    ) -> "PySerialByteStream":
        if not port:
            raise ValueError("serial port must not be empty")
        if baudrate <= 0:
            raise ValueError("serial baudrate must be positive")
        if not 0.01 <= read_timeout_s <= 1.0:
            raise ValueError("read_timeout_s must be in [0.01, 1]")
        try:
            import serial
        except ImportError as exc:  # pragma: no cover - packaging guard
            raise SerialTransportError("pyserial is required for P9 transport") from exc

        serial_port = await asyncio.to_thread(
            serial.Serial,
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=read_timeout_s,
            write_timeout=write_timeout_s,
        )
        return cls(serial_port)

    async def read(self, max_bytes: int) -> bytes:
        if self._closed:
            return b""
        size = min(max(1, max_bytes), self._read_size)
        return bytes(await asyncio.to_thread(self._serial.read, size))

    async def write(self, data: bytes) -> None:
        if self._closed:
            raise SerialChannelClosed("serial byte stream is closed")
        async with self._write_lock:
            written = await asyncio.to_thread(self._serial.write, data)
            if written != len(data):
                raise SerialTransportError(
                    f"serial write was truncated: {written}/{len(data)} bytes"
                )

    async def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        await asyncio.to_thread(self._serial.close)


async def pyserial_stream_factory(port: str, baudrate: int) -> AsyncByteStream:
    return await PySerialByteStream.open(port, baudrate)


class ReliableSerialLink:
    """One full-duplex framed link with selective delivery acknowledgement."""

    def __init__(
        self,
        stream: AsyncByteStream,
        *,
        send_direction: SerialDirection,
        ack_timeout_s: float = 0.4,
        max_retries: int = 4,
        max_payload_bytes: int = DEFAULT_MAX_PAYLOAD_BYTES,
        duplicate_history: int = 512,
        initial_sequence: int | None = None,
        accepted_vehicle_ids: set[int] | None = None,
    ) -> None:
        if ack_timeout_s <= 0.0:
            raise ValueError("ack_timeout_s must be positive")
        if not 0 <= max_retries <= 20:
            raise ValueError("max_retries must be in [0, 20]")
        if duplicate_history < 16:
            raise ValueError("duplicate_history must be at least 16")
        self._stream = stream
        self.send_direction = send_direction
        self.receive_direction = send_direction.opposite
        self._ack_timeout_s = ack_timeout_s
        self._max_retries = max_retries
        self._decoder = SerialFrameDecoder(max_payload_bytes=max_payload_bytes)
        self._max_payload_bytes = max_payload_bytes
        self._duplicate_history = duplicate_history
        if accepted_vehicle_ids is not None and not accepted_vehicle_ids:
            raise ValueError("accepted_vehicle_ids must not be empty")
        if accepted_vehicle_ids is not None and any(
            not 1 <= vehicle_id <= 255 for vehicle_id in accepted_vehicle_ids
        ):
            raise ValueError("accepted vehicle IDs must be in [1, 255]")
        self._accepted_vehicle_ids = (
            None if accepted_vehicle_ids is None else frozenset(accepted_vehicle_ids)
        )
        self._next_sequence = (
            secrets.randbits(32) if initial_sequence is None else initial_sequence
        )
        self._incoming: asyncio.Queue[SerialFrame | None] = asyncio.Queue()
        self._pending: dict[tuple[int, int], asyncio.Future[None]] = {}
        self._seen: OrderedDict[tuple[int, int], None] = OrderedDict()
        self._write_lock = asyncio.Lock()
        self._sequence_lock = asyncio.Lock()
        self._reader_task: asyncio.Task[None] | None = None
        self._closed = False
        self._close_complete = False
        self._received_frames = 0
        self._sent_frames = 0
        self._duplicate_frames = 0
        self._retries = 0

    @property
    def stats(self) -> SerialLinkStats:
        return SerialLinkStats(
            received_frames=self._received_frames,
            sent_frames=self._sent_frames,
            duplicate_frames=self._duplicate_frames,
            discarded_bytes=self._decoder.discarded_bytes,
            crc_errors=self._decoder.crc_errors,
            retries=self._retries,
        )

    async def start(self) -> None:
        if self._reader_task is not None:
            raise SerialTransportError("serial link can only be started once")
        self._reader_task = asyncio.create_task(
            self._reader_loop(), name=f"serial-link-{self.send_direction.name.lower()}"
        )

    async def close(self) -> None:
        if self._close_complete:
            return
        self._closed = True
        self._close_complete = True
        task = self._reader_task
        self._reader_task = None
        if task is not None:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        error = SerialChannelClosed("serial link is closed")
        for pending in self._pending.values():
            if not pending.done():
                pending.set_exception(error)
        self._pending.clear()
        await self._incoming.put(None)
        await self._stream.close()

    async def send(self, vehicle_id: int, payload: str | bytes, *, reliable: bool) -> int:
        if self._closed or self._reader_task is None:
            raise SerialChannelClosed("serial link is not running")
        encoded_payload = payload.encode("utf-8") if isinstance(payload, str) else payload
        async with self._sequence_lock:
            sequence = self._next_sequence
            self._next_sequence = (self._next_sequence + 1) & 0xFFFFFFFF
        frame = SerialFrame(
            kind=SerialFrameKind.DATA,
            direction=self.send_direction,
            vehicle_id=vehicle_id,
            sequence=sequence,
            payload=encoded_payload,
            reliable=reliable,
        )
        encoded = encode_serial_frame(
            frame, max_payload_bytes=self._max_payload_bytes
        )
        if not reliable:
            await self._write(encoded)
            return sequence

        loop = asyncio.get_running_loop()
        acknowledged: asyncio.Future[None] = loop.create_future()
        key = (vehicle_id, sequence)
        self._pending[key] = acknowledged
        try:
            for attempt in range(self._max_retries + 1):
                await self._write(encoded)
                try:
                    await asyncio.wait_for(
                        asyncio.shield(acknowledged), timeout=self._ack_timeout_s
                    )
                    return sequence
                except asyncio.TimeoutError:
                    if attempt >= self._max_retries:
                        break
                    self._retries += 1
            raise SerialDeliveryError(
                f"vehicle {vehicle_id} frame {sequence} wasn't acknowledged"
            )
        finally:
            self._pending.pop(key, None)

    async def receive(self) -> SerialFrame:
        frame = await self._incoming.get()
        if frame is None:
            raise SerialChannelClosed("serial link is closed")
        return frame

    async def _write(self, encoded: bytes) -> None:
        async with self._write_lock:
            await self._stream.write(encoded)
            self._sent_frames += 1

    async def _reader_loop(self) -> None:
        try:
            while True:
                data = await self._stream.read(1024)
                if not data:
                    await asyncio.sleep(0)
                    continue
                for frame in self._decoder.feed(data):
                    if frame.direction != self.receive_direction:
                        continue
                    if (
                        self._accepted_vehicle_ids is not None
                        and frame.vehicle_id not in self._accepted_vehicle_ids
                    ):
                        continue
                    self._received_frames += 1
                    if frame.kind == SerialFrameKind.ACK:
                        pending = self._pending.get((frame.vehicle_id, frame.sequence))
                        if pending is not None and not pending.done():
                            pending.set_result(None)
                        continue
                    if frame.reliable:
                        await self._send_ack(frame)
                    key = (frame.vehicle_id, frame.sequence)
                    if key in self._seen:
                        self._duplicate_frames += 1
                        self._seen.move_to_end(key)
                        continue
                    self._seen[key] = None
                    if len(self._seen) > self._duplicate_history:
                        self._seen.popitem(last=False)
                    await self._incoming.put(frame)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            if not self._closed:
                self._closed = True
                for pending in self._pending.values():
                    if not pending.done():
                        pending.set_exception(exc)
                await self._incoming.put(None)

    async def _send_ack(self, frame: SerialFrame) -> None:
        acknowledgement = SerialFrame(
            kind=SerialFrameKind.ACK,
            direction=self.send_direction,
            vehicle_id=frame.vehicle_id,
            sequence=frame.sequence,
        )
        await self._write(
            encode_serial_frame(
                acknowledgement, max_payload_bytes=self._max_payload_bytes
            )
        )


def payload_requires_delivery(payload: str | bytes) -> bool:
    """Reliably deliver events, while periodically refreshing replaceable state."""
    try:
        data = json.loads(payload)
    except (json.JSONDecodeError, TypeError, UnicodeDecodeError):
        return True
    if not isinstance(data, dict) or data.get("frame_type") != "signed_message":
        return True
    message = data.get("message")
    if not isinstance(message, dict):
        return True
    return message.get("message_type") not in {
        "heartbeat",
        "vehicle_telemetry",
    }


class SerialMessageChannel:
    """WebSocket-shaped message channel used by the onboard agent."""

    def __init__(
        self,
        link: ReliableSerialLink,
        *,
        vehicle_id: int,
        owns_link: bool = True,
    ) -> None:
        self._link = link
        self._vehicle_id = vehicle_id
        self._owns_link = owns_link
        self._closed = False

    async def send(self, payload: str | bytes) -> None:
        if self._closed:
            raise SerialChannelClosed("serial message channel is closed")
        await self._link.send(
            self._vehicle_id,
            payload,
            reliable=payload_requires_delivery(payload),
        )

    async def recv(self) -> str:
        if self._closed:
            raise SerialChannelClosed("serial message channel is closed")
        while True:
            frame = await self._link.receive()
            if frame.vehicle_id != self._vehicle_id:
                continue
            try:
                return frame.payload.decode("utf-8")
            except UnicodeDecodeError as exc:
                raise SerialTransportError("serial message payload is not UTF-8") from exc

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
        if self._owns_link:
            await self._link.close()


class SerialOnboardConnection:
    """Re-open a Pi P9 serial port for each authenticated agent session."""

    def __init__(
        self,
        *,
        port: str,
        baudrate: int,
        vehicle_id: int,
        stream_factory: ByteStreamFactory | None = None,
        ack_timeout_s: float = 0.4,
        max_retries: int = 4,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._vehicle_id = vehicle_id
        self._stream_factory = stream_factory
        self._ack_timeout_s = ack_timeout_s
        self._max_retries = max_retries
        self._channel: SerialMessageChannel | None = None

    async def __aenter__(self) -> SerialMessageChannel:
        if self._stream_factory is None:
            stream = await pyserial_stream_factory(self._port, self._baudrate)
        else:
            stream = await self._stream_factory()
        link = ReliableSerialLink(
            stream,
            send_direction=SerialDirection.ONBOARD_TO_GROUND,
            ack_timeout_s=self._ack_timeout_s,
            max_retries=self._max_retries,
            accepted_vehicle_ids={self._vehicle_id},
        )
        await link.start()
        self._channel = SerialMessageChannel(
            link, vehicle_id=self._vehicle_id, owns_link=True
        )
        return self._channel

    async def __aexit__(self, *_: object) -> None:
        channel = self._channel
        self._channel = None
        if channel is not None:
            await channel.close()


class SerialOnboardChannelFactory:
    def __init__(
        self,
        *,
        port: str,
        baudrate: int,
        vehicle_id: int,
        stream_factory: ByteStreamFactory | None = None,
        ack_timeout_s: float = 0.4,
        max_retries: int = 4,
    ) -> None:
        self._options = {
            "port": port,
            "baudrate": baudrate,
            "vehicle_id": vehicle_id,
            "stream_factory": stream_factory,
            "ack_timeout_s": ack_timeout_s,
            "max_retries": max_retries,
        }

    def __call__(self) -> SerialOnboardConnection:
        return SerialOnboardConnection(**self._options)
