"""Deterministic fake MAVLink transport for link and mission tests."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Sequence

from .models import MavlinkEnvelope
from .transport import (
    MAV_CMD_SET_MESSAGE_INTERVAL,
    message_interval_command_params,
)


@dataclass(frozen=True, slots=True)
class SentCommandLong:
    command_id: int
    params: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class SentHeartbeat:
    pass


@dataclass(frozen=True, slots=True)
class SentLocalPositionTarget:
    north_m: float
    east_m: float
    down_m: float
    yaw_rad: float | None


SentRecord = SentCommandLong | SentHeartbeat | SentLocalPositionTarget


class FakeMavlinkTransport:
    def __init__(self, *, mode_mapping: dict[str, int] | None = None) -> None:
        self._incoming: asyncio.Queue[MavlinkEnvelope] = asyncio.Queue()
        self._sent_event = asyncio.Event()
        self._mode_mapping = {
            key.upper(): value
            for key, value in (
                mode_mapping or {"GUIDED": 4, "LOITER": 5, "RTL": 6}
            ).items()
        }
        self._owner: asyncio.Task[object] | None = None
        self.opened = False
        self.closed = False
        self.owner_violations = 0
        self.sent: list[SentRecord] = []

    def feed(self, message: MavlinkEnvelope) -> None:
        self._incoming.put_nowait(message)

    async def wait_for_sent(
        self,
        count: int,
        *,
        timeout_s: float = 1.0,
    ) -> SentRecord:
        async def wait() -> SentRecord:
            while len(self.sent) < count:
                self._sent_event.clear()
                if len(self.sent) < count:
                    await self._sent_event.wait()
            return self.sent[count - 1]

        return await asyncio.wait_for(wait(), timeout=timeout_s)

    async def wait_for_command_long(
        self,
        command_id: int,
        *,
        timeout_s: float = 1.0,
    ) -> SentCommandLong:
        async def wait() -> SentCommandLong:
            inspected = 0
            while True:
                for item in self.sent[inspected:]:
                    if (
                        isinstance(item, SentCommandLong)
                        and item.command_id == command_id
                    ):
                        return item
                inspected = len(self.sent)
                self._sent_event.clear()
                if len(self.sent) == inspected:
                    await self._sent_event.wait()

        return await asyncio.wait_for(wait(), timeout=timeout_s)

    async def open(self) -> None:
        if self.opened:
            raise RuntimeError("fake transport is already open")
        self._owner = asyncio.current_task()
        self.opened = True

    async def close(self) -> None:
        self._assert_owner()
        self.closed = True

    async def receive(self, timeout_s: float) -> MavlinkEnvelope | None:
        self._assert_owner()
        try:
            return await asyncio.wait_for(
                self._incoming.get(),
                timeout=max(timeout_s, 0.0),
            )
        except asyncio.TimeoutError:
            return None

    async def send_command_long(
        self,
        command_id: int,
        params: Sequence[float],
    ) -> None:
        self._assert_owner()
        values = tuple(float(value) for value in params)
        if len(values) != 7:
            raise ValueError("COMMAND_LONG requires exactly seven parameters")
        self.sent.append(SentCommandLong(command_id=command_id, params=values))
        self._sent_event.set()

    async def send_heartbeat(self) -> None:
        self._assert_owner()
        self.sent.append(SentHeartbeat())
        self._sent_event.set()

    async def request_message_interval(
        self,
        message_id: int,
        frequency_hz: float,
    ) -> None:
        self._assert_owner()
        await self.send_command_long(
            MAV_CMD_SET_MESSAGE_INTERVAL,
            message_interval_command_params(message_id, frequency_hz),
        )

    async def send_local_position_target(
        self,
        north_m: float,
        east_m: float,
        down_m: float,
        yaw_rad: float | None,
    ) -> None:
        self._assert_owner()
        self.sent.append(
            SentLocalPositionTarget(
                north_m=north_m,
                east_m=east_m,
                down_m=down_m,
                yaw_rad=yaw_rad,
            )
        )
        self._sent_event.set()

    async def mode_id(self, mode: str) -> int:
        self._assert_owner()
        try:
            return self._mode_mapping[mode.upper()]
        except KeyError as exc:
            raise ValueError(f"unknown FCU mode: {mode}") from exc

    def _assert_owner(self) -> None:
        if asyncio.current_task() is not self._owner:
            self.owner_violations += 1
            raise RuntimeError(
                "MAVLink transport accessed outside its owner task"
            )
