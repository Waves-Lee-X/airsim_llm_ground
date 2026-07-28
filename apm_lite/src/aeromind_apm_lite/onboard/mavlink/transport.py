"""The single-owner transport boundary used by :mod:`onboard.apm_link`."""

from __future__ import annotations

from typing import Protocol, Sequence

from .models import MavlinkEnvelope

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MIN_MESSAGE_FREQUENCY_HZ = 0.1
MAX_MESSAGE_FREQUENCY_HZ = 100.0
MAX_MAVLINK_MESSAGE_ID = 0xFFFFFF


def message_interval_command_params(
    message_id: int,
    frequency_hz: float,
) -> tuple[float, ...]:
    """Build validated MAV_CMD_SET_MESSAGE_INTERVAL parameters."""
    if not isinstance(message_id, int) or isinstance(message_id, bool):
        raise ValueError("message_id must be an integer")
    if not 0 <= message_id <= MAX_MAVLINK_MESSAGE_ID:
        raise ValueError("message_id must be in [0, 16777215]")
    if isinstance(frequency_hz, bool) or not isinstance(
        frequency_hz,
        (int, float),
    ):
        raise ValueError("frequency_hz must be a real number")
    frequency = float(frequency_hz)
    if not (
        MIN_MESSAGE_FREQUENCY_HZ
        <= frequency
        <= MAX_MESSAGE_FREQUENCY_HZ
    ):
        raise ValueError("frequency_hz must be in [0.1, 100.0]")
    interval_us = round(1_000_000.0 / frequency)
    return (
        float(message_id),
        float(interval_us),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


class MavlinkTransport(Protocol):
    async def open(self) -> None:
        """Open the connection and bind it to the current owner task."""

    async def close(self) -> None:
        """Close the connection from the owner task."""

    async def receive(self, timeout_s: float) -> MavlinkEnvelope | None:
        """Receive one message, returning ``None`` at the timeout."""

    async def send_command_long(
        self,
        command_id: int,
        params: Sequence[float],
    ) -> None:
        """Send a COMMAND_LONG with exactly seven parameters."""

    async def send_heartbeat(self) -> None:
        """Send one companion heartbeat using a GCS MAVLink identity."""

    async def request_message_interval(
        self,
        message_id: int,
        frequency_hz: float,
    ) -> None:
        """Request an FCU message rate with MAV_CMD_SET_MESSAGE_INTERVAL."""

    async def send_local_position_target(
        self,
        north_m: float,
        east_m: float,
        down_m: float,
        yaw_rad: float | None,
    ) -> None:
        """Send a position-only SET_POSITION_TARGET_LOCAL_NED."""

    async def mode_id(self, mode: str) -> int:
        """Resolve an ArduCopter mode name after heartbeat discovery."""
