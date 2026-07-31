"""Real serial/UDP transport backed by pymavlink.

The connection object never leaves this class. ``ApmLink`` calls every method
from one asyncio task, so receive and send operations cannot race each other.
"""

from __future__ import annotations

import asyncio
import time
from collections.abc import Callable, Sequence
from typing import Any

from pymavlink import mavutil

from aeromind_apm_lite.common.config import FcuConnection, FcuTransport

from .models import MavlinkEnvelope
from .transport import message_interval_command_params

ConnectionFactory = Callable[..., Any]


class PymavlinkTransport:
    def __init__(
        self,
        config: FcuConnection,
        *,
        poll_interval_s: float = 0.01,
        connection_factory: ConnectionFactory | None = None,
    ) -> None:
        if not 0.001 <= poll_interval_s <= 0.1:
            raise ValueError("poll_interval_s must be in [0.001, 0.1]")
        self._config = config
        self._poll_interval_s = poll_interval_s
        self._connection_factory = (
            connection_factory or mavutil.mavlink_connection
        )
        self._connection: Any | None = None
        self._owner: asyncio.Task[object] | None = None

    async def open(self) -> None:
        if self._connection is not None:
            raise RuntimeError("pymavlink transport is already open")
        self._owner = asyncio.current_task()
        kwargs: dict[str, Any] = {
            "source_system": self._config.source_system,
            "source_component": self._config.source_component,
            "autoreconnect": False,
        }
        if self._config.transport == FcuTransport.SERIAL:
            kwargs["baud"] = self._config.baudrate
        self._connection = self._connection_factory(
            self._config.endpoint,
            **kwargs,
        )

    @property
    def is_open(self) -> bool:
        return self._connection is not None

    async def close(self) -> None:
        self._assert_owner()
        connection = self._require_connection()
        connection.close()
        self._connection = None

    async def receive(self, timeout_s: float) -> MavlinkEnvelope | None:
        self._assert_owner()
        connection = self._require_connection()
        deadline = time.monotonic() + max(timeout_s, 0.0)
        while True:
            message = connection.recv_match(blocking=False)
            if message is not None and message.get_type() != "BAD_DATA":
                fields = dict(message.to_dict())
                if message.get_type() == "HEARTBEAT":
                    fields["mode_name"] = mavutil.mode_string_v10(message)
                return MavlinkEnvelope(
                    name=message.get_type(),
                    fields=fields,
                    source_system=int(message.get_srcSystem()),
                    source_component=int(message.get_srcComponent()),
                )
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            await asyncio.sleep(min(self._poll_interval_s, remaining))

    async def send_command_long(
        self,
        command_id: int,
        params: Sequence[float],
    ) -> None:
        self._assert_owner()
        connection = self._require_connection()
        values = tuple(float(value) for value in params)
        if len(values) != 7:
            raise ValueError("COMMAND_LONG requires exactly seven parameters")
        connection.mav.command_long_send(
            self._config.target_system,
            self._config.target_component,
            int(command_id),
            0,
            *values,
        )

    async def send_heartbeat(self) -> None:
        self._assert_owner()
        connection = self._require_connection()
        connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
            3,
        )

    async def request_message_interval(
        self,
        message_id: int,
        frequency_hz: float,
    ) -> None:
        self._assert_owner()
        await self.send_command_long(
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
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
        connection = self._require_connection()
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        if yaw_rad is None:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        connection.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self._config.target_system,
            self._config.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            float(north_m),
            float(east_m),
            float(down_m),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0 if yaw_rad is None else float(yaw_rad),
            0.0,
        )

    async def mode_id(self, mode: str) -> int:
        self._assert_owner()
        connection = self._require_connection()
        mapping = connection.mode_mapping() or {}
        normalized = mode.upper()
        try:
            return int(mapping[normalized])
        except KeyError as exc:
            available = ", ".join(sorted(mapping))
            raise ValueError(
                f"unknown FCU mode {normalized}; available: {available}"
            ) from exc

    def _require_connection(self) -> Any:
        if self._connection is None:
            raise RuntimeError("pymavlink transport is not open")
        return self._connection

    def _assert_owner(self) -> None:
        if asyncio.current_task() is not self._owner:
            raise RuntimeError(
                "pymavlink transport accessed outside its owner task"
            )
