"""Small single-owner MAVLink I/O boundary for the bridge adapters."""

from __future__ import annotations

import asyncio
import time
from collections.abc import Callable, Sequence
from typing import Any, Protocol

from pymavlink import mavutil

from aeromind_apm_lite.common.config import FcuConnection, FcuTransport
from aeromind_apm_lite.onboard.mavlink.models import MavlinkEnvelope

from .base_types import MissionWireItem


class MavlinkIO(Protocol):
    async def open(self) -> None: ...

    async def close(self) -> None: ...

    async def recv(self, timeout_s: float) -> MavlinkEnvelope | None: ...

    async def send_heartbeat(self) -> None: ...

    async def send_command_long(self, command_id: int, params: Sequence[float]) -> None: ...

    async def set_mode(self, mode: str) -> None: ...

    async def send_position_target_local_ned(
        self,
        north_m: float,
        east_m: float,
        down_m: float,
    ) -> None: ...

    async def send_position_target_global_int(
        self,
        latitude_deg: float,
        longitude_deg: float,
        relative_altitude_m: float,
    ) -> None: ...

    async def mission_clear_all(self) -> None: ...

    async def mission_count(self, count: int) -> None: ...

    async def mission_item_int(self, item: MissionWireItem) -> None: ...

    async def mission_set_current(self, sequence: int) -> None: ...


class PymavlinkIO:
    """Pymavlink wrapper that never exposes its connection object."""

    def __init__(
        self,
        config: FcuConnection,
        *,
        poll_interval_s: float = 0.01,
        dialect: str = "common",
        connection_factory: Callable[..., Any] | None = None,
    ) -> None:
        if not 0.001 <= poll_interval_s <= 0.1:
            raise ValueError("poll_interval_s must be in [0.001, 0.1]")
        self.config = config
        self.poll_interval_s = poll_interval_s
        if dialect not in {"common", "ardupilotmega"}:
            raise ValueError("unsupported MAVLink dialect")
        self.dialect = dialect
        self.connection_factory = connection_factory or mavutil.mavlink_connection
        self._connection: Any | None = None
        self._owner: asyncio.Task[object] | None = None

    async def open(self) -> None:
        if self._connection is not None:
            raise RuntimeError("MAVLink connection is already open")
        self._owner = asyncio.current_task()
        kwargs: dict[str, Any] = {
            "source_system": self.config.source_system,
            "source_component": self.config.source_component,
            "autoreconnect": False,
            "dialect": self.dialect,
        }
        if self.config.transport == FcuTransport.SERIAL:
            kwargs["baud"] = self.config.baudrate
        self._connection = self.connection_factory(self.config.endpoint, **kwargs)

    async def close(self) -> None:
        self._assert_owner()
        if self._connection is not None:
            self._connection.close()
            self._connection = None

    async def recv(self, timeout_s: float) -> MavlinkEnvelope | None:
        self._assert_owner()
        connection = self._require_connection()
        deadline = time.monotonic() + max(timeout_s, 0.0)
        while True:
            message = connection.recv_match(blocking=False)
            if message is not None and message.get_type() != "BAD_DATA":
                fields = dict(message.to_dict())
                name = message.get_type()
                if name == "HEARTBEAT":
                    try:
                        fields["mode_name"] = mavutil.mode_string_v10(message)
                    except (AttributeError, TypeError, ValueError):
                        fields["mode_name"] = str(fields.get("custom_mode", "UNKNOWN"))
                return MavlinkEnvelope(
                    name=name,
                    fields=fields,
                    source_system=int(message.get_srcSystem()),
                    source_component=int(message.get_srcComponent()),
                )
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            await asyncio.sleep(min(self.poll_interval_s, remaining))

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

    async def send_command_long(self, command_id: int, params: Sequence[float]) -> None:
        self._assert_owner()
        values = tuple(float(value) for value in params)
        if len(values) != 7:
            raise ValueError("COMMAND_LONG requires exactly seven parameters")
        connection = self._require_connection()
        connection.mav.command_long_send(
            self.config.target_system,
            self.config.target_component,
            int(command_id),
            0,
            *values,
        )

    async def set_mode(self, mode: str) -> None:
        self._assert_owner()
        connection = self._require_connection()
        mapping = connection.mode_mapping() or {}
        mode_value: Any | None = None
        for name, value in mapping.items():
            if str(name).upper() == mode.upper():
                mode_value = value
                break
        if mode_value is None:
            raise ValueError(f"FCU does not advertise mode {mode}")
        # pymavlink represents PX4 modes as (base_mode, custom_mode,
        # custom_sub_mode), while ArduPilot modes are a single integer.
        # Keep this dialect-specific wire detail inside the I/O boundary.
        if isinstance(mode_value, (tuple, list)):
            if len(mode_value) != 3:
                raise ValueError(f"invalid PX4 mode mapping for {mode}")
            connection.mav.command_long_send(
                self.config.target_system,
                self.config.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                float(mode_value[0]),
                float(mode_value[1]),
                float(mode_value[2]),
                0.0,
                0.0,
                0.0,
                0.0,
            )
            return
        mode_id = int(mode_value)
        connection.mav.set_mode_send(
            self.config.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    async def send_position_target_local_ned(
        self,
        north_m: float,
        east_m: float,
        down_m: float,
    ) -> None:
        """Send a position-only setpoint; velocity, acceleration and yaw are ignored."""

        self._assert_owner()
        connection = self._require_connection()
        connection.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1_000.0) & 0xFFFFFFFF,
            self.config.target_system,
            self.config.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            _POSITION_ONLY_TYPE_MASK,
            float(north_m),
            float(east_m),
            float(down_m),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    async def send_position_target_global_int(
        self,
        latitude_deg: float,
        longitude_deg: float,
        relative_altitude_m: float,
    ) -> None:
        """Send one GUIDED global position target without low-level control fields."""

        self._assert_owner()
        connection = self._require_connection()
        connection.mav.set_position_target_global_int_send(
            int(time.monotonic() * 1_000.0) & 0xFFFFFFFF,
            self.config.target_system,
            self.config.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            _POSITION_ONLY_TYPE_MASK,
            round(float(latitude_deg) * 10_000_000.0),
            round(float(longitude_deg) * 10_000_000.0),
            float(relative_altitude_m),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    async def mission_clear_all(self) -> None:
        self._assert_owner()
        connection = self._require_connection()
        connection.mav.mission_clear_all_send(
            self.config.target_system,
            self.config.target_component,
            0,
        )

    async def mission_count(self, count: int) -> None:
        self._assert_owner()
        if count < 1:
            raise ValueError("mission count must be positive")
        connection = self._require_connection()
        connection.mav.mission_count_send(
            self.config.target_system,
            self.config.target_component,
            count,
            0,
        )

    async def mission_item_int(self, item: MissionWireItem) -> None:
        self._assert_owner()
        connection = self._require_connection()
        connection.mav.mission_item_int_send(
            self.config.target_system,
            self.config.target_component,
            item.sequence,
            item.frame,
            item.command,
            item.current,
            item.autocontinue,
            *item.params,
            item.x,
            item.y,
            item.z,
            0,
        )

    async def mission_set_current(self, sequence: int) -> None:
        self._assert_owner()
        connection = self._require_connection()
        connection.mav.mission_set_current_send(
            self.config.target_system,
            self.config.target_component,
            sequence,
        )

    def _assert_owner(self) -> None:
        if asyncio.current_task() is not self._owner:
            raise RuntimeError("MAVLink I/O accessed outside its owner task")

    def _require_connection(self) -> Any:
        if self._connection is None:
            raise RuntimeError("MAVLink connection is not open")
        return self._connection


_POSITION_ONLY_TYPE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


__all__ = ["MavlinkIO", "PymavlinkIO"]
