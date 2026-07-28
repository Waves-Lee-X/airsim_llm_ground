"""Transport-neutral MAVLink command and telemetry models."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, IntEnum
from types import MappingProxyType
from typing import Any, Mapping
from uuid import UUID


class MavCommandId(IntEnum):
    NAV_RETURN_TO_LAUNCH = 20
    NAV_LAND = 21
    NAV_TAKEOFF = 22
    DO_SET_MODE = 176
    COMPONENT_ARM_DISARM = 400
    SET_MESSAGE_INTERVAL = 511
    REQUEST_MESSAGE = 512


class MavMessageId(IntEnum):
    SYS_STATUS = 1
    GPS_RAW_INT = 24
    ATTITUDE_QUATERNION = 31
    LOCAL_POSITION_NED = 32
    GLOBAL_POSITION_INT = 33
    BATTERY_STATUS = 147
    AUTOPILOT_VERSION = 148
    EKF_STATUS_REPORT = 193
    HOME_POSITION = 242
    EXTENDED_SYS_STATE = 245


class MavResult(IntEnum):
    ACCEPTED = 0
    TEMPORARILY_REJECTED = 1
    DENIED = 2
    UNSUPPORTED = 3
    FAILED = 4
    IN_PROGRESS = 5
    CANCELLED = 6


class FcuAction(str, Enum):
    ARM = "arm"
    DISARM = "disarm"
    TAKEOFF = "takeoff"
    SET_MODE = "set_mode"
    GOTO_LOCAL_NED = "goto_local_ned"
    HOLD = "hold"
    LAND = "land"
    RTL = "rtl"


class CommandStatus(str, Enum):
    COMPLETED = "completed"
    APPLICATION_REJECTED = "application_rejected"
    DISPATCH_FAILED = "dispatch_failed"
    MAVLINK_REJECTED = "mavlink_rejected"
    ACK_TIMEOUT = "ack_timeout"
    PHYSICAL_TIMEOUT = "physical_timeout"
    EXPIRED_BEFORE_SEND = "expired_before_send"
    EXPIRED_DURING_EXECUTION = "expired_during_execution"
    PREEMPTED = "preempted"
    FCU_LINK_LOST = "fcu_link_lost"
    LINK_STOPPED = "link_stopped"


class MavLandedState(IntEnum):
    UNDEFINED = 0
    ON_GROUND = 1
    IN_AIR = 2
    TAKEOFF = 3
    LANDING = 4


@dataclass(frozen=True, slots=True)
class FcuCommand:
    action: FcuAction
    target_altitude_m: float | None = None
    target_position_ned_m: tuple[float, float, float] | None = None
    yaw_rad: float | None = None
    mode: str | None = None

    def __post_init__(self) -> None:
        if self.action == FcuAction.TAKEOFF:
            if self.target_altitude_m is None or not 0.0 < self.target_altitude_m <= 120.0:
                raise ValueError("takeoff requires target_altitude_m in (0, 120]")
        elif self.target_altitude_m is not None:
            raise ValueError("target_altitude_m is only valid for takeoff")

        if self.action == FcuAction.GOTO_LOCAL_NED:
            if self.target_position_ned_m is None:
                raise ValueError("goto_local_ned requires target_position_ned_m")
            if len(self.target_position_ned_m) != 3:
                raise ValueError("target_position_ned_m must contain north, east and down")
            if not all(math.isfinite(value) for value in self.target_position_ned_m):
                raise ValueError("target_position_ned_m must contain finite values")
        elif self.target_position_ned_m is not None:
            raise ValueError("target_position_ned_m is only valid for goto_local_ned")

        if self.action == FcuAction.SET_MODE:
            if self.mode is None or not self.mode.strip():
                raise ValueError("set_mode requires a non-empty mode")
        elif self.mode is not None:
            raise ValueError("mode is only valid for set_mode")

        if self.yaw_rad is not None:
            if self.action != FcuAction.GOTO_LOCAL_NED:
                raise ValueError("yaw_rad is only valid for goto_local_ned")
            if not -math.pi <= self.yaw_rad <= math.pi:
                raise ValueError("yaw_rad must be in [-pi, pi]")


@dataclass(frozen=True, slots=True)
class MavlinkEnvelope:
    name: str
    fields: Mapping[str, Any]
    source_system: int
    source_component: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", self.name.upper())
        object.__setattr__(self, "fields", MappingProxyType(dict(self.fields)))


@dataclass(frozen=True, slots=True)
class FcuIdentity:
    system_id: int
    component_id: int
    autopilot_type: int
    vehicle_type: int
    flight_sw_version_raw: int
    flight_sw_version: str
    board_version: int
    vendor_id: int
    product_id: int
    uid: int
    capabilities: int
    custom_version_hex: str


@dataclass(frozen=True, slots=True)
class TelemetrySnapshot:
    observed_monotonic_s: float
    fcu_link_ok: bool
    armed: bool | None
    mode: str | None
    relative_altitude_m: float | None
    local_position_ned_m: tuple[float, float, float] | None
    velocity_ned_m_s: tuple[float, float, float] | None
    global_position_deg_m: tuple[float, float, float] | None
    attitude_rpy_rad: tuple[float, float, float] | None
    battery_remaining: float | None
    battery_voltage_v: float | None
    gps_fix_type: int | None
    satellites_visible: int | None
    gps_healthy: bool | None
    prearm_ok: bool | None
    ekf_flags: int | None
    landed_state: int | None
    home_position_deg_m: tuple[float, float, float] | None
    home_position_ned_m: tuple[float, float, float] | None
    last_status_text: str | None
    last_heartbeat_monotonic_s: float | None
    field_ages_s: Mapping[str, float]

    def __post_init__(self) -> None:
        object.__setattr__(self, "field_ages_s", MappingProxyType(dict(self.field_ages_s)))


@dataclass(frozen=True, slots=True)
class ApplicationAcceptance:
    accepted: bool
    observed_monotonic_s: float
    detail: str


@dataclass(frozen=True, slots=True)
class MavlinkAckEvidence:
    applicable: bool
    received: bool
    command_id: int | None
    result: int | None
    observed_monotonic_s: float | None
    detail: str


@dataclass(frozen=True, slots=True)
class PhysicalCompletionEvidence:
    confirmed: bool
    observed_monotonic_s: float | None
    detail: str


@dataclass(frozen=True, slots=True)
class CommandResult:
    request_id: UUID
    action: FcuAction
    status: CommandStatus
    application: ApplicationAcceptance
    sent_monotonic_s: float | None
    mavlink_ack: MavlinkAckEvidence
    physical_completion: PhysicalCompletionEvidence
    detail: str
