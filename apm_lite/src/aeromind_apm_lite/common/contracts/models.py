"""Strict v1 wire contracts for ground, simulated and real vehicles."""

from __future__ import annotations

from datetime import datetime, timezone
from enum import Enum
from typing import Annotated, Literal, Union
from uuid import UUID, uuid4

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

SCHEMA_VERSION = "1.0"


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _as_utc(value: datetime) -> datetime:
    if value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("timestamp must include a timezone")
    return value.astimezone(timezone.utc)


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)


class CoordinateFrame(str, Enum):
    NONE = "none"
    MAP = "map"
    LOCAL_NED = "local_ned"
    GLOBAL_WGS84 = "global_wgs84"
    BODY_FRD = "body_frd"
    CAMERA_OPTICAL = "camera_optical"


class VehicleCommandType(str, Enum):
    ARM = "arm"
    DISARM = "disarm"
    TAKEOFF = "takeoff"
    LAND = "land"
    RTL = "rtl"
    HOLD = "hold"
    CANCEL = "cancel"


class FormationShape(str, Enum):
    LINE = "line"
    V = "v"
    SQUARE = "square"


class AckStatus(str, Enum):
    ACCEPTED = "accepted"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"
    EXPIRED = "expired"


class BoxFace(str, Enum):
    FRONT = "front"
    BACK = "back"
    LEFT = "left"
    RIGHT = "right"
    TOP = "top"
    UNKNOWN = "unknown"


class Vector3(StrictModel):
    x: float
    y: float
    z: float


class Quaternion(StrictModel):
    w: float
    x: float
    y: float
    z: float

    @model_validator(mode="after")
    def approximately_unit_length(self) -> "Quaternion":
        norm_squared = self.w**2 + self.x**2 + self.y**2 + self.z**2
        if not 0.98 <= norm_squared <= 1.02:
            raise ValueError("attitude quaternion must be normalized")
        return self


class MessageBase(StrictModel):
    schema_version: Literal["1.0"] = SCHEMA_VERSION
    message_type: str
    message_id: UUID = Field(default_factory=uuid4)
    mission_id: UUID | None = None
    vehicle_id: int = Field(ge=1, le=255)
    sequence: int = Field(ge=0)
    session_id: UUID
    created_at_utc: datetime = Field(default_factory=utc_now)
    ttl_ms: int = Field(default=5_000, ge=100, le=300_000)
    frame: CoordinateFrame = CoordinateFrame.NONE
    frame_calibration_id: str | None = Field(default=None, min_length=1, max_length=128)

    _created_at_is_utc = field_validator("created_at_utc")(_as_utc)

    @model_validator(mode="after")
    def coordinate_messages_require_calibration(self) -> "MessageBase":
        if self.frame != CoordinateFrame.NONE and not self.frame_calibration_id:
            raise ValueError("coordinate-bearing messages require frame_calibration_id")
        return self


class VehicleCommand(MessageBase):
    message_type: Literal["vehicle_command"] = "vehicle_command"
    command: VehicleCommandType
    target_altitude_m: float | None = Field(default=None, gt=0.0, le=120.0)
    reason: str = Field(default="", max_length=256)

    @model_validator(mode="after")
    def takeoff_requires_altitude(self) -> "VehicleCommand":
        if self.command == VehicleCommandType.TAKEOFF and self.target_altitude_m is None:
            raise ValueError("takeoff requires target_altitude_m")
        if self.command != VehicleCommandType.TAKEOFF and self.target_altitude_m is not None:
            raise ValueError("target_altitude_m is only valid for takeoff")
        return self


class TrajectoryPoint(StrictModel):
    time_from_start_ms: int = Field(ge=0, le=300_000)
    position_m: Vector3
    velocity_m_s: Vector3 | None = None
    yaw_rad: float | None = Field(default=None, ge=-3.141593, le=3.141593)


class TrajectorySegment(MessageBase):
    message_type: Literal["trajectory_segment"] = "trajectory_segment"
    segment_id: UUID = Field(default_factory=uuid4)
    starts_at_utc: datetime
    points: list[TrajectoryPoint] = Field(min_length=2, max_length=500)
    max_speed_m_s: float = Field(gt=0.0, le=15.0)
    max_accel_m_s2: float = Field(gt=0.0, le=10.0)

    _starts_at_is_utc = field_validator("starts_at_utc")(_as_utc)

    @model_validator(mode="after")
    def validate_segment(self) -> "TrajectorySegment":
        if self.frame not in {CoordinateFrame.MAP, CoordinateFrame.LOCAL_NED}:
            raise ValueError("trajectory frame must be map or local_ned")
        times = [point.time_from_start_ms for point in self.points]
        if any(current <= previous for previous, current in zip(times, times[1:])):
            raise ValueError("trajectory point times must be strictly increasing")
        return self


class FormationCommand(MessageBase):
    message_type: Literal["formation_command"] = "formation_command"
    shape: FormationShape
    spacing_m: float = Field(ge=2.0, le=50.0)
    transition_duration_s: float = Field(ge=1.0, le=60.0)
    starts_at_utc: datetime
    virtual_leader_position_m: Vector3

    _starts_at_is_utc = field_validator("starts_at_utc")(_as_utc)

    @model_validator(mode="after")
    def formation_uses_map_frame(self) -> "FormationCommand":
        if self.frame != CoordinateFrame.MAP:
            raise ValueError("formation commands must use the map frame")
        return self


class VehicleHealth(StrictModel):
    fcu_link_ok: bool
    ekf_ok: bool | None = None
    gps_fix_type: int = Field(default=0, ge=0, le=6)
    gps_healthy: bool | None = None
    prearm_ok: bool | None = None
    depth_ok: bool | None = None


class VehicleTelemetry(MessageBase):
    message_type: Literal["vehicle_telemetry"] = "vehicle_telemetry"
    observed_at_utc: datetime
    armed: bool
    mode: str = Field(min_length=1, max_length=64)
    position_m: Vector3 | None = None
    velocity_m_s: Vector3 | None = None
    attitude: Quaternion | None = None
    battery_remaining: float | None = Field(default=None, ge=0.0, le=1.0)
    health: VehicleHealth

    _observed_at_is_utc = field_validator("observed_at_utc")(_as_utc)

    @model_validator(mode="after")
    def geometry_requires_frame(self) -> "VehicleTelemetry":
        if (
            self.position_m is not None
            or self.velocity_m_s is not None
            or self.attitude is not None
        ) and self.frame == CoordinateFrame.NONE:
            raise ValueError("telemetry geometry requires a coordinate frame")
        return self


class SemanticObservation(MessageBase):
    message_type: Literal["semantic_observation"] = "semantic_observation"
    observed_at_utc: datetime
    box_id: str = Field(min_length=1, max_length=64)
    color: str = Field(min_length=1, max_length=32, pattern=r"^[a-z][a-z0-9_-]*$")
    face: BoxFace
    confidence: float = Field(ge=0.0, le=1.0)
    position_m: Vector3 | None = None
    evidence_ids: list[str] = Field(min_length=1, max_length=16)
    model_name: str | None = Field(default=None, max_length=128)

    _observed_at_is_utc = field_validator("observed_at_utc")(_as_utc)

    @model_validator(mode="after")
    def position_requires_frame(self) -> "SemanticObservation":
        if self.position_m is not None and self.frame == CoordinateFrame.NONE:
            raise ValueError("semantic position requires a coordinate frame")
        return self


class MissionAck(MessageBase):
    message_type: Literal["mission_ack"] = "mission_ack"
    acknowledged_message_id: UUID
    status: AckStatus
    detail: str = Field(default="", max_length=512)
    apm_command_result: int | None = None
    physical_completion_confirmed: bool = False

    @model_validator(mode="after")
    def completion_requires_physical_confirmation(self) -> "MissionAck":
        if self.status == AckStatus.COMPLETED and not self.physical_completion_confirmed:
            raise ValueError("completed ACK requires physical telemetry confirmation")
        return self


class Heartbeat(MessageBase):
    message_type: Literal["heartbeat"] = "heartbeat"
    software_version: str = Field(min_length=1, max_length=64)
    configuration_hash: str = Field(min_length=8, max_length=128)
    last_command_message_id: UUID | None = None


WireMessage = Annotated[
    Union[
        VehicleCommand,
        TrajectorySegment,
        FormationCommand,
        VehicleTelemetry,
        SemanticObservation,
        MissionAck,
        Heartbeat,
    ],
    Field(discriminator="message_type"),
]
