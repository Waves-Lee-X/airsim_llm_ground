"""Strict contracts for the parallel-domain bridge."""

from __future__ import annotations

import hashlib
import json
import math
from datetime import datetime, timezone
from enum import Enum
from typing import Literal
from uuid import UUID, uuid4

from pydantic import ConfigDict, Field, field_validator, model_validator

from aeromind_apm_lite.common.contracts import (
    CoordinateFrame,
    TwinLine,
    TwinSource,
    TwinState,
    Vector3,
)
from aeromind_apm_lite.common.contracts.models import Quaternion, StrictModel
from aeromind_apm_lite.common.coordinates import CalibrationStatus, Wgs84Position
from aeromind_apm_lite.common.trajectory import TrajectoryComparisonReport


PARALLEL_DOMAIN_SCHEMA_VERSION = "1.0"


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _as_utc(value: datetime) -> datetime:
    if value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("timestamp must include a timezone")
    return value.astimezone(timezone.utc)


def canonical_hash(value: object) -> str:
    if hasattr(value, "model_dump"):
        value = value.model_dump(mode="json")  # type: ignore[union-attr]
    encoded = json.dumps(
        value,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class ParallelModel(StrictModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)
    schema_version: Literal["1.0"] = PARALLEL_DOMAIN_SCHEMA_VERSION


class EndpointRole(str, Enum):
    REAL = "real"
    VIRTUAL = "virtual"


class AdapterKind(str, Enum):
    PX4 = "px4"
    ARDUPILOT = "ardupilot"


class ExecutionSemantic(str, Enum):
    PX4_OFFBOARD_TASK = "px4_offboard_task"
    ARDUPILOT_GUIDED_TASK = "ardupilot_guided_task"
    FAKE_TASK = "fake_task"


class MirrorState(str, Enum):
    WAITING = "waiting"
    FRESH = "fresh"
    STALE = "stale"
    RECOVERING = "recovering"


class MissionAction(str, Enum):
    TAKEOFF = "takeoff"
    WAYPOINT = "waypoint"
    LAND = "land"
    RTL = "rtl"


class LiveFaultType(str, Enum):
    HEARTBEAT_GPS_LOSS = "heartbeat_gps_loss"


class MissionStep(ParallelModel):
    action: MissionAction
    position_map_m: tuple[float, float, float] | None = None
    altitude_m: float | None = Field(default=None, gt=0.0, le=500.0)
    acceptance_radius_m: float = Field(default=1.0, gt=0.0, le=50.0)

    @model_validator(mode="after")
    def parameters_match_action(self) -> "MissionStep":
        if self.action == MissionAction.WAYPOINT:
            if self.position_map_m is None:
                raise ValueError("waypoint requires position_map_m")
        elif self.position_map_m is not None:
            raise ValueError("position_map_m is only valid for waypoint")
        if self.action == MissionAction.TAKEOFF:
            if self.altitude_m is None:
                raise ValueError("takeoff requires altitude_m")
        elif self.altitude_m is not None:
            raise ValueError("altitude_m is only valid for takeoff")
        return self

    @field_validator("position_map_m")
    @classmethod
    def finite_position(
        cls,
        value: tuple[float, float, float] | None,
    ) -> tuple[float, float, float] | None:
        if value is not None and not all(math.isfinite(item) for item in value):
            raise ValueError("position_map_m must contain finite values")
        return value


class MissionPlan(ParallelModel):
    """A high-level task plan; no attitude, rate or actuator fields exist."""

    mission_id: UUID = Field(default_factory=uuid4)
    line_id: str = Field(
        min_length=1,
        max_length=64,
        pattern=r"^[a-z][a-z0-9_-]*$",
    )
    vehicle_id: int = Field(ge=1, le=255)
    frame: Literal["map"] = "map"
    frame_calibration_id: str = Field(min_length=1, max_length=128)
    source: Literal["predicted/sim"] = "predicted/sim"
    cruise_speed_m_s: float = Field(gt=0.0, le=100.0)
    steps: tuple[MissionStep, ...] = Field(min_length=2, max_length=500)
    created_at_utc: datetime = Field(default_factory=utc_now)
    rehearsal_timeout_s: float = Field(default=180.0, ge=10.0, le=3_600.0)

    _created_at_is_utc = field_validator("created_at_utc")(_as_utc)

    @model_validator(mode="after")
    def mission_has_safe_shape(self) -> "MissionPlan":
        if self.steps[0].action != MissionAction.TAKEOFF:
            raise ValueError("mission must start with takeoff")
        if self.steps[-1].action not in {MissionAction.LAND, MissionAction.RTL}:
            raise ValueError("mission must end with land or rtl")
        return self

    @property
    def mission_hash(self) -> str:
        return canonical_hash(self)


class SafetyPolicy(ParallelModel):
    allowed_actions: tuple[MissionAction, ...] = Field(min_length=1)
    confirmation_required: bool = True
    min_takeoff_altitude_m: float = Field(default=2.0, gt=0.0, le=500.0)
    max_altitude_m: float = Field(default=30.0, gt=0.0, le=500.0)
    max_distance_from_home_m: float = Field(default=100.0, gt=0.0, le=100_000.0)
    max_speed_m_s: float = Field(default=10.0, gt=0.0, le=100.0)
    min_gps_fix_type: int = Field(default=3, ge=0, le=6)
    max_telemetry_age_s: float = Field(default=2.0, gt=0.0, le=60.0)
    approval_ttl_s: int = Field(default=600, ge=30, le=86_400)

    @field_validator("allowed_actions")
    @classmethod
    def actions_are_unique(
        cls,
        value: tuple[MissionAction, ...],
    ) -> tuple[MissionAction, ...]:
        if len(value) != len(set(value)):
            raise ValueError("allowed_actions must be unique")
        return value

    @model_validator(mode="after")
    def altitude_bounds_are_ordered(self) -> "SafetyPolicy":
        if self.min_takeoff_altitude_m > self.max_altitude_m:
            raise ValueError("min_takeoff_altitude_m exceeds max_altitude_m")
        return self


class AdapterTelemetry(ParallelModel):
    vehicle_id: int = Field(ge=1, le=255)
    sampled_at_utc: datetime
    received_monotonic_s: float = Field(ge=0.0)
    source_sequence: int = Field(default=0, ge=0)
    connection_generation: int = Field(default=1, ge=1)
    sample_kind: Literal["global_position", "local_position"] = "global_position"
    sim_time_s: float | None = Field(default=None, ge=0.0)
    global_position: Wgs84Position | None = None
    local_position_ned_m: tuple[float, float, float] | None = None
    velocity_ned_m_s: tuple[float, float, float] | None = None
    attitude_quaternion_wxyz: tuple[float, float, float, float] | None = None
    armed: bool | None = None
    mode: str = Field(default="UNKNOWN", min_length=1, max_length=64)
    mission_phase: str = Field(default="idle", min_length=1, max_length=64)
    mission_sequence: int | None = Field(default=None, ge=0, le=65_535)
    mission_complete: bool = False
    battery_remaining: float | None = Field(default=None, ge=0.0, le=1.0)
    gps_fix_type: int | None = Field(default=None, ge=0, le=6)
    gps_hdop: float | None = Field(default=None, ge=0.0, le=100.0)
    satellites_visible: int | None = Field(default=None, ge=0, le=255)
    heartbeat_age_s: float | None = Field(default=None, ge=0.0)
    gps_age_s: float | None = Field(default=None, ge=0.0)
    link_ok: bool = True
    health_ok: bool = True

    _sampled_at_is_utc = field_validator("sampled_at_utc")(_as_utc)

    @model_validator(mode="after")
    def has_position(self) -> "AdapterTelemetry":
        if self.global_position is None and self.local_position_ned_m is None:
            raise ValueError("telemetry requires global or local position")
        if self.attitude_quaternion_wxyz is not None:
            norm = sum(value * value for value in self.attitude_quaternion_wxyz)
            if not 0.98 <= norm <= 1.02:
                raise ValueError("attitude quaternion must be normalized")
        return self


class ApprovalRequest(ParallelModel):
    request_id: UUID = Field(default_factory=uuid4)
    mission_id: UUID
    mission_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    line_id: str
    vehicle_id: int = Field(ge=1, le=255)
    predicted_evidence_id: UUID
    predicted_evidence_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    requested_at_utc: datetime
    expires_at_utc: datetime
    status: Literal["pending"] = "pending"

    @field_validator("requested_at_utc", "expires_at_utc")
    @classmethod
    def request_timestamps_are_utc(cls, value: datetime) -> datetime:
        return _as_utc(value)

    @model_validator(mode="after")
    def expiry_is_after_request(self) -> "ApprovalRequest":
        if self.expires_at_utc <= self.requested_at_utc:
            raise ValueError("approval request expiry must follow request time")
        return self


class ApprovalReceipt(ParallelModel):
    request_id: UUID
    mission_id: UUID
    mission_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    predicted_evidence_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    operator_id: str = Field(min_length=1, max_length=128)
    confirmation_digest: str = Field(pattern=r"^[0-9a-f]{64}$")
    approved_at_utc: datetime = Field(default_factory=utc_now)

    _approved_at_is_utc = field_validator("approved_at_utc")(_as_utc)


class ApprovalCommand(ParallelModel):
    """Operator input placed in the bridge approval spool."""

    mission_id: UUID
    mission_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    operator_id: str = Field(min_length=1, max_length=128)
    confirmation_digest: str = Field(pattern=r"^[0-9a-f]{64}$")


class RealDispatchRequest(ParallelModel):
    """Explicit dispatch request; the core still requires an approval receipt."""

    request_id: UUID = Field(default_factory=uuid4)
    mission_id: UUID
    line_id: str
    vehicle_id: int = Field(ge=1, le=255)
    requested_at_utc: datetime = Field(default_factory=utc_now)

    _requested_at_is_utc = field_validator("requested_at_utc")(_as_utc)


class LiveFaultCommand(ParallelModel):
    """Short-lived SITL-only fault injected at the live telemetry gate."""

    fault_id: UUID = Field(default_factory=uuid4)
    mission_id: UUID
    line_id: str
    vehicle_id: int = Field(ge=1, le=255)
    fault: LiveFaultType
    requested_at_utc: datetime
    expires_at_utc: datetime
    authorization: Literal["SITL_FAULT_INJECTION"] = "SITL_FAULT_INJECTION"

    @field_validator("requested_at_utc", "expires_at_utc")
    @classmethod
    def fault_timestamps_are_utc(cls, value: datetime) -> datetime:
        return _as_utc(value)

    @model_validator(mode="after")
    def fault_expiry_follows_request(self) -> "LiveFaultCommand":
        if self.expires_at_utc <= self.requested_at_utc:
            raise ValueError("fault injection expiry must follow request time")
        return self


class SafetyActionRecord(ParallelModel):
    mission_id: UUID
    line_id: str
    vehicle_id: int = Field(ge=1, le=255)
    action: Literal["safety_hold", "recovery_land"]
    command: str = Field(min_length=1, max_length=128)
    reason: str = Field(min_length=1, max_length=256)
    wall_time: datetime = Field(default_factory=utc_now)

    _wall_time_is_utc = field_validator("wall_time")(_as_utc)


class GateCheck(ParallelModel):
    name: str = Field(min_length=1, max_length=64)
    passed: bool
    detail: str = Field(min_length=1, max_length=512)


class GateDecision(ParallelModel):
    mission_id: UUID
    stage: Literal["rehearsal", "real_dispatch"]
    checks: tuple[GateCheck, ...] = Field(min_length=1)

    @property
    def allowed(self) -> bool:
        return all(check.passed for check in self.checks)


class DispatchRecord(ParallelModel):
    mission_id: UUID
    line_id: str
    role: EndpointRole
    adapter: AdapterKind
    execution_semantic: ExecutionSemantic
    accepted_at_utc: datetime = Field(default_factory=utc_now)
    mission_item_count: int = Field(ge=1)
    detail: str = Field(min_length=1, max_length=512)

    _accepted_at_is_utc = field_validator("accepted_at_utc")(_as_utc)


class DispatchAuthorizationUse(ParallelModel):
    """Append-safe record that makes one approval receipt single-use."""

    request_id: UUID
    mission_id: UUID
    mission_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    predicted_evidence_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    operator_id: str = Field(min_length=1, max_length=128)
    receipt_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    status: Literal["claimed", "dispatched", "failed"] = "claimed"
    claimed_at_utc: datetime = Field(default_factory=utc_now)
    completed_at_utc: datetime | None = None
    detail: str = Field(default="authorization claimed", min_length=1, max_length=512)

    @field_validator("claimed_at_utc", "completed_at_utc")
    @classmethod
    def authorization_timestamps_are_utc(
        cls,
        value: datetime | None,
    ) -> datetime | None:
        return _as_utc(value) if value is not None else None

    @model_validator(mode="after")
    def completion_matches_status(self) -> "DispatchAuthorizationUse":
        if (self.status == "claimed") != (self.completed_at_utc is None):
            raise ValueError("only a claimed authorization may omit completed_at_utc")
        return self


class LiveTrajectoryComparison(ParallelModel):
    """Predicted/observed report aligned only on relative UTC wall time."""

    mission_id: UUID
    line: TwinLine
    vehicle_id: int = Field(ge=1, le=255)
    alignment_clock: Literal["wall_time"] = "wall_time"
    alignment_method: Literal["relative_start_linear_interpolation"] = (
        "relative_start_linear_interpolation"
    )
    predicted_started_at_utc: datetime
    observed_started_at_utc: datetime
    generated_at_utc: datetime = Field(default_factory=utc_now)
    report: TrajectoryComparisonReport

    @field_validator(
        "predicted_started_at_utc",
        "observed_started_at_utc",
        "generated_at_utc",
    )
    @classmethod
    def comparison_timestamps_are_utc(cls, value: datetime) -> datetime:
        return _as_utc(value)

    @model_validator(mode="after")
    def report_matches_mission(self) -> "LiveTrajectoryComparison":
        if (
            self.report.mission_id != self.mission_id
            or self.report.vehicle_id != self.vehicle_id
        ):
            raise ValueError("comparison report target does not match live comparison")
        return self


class TwinTelemetryRecord(ParallelModel):
    line_id: str
    role: EndpointRole
    flight_mode: str
    mission_sequence: int | None = None
    gps_fix_type: int | None = None
    gps_hdop: float | None = None
    satellites_visible: int | None = None
    mirror_state: MirrorState = MirrorState.FRESH
    received_monotonic_s: float = Field(ge=0.0)
    received_wall_time: datetime
    mirrored_monotonic_s: float = Field(ge=0.0)
    mirrored_wall_time: datetime
    state: dict[str, object]

    @field_validator("received_wall_time", "mirrored_wall_time")
    @classmethod
    def telemetry_timestamps_are_utc(cls, value: datetime) -> datetime:
        return _as_utc(value)

    @model_validator(mode="after")
    def mirror_follows_receive(self) -> "TwinTelemetryRecord":
        if self.mirrored_monotonic_s < self.received_monotonic_s:
            raise ValueError("mirrored monotonic time must not precede receive time")
        if self.mirrored_wall_time < self.received_wall_time:
            raise ValueError("mirrored wall time must not precede receive time")
        return self


class LiveMirrorStatus(ParallelModel):
    line: TwinLine
    vehicle_id: int = Field(ge=1, le=255)
    state: MirrorState = MirrorState.WAITING
    mirror_active: bool = False
    reason: str = Field(default="waiting_for_telemetry", min_length=1, max_length=256)
    transition_sequence: int = Field(default=0, ge=0)
    observed_sequence: int = Field(default=0, ge=0)
    recovery_samples: int = Field(default=0, ge=0)
    last_fresh_monotonic_s: float | None = Field(default=None, ge=0.0)
    last_fresh_wall_time: datetime | None = None
    updated_monotonic_s: float = Field(ge=0.0)
    updated_wall_time: datetime

    @field_validator("last_fresh_wall_time", "updated_wall_time")
    @classmethod
    def status_timestamps_are_utc(cls, value: datetime | None) -> datetime | None:
        return _as_utc(value) if value is not None else None

    @model_validator(mode="after")
    def active_only_when_fresh(self) -> "LiveMirrorStatus":
        if self.mirror_active != (self.state == MirrorState.FRESH):
            raise ValueError("mirror_active must match the fresh state")
        return self


class ObservedReplayRecord(ParallelModel):
    """One fresh, map-normalized observed sample in append-only order."""

    sequence: int = Field(ge=1)
    source_sequence: int = Field(ge=0)
    twin_id: str = Field(min_length=1, max_length=128)
    vehicle_id: int = Field(ge=1, le=255)
    line: TwinLine
    source: Literal[TwinSource.OBSERVED] = TwinSource.OBSERVED
    frame: Literal[CoordinateFrame.MAP] = CoordinateFrame.MAP
    frame_calibration_id: str = Field(min_length=1, max_length=128)
    frame_calibration_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    calibration_status: CalibrationStatus = CalibrationStatus.DRAFT
    preview_only: bool = True
    position: Vector3
    attitude: Quaternion | None = None
    mode: str = Field(min_length=1, max_length=64)
    task_phase: str = Field(min_length=1, max_length=64)
    wall_time: datetime
    sim_time: float | None = Field(default=None, ge=0.0)
    monotonic_time_s: float = Field(ge=0.0)
    mirrored_monotonic_s: float = Field(ge=0.0)
    mirrored_wall_time: datetime
    state: TwinState

    @field_validator("wall_time", "mirrored_wall_time")
    @classmethod
    def replay_timestamps_are_utc(cls, value: datetime) -> datetime:
        return _as_utc(value)

    @model_validator(mode="after")
    def replay_matches_state(self) -> "ObservedReplayRecord":
        if self.state.source != TwinSource.OBSERVED:
            raise ValueError("observed replay requires an observed TwinState")
        if (
            self.state.twin_id != self.twin_id
            or self.state.vehicle_id != self.vehicle_id
            or self.state.line != self.line
            or self.state.frame != self.frame
            or self.state.position != self.position
        ):
            raise ValueError("observed replay metadata does not match TwinState")
        if self.mirrored_monotonic_s < self.monotonic_time_s:
            raise ValueError("mirrored monotonic time must not precede receive time")
        if self.mirrored_wall_time < self.wall_time:
            raise ValueError("mirrored wall time must not precede receive time")
        if self.preview_only != (self.calibration_status != CalibrationStatus.SURVEYED):
            raise ValueError("preview_only must match calibration status")
        return self


__all__ = [
    "PARALLEL_DOMAIN_SCHEMA_VERSION",
    "AdapterKind",
    "AdapterTelemetry",
    "DispatchAuthorizationUse",
    "ExecutionSemantic",
    "LiveFaultCommand",
    "LiveFaultType",
    "LiveTrajectoryComparison",
    "LiveMirrorStatus",
    "MirrorState",
    "ObservedReplayRecord",
    "ApprovalReceipt",
    "ApprovalCommand",
    "ApprovalRequest",
    "RealDispatchRequest",
    "SafetyActionRecord",
    "DispatchRecord",
    "EndpointRole",
    "GateCheck",
    "GateDecision",
    "MissionAction",
    "MissionPlan",
    "MissionStep",
    "SafetyPolicy",
    "TwinTelemetryRecord",
    "canonical_hash",
    "utc_now",
]
