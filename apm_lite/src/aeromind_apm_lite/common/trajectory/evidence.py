"""Versioned trajectory evidence and deterministic sim-to-real comparison."""

from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
import threading
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Literal
from uuid import UUID, uuid4

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from aeromind_apm_lite.common.coordinates import CalibrationStatus, Wgs84Position


TRAJECTORY_SCHEMA_VERSION = "1.0"


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _as_utc(value: datetime) -> datetime:
    if value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("timestamp must include a timezone")
    return value.astimezone(timezone.utc)


def _canonical_hash(payload: Any) -> str:
    encoded = json.dumps(
        payload,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class TrajectoryModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)


class TrajectoryRole(str, Enum):
    PLANNED = "planned"
    PREDICTED = "predicted"
    OBSERVED = "observed"


class TrajectorySource(str, Enum):
    MISSION_PLAN = "mission_plan"
    AIRSIM = "airsim"
    SITL = "sitl"
    REAL_TELEMETRY = "real_telemetry"
    GPS_LOG_REPLAY = "gps_log_replay"


class GpsQuality(TrajectoryModel):
    fix_type: int | None = Field(default=None, ge=0, le=6)
    hdop: float | None = Field(default=None, ge=0.0, le=100.0)
    satellites_visible: int | None = Field(default=None, ge=0, le=255)


class ArtifactVersions(TrajectoryModel):
    """Versions needed to reproduce one trajectory source."""

    mission_plan_hash: str | None = Field(
        default=None,
        pattern=r"^[0-9a-f]{64}$",
    )
    software_commit: str | None = Field(default=None, min_length=1, max_length=128)
    arducopter_version: str | None = Field(default=None, min_length=1, max_length=128)
    parameter_hash: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    scene_hash: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")

    @property
    def missing_fields(self) -> tuple[str, ...]:
        return tuple(
            name
            for name in (
                "mission_plan_hash",
                "software_commit",
                "arducopter_version",
                "parameter_hash",
                "scene_hash",
            )
            if getattr(self, name) is None
        )


class TrajectorySample(TrajectoryModel):
    time_from_start_ms: int = Field(ge=0, le=86_400_000)
    position_map_m: tuple[float, float, float]
    phase: str | None = Field(default=None, min_length=1, max_length=64)
    observed_at_utc: datetime | None = None
    source_wgs84: Wgs84Position | None = None
    gps_quality: GpsQuality | None = None

    @field_validator("observed_at_utc")
    @classmethod
    def observed_timestamp_is_utc(
        cls,
        value: datetime | None,
    ) -> datetime | None:
        return _as_utc(value) if value is not None else None


class TrajectoryEvidence(TrajectoryModel):
    """One immutable planned, predicted or observed trajectory in map metres."""

    schema_version: Literal["1.0"] = TRAJECTORY_SCHEMA_VERSION
    evidence_id: UUID = Field(default_factory=uuid4)
    mission_id: UUID
    vehicle_id: int = Field(ge=1, le=255)
    role: TrajectoryRole
    source: TrajectorySource
    coordinate_frame: Literal["map"] = "map"
    frame_calibration_id: str = Field(
        min_length=1,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9._-]*$",
    )
    frame_calibration_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    calibration_status: CalibrationStatus
    preview_only: bool = True
    producer_version: str = Field(min_length=1, max_length=128)
    created_at_utc: datetime = Field(default_factory=utc_now)
    started_at_utc: datetime | None = None
    versions: ArtifactVersions = Field(default_factory=ArtifactVersions)
    source_metadata: dict[str, str | int | float | bool | None] = Field(
        default_factory=dict
    )
    samples: tuple[TrajectorySample, ...] = Field(min_length=2, max_length=200_000)

    _created_at_is_utc = field_validator("created_at_utc")(_as_utc)

    @field_validator("started_at_utc")
    @classmethod
    def started_timestamp_is_utc(
        cls,
        value: datetime | None,
    ) -> datetime | None:
        return _as_utc(value) if value is not None else None

    @model_validator(mode="after")
    def validate_evidence(self) -> "TrajectoryEvidence":
        times = [sample.time_from_start_ms for sample in self.samples]
        if any(current <= previous for previous, current in zip(times, times[1:])):
            raise ValueError("trajectory sample times must be strictly increasing")
        timestamps = [sample.observed_at_utc for sample in self.samples]
        if any(value is not None for value in timestamps):
            if any(value is None for value in timestamps):
                raise ValueError("observed_at_utc must be present on every sample or none")
            actual = [value for value in timestamps if value is not None]
            if any(current <= previous for previous, current in zip(actual, actual[1:])):
                raise ValueError("trajectory observed timestamps must be strictly increasing")
        if self.calibration_status != CalibrationStatus.SURVEYED and not self.preview_only:
            raise ValueError("draft calibration evidence must remain preview_only")
        return self

    @property
    def evidence_hash(self) -> str:
        return _canonical_hash(self.model_dump(mode="json"))

    def public_payload(self) -> dict[str, Any]:
        return {
            "evidence_hash": self.evidence_hash,
            "evidence": self.model_dump(mode="json"),
        }


class TrajectoryThresholds(TrajectoryModel):
    horizontal_rmse_m: float = Field(default=3.0, gt=0.0, le=100.0)
    vertical_rmse_m: float = Field(default=2.0, gt=0.0, le=100.0)
    maximum_3d_error_m: float = Field(default=8.0, gt=0.0, le=200.0)
    endpoint_3d_error_m: float = Field(default=5.0, gt=0.0, le=200.0)
    sample_period_ms: int = Field(default=200, ge=20, le=10_000)
    validated_for_venue: bool = False


class TrajectoryPairMetrics(TrajectoryModel):
    reference_role: TrajectoryRole
    compared_role: TrajectoryRole
    sample_count: int = Field(ge=2)
    overlap_start_ms: int = Field(ge=0)
    overlap_end_ms: int = Field(ge=0)
    horizontal_rmse_m: float = Field(ge=0.0)
    vertical_rmse_m: float = Field(ge=0.0)
    three_dimensional_rmse_m: float = Field(ge=0.0)
    maximum_3d_error_m: float = Field(ge=0.0)
    p95_3d_error_m: float = Field(ge=0.0)
    endpoint_3d_error_m: float = Field(ge=0.0)
    within_thresholds: bool


class TrajectoryComparisonReport(TrajectoryModel):
    schema_version: Literal["1.0"] = TRAJECTORY_SCHEMA_VERSION
    report_id: UUID = Field(default_factory=uuid4)
    generated_at_utc: datetime = Field(default_factory=utc_now)
    mission_id: UUID
    vehicle_id: int = Field(ge=1, le=255)
    frame_calibration_id: str = Field(min_length=1, max_length=128)
    frame_calibration_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    evidence_hashes: dict[TrajectoryRole, str]
    thresholds: TrajectoryThresholds
    comparisons: tuple[TrajectoryPairMetrics, ...]
    ready: bool
    preview_only: bool
    metrics_within_thresholds: bool
    acceptance_passed: bool | None
    limitations: tuple[str, ...] = ()

    _generated_at_is_utc = field_validator("generated_at_utc")(_as_utc)

    @property
    def report_hash(self) -> str:
        return _canonical_hash(self.model_dump(mode="json"))

    def public_payload(self) -> dict[str, Any]:
        return {
            "report_hash": self.report_hash,
            "report": self.model_dump(mode="json"),
        }


class TrajectoryEvidenceError(ValueError):
    pass


class TrajectoryEvidenceStore:
    """Atomically persist immutable evidence envelopes by evidence UUID."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self._lock = threading.RLock()

    def _path(self, evidence_id: UUID) -> Path:
        return self.root / f"{evidence_id}.json"

    def save(self, evidence: TrajectoryEvidence) -> TrajectoryEvidence:
        with self._lock:
            path = self._path(evidence.evidence_id)
            if path.exists():
                existing = self.load(evidence.evidence_id)
                if existing.evidence_hash != evidence.evidence_hash:
                    raise TrajectoryEvidenceError(
                        "evidence_id is immutable and already contains different data"
                    )
                return existing
            _atomic_json_write(path, evidence.public_payload())
            return evidence

    def load(self, evidence_id: UUID | str) -> TrajectoryEvidence:
        with self._lock:
            try:
                parsed_id = UUID(str(evidence_id))
                payload = json.loads(self._path(parsed_id).read_text(encoding="utf-8"))
                evidence = TrajectoryEvidence.model_validate(payload["evidence"])
            except (OSError, ValueError, KeyError, TypeError) as exc:
                raise TrajectoryEvidenceError(
                    f"failed to load trajectory evidence {evidence_id}: {exc}"
                ) from exc
            if payload.get("evidence_hash") != evidence.evidence_hash:
                raise TrajectoryEvidenceError("trajectory evidence hash mismatch")
            return evidence

    def list_summaries(self) -> list[dict[str, Any]]:
        if not self.root.is_dir():
            return []
        summaries = []
        for path in sorted(self.root.glob("*.json")):
            evidence = self.load(path.stem)
            summaries.append(
                {
                    "evidence_id": str(evidence.evidence_id),
                    "evidence_hash": evidence.evidence_hash,
                    "mission_id": str(evidence.mission_id),
                    "vehicle_id": evidence.vehicle_id,
                    "role": evidence.role.value,
                    "source": evidence.source.value,
                    "sample_count": len(evidence.samples),
                    "preview_only": evidence.preview_only,
                    "created_at_utc": evidence.created_at_utc.isoformat(),
                }
            )
        return summaries


def compare_trajectories(
    evidence_items: list[TrajectoryEvidence] | tuple[TrajectoryEvidence, ...],
    thresholds: TrajectoryThresholds | None = None,
) -> TrajectoryComparisonReport:
    if len(evidence_items) < 2:
        raise TrajectoryEvidenceError("at least two trajectory evidence items are required")
    by_role = {evidence.role: evidence for evidence in evidence_items}
    if len(by_role) != len(evidence_items):
        raise TrajectoryEvidenceError("trajectory roles must be unique within one report")

    first = evidence_items[0]
    for evidence in evidence_items[1:]:
        if evidence.mission_id != first.mission_id:
            raise TrajectoryEvidenceError("trajectory mission_id values do not match")
        if evidence.vehicle_id != first.vehicle_id:
            raise TrajectoryEvidenceError("trajectory vehicle_id values do not match")
        if evidence.frame_calibration_id != first.frame_calibration_id:
            raise TrajectoryEvidenceError("trajectory frame_calibration_id values do not match")
        if evidence.frame_calibration_hash != first.frame_calibration_hash:
            raise TrajectoryEvidenceError("trajectory calibration hashes do not match")

    limits = thresholds or TrajectoryThresholds()
    role_pairs = (
        (TrajectoryRole.PLANNED, TrajectoryRole.PREDICTED),
        (TrajectoryRole.PLANNED, TrajectoryRole.OBSERVED),
        (TrajectoryRole.PREDICTED, TrajectoryRole.OBSERVED),
    )
    comparisons = tuple(
        _compare_pair(by_role[left], by_role[right], limits)
        for left, right in role_pairs
        if left in by_role and right in by_role
    )
    required_roles = set(TrajectoryRole)
    missing_roles = sorted(role.value for role in required_roles - set(by_role))
    ready = not missing_roles and len(comparisons) == 3
    missing_versions = sorted(
        {
            field
            for evidence in evidence_items
            for field in evidence.versions.missing_fields
        }
    )
    preview_only = (
        not limits.validated_for_venue
        or any(evidence.preview_only for evidence in evidence_items)
        or any(
            evidence.calibration_status != CalibrationStatus.SURVEYED
            for evidence in evidence_items
        )
        or bool(missing_versions)
    )
    metrics_within_thresholds = all(item.within_thresholds for item in comparisons)
    limitations: list[str] = []
    if missing_roles:
        limitations.append("missing trajectory roles: " + ", ".join(missing_roles))
    if not limits.validated_for_venue:
        limitations.append("acceptance thresholds are provisional and not venue-validated")
    if any(evidence.preview_only for evidence in evidence_items):
        limitations.append("one or more evidence items are preview_only")
    if any(
        evidence.calibration_status != CalibrationStatus.SURVEYED
        for evidence in evidence_items
    ):
        limitations.append("one or more evidence items use a draft calibration")
    if missing_versions:
        limitations.append("missing reproduction metadata: " + ", ".join(missing_versions))
    acceptance_passed = (
        metrics_within_thresholds if ready and not preview_only else None
    )
    return TrajectoryComparisonReport(
        mission_id=first.mission_id,
        vehicle_id=first.vehicle_id,
        frame_calibration_id=first.frame_calibration_id,
        frame_calibration_hash=first.frame_calibration_hash,
        evidence_hashes={
            role: evidence.evidence_hash for role, evidence in by_role.items()
        },
        thresholds=limits,
        comparisons=comparisons,
        ready=ready,
        preview_only=preview_only,
        metrics_within_thresholds=metrics_within_thresholds,
        acceptance_passed=acceptance_passed,
        limitations=tuple(limitations),
    )


def load_evidence_file(path: str | Path) -> TrajectoryEvidence:
    try:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if isinstance(payload, dict) and "evidence" in payload:
            expected_hash = payload.get("evidence_hash")
            payload = payload["evidence"]
        else:
            expected_hash = None
        evidence = TrajectoryEvidence.model_validate(payload)
    except (OSError, ValueError, TypeError) as exc:
        raise TrajectoryEvidenceError(f"failed to read evidence {path}: {exc}") from exc
    if expected_hash is not None and expected_hash != evidence.evidence_hash:
        raise TrajectoryEvidenceError(f"evidence hash mismatch in {path}")
    return evidence


def write_evidence_file(path: str | Path, evidence: TrajectoryEvidence) -> None:
    _atomic_json_write(Path(path), evidence.public_payload())


def write_report_file(path: str | Path, report: TrajectoryComparisonReport) -> None:
    _atomic_json_write(Path(path), report.public_payload())


def _compare_pair(
    reference: TrajectoryEvidence,
    compared: TrajectoryEvidence,
    thresholds: TrajectoryThresholds,
) -> TrajectoryPairMetrics:
    overlap_start = max(
        reference.samples[0].time_from_start_ms,
        compared.samples[0].time_from_start_ms,
    )
    overlap_end = min(
        reference.samples[-1].time_from_start_ms,
        compared.samples[-1].time_from_start_ms,
    )
    if overlap_end <= overlap_start:
        raise TrajectoryEvidenceError(
            f"{reference.role.value}/{compared.role.value} trajectories do not overlap"
        )
    times = list(range(overlap_start, overlap_end + 1, thresholds.sample_period_ms))
    if not times or times[-1] != overlap_end:
        times.append(overlap_end)
    if len(times) < 2:
        times = [overlap_start, overlap_end]

    horizontal_errors: list[float] = []
    vertical_errors: list[float] = []
    errors_3d: list[float] = []
    for current_time in times:
        left = _interpolate(reference.samples, current_time)
        right = _interpolate(compared.samples, current_time)
        dx = right[0] - left[0]
        dy = right[1] - left[1]
        dz = right[2] - left[2]
        horizontal_errors.append(math.hypot(dx, dy))
        vertical_errors.append(abs(dz))
        errors_3d.append(math.sqrt(dx * dx + dy * dy + dz * dz))

    horizontal_rmse = _rmse(horizontal_errors)
    vertical_rmse = _rmse(vertical_errors)
    rmse_3d = _rmse(errors_3d)
    maximum_3d = max(errors_3d)
    endpoint_3d = errors_3d[-1]
    within = (
        horizontal_rmse <= thresholds.horizontal_rmse_m
        and vertical_rmse <= thresholds.vertical_rmse_m
        and maximum_3d <= thresholds.maximum_3d_error_m
        and endpoint_3d <= thresholds.endpoint_3d_error_m
    )
    return TrajectoryPairMetrics(
        reference_role=reference.role,
        compared_role=compared.role,
        sample_count=len(times),
        overlap_start_ms=overlap_start,
        overlap_end_ms=overlap_end,
        horizontal_rmse_m=horizontal_rmse,
        vertical_rmse_m=vertical_rmse,
        three_dimensional_rmse_m=rmse_3d,
        maximum_3d_error_m=maximum_3d,
        p95_3d_error_m=_percentile(errors_3d, 0.95),
        endpoint_3d_error_m=endpoint_3d,
        within_thresholds=within,
    )


def _interpolate(
    samples: tuple[TrajectorySample, ...],
    target_time_ms: int,
) -> tuple[float, float, float]:
    if target_time_ms <= samples[0].time_from_start_ms:
        return samples[0].position_map_m
    for left, right in zip(samples, samples[1:]):
        if target_time_ms <= right.time_from_start_ms:
            duration = right.time_from_start_ms - left.time_from_start_ms
            ratio = (target_time_ms - left.time_from_start_ms) / duration
            return tuple(
                left_value + ratio * (right_value - left_value)
                for left_value, right_value in zip(
                    left.position_map_m,
                    right.position_map_m,
                )
            )
    return samples[-1].position_map_m


def _rmse(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def _atomic_json_write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temporary_path = Path(temporary.name)
            json.dump(payload, temporary, ensure_ascii=False, indent=2, sort_keys=True)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
        os.replace(temporary_path, path)
    except OSError as exc:
        raise TrajectoryEvidenceError(f"failed to write {path}: {exc}") from exc
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
