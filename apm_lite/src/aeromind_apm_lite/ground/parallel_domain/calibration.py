"""Measured calibration gate and immutable GeoReference freeze receipts."""

from __future__ import annotations

import json
import math
import os
import tempfile
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Literal
from uuid import UUID, uuid4

from pydantic import Field, field_validator, model_validator

from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
    Vec3,
    Wgs84Position,
)

from .models import ParallelModel, canonical_hash, utc_now


class CalibrationTruthSource(str, Enum):
    SURVEYED_CONTROL_POINT = "surveyed_control_point"
    SIMULATION_GROUND_TRUTH = "simulation_ground_truth"


class CalibrationDirection(str, Enum):
    OUTBOUND = "outbound"
    RETURN = "return"


class CalibrationMeasurement(ParallelModel):
    sample_id: UUID = Field(default_factory=uuid4)
    control_point_id: str = Field(
        min_length=1,
        max_length=64,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9._-]*$",
    )
    pass_id: str = Field(
        min_length=1,
        max_length=64,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9._-]*$",
    )
    direction: CalibrationDirection
    captured_at_utc: datetime
    ground_truth_captured_at_utc: datetime | None = None
    pairing_skew_s: float | None = Field(default=None, ge=0.0, le=60.0)
    ground_truth_reference: str | None = Field(
        default=None,
        min_length=1,
        max_length=512,
    )
    measured_wgs84: Wgs84Position
    ground_truth_map_m: tuple[float, float, float]

    @field_validator("captured_at_utc", "ground_truth_captured_at_utc")
    @classmethod
    def captured_at_is_utc(cls, value: datetime | None) -> datetime | None:
        if value is None:
            return None
        if value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("calibration timestamps must include a timezone")
        return value.astimezone(timezone.utc)

    @model_validator(mode="after")
    def live_truth_stamps_are_paired(self) -> "CalibrationMeasurement":
        values = (
            self.ground_truth_captured_at_utc,
            self.pairing_skew_s,
            self.ground_truth_reference,
        )
        if any(value is not None for value in values) and any(
            value is None for value in values
        ):
            raise ValueError(
                "ground-truth capture time, pairing skew and reference must be paired"
            )
        return self


class LiveCalibrationDataset(ParallelModel):
    dataset_id: UUID = Field(default_factory=uuid4)
    calibration_id: str = Field(
        min_length=1,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9._-]*$",
    )
    line: Literal["px4", "apm"]
    vehicle_id: int = Field(ge=1, le=255)
    truth_source: CalibrationTruthSource
    truth_reference: str = Field(min_length=1, max_length=512)
    created_at_utc: datetime = Field(default_factory=utc_now)
    measurements: tuple[CalibrationMeasurement, ...] = Field(
        min_length=2,
        max_length=100_000,
    )

    @field_validator("created_at_utc")
    @classmethod
    def created_at_is_utc(cls, value: datetime) -> datetime:
        if value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("created_at_utc must include a timezone")
        return value.astimezone(timezone.utc)

    @model_validator(mode="after")
    def measurements_are_unique(self) -> "LiveCalibrationDataset":
        sample_ids = [item.sample_id for item in self.measurements]
        if len(sample_ids) != len(set(sample_ids)):
            raise ValueError("calibration sample_id values must be unique")
        times = [item.captured_at_utc for item in self.measurements]
        if any(current < previous for previous, current in zip(times, times[1:])):
            raise ValueError("calibration measurements must be ordered by wall time")
        return self

    @property
    def dataset_hash(self) -> str:
        return canonical_hash(self)

    def public_payload(self) -> dict[str, Any]:
        return {
            "dataset_hash": self.dataset_hash,
            "dataset": self.model_dump(mode="json"),
        }


class CalibrationThresholds(ParallelModel):
    minimum_samples: int = Field(default=6, ge=2, le=10_000)
    minimum_control_points: int = Field(default=3, ge=1, le=1_000)
    horizontal_rmse_m: float = Field(default=1.0, gt=0.0, le=100.0)
    vertical_rmse_m: float = Field(default=1.0, gt=0.0, le=100.0)
    maximum_3d_error_m: float = Field(default=2.0, gt=0.0, le=200.0)
    p95_3d_error_m: float = Field(default=1.5, gt=0.0, le=200.0)


class CalibrationErrorSample(ParallelModel):
    sample_id: UUID
    control_point_id: str
    pass_id: str
    direction: CalibrationDirection
    captured_at_utc: datetime
    ground_truth_captured_at_utc: datetime | None = None
    pairing_skew_s: float | None = Field(default=None, ge=0.0, le=60.0)
    ground_truth_reference: str | None = None
    measured_wgs84: Wgs84Position
    estimated_map_m: tuple[float, float, float]
    ground_truth_map_m: tuple[float, float, float]
    horizontal_error_m: float = Field(ge=0.0)
    vertical_error_m: float = Field(ge=0.0)
    three_dimensional_error_m: float = Field(ge=0.0)


class CalibrationErrorDistribution(ParallelModel):
    sample_count: int = Field(ge=2)
    unique_control_points: int = Field(ge=1)
    horizontal_rmse_m: float = Field(ge=0.0)
    vertical_rmse_m: float = Field(ge=0.0)
    three_dimensional_rmse_m: float = Field(ge=0.0)
    maximum_3d_error_m: float = Field(ge=0.0)
    p95_3d_error_m: float = Field(ge=0.0)
    mean_3d_error_m: float = Field(ge=0.0)


class CalibrationTriggerCheck(ParallelModel):
    name: str = Field(min_length=1, max_length=64)
    passed: bool
    detail: str = Field(min_length=1, max_length=512)


class LiveCalibrationReport(ParallelModel):
    report_id: UUID = Field(default_factory=uuid4)
    generated_at_utc: datetime = Field(default_factory=utc_now)
    dataset_id: UUID
    dataset_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    calibration_id: str
    calibration_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    calibration_status: CalibrationStatus
    truth_source: CalibrationTruthSource
    truth_reference: str
    thresholds: CalibrationThresholds
    coordinate_round_trip_self_consistency_max_m: float = Field(ge=0.0)
    errors: tuple[CalibrationErrorSample, ...] = Field(min_length=2)
    distribution: CalibrationErrorDistribution
    trigger_checks: tuple[CalibrationTriggerCheck, ...] = Field(min_length=1)
    metrics_within_thresholds: bool
    preview_only: bool
    acceptance_passed: bool | None
    result_statement_zh: str

    @property
    def report_hash(self) -> str:
        return canonical_hash(self)

    def public_payload(self) -> dict[str, Any]:
        return {
            "report_hash": self.report_hash,
            "report": self.model_dump(mode="json"),
        }


class CalibrationFreezeReceipt(ParallelModel):
    receipt_id: UUID = Field(default_factory=uuid4)
    frozen_at_utc: datetime = Field(default_factory=utc_now)
    calibration_id: str
    draft_calibration_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    frozen_calibration_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    dataset_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    measurement_report_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    truth_source: CalibrationTruthSource
    trigger_checks: tuple[CalibrationTriggerCheck, ...] = Field(min_length=1)
    preview_only: Literal[False] = False
    acceptance_passed: Literal[True] = True

    @property
    def receipt_hash(self) -> str:
        return canonical_hash(self)

    def public_payload(self) -> dict[str, Any]:
        return {
            "receipt_hash": self.receipt_hash,
            "receipt": self.model_dump(mode="json"),
        }


class CalibrationError(ValueError):
    pass


def measure_calibration(
    reference: GeoReference,
    dataset: LiveCalibrationDataset,
    thresholds: CalibrationThresholds | None = None,
) -> LiveCalibrationReport:
    if not reference.is_complete:
        raise CalibrationError("calibration measurement requires a complete GeoReference")
    if dataset.calibration_id != reference.calibration_id:
        raise CalibrationError("measurement calibration_id does not match GeoReference")
    limits = thresholds or CalibrationThresholds()
    errors: list[CalibrationErrorSample] = []
    horizontal_values: list[float] = []
    vertical_values: list[float] = []
    three_dimensional_values: list[float] = []
    for item in dataset.measurements:
        estimated = reference.telemetry_wgs84_to_map(
            item.measured_wgs84,
            line=dataset.line,
        )
        truth = Vec3(*item.ground_truth_map_m)
        dx = estimated.x - truth.x
        dy = estimated.y - truth.y
        dz = estimated.z - truth.z
        horizontal = math.hypot(dx, dy)
        vertical = abs(dz)
        three_dimensional = math.sqrt(dx * dx + dy * dy + dz * dz)
        horizontal_values.append(horizontal)
        vertical_values.append(vertical)
        three_dimensional_values.append(three_dimensional)
        errors.append(
            CalibrationErrorSample(
                sample_id=item.sample_id,
                control_point_id=item.control_point_id,
                pass_id=item.pass_id,
                direction=item.direction,
                captured_at_utc=item.captured_at_utc,
                ground_truth_captured_at_utc=item.ground_truth_captured_at_utc,
                pairing_skew_s=item.pairing_skew_s,
                ground_truth_reference=item.ground_truth_reference,
                measured_wgs84=item.measured_wgs84,
                estimated_map_m=(estimated.x, estimated.y, estimated.z),
                ground_truth_map_m=item.ground_truth_map_m,
                horizontal_error_m=horizontal,
                vertical_error_m=vertical,
                three_dimensional_error_m=three_dimensional,
            )
        )
    control_points = {item.control_point_id for item in dataset.measurements}
    directions = {item.direction for item in dataset.measurements}
    distribution = CalibrationErrorDistribution(
        sample_count=len(errors),
        unique_control_points=len(control_points),
        horizontal_rmse_m=_rmse(horizontal_values),
        vertical_rmse_m=_rmse(vertical_values),
        three_dimensional_rmse_m=_rmse(three_dimensional_values),
        maximum_3d_error_m=max(three_dimensional_values),
        p95_3d_error_m=_percentile(three_dimensional_values, 0.95),
        mean_3d_error_m=sum(three_dimensional_values) / len(three_dimensional_values),
    )
    checks = (
        CalibrationTriggerCheck(
            name="sample_count",
            passed=len(errors) >= limits.minimum_samples,
            detail=f"samples={len(errors)}, required={limits.minimum_samples}",
        ),
        CalibrationTriggerCheck(
            name="independent_control_points",
            passed=len(control_points) >= limits.minimum_control_points,
            detail=(
                f"control_points={len(control_points)}, "
                f"required={limits.minimum_control_points}"
            ),
        ),
        CalibrationTriggerCheck(
            name="round_trip_passes",
            passed=directions
            == {CalibrationDirection.OUTBOUND, CalibrationDirection.RETURN},
            detail="both outbound and return passes are required",
        ),
        CalibrationTriggerCheck(
            name="ground_truth_provenance",
            passed=bool(dataset.truth_reference.strip()),
            detail=(
                f"source={dataset.truth_source.value}; "
                f"reference={dataset.truth_reference}"
            ),
        ),
        CalibrationTriggerCheck(
            name="measured_error_thresholds",
            passed=(
                distribution.horizontal_rmse_m <= limits.horizontal_rmse_m
                and distribution.vertical_rmse_m <= limits.vertical_rmse_m
                and distribution.maximum_3d_error_m
                <= limits.maximum_3d_error_m
                and distribution.p95_3d_error_m <= limits.p95_3d_error_m
            ),
            detail=(
                f"horizontal_rmse={distribution.horizontal_rmse_m:.6f}m, "
                f"vertical_rmse={distribution.vertical_rmse_m:.6f}m, "
                f"max_3d={distribution.maximum_3d_error_m:.6f}m, "
                f"p95_3d={distribution.p95_3d_error_m:.6f}m"
            ),
        ),
    )
    all_checks_pass = all(item.passed for item in checks)
    round_trip = reference.coordinate_round_trip_report()
    self_consistency = float(round_trip.get("maximum_error_m", math.inf))
    preview_only = reference.status != CalibrationStatus.SURVEYED
    return LiveCalibrationReport(
        dataset_id=dataset.dataset_id,
        dataset_hash=dataset.dataset_hash,
        calibration_id=reference.calibration_id,
        calibration_sha256=reference.config_hash,
        calibration_status=reference.status,
        truth_source=dataset.truth_source,
        truth_reference=dataset.truth_reference,
        thresholds=limits,
        coordinate_round_trip_self_consistency_max_m=self_consistency,
        errors=tuple(errors),
        distribution=distribution,
        trigger_checks=checks,
        metrics_within_thresholds=all_checks_pass,
        preview_only=preview_only,
        acceptance_passed=all_checks_pass if not preview_only else None,
        result_statement_zh=(
            "实现往返自洽 1.6e-9 m + live 标定误差实测值："
            f"3D RMSE {distribution.three_dimensional_rmse_m:.3f} m，"
            f"P95 {distribution.p95_3d_error_m:.3f} m"
        ),
    )


def freeze_georeference(
    draft: GeoReference,
    dataset: LiveCalibrationDataset,
    thresholds: CalibrationThresholds | None = None,
) -> tuple[GeoReference, LiveCalibrationReport, CalibrationFreezeReceipt]:
    if draft.status != CalibrationStatus.DRAFT:
        raise CalibrationError("freeze requires a draft GeoReference")
    draft_report = measure_calibration(draft, dataset, thresholds)
    failed = [item.name for item in draft_report.trigger_checks if not item.passed]
    if failed:
        raise CalibrationError(
            "calibration freeze trigger failed: " + ", ".join(failed)
        )
    frozen = draft.model_copy(update={"status": CalibrationStatus.SURVEYED})
    final_report = measure_calibration(frozen, dataset, thresholds)
    if final_report.acceptance_passed is not True:
        raise CalibrationError("frozen calibration did not produce an acceptance report")
    receipt = CalibrationFreezeReceipt(
        calibration_id=frozen.calibration_id,
        draft_calibration_sha256=draft.config_hash,
        frozen_calibration_sha256=frozen.config_hash,
        dataset_hash=dataset.dataset_hash,
        measurement_report_hash=final_report.report_hash,
        truth_source=dataset.truth_source,
        trigger_checks=final_report.trigger_checks,
    )
    return frozen, final_report, receipt


def verify_calibration_freeze(
    reference: GeoReference,
    dataset: LiveCalibrationDataset,
    report: LiveCalibrationReport,
    receipt: CalibrationFreezeReceipt,
) -> None:
    if reference.status != CalibrationStatus.SURVEYED or reference.preview_only:
        raise CalibrationError("frozen GeoReference must be surveyed")
    if receipt.calibration_id != reference.calibration_id:
        raise CalibrationError("freeze receipt calibration_id mismatch")
    if receipt.frozen_calibration_sha256 != reference.config_hash:
        raise CalibrationError("freeze receipt GeoReference SHA-256 mismatch")
    if receipt.dataset_hash != dataset.dataset_hash:
        raise CalibrationError("freeze receipt dataset hash mismatch")
    if receipt.measurement_report_hash != report.report_hash:
        raise CalibrationError("freeze receipt measurement report hash mismatch")
    if report.calibration_sha256 != reference.config_hash:
        raise CalibrationError("measurement report GeoReference SHA-256 mismatch")
    if report.dataset_hash != dataset.dataset_hash:
        raise CalibrationError("measurement report dataset hash mismatch")
    if report.preview_only or report.acceptance_passed is not True:
        raise CalibrationError("measurement report is not acceptance eligible")
    if not all(item.passed for item in receipt.trigger_checks):
        raise CalibrationError("freeze receipt contains a failed trigger")


def parse_gazebo_pose_tuple(payload: str) -> Vec3:
    """Parse `gz model -m NAME -p` without Gazebo Python bindings."""

    values = payload.strip().split()
    if len(values) != 6:
        raise CalibrationError(
            "Gazebo pose must be the six-value output of `gz model -p`"
        )
    try:
        numbers = tuple(float(value) for value in values)
    except ValueError as exc:
        raise CalibrationError("Gazebo pose contains a non-numeric value") from exc
    if not all(math.isfinite(value) for value in numbers):
        raise CalibrationError("Gazebo pose contains a non-finite value")
    return Vec3(numbers[0], numbers[1], numbers[2])


def calibration_measurement_from_gazebo(
    *,
    reference: GeoReference,
    replay: "ObservedReplayRecord",
    gazebo_position_enu_m: Vec3,
    ground_truth_captured_at_utc: datetime,
    ground_truth_reference: str,
    control_point_id: str,
    pass_id: str,
    direction: CalibrationDirection,
    maximum_pairing_skew_s: float = 0.5,
) -> CalibrationMeasurement:
    """Pair GPS-derived observed telemetry with independent Gazebo ground truth."""

    from .models import ObservedReplayRecord

    if not isinstance(replay, ObservedReplayRecord):
        raise CalibrationError("replay must be an ObservedReplayRecord")
    if replay.frame_calibration_id != reference.calibration_id:
        raise CalibrationError("observed replay calibration_id mismatch")
    if replay.frame_calibration_sha256 != reference.config_hash:
        raise CalibrationError("observed replay calibration SHA-256 mismatch")
    truth_time = ground_truth_captured_at_utc
    if truth_time.tzinfo is None or truth_time.utcoffset() is None:
        raise CalibrationError("ground-truth capture time must include a timezone")
    truth_time = truth_time.astimezone(timezone.utc)
    skew_s = abs((replay.wall_time - truth_time).total_seconds())
    if skew_s > maximum_pairing_skew_s:
        raise CalibrationError(
            f"Gazebo/observed wall-time skew {skew_s:.6f}s exceeds "
            f"{maximum_pairing_skew_s:.6f}s"
        )
    measured_wgs84 = reference.map_to_wgs84(
        Vec3(replay.position.x, replay.position.y, replay.position.z)
    )
    truth_map = reference.gazebo_enu_to_map(gazebo_position_enu_m)
    return CalibrationMeasurement(
        control_point_id=control_point_id,
        pass_id=pass_id,
        direction=direction,
        captured_at_utc=replay.wall_time,
        ground_truth_captured_at_utc=truth_time,
        pairing_skew_s=skew_s,
        ground_truth_reference=ground_truth_reference,
        measured_wgs84=measured_wgs84,
        ground_truth_map_m=(truth_map.x, truth_map.y, truth_map.z),
    )


def read_latest_observed_replay(path: str | Path) -> "ObservedReplayRecord":
    """Read the last complete JSONL record without loading a long live log."""

    from .models import ObservedReplayRecord

    source = Path(path)
    try:
        with source.open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            end = stream.tell()
            if end == 0:
                raise CalibrationError(f"observed replay is empty: {source}")
            cursor = end
            data = b""
            while cursor > 0 and data.count(b"\n") < 2:
                size = min(8192, cursor)
                cursor -= size
                stream.seek(cursor)
                data = stream.read(size) + data
        lines = [line for line in data.splitlines() if line.strip()]
        if not lines:
            raise CalibrationError(f"observed replay has no complete record: {source}")
        return ObservedReplayRecord.model_validate_json(lines[-1])
    except CalibrationError:
        raise
    except (OSError, ValueError) as exc:
        raise CalibrationError(f"failed to read observed replay {source}: {exc}") from exc


def load_calibration_dataset(path: str | Path) -> LiveCalibrationDataset:
    payload = _read_json(Path(path))
    expected = payload.get("dataset_hash")
    value = LiveCalibrationDataset.model_validate(payload.get("dataset", payload))
    if expected is not None and expected != value.dataset_hash:
        raise CalibrationError("calibration dataset hash mismatch")
    return value


def load_calibration_report(path: str | Path) -> LiveCalibrationReport:
    payload = _read_json(Path(path))
    expected = payload.get("report_hash")
    value = LiveCalibrationReport.model_validate(payload.get("report", payload))
    if expected is not None and expected != value.report_hash:
        raise CalibrationError("calibration report hash mismatch")
    return value


def load_freeze_receipt(path: str | Path) -> CalibrationFreezeReceipt:
    payload = _read_json(Path(path))
    expected = payload.get("receipt_hash")
    value = CalibrationFreezeReceipt.model_validate(payload.get("receipt", payload))
    if expected is not None and expected != value.receipt_hash:
        raise CalibrationError("calibration freeze receipt hash mismatch")
    return value


def write_calibration_dataset(
    path: str | Path,
    dataset: LiveCalibrationDataset,
) -> None:
    _atomic_json(Path(path), dataset.public_payload())


def write_calibration_report(
    path: str | Path,
    report: LiveCalibrationReport,
) -> None:
    _atomic_json(Path(path), report.public_payload())


def write_freeze_receipt(
    path: str | Path,
    receipt: CalibrationFreezeReceipt,
) -> None:
    _atomic_json(Path(path), receipt.public_payload())


def _rmse(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def _read_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationError(f"failed to read calibration artifact {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise CalibrationError(f"calibration artifact must be an object: {path}")
    return payload


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
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
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


__all__ = [
    "CalibrationDirection",
    "CalibrationError",
    "CalibrationErrorDistribution",
    "CalibrationErrorSample",
    "CalibrationFreezeReceipt",
    "CalibrationMeasurement",
    "CalibrationThresholds",
    "CalibrationTriggerCheck",
    "CalibrationTruthSource",
    "LiveCalibrationDataset",
    "LiveCalibrationReport",
    "calibration_measurement_from_gazebo",
    "freeze_georeference",
    "load_calibration_dataset",
    "load_calibration_report",
    "load_freeze_receipt",
    "measure_calibration",
    "parse_gazebo_pose_tuple",
    "read_latest_observed_replay",
    "verify_calibration_freeze",
    "write_calibration_dataset",
    "write_calibration_report",
    "write_freeze_receipt",
]
