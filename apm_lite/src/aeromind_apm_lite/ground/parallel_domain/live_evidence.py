"""Self-verifying live planned/predicted/observed evidence packages."""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
import zipfile
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, Literal
from uuid import UUID

import yaml
from pydantic import Field, field_validator, model_validator

from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
)
from aeromind_apm_lite.common.trajectory import (
    TrajectoryComparisonReport,
    TrajectoryEvidence,
    TrajectoryPairMetrics,
    TrajectoryRole,
    TrajectorySource,
    TrajectoryThresholds,
    compare_trajectories,
    load_evidence_file,
)

from .calibration import (
    CalibrationFreezeReceipt,
    LiveCalibrationDataset,
    LiveCalibrationReport,
    verify_calibration_freeze,
)
from .models import MissionPlan, ObservedReplayRecord, ParallelModel, canonical_hash, utc_now


REQUIRED_LIVE_VERSIONS = frozenset(
    {
        "aeromind_apm_lite",
        "bridge",
        "flight_controller",
        "simulator",
        "world",
    }
)


class LiveEvidenceArtifact(ParallelModel):
    name: str = Field(
        min_length=1,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9._-]*$",
    )
    role: str = Field(
        min_length=1,
        max_length=64,
        pattern=r"^[a-z][a-z0-9_-]*$",
    )
    relative_path: str = Field(min_length=1, max_length=256)
    media_type: str = Field(min_length=1, max_length=128)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    size_bytes: int = Field(ge=0)

    @field_validator("relative_path")
    @classmethod
    def path_is_safe(cls, value: str) -> str:
        path = PurePosixPath(value)
        if path.is_absolute() or ".." in path.parts or path.parts[0] != "artifacts":
            raise ValueError("live evidence artifact path must stay under artifacts/")
        return value


class LiveEvidenceTimeRange(ParallelModel):
    started_at_utc: datetime
    ended_at_utc: datetime

    @field_validator("started_at_utc", "ended_at_utc")
    @classmethod
    def timestamps_are_utc(cls, value: datetime) -> datetime:
        if value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("evidence time range must include a timezone")
        return value.astimezone(timezone.utc)

    @model_validator(mode="after")
    def end_follows_start(self) -> "LiveEvidenceTimeRange":
        if self.ended_at_utc < self.started_at_utc:
            raise ValueError("evidence time range end precedes start")
        return self


class LiveEvidenceManifest(ParallelModel):
    package_kind: Literal["parallel_domain_live_trajectory"] = (
        "parallel_domain_live_trajectory"
    )
    mission_id: UUID
    line: Literal["px4", "apm"]
    vehicle_id: int = Field(ge=1, le=255)
    coordinate_frame: Literal["map"] = "map"
    created_at_utc: datetime = Field(default_factory=utc_now)
    time_range_utc: LiveEvidenceTimeRange
    versions: dict[str, str]
    software_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    software_hash_scope: str = Field(min_length=1, max_length=512)
    software_commit: str | None = Field(default=None, min_length=1, max_length=128)
    random_seed: int
    frame_calibration_id: str
    frame_calibration_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    calibration_receipt_hash: str | None = Field(
        default=None,
        pattern=r"^[0-9a-f]{64}$",
    )
    evidence_hashes: dict[TrajectoryRole, str]
    comparison_report_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    preview_only: bool
    acceptance_passed: bool | None
    limitations: tuple[str, ...] = ()
    artifacts: tuple[LiveEvidenceArtifact, ...] = Field(min_length=1)
    content_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")

    @model_validator(mode="after")
    def manifest_is_complete(self) -> "LiveEvidenceManifest":
        if set(self.evidence_hashes) != set(TrajectoryRole):
            raise ValueError("manifest requires planned, predicted and observed hashes")
        if not REQUIRED_LIVE_VERSIONS <= set(self.versions):
            missing = sorted(REQUIRED_LIVE_VERSIONS - set(self.versions))
            raise ValueError("manifest missing versions: " + ", ".join(missing))
        if any(not str(value).strip() for value in self.versions.values()):
            raise ValueError("manifest version values must not be blank")
        if any(
            len(value) != 64 or any(character not in "0123456789abcdef" for character in value)
            for value in self.evidence_hashes.values()
        ):
            raise ValueError("manifest evidence hashes must be lowercase SHA-256 values")
        names = [item.name for item in self.artifacts]
        paths = [item.relative_path for item in self.artifacts]
        if len(names) != len(set(names)) or len(paths) != len(set(paths)):
            raise ValueError("manifest artifact names and paths must be unique")
        if self.preview_only and self.acceptance_passed is not None:
            raise ValueError("preview package acceptance_passed must be null")
        if not self.preview_only and self.acceptance_passed is None:
            raise ValueError("non-preview package requires an acceptance result")
        expected = canonical_hash(
            self.model_dump(mode="json", exclude={"content_sha256"})
        )
        if self.content_sha256 != expected:
            raise ValueError("live evidence package content hash mismatch")
        return self


class LiveEvidenceError(ValueError):
    pass


def software_tree_sha256(root: str | Path) -> str:
    source = Path(root).resolve()
    if not source.is_dir():
        raise LiveEvidenceError(f"software hash root is not a directory: {source}")
    digest = hashlib.sha256()
    files = sorted(
        path
        for path in source.rglob("*")
        if path.is_file()
        and path.suffix in {".py", ".json", ".yaml", ".yml"}
        and "__pycache__" not in path.parts
        and ".runtime" not in path.parts
    )
    if not files:
        raise LiveEvidenceError(f"software hash root has no source files: {source}")
    for path in files:
        relative = path.relative_to(source).as_posix().encode("utf-8")
        payload = path.read_bytes()
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(len(payload).to_bytes(8, "big"))
        digest.update(payload)
    return digest.hexdigest()


def build_live_evidence_package(
    *,
    state_directory: str | Path,
    mission_id: UUID | str,
    output: str | Path,
    georeference: GeoReference,
    versions: dict[str, str],
    software_sha256: str,
    software_hash_scope: str,
    random_seed: int,
    software_commit: str | None = None,
    calibration_dataset: LiveCalibrationDataset | None = None,
    calibration_report: LiveCalibrationReport | None = None,
    calibration_receipt: CalibrationFreezeReceipt | None = None,
    allow_preview: bool = False,
    thresholds: TrajectoryThresholds | None = None,
) -> LiveEvidenceManifest:
    root = Path(state_directory)
    destination = Path(output)
    if destination.exists():
        raise LiveEvidenceError(f"live evidence destination already exists: {destination}")
    parsed_id = UUID(str(mission_id))
    plan = MissionPlan.model_validate(
        _read_json(root / "plans" / f"{parsed_id}.json")
    )
    if plan.mission_id != parsed_id:
        raise LiveEvidenceError("mission plan ID does not match requested mission")
    if plan.frame_calibration_id != georeference.calibration_id:
        raise LiveEvidenceError("mission plan calibration_id does not match GeoReference")
    evidence_by_role = _load_mission_evidence(root / "evidence", parsed_id)
    evidence_items = tuple(evidence_by_role[role] for role in TrajectoryRole)
    for evidence in evidence_items:
        if evidence.vehicle_id != plan.vehicle_id:
            raise LiveEvidenceError("trajectory vehicle_id does not match mission plan")
        if (
            evidence.versions.mission_plan_hash is not None
            and evidence.versions.mission_plan_hash != plan.mission_hash
        ):
            raise LiveEvidenceError("trajectory mission plan hash does not match mission plan")
        if evidence.frame_calibration_hash != georeference.config_hash:
            raise LiveEvidenceError(
                "evidence calibration hash does not match supplied GeoReference"
            )
        if evidence.frame_calibration_id != georeference.calibration_id:
            raise LiveEvidenceError(
                "evidence calibration_id does not match supplied GeoReference"
            )

    freeze_verified = False
    if any(
        item is not None
        for item in (
            calibration_dataset,
            calibration_report,
            calibration_receipt,
        )
    ):
        if any(
            item is None
            for item in (
                calibration_dataset,
                calibration_report,
                calibration_receipt,
            )
        ):
            raise LiveEvidenceError(
                "calibration dataset, report and receipt must be supplied together"
            )
        assert calibration_dataset is not None
        assert calibration_report is not None
        assert calibration_receipt is not None
        verify_calibration_freeze(
            georeference,
            calibration_dataset,
            calibration_report,
            calibration_receipt,
        )
        if (
            calibration_dataset.line != plan.line_id
            or calibration_dataset.vehicle_id != plan.vehicle_id
        ):
            raise LiveEvidenceError(
                "calibration dataset line/vehicle does not match mission plan"
            )
        freeze_verified = True

    limits = thresholds or TrajectoryThresholds()
    limits = limits.model_copy(update={"validated_for_venue": freeze_verified})
    base_report = compare_trajectories(evidence_items, limits)
    version_lock_complete = REQUIRED_LIVE_VERSIONS <= set(versions) and all(
        versions[name].strip() for name in REQUIRED_LIVE_VERSIONS
    )
    preview_only = (
        not freeze_verified
        or georeference.status != CalibrationStatus.SURVEYED
        or any(item.preview_only for item in evidence_items)
        or not version_lock_complete
    )
    limitations = [
        item
        for item in base_report.limitations
        if not item.startswith("missing reproduction metadata:")
    ]
    if not freeze_verified:
        limitations.append("calibration freeze receipt was not verified")
    if not version_lock_complete:
        limitations.append("live package version lock is incomplete")
    report = base_report.model_copy(
        update={
            "preview_only": preview_only,
            "acceptance_passed": (
                base_report.metrics_within_thresholds
                if base_report.ready and not preview_only
                else None
            ),
            "limitations": tuple(dict.fromkeys(limitations)),
        }
    )
    if preview_only and not allow_preview:
        raise LiveEvidenceError(
            "live evidence is preview_only; supply a verified calibration freeze "
            "receipt or pass allow_preview explicitly"
        )

    timestamps = [
        sample.observed_at_utc
        for evidence in evidence_items
        for sample in evidence.samples
        if sample.observed_at_utc is not None
    ]
    if not timestamps:
        raise LiveEvidenceError("live evidence requires wall-clock trajectory samples")
    time_range = LiveEvidenceTimeRange(
        started_at_utc=min(timestamps),
        ended_at_utc=max(timestamps),
    )

    payloads: dict[str, tuple[str, str, bytes, str]] = {}
    for role, evidence in evidence_by_role.items():
        _add_payload(
            payloads,
            f"trajectory-{role.value}",
            role.value,
            "application/json",
            _json_bytes(evidence.public_payload()),
            suffix="json",
        )
    _add_payload(
        payloads,
        "comparison-report",
        "metrics",
        "application/json",
        _json_bytes(report.public_payload()),
        suffix="json",
    )
    _add_payload(
        payloads,
        "mission-plan",
        "planned",
        "application/json",
        _json_bytes(plan.model_dump(mode="json")),
        suffix="json",
    )
    _add_payload(
        payloads,
        "audit-events",
        "raw_log",
        "application/x-ndjson",
        _mission_audit_bytes(root / "audit" / "events.jsonl", parsed_id),
        suffix="jsonl",
    )
    observed = evidence_by_role[TrajectoryRole.OBSERVED]
    _add_payload(
        payloads,
        "observed-replay",
        "raw_log",
        "application/x-ndjson",
        _observed_replay_bytes(
            root / "replay" / plan.line_id / "observed.jsonl",
            observed,
        ),
        suffix="jsonl",
    )
    _add_payload(
        payloads,
        "georeference",
        "calibration",
        "application/yaml",
        yaml.safe_dump(
            georeference.model_dump(mode="json"),
            allow_unicode=True,
            sort_keys=False,
        ).encode("utf-8"),
        suffix="yaml",
    )
    _add_optional_state_files(payloads, root, parsed_id)
    if freeze_verified:
        assert calibration_dataset is not None
        assert calibration_report is not None
        assert calibration_receipt is not None
        _add_payload(
            payloads,
            "calibration-dataset",
            "calibration",
            "application/json",
            _json_bytes(calibration_dataset.public_payload()),
            suffix="json",
        )
        _add_payload(
            payloads,
            "calibration-report",
            "calibration",
            "application/json",
            _json_bytes(calibration_report.public_payload()),
            suffix="json",
        )
        _add_payload(
            payloads,
            "calibration-freeze-receipt",
            "calibration",
            "application/json",
            _json_bytes(calibration_receipt.public_payload()),
            suffix="json",
        )

    artifacts = tuple(
        LiveEvidenceArtifact(
            name=name,
            role=role,
            relative_path=relative_path,
            media_type=media_type,
            sha256=_sha256(payload),
            size_bytes=len(payload),
        )
        for name, (role, media_type, payload, relative_path) in sorted(payloads.items())
    )
    manifest_payload: dict[str, Any] = {
        "mission_id": parsed_id,
        "line": plan.line_id,
        "vehicle_id": plan.vehicle_id,
        "created_at_utc": utc_now(),
        "time_range_utc": time_range,
        "versions": versions,
        "software_sha256": software_sha256,
        "software_hash_scope": software_hash_scope,
        "software_commit": software_commit,
        "random_seed": random_seed,
        "frame_calibration_id": georeference.calibration_id,
        "frame_calibration_sha256": georeference.config_hash,
        "calibration_receipt_hash": (
            calibration_receipt.receipt_hash
            if calibration_receipt is not None
            else None
        ),
        "evidence_hashes": {
            role: evidence.evidence_hash
            for role, evidence in evidence_by_role.items()
        },
        "comparison_report_hash": report.report_hash,
        "preview_only": preview_only,
        "acceptance_passed": report.acceptance_passed,
        "limitations": report.limitations,
        "artifacts": artifacts,
    }
    content_hash = canonical_hash(
        LiveEvidenceManifest.model_construct(
            **manifest_payload,
            content_sha256="0" * 64,
        ).model_dump(mode="json", exclude={"content_sha256"})
    )
    manifest = LiveEvidenceManifest(
        **manifest_payload,
        content_sha256=content_hash,
    )
    files = {
        relative_path: payload
        for _name, (_role, _media_type, payload, relative_path) in payloads.items()
    }
    files["manifest.json"] = _json_bytes(manifest.model_dump(mode="json"))
    _export_package(destination, files)
    return manifest


def verify_live_evidence_package(path: str | Path) -> LiveEvidenceManifest:
    files = _read_package(Path(path))
    try:
        manifest = LiveEvidenceManifest.model_validate(
            json.loads(files["manifest.json"])
        )
    except (KeyError, ValueError, json.JSONDecodeError) as exc:
        raise LiveEvidenceError(f"invalid live evidence manifest: {exc}") from exc
    expected_paths = {"manifest.json"}
    by_name = {item.name: item for item in manifest.artifacts}
    required_artifacts = {
        "trajectory-planned",
        "trajectory-predicted",
        "trajectory-observed",
        "comparison-report",
        "mission-plan",
        "audit-events",
        "observed-replay",
        "georeference",
    }
    if not required_artifacts <= set(by_name):
        missing = sorted(required_artifacts - set(by_name))
        raise LiveEvidenceError("missing required live artifacts: " + ", ".join(missing))
    for artifact in manifest.artifacts:
        expected_paths.add(artifact.relative_path)
        payload = files.get(artifact.relative_path)
        if payload is None:
            raise LiveEvidenceError(f"missing live evidence artifact {artifact.name}")
        if _sha256(payload) != artifact.sha256:
            raise LiveEvidenceError(f"hash mismatch for live artifact {artifact.name}")
        if len(payload) != artifact.size_bytes:
            raise LiveEvidenceError(f"size mismatch for live artifact {artifact.name}")
    extras = set(files) - expected_paths
    if extras:
        raise LiveEvidenceError("unindexed live evidence files: " + ", ".join(sorted(extras)))

    evidence_by_role: dict[TrajectoryRole, TrajectoryEvidence] = {}
    for role in TrajectoryRole:
        artifact = by_name[f"trajectory-{role.value}"]
        envelope = json.loads(files[artifact.relative_path])
        evidence = TrajectoryEvidence.model_validate(envelope["evidence"])
        if envelope.get("evidence_hash") != evidence.evidence_hash:
            raise LiveEvidenceError(f"{role.value} evidence hash mismatch")
        if manifest.evidence_hashes[role] != evidence.evidence_hash:
            raise LiveEvidenceError(f"manifest {role.value} evidence hash mismatch")
        evidence_by_role[role] = evidence

    expected_sources = {
        TrajectoryRole.PLANNED: TrajectorySource.MISSION_PLAN,
        TrajectoryRole.PREDICTED: TrajectorySource.AIRSIM,
        TrajectoryRole.OBSERVED: TrajectorySource.REAL_TELEMETRY,
    }
    for role, evidence in evidence_by_role.items():
        if evidence.mission_id != manifest.mission_id:
            raise LiveEvidenceError(f"{role.value} mission_id does not match manifest")
        if evidence.vehicle_id != manifest.vehicle_id:
            raise LiveEvidenceError(f"{role.value} vehicle_id does not match manifest")
        if evidence.source != expected_sources[role]:
            raise LiveEvidenceError(
                f"{role.value} source must be {expected_sources[role].value} for live evidence"
            )
        if (
            evidence.frame_calibration_id != manifest.frame_calibration_id
            or evidence.frame_calibration_hash != manifest.frame_calibration_sha256
        ):
            raise LiveEvidenceError(f"{role.value} calibration does not match manifest")

    plan = MissionPlan.model_validate(
        json.loads(files[by_name["mission-plan"].relative_path])
    )
    if (
        plan.mission_id != manifest.mission_id
        or plan.vehicle_id != manifest.vehicle_id
        or plan.line_id != manifest.line
        or plan.frame_calibration_id != manifest.frame_calibration_id
    ):
        raise LiveEvidenceError("mission plan identity does not match manifest")
    if any(
        evidence.versions.mission_plan_hash is not None
        and evidence.versions.mission_plan_hash != plan.mission_hash
        for evidence in evidence_by_role.values()
    ):
        raise LiveEvidenceError("trajectory mission plan hash does not match packaged plan")

    report_artifact = by_name["comparison-report"]
    report_envelope = json.loads(files[report_artifact.relative_path])
    report = TrajectoryComparisonReport.model_validate(report_envelope["report"])
    if report_envelope.get("report_hash") != report.report_hash:
        raise LiveEvidenceError("trajectory comparison report hash mismatch")
    if manifest.comparison_report_hash != report.report_hash:
        raise LiveEvidenceError("manifest comparison report hash mismatch")
    if (
        report.preview_only != manifest.preview_only
        or report.acceptance_passed != manifest.acceptance_passed
        or report.evidence_hashes != manifest.evidence_hashes
    ):
        raise LiveEvidenceError("comparison report result does not match manifest")
    if len(report.comparisons) != 3:
        raise LiveEvidenceError("comparison report must contain all three trajectory pairs")
    recomputed = compare_trajectories(
        tuple(evidence_by_role[role] for role in TrajectoryRole),
        report.thresholds,
    )
    if tuple(item.model_dump(mode="json") for item in recomputed.comparisons) != tuple(
        item.model_dump(mode="json") for item in report.comparisons
    ):
        raise LiveEvidenceError("comparison metrics do not match trajectory artifacts")
    if (
        recomputed.ready != report.ready
        or recomputed.metrics_within_thresholds != report.metrics_within_thresholds
    ):
        raise LiveEvidenceError("comparison readiness does not match trajectory artifacts")
    _verify_raw_replay(files, by_name, evidence_by_role[TrajectoryRole.OBSERVED])

    timestamps = [
        sample.observed_at_utc
        for evidence in evidence_by_role.values()
        for sample in evidence.samples
        if sample.observed_at_utc is not None
    ]
    if not timestamps or (
        min(timestamps) != manifest.time_range_utc.started_at_utc
        or max(timestamps) != manifest.time_range_utc.ended_at_utc
    ):
        raise LiveEvidenceError("manifest wall-time range does not match trajectories")

    reference = GeoReference.model_validate(
        yaml.safe_load(files[by_name["georeference"].relative_path])
    )
    if (
        reference.calibration_id != manifest.frame_calibration_id
        or reference.config_hash != manifest.frame_calibration_sha256
    ):
        raise LiveEvidenceError("packaged GeoReference does not match manifest")

    calibration_artifacts = {
            "calibration-dataset",
            "calibration-report",
            "calibration-freeze-receipt",
        }
    present_calibration = calibration_artifacts & set(by_name)
    if present_calibration and present_calibration != calibration_artifacts:
        raise LiveEvidenceError("calibration freeze artifact set is incomplete")
    if not manifest.preview_only and present_calibration != calibration_artifacts:
            raise LiveEvidenceError("acceptance package is missing calibration freeze artifacts")
    if present_calibration == calibration_artifacts:
        dataset_envelope = json.loads(
            files[by_name["calibration-dataset"].relative_path]
        )
        calibration_report_envelope = json.loads(
            files[by_name["calibration-report"].relative_path]
        )
        receipt_envelope = json.loads(
            files[by_name["calibration-freeze-receipt"].relative_path]
        )
        dataset = LiveCalibrationDataset.model_validate(dataset_envelope["dataset"])
        calibration_report = LiveCalibrationReport.model_validate(
            calibration_report_envelope["report"]
        )
        receipt = CalibrationFreezeReceipt.model_validate(
            receipt_envelope["receipt"]
        )
        if dataset_envelope.get("dataset_hash") != dataset.dataset_hash:
            raise LiveEvidenceError("packaged calibration dataset hash mismatch")
        if calibration_report_envelope.get("report_hash") != calibration_report.report_hash:
            raise LiveEvidenceError("packaged calibration report hash mismatch")
        if receipt_envelope.get("receipt_hash") != receipt.receipt_hash:
            raise LiveEvidenceError("packaged calibration receipt hash mismatch")
        verify_calibration_freeze(reference, dataset, calibration_report, receipt)
        if dataset.line != manifest.line or dataset.vehicle_id != manifest.vehicle_id:
            raise LiveEvidenceError("packaged calibration dataset line/vehicle mismatch")
        if manifest.calibration_receipt_hash != receipt.receipt_hash:
            raise LiveEvidenceError("manifest calibration receipt hash mismatch")
    elif manifest.calibration_receipt_hash is not None:
        raise LiveEvidenceError("manifest references an absent calibration receipt")
    return manifest


def comparison_metrics_by_pair(
    report: TrajectoryComparisonReport,
) -> dict[str, TrajectoryPairMetrics]:
    return {
        f"{item.reference_role.value}-{item.compared_role.value}": item
        for item in report.comparisons
    }


def _load_mission_evidence(
    root: Path,
    mission_id: UUID,
) -> dict[TrajectoryRole, TrajectoryEvidence]:
    found: dict[TrajectoryRole, TrajectoryEvidence] = {}
    for path in sorted(root.glob("*.json")):
        evidence = load_evidence_file(path)
        if evidence.mission_id != mission_id:
            continue
        if evidence.role in found:
            raise LiveEvidenceError(
                f"mission has duplicate {evidence.role.value} evidence"
            )
        found[evidence.role] = evidence
    missing = set(TrajectoryRole) - set(found)
    if missing:
        raise LiveEvidenceError(
            "mission is missing evidence roles: "
            + ", ".join(sorted(item.value for item in missing))
        )
    return found


def _mission_audit_bytes(path: Path, mission_id: UUID) -> bytes:
    selected: list[str] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw.strip():
            continue
        event = json.loads(raw)
        payload = event.get("payload")
        if isinstance(payload, dict) and payload.get("mission_id") == str(mission_id):
            selected.append(json.dumps(event, sort_keys=True, separators=(",", ":")))
    if not selected:
        raise LiveEvidenceError("mission has no raw audit events")
    return ("\n".join(selected) + "\n").encode("utf-8")


def _observed_replay_bytes(path: Path, evidence: TrajectoryEvidence) -> bytes:
    start = evidence.samples[0].observed_at_utc
    end = evidence.samples[-1].observed_at_utc
    if start is None or end is None:
        raise LiveEvidenceError("observed evidence is missing wall timestamps")
    selected: list[str] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw.strip():
            continue
        record = ObservedReplayRecord.model_validate_json(raw)
        if start <= record.wall_time <= end:
            selected.append(
                json.dumps(
                    record.model_dump(mode="json"),
                    ensure_ascii=True,
                    sort_keys=True,
                    separators=(",", ":"),
                )
            )
    if len(selected) < 2:
        raise LiveEvidenceError("observed trajectory has fewer than two raw replay records")
    return ("\n".join(selected) + "\n").encode("utf-8")


def _verify_raw_replay(
    files: dict[str, bytes],
    by_name: dict[str, LiveEvidenceArtifact],
    observed: TrajectoryEvidence,
) -> None:
    payload = files[by_name["observed-replay"].relative_path].decode("utf-8")
    records = tuple(
        ObservedReplayRecord.model_validate_json(raw)
        for raw in payload.splitlines()
        if raw.strip()
    )
    if len(records) < 2:
        raise LiveEvidenceError("packaged raw observed replay is incomplete")
    for sample in observed.samples:
        if sample.observed_at_utc is None:
            raise LiveEvidenceError("observed evidence sample lacks wall time")
        matched = any(
            record.wall_time == sample.observed_at_utc
            and _position_distance(record.position, sample.position_map_m) <= 1e-9
            for record in records
        )
        if not matched:
            raise LiveEvidenceError(
                "observed evidence sample cannot be reconciled with raw replay"
            )


def _position_distance(position: Any, expected: tuple[float, float, float]) -> float:
    return (
        (position.x - expected[0]) ** 2
        + (position.y - expected[1]) ** 2
        + (position.z - expected[2]) ** 2
    ) ** 0.5


def _add_optional_state_files(
    payloads: dict[str, tuple[str, str, bytes, str]],
    root: Path,
    mission_id: UUID,
) -> None:
    candidates = (
        ("approval-request", "approval", root / "approval_requests" / f"{mission_id}.json"),
        ("approval-receipt", "approval", root / "approvals" / f"{mission_id}.json"),
        (
            "dispatch-authorization",
            "approval",
            root / "dispatch_authorizations" / f"{mission_id}.json",
        ),
    )
    for name, role, path in candidates:
        if path.is_file():
            _add_payload(
                payloads,
                name,
                role,
                "application/json",
                path.read_bytes(),
                suffix="json",
            )


def _add_payload(
    payloads: dict[str, tuple[str, str, bytes, str]],
    name: str,
    role: str,
    media_type: str,
    payload: bytes,
    *,
    suffix: str,
) -> None:
    if name in payloads:
        raise LiveEvidenceError(f"duplicate live evidence artifact {name}")
    payloads[name] = (role, media_type, payload, f"artifacts/{name}.{suffix}")


def _export_package(destination: Path, files: dict[str, bytes]) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.suffix.lower() == ".zip":
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{destination.name}.",
            suffix=".tmp",
            dir=destination.parent,
        )
        os.close(descriptor)
        temporary = Path(temporary_name)
        try:
            with zipfile.ZipFile(temporary, "w") as archive:
                for relative_path, payload in sorted(files.items()):
                    info = zipfile.ZipInfo(
                        relative_path,
                        date_time=(1980, 1, 1, 0, 0, 0),
                    )
                    info.compress_type = zipfile.ZIP_DEFLATED
                    info.create_system = 3
                    info.external_attr = 0o100644 << 16
                    archive.writestr(info, payload)
            os.replace(temporary, destination)
        finally:
            if temporary.exists():
                temporary.unlink()
        return
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{destination.name}.", dir=destination.parent)
    )
    try:
        for relative_path, payload in files.items():
            path = temporary / PurePosixPath(relative_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            for path in sorted(temporary.rglob("*"), reverse=True):
                if path.is_file():
                    path.unlink()
                elif path.is_dir():
                    path.rmdir()
            temporary.rmdir()


def _read_package(path: Path) -> dict[str, bytes]:
    try:
        if path.is_dir():
            return {
                item.relative_to(path).as_posix(): item.read_bytes()
                for item in path.rglob("*")
                if item.is_file()
            }
        with zipfile.ZipFile(path, "r") as archive:
            names = archive.namelist()
            if len(names) != len(set(names)):
                raise LiveEvidenceError("live evidence ZIP contains duplicate paths")
            return {name: archive.read(name) for name in names}
    except LiveEvidenceError:
        raise
    except (OSError, zipfile.BadZipFile) as exc:
        raise LiveEvidenceError(f"failed to read live evidence package {path}: {exc}") from exc


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise LiveEvidenceError(f"failed to read {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise LiveEvidenceError(f"expected an object in {path}")
    return value


def _json_bytes(payload: object) -> bytes:
    return (
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


__all__ = [
    "LiveEvidenceArtifact",
    "LiveEvidenceError",
    "LiveEvidenceManifest",
    "LiveEvidenceTimeRange",
    "REQUIRED_LIVE_VERSIONS",
    "build_live_evidence_package",
    "comparison_metrics_by_pair",
    "software_tree_sha256",
    "verify_live_evidence_package",
]
