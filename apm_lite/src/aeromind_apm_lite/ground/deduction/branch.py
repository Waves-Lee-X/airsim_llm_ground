"""Baseline snapshots, deterministic branches and ordered execution."""

from __future__ import annotations

import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from typing import Any, Literal, Protocol, Sequence

from pydantic import Field, field_validator, model_validator

from aeromind_apm_lite.common.contracts.models import StrictModel
from aeromind_apm_lite.common.contracts.twin import (
    MetricRecord,
    StrategyKind,
    StrategySpec,
    TwinState,
    WorldEvent,
)


DEDUCTION_SCHEMA_VERSION = "1.0"


def canonical_hash(payload: Any) -> str:
    if hasattr(payload, "model_dump"):
        payload = payload.model_dump(mode="json")
    encoded = json.dumps(
        payload,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _as_utc(value: datetime) -> datetime:
    if value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("timestamp must include a timezone")
    return value.astimezone(timezone.utc)


class DeductionModel(StrictModel):
    schema_version: Literal["1.0"] = DEDUCTION_SCHEMA_VERSION


class BaselineSnapshot(DeductionModel):
    snapshot_id: str = Field(min_length=1, max_length=128)
    captured_at_utc: datetime
    duration_s: int = Field(ge=1, le=86_400)
    target_count: int = Field(ge=1, le=1_000_000)
    minimum_separation_m: float = Field(gt=0.0, le=10_000.0)
    reserve_energy: float = Field(ge=0.0, lt=1.0)
    twins: tuple[TwinState, ...] = Field(min_length=1, max_length=10_000)

    _captured_at_is_utc = field_validator("captured_at_utc")(_as_utc)

    @model_validator(mode="after")
    def validate_twins(self) -> "BaselineSnapshot":
        vehicle_ids = [item.vehicle_id for item in self.twins]
        if len(vehicle_ids) != len(set(vehicle_ids)):
            raise ValueError("baseline contains duplicate vehicle_id")
        frames = {(item.frame, item.frame_calibration_id) for item in self.twins}
        if len(frames) != 1:
            raise ValueError("baseline twins must share one calibrated coordinate frame")
        return self

    @property
    def baseline_hash(self) -> str:
        return canonical_hash(self.model_dump(mode="json"))


class BranchSpec(DeductionModel):
    branch_id: str = Field(min_length=1, max_length=128)
    baseline: BaselineSnapshot
    strategy: StrategySpec
    events: tuple[WorldEvent, ...] = ()
    model_version: str = Field(default="l0-event-v1", min_length=1, max_length=128)
    software_version: str = Field(default="aeromind-apm-lite-0.1.0", min_length=1, max_length=128)

    @model_validator(mode="after")
    def validate_events(self) -> "BranchSpec":
        known = {item.vehicle_id for item in self.baseline.twins}
        end = self.baseline.captured_at_utc.timestamp() + self.baseline.duration_s
        for event in self.events:
            if not set(event.affected_vehicle_ids).issubset(known):
                raise ValueError("branch event references an unknown vehicle")
            timestamp = event.occurs_at_utc.timestamp()
            if not self.baseline.captured_at_utc.timestamp() <= timestamp <= end:
                raise ValueError("branch event falls outside the simulation timeline")
        return self


class BranchResult(DeductionModel):
    branch_id: str = Field(min_length=1, max_length=128)
    strategy_id: str = Field(min_length=1, max_length=128)
    strategy_kind: StrategyKind
    baseline_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    event_count: int = Field(ge=0)
    final_states: tuple[TwinState, ...]
    metrics: tuple[MetricRecord, ...]
    hard_constraint_violations: tuple[str, ...]
    failure_reasons: tuple[str, ...]
    reproducibility_hash: str = Field(pattern=r"^[0-9a-f]{64}$")

    def metric(self, name: str) -> float:
        for record in self.metrics:
            if record.metric_name == name:
                return record.value
        raise KeyError(name)


class ParetoReport(DeductionModel):
    selected_branch_ids: tuple[str, ...]
    rejected_branch_ids: tuple[str, ...]
    rejection_reasons: dict[str, tuple[str, ...]]
    objective_names: tuple[str, ...]


class DeductionRun(DeductionModel):
    run_id: str = Field(min_length=1, max_length=128)
    baseline_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    results: tuple[BranchResult, ...]
    pareto_report: ParetoReport
    wall_time_s: float = Field(ge=0.0)
    simulated_time_s: float = Field(ge=0.0)
    realtime_factor: float = Field(ge=0.0)


class RunnableModel(Protocol):
    def run(self, branch: BranchSpec) -> BranchResult:
        ...


class BranchManager:
    """Create content-addressed branches and execute them in stable order."""

    def __init__(self, model: RunnableModel | None = None) -> None:
        if model is None:
            from .l0_model import L0Model

            model = L0Model()
        self.model = model

    def create_branch(
        self,
        baseline: BaselineSnapshot,
        strategy: StrategySpec,
        *,
        events: Sequence[WorldEvent] = (),
        model_version: str = "l0-event-v1",
        software_version: str = "aeromind-apm-lite-0.1.0",
    ) -> BranchSpec:
        ordered_events = tuple(
            sorted(events, key=lambda item: (item.occurs_at_utc, str(item.event_id)))
        )
        identity = canonical_hash(
            {
                "baseline_hash": baseline.baseline_hash,
                "strategy": strategy.model_dump(mode="json"),
                "events": [item.model_dump(mode="json") for item in ordered_events],
                "model_version": model_version,
                "software_version": software_version,
            }
        )
        return BranchSpec(
            branch_id=f"branch-{identity[:20]}",
            baseline=baseline,
            strategy=strategy,
            events=ordered_events,
            model_version=model_version,
            software_version=software_version,
        )

    def run(
        self,
        branches: Sequence[BranchSpec],
        *,
        workers: int = 1,
    ) -> tuple[BranchResult, ...]:
        if workers < 1:
            raise ValueError("workers must be at least one")
        branch_tuple = tuple(branches)
        if workers == 1 or len(branch_tuple) < 2:
            return tuple(self.model.run(branch) for branch in branch_tuple)
        with ThreadPoolExecutor(max_workers=workers) as executor:
            return tuple(executor.map(self.model.run, branch_tuple))


__all__ = [
    "DEDUCTION_SCHEMA_VERSION",
    "BaselineSnapshot",
    "BranchManager",
    "BranchResult",
    "BranchSpec",
    "DeductionRun",
    "ParetoReport",
    "canonical_hash",
]
