"""Versioned contracts shared by twin, deduction and orchestration layers."""

from __future__ import annotations

import math
from datetime import datetime, timezone
from enum import Enum
from typing import Literal
from uuid import UUID, uuid4

from pydantic import ConfigDict, Field, field_validator, model_validator

from .models import CoordinateFrame, Quaternion, StrictModel, Vector3


TWIN_SCHEMA_VERSION = "1.0"
JsonScalar = str | int | float | bool | None


def _as_utc(value: datetime) -> datetime:
    if value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("timestamp must include a timezone")
    return value.astimezone(timezone.utc)


class TwinSource(str, Enum):
    REAL = "real"
    SIM = "sim"
    PREDICTED = "predicted"
    OBSERVED = "observed"


class FidelityLevel(str, Enum):
    L0 = "L0"
    L1 = "L1"
    L2 = "L2"
    L3 = "L3"


class HealthStatus(str, Enum):
    NOMINAL = "nominal"
    DEGRADED = "degraded"
    FAILED = "failed"
    OFFLINE = "offline"
    UNKNOWN = "unknown"


class MissionEdgeKind(str, Enum):
    DEPENDENCY = "dependency"
    CONDITION = "condition"
    FALLBACK = "fallback"


class StrategyKind(str, Enum):
    FIXED_PARTITION = "fixed_partition"
    CENTRALIZED_OPTIMIZATION = "centralized_optimization"
    DISTRIBUTED_COLLABORATION = "distributed_collaboration"
    AI_ASSISTED_HYBRID = "ai_assisted_hybrid"


class MetricDirection(str, Enum):
    MAXIMIZE = "maximize"
    MINIMIZE = "minimize"


class TwinContract(StrictModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)
    schema_version: Literal["1.0"] = TWIN_SCHEMA_VERSION


class TwinState(TwinContract):
    """One source-labelled dynamic state for a physical or simulated platform."""

    vehicle_id: int = Field(ge=1, le=65_535)
    platform_type: str = Field(
        min_length=1,
        max_length=64,
        pattern=r"^[a-z][a-z0-9_-]*$",
    )
    frame: CoordinateFrame
    frame_calibration_id: str = Field(min_length=1, max_length=128)
    position_m: Vector3
    velocity_m_s: Vector3 | None = None
    attitude: Quaternion | None = None
    energy_remaining: float = Field(ge=0.0, le=1.0)
    communication_quality: float = Field(ge=0.0, le=1.0)
    mission_phase: str = Field(min_length=1, max_length=64)
    health_status: HealthStatus
    source: TwinSource
    confidence: float = Field(ge=0.0, le=1.0)
    fidelity: FidelityLevel
    sampled_at_utc: datetime
    simulated_at_utc: datetime | None = None
    mapped_at_utc: datetime | None = None

    @field_validator("sampled_at_utc", "simulated_at_utc", "mapped_at_utc")
    @classmethod
    def timestamps_are_utc(cls, value: datetime | None) -> datetime | None:
        return _as_utc(value) if value is not None else None

    @model_validator(mode="after")
    def validate_state_provenance(self) -> "TwinState":
        if self.frame == CoordinateFrame.NONE:
            raise ValueError("twin state requires an explicit coordinate frame")
        simulated = self.source in {TwinSource.SIM, TwinSource.PREDICTED}
        if simulated and self.simulated_at_utc is None:
            raise ValueError("sim/predicted state requires simulated_at_utc")
        if not simulated and self.mapped_at_utc is None:
            raise ValueError("real/observed state requires mapped_at_utc")
        if self.simulated_at_utc is not None and self.mapped_at_utc is not None:
            raise ValueError("one state must not mix simulated and mapped timestamps")
        return self


class VehicleProfile(TwinContract):
    """Static capability and authorization information for one platform."""

    vehicle_id: int = Field(ge=1, le=65_535)
    display_name: str = Field(min_length=1, max_length=64)
    platform_type: str = Field(
        min_length=1,
        max_length=64,
        pattern=r"^[a-z][a-z0-9_-]*$",
    )
    sensors: tuple[str, ...] = ()
    payloads: tuple[str, ...] = ()
    max_endurance_min: float = Field(gt=0.0, le=10_000.0)
    max_speed_m_s: float = Field(gt=0.0, le=1_000.0)
    capabilities: tuple[str, ...] = Field(min_length=1)
    whitelist_level: int = Field(ge=0, le=10)

    @field_validator("sensors", "payloads", "capabilities")
    @classmethod
    def normalize_catalog(cls, values: tuple[str, ...]) -> tuple[str, ...]:
        normalized = tuple(sorted({str(value).strip().lower() for value in values}))
        if any(not value for value in normalized):
            raise ValueError("catalog entries must not be empty")
        return normalized


class MissionNode(TwinContract):
    node_id: str = Field(min_length=1, max_length=64)
    action: str = Field(min_length=1, max_length=64)
    parameters: dict[str, JsonScalar] = Field(default_factory=dict)


class MissionEdge(TwinContract):
    source_node_id: str = Field(min_length=1, max_length=64)
    target_node_id: str = Field(min_length=1, max_length=64)
    kind: MissionEdgeKind = MissionEdgeKind.DEPENDENCY
    condition: str | None = Field(default=None, min_length=1, max_length=256)

    @model_validator(mode="after")
    def condition_matches_kind(self) -> "MissionEdge":
        if self.kind == MissionEdgeKind.CONDITION and self.condition is None:
            raise ValueError("condition edge requires condition")
        return self


class MissionGraph(TwinContract):
    mission_id: UUID
    graph_id: UUID = Field(default_factory=uuid4)
    nodes: tuple[MissionNode, ...] = Field(min_length=1, max_length=10_000)
    edges: tuple[MissionEdge, ...] = ()

    @model_validator(mode="after")
    def validate_graph(self) -> "MissionGraph":
        node_ids = [node.node_id for node in self.nodes]
        if len(node_ids) != len(set(node_ids)):
            raise ValueError("mission graph contains duplicate node_id")
        known = set(node_ids)
        for edge in self.edges:
            if edge.source_node_id not in known or edge.target_node_id not in known:
                raise ValueError("mission edge references an unknown node")
            if edge.source_node_id == edge.target_node_id:
                raise ValueError("mission graph dependency cycle detected")

        dependencies = {
            node_id: [] for node_id in node_ids
        }
        for edge in self.edges:
            if edge.kind == MissionEdgeKind.DEPENDENCY:
                dependencies[edge.source_node_id].append(edge.target_node_id)
        visiting: set[str] = set()
        visited: set[str] = set()

        def visit(node_id: str) -> None:
            if node_id in visiting:
                raise ValueError("mission graph dependency cycle detected")
            if node_id in visited:
                return
            visiting.add(node_id)
            for target in dependencies[node_id]:
                visit(target)
            visiting.remove(node_id)
            visited.add(node_id)

        for node_id in node_ids:
            visit(node_id)
        return self


class WorldEvent(TwinContract):
    event_id: UUID = Field(default_factory=uuid4)
    event_type: str = Field(
        min_length=1,
        max_length=64,
        pattern=r"^[a-z][a-z0-9_-]*$",
    )
    occurs_at_utc: datetime
    affected_vehicle_ids: tuple[int, ...] = ()
    parameters: dict[str, JsonScalar] = Field(default_factory=dict)
    source: TwinSource
    confidence: float = Field(default=1.0, ge=0.0, le=1.0)

    _occurs_at_is_utc = field_validator("occurs_at_utc")(_as_utc)

    @field_validator("affected_vehicle_ids")
    @classmethod
    def vehicle_ids_are_unique(cls, values: tuple[int, ...]) -> tuple[int, ...]:
        if any(value < 1 or value > 65_535 for value in values):
            raise ValueError("affected vehicle_id is out of range")
        if len(values) != len(set(values)):
            raise ValueError("affected_vehicle_ids must be unique")
        return tuple(sorted(values))


class StrategySpec(TwinContract):
    strategy_id: str = Field(min_length=1, max_length=128)
    kind: StrategyKind
    random_seed: int = Field(ge=0, le=2**63 - 1)
    parameters: dict[str, JsonScalar] = Field(default_factory=dict)
    metric_names: tuple[str, ...] = Field(min_length=1)

    @field_validator("metric_names")
    @classmethod
    def metrics_are_unique(cls, values: tuple[str, ...]) -> tuple[str, ...]:
        if len(values) != len(set(values)):
            raise ValueError("metric_names must be unique")
        return values


class MetricRecord(TwinContract):
    branch_id: str = Field(min_length=1, max_length=128)
    metric_name: str = Field(
        min_length=1,
        max_length=128,
        pattern=r"^[a-z][a-z0-9_]*$",
    )
    value: float
    unit: str = Field(min_length=1, max_length=32)
    direction: MetricDirection
    source: str = Field(min_length=1, max_length=64)
    recorded_at_utc: datetime

    _recorded_at_is_utc = field_validator("recorded_at_utc")(_as_utc)


def source_for_runtime_mode(mode: str) -> TwinSource:
    """Map existing ground-station modes without conflating sim and real state."""

    normalized = str(mode).strip().lower()
    if normalized in {"demo", "sitl", "sim"}:
        return TwinSource.SIM
    if normalized == "real":
        return TwinSource.REAL
    raise ValueError(f"unsupported runtime mode: {mode}")


def distance_between(left: Vector3, right: Vector3) -> float:
    return math.sqrt(
        (left.x - right.x) ** 2
        + (left.y - right.y) ** 2
        + (left.z - right.z) ** 2
    )


__all__ = [
    "TWIN_SCHEMA_VERSION",
    "FidelityLevel",
    "HealthStatus",
    "MetricDirection",
    "MetricRecord",
    "MissionEdge",
    "MissionEdgeKind",
    "MissionGraph",
    "MissionNode",
    "StrategyKind",
    "StrategySpec",
    "TwinSource",
    "TwinState",
    "VehicleProfile",
    "WorldEvent",
    "distance_between",
    "source_for_runtime_mode",
]
