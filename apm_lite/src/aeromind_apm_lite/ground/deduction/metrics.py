"""Metric construction and geometric helpers for deduction results."""

from __future__ import annotations

from datetime import datetime
from itertools import combinations

from aeromind_apm_lite.common.contracts.twin import (
    MetricDirection,
    MetricRecord,
    TwinState,
    distance_between,
)


METRIC_CONTRACT = {
    "task_completion_rate": ("ratio", MetricDirection.MAXIMIZE),
    "minimum_separation_m": ("m", MetricDirection.MAXIMIZE),
    "elapsed_time_s": ("s", MetricDirection.MINIMIZE),
    "energy_consumption_ratio": ("ratio", MetricDirection.MINIMIZE),
    "communication_load_ratio": ("ratio", MetricDirection.MINIMIZE),
    "recovery_rate": ("ratio", MetricDirection.MAXIMIZE),
}


def minimum_separation(states: tuple[TwinState, ...]) -> float:
    if len(states) < 2:
        # A singleton fleet has no pairwise collision candidate; keep the
        # metric finite because all contract models reject NaN/Infinity.
        return 1_000_000.0
    return min(
        distance_between(left.position_m, right.position_m)
        for left, right in combinations(states, 2)
    )


def metric_records(
    branch_id: str,
    values: dict[str, float],
    recorded_at_utc: datetime,
) -> tuple[MetricRecord, ...]:
    return tuple(
        MetricRecord(
            branch_id=branch_id,
            metric_name=name,
            value=round(float(values[name]), 9),
            unit=METRIC_CONTRACT[name][0],
            direction=METRIC_CONTRACT[name][1],
            source="l0-event-v1",
            recorded_at_utc=recorded_at_utc,
        )
        for name in METRIC_CONTRACT
    )


__all__ = ["METRIC_CONTRACT", "metric_records", "minimum_separation"]
