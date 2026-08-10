"""Hard-constraint filtering and direction-aware Pareto selection."""

from __future__ import annotations

from collections.abc import Sequence

from aeromind_apm_lite.common.contracts.twin import MetricDirection

from .branch import BranchResult, ParetoReport


def pareto_front(results: Sequence[BranchResult]) -> ParetoReport:
    result_tuple = tuple(results)
    rejected = {
        item.branch_id: item.hard_constraint_violations
        for item in result_tuple
        if item.hard_constraint_violations
    }
    eligible = [item for item in result_tuple if not item.hard_constraint_violations]
    objective_names = _shared_objectives(eligible)
    selected = []
    for candidate in eligible:
        if not any(
            _dominates(other, candidate, objective_names)
            for other in eligible
            if other.branch_id != candidate.branch_id
        ):
            selected.append(candidate.branch_id)
    return ParetoReport(
        selected_branch_ids=tuple(selected),
        rejected_branch_ids=tuple(rejected),
        rejection_reasons=rejected,
        objective_names=objective_names,
    )


def _shared_objectives(results: Sequence[BranchResult]) -> tuple[str, ...]:
    if not results:
        return ()
    common = {metric.metric_name for metric in results[0].metrics}
    for result in results[1:]:
        common &= {metric.metric_name for metric in result.metrics}
    return tuple(metric.metric_name for metric in results[0].metrics if metric.metric_name in common)


def _dominates(
    left: BranchResult,
    right: BranchResult,
    objectives: tuple[str, ...],
) -> bool:
    if not objectives:
        return False
    left_metrics = {item.metric_name: item for item in left.metrics}
    right_metrics = {item.metric_name: item for item in right.metrics}
    weakly_better = True
    strictly_better = False
    for name in objectives:
        left_metric = left_metrics[name]
        right_metric = right_metrics[name]
        if left_metric.direction != right_metric.direction:
            raise ValueError(f"metric direction mismatch for {name}")
        if left_metric.direction == MetricDirection.MAXIMIZE:
            weakly_better &= left_metric.value >= right_metric.value
            strictly_better |= left_metric.value > right_metric.value
        else:
            weakly_better &= left_metric.value <= right_metric.value
            strictly_better |= left_metric.value < right_metric.value
    return weakly_better and strictly_better


__all__ = ["pareto_front"]
