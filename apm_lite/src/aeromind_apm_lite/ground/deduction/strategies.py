"""Constrained strategy specifications used by the M1 deduction demo."""

from __future__ import annotations

from aeromind_apm_lite.common.contracts.twin import StrategyKind, StrategySpec


METRIC_NAMES = (
    "task_completion_rate",
    "minimum_separation_m",
    "elapsed_time_s",
    "energy_consumption_ratio",
    "communication_load_ratio",
    "recovery_rate",
)


def default_strategies(seed: int) -> tuple[StrategySpec, ...]:
    """Return the four declared strategies as bounded, auditable parameters."""

    definitions = (
        (
            "fixed-partition-v1",
            StrategyKind.FIXED_PARTITION,
            {
                "productivity": 0.78,
                "cruise_speed_m_s": 6.0,
                "energy_per_hour": 0.18,
                "communication_load": 0.35,
                "recovery_factor": 0.35,
            },
        ),
        (
            "centralized-optimization-v1",
            StrategyKind.CENTRALIZED_OPTIMIZATION,
            {
                "productivity": 0.94,
                "cruise_speed_m_s": 7.5,
                "energy_per_hour": 0.22,
                "communication_load": 0.78,
                "recovery_factor": 0.62,
            },
        ),
        (
            "distributed-collaboration-v1",
            StrategyKind.DISTRIBUTED_COLLABORATION,
            {
                "productivity": 0.88,
                "cruise_speed_m_s": 7.0,
                "energy_per_hour": 0.19,
                "communication_load": 0.52,
                "recovery_factor": 0.88,
            },
        ),
        (
            "ai-assisted-hybrid-v1",
            StrategyKind.AI_ASSISTED_HYBRID,
            {
                "productivity": 0.97,
                "cruise_speed_m_s": 7.2,
                "energy_per_hour": 0.20,
                "communication_load": 0.61,
                "recovery_factor": 0.80,
                "control_authority": "parameters_only",
            },
        ),
    )
    return tuple(
        StrategySpec(
            strategy_id=strategy_id,
            kind=kind,
            random_seed=seed,
            parameters=parameters,
            metric_names=METRIC_NAMES,
        )
        for strategy_id, kind, parameters in definitions
    )


__all__ = ["METRIC_NAMES", "default_strategies"]
