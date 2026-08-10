import json
from datetime import datetime, timedelta, timezone

from aeromind_apm_lite.common.contracts import CoordinateFrame, Vector3
from aeromind_apm_lite.common.contracts.twin import (
    FidelityLevel,
    HealthStatus,
    MetricDirection,
    MetricRecord,
    TwinSource,
    TwinState,
    WorldEvent,
)
from aeromind_apm_lite.ground.deduction import (
    BaselineSnapshot,
    BranchManager,
    BranchResult,
    L0Model,
    default_strategies,
    pareto_front,
)
from aeromind_apm_lite.ground.deduction.cli import main as deduction_cli_main


NOW = datetime(2026, 8, 10, 8, 0, tzinfo=timezone.utc)


def baseline(vehicle_count=4):
    twins = tuple(
        TwinState(
            vehicle_id=index + 1,
            platform_type="multirotor",
            frame=CoordinateFrame.MAP,
            frame_calibration_id="test-field-v1",
            position_m=Vector3(x=float((index % 16) * 5), y=float((index // 16) * 5), z=5.0),
            energy_remaining=0.9,
            communication_quality=0.95,
            mission_phase="ready",
            health_status=HealthStatus.NOMINAL,
            source=TwinSource.SIM,
            confidence=1.0,
            fidelity=FidelityLevel.L0,
            sampled_at_utc=NOW,
            simulated_at_utc=NOW,
        )
        for index in range(vehicle_count)
    )
    return BaselineSnapshot(
        snapshot_id=f"baseline-{vehicle_count}",
        captured_at_utc=NOW,
        duration_s=120,
        target_count=max(1, vehicle_count * 2),
        minimum_separation_m=2.0,
        reserve_energy=0.1,
        twins=twins,
    )


def test_same_seed_has_identical_terminal_state_for_ten_runs():
    manager = BranchManager(model=L0Model())
    branch = manager.create_branch(baseline(), default_strategies(seed=20260810)[0])

    results = [manager.run((branch,))[0] for _ in range(10)]

    assert all(
        result.reproducibility_hash == results[0].reproducibility_hash
        for result in results
    )
    assert all(result.final_states == results[0].final_states for result in results)
    assert all(result.metrics == results[0].metrics for result in results)


def test_four_strategies_emit_complete_metrics_and_pareto_filters_constraints():
    manager = BranchManager(model=L0Model())
    branches = tuple(
        manager.create_branch(baseline(), strategy)
        for strategy in default_strategies(seed=7)
    )
    results = manager.run(branches, workers=2)
    expected_metrics = {
        "task_completion_rate",
        "minimum_separation_m",
        "elapsed_time_s",
        "energy_consumption_ratio",
        "communication_load_ratio",
        "recovery_rate",
    }

    assert len(results) == 4
    assert all(
        {metric.metric_name for metric in result.metrics} == expected_metrics
        for result in results
    )
    assert {result.strategy_kind for result in results} == {
        item.kind for item in default_strategies(7)
    }

    unsafe = results[0].model_copy(
        update={"branch_id": "unsafe", "hard_constraint_violations": ("separation",)}
    )
    report = pareto_front((*results, unsafe))
    assert "unsafe" in report.rejected_branch_ids
    assert report.selected_branch_ids


def test_failure_event_is_an_event_difference_and_recovery_is_reported():
    snapshot = baseline()
    event = WorldEvent(
        event_type="vehicle_failure",
        occurs_at_utc=NOW + timedelta(seconds=30),
        affected_vehicle_ids=(1,),
        parameters={},
        source=TwinSource.SIM,
    )
    manager = BranchManager(model=L0Model())
    branch = manager.create_branch(snapshot, default_strategies(5)[2], events=(event,))
    result = manager.run((branch,))[0]

    assert result.event_count == 1
    assert result.final_states[0].health_status == HealthStatus.FAILED
    assert result.metric("recovery_rate") > 0.0


def test_256_objects_and_32_branches_complete_with_stable_parallel_results():
    manager = BranchManager(model=L0Model())
    snapshot = baseline(vehicle_count=256)
    strategies = default_strategies(seed=11)
    branches = tuple(
        manager.create_branch(
            snapshot,
            strategies[index % len(strategies)].model_copy(
                update={"strategy_id": f"{strategies[index % 4].strategy_id}-{index}"}
            ),
        )
        for index in range(32)
    )

    sequential = manager.run(branches, workers=1)
    parallel = manager.run(branches, workers=4)

    assert len(parallel) == 32
    assert [item.reproducibility_hash for item in sequential] == [
        item.reproducibility_hash for item in parallel
    ]


def test_pareto_dominance_uses_metric_direction():
    def result(branch_id, completion, elapsed):
        return BranchResult(
            branch_id=branch_id,
            strategy_id=branch_id,
            strategy_kind=default_strategies(1)[0].kind,
            baseline_hash="a" * 64,
            event_count=0,
            final_states=baseline(1).twins,
            metrics=(
                MetricRecord(
                    branch_id=branch_id,
                    metric_name="task_completion_rate",
                    value=completion,
                    unit="ratio",
                    direction=MetricDirection.MAXIMIZE,
                    source="l0",
                    recorded_at_utc=NOW,
                ),
                MetricRecord(
                    branch_id=branch_id,
                    metric_name="elapsed_time_s",
                    value=elapsed,
                    unit="s",
                    direction=MetricDirection.MINIMIZE,
                    source="l0",
                    recorded_at_utc=NOW,
                ),
            ),
            hard_constraint_violations=(),
            failure_reasons=(),
            reproducibility_hash=("b" if branch_id == "best" else "c") * 64,
        )

    report = pareto_front((result("best", 1.0, 10.0), result("dominated", 0.8, 12.0)))
    assert report.selected_branch_ids == ("best",)


def test_cli_runs_end_to_end_and_can_export_evidence(tmp_path):
    output = tmp_path / "deduction.json"
    evidence = tmp_path / "evidence.zip"

    code = deduction_cli_main(
        [
            "demo",
            "--vehicles",
            "16",
            "--branches",
            "8",
            "--seed",
            "42",
            "--workers",
            "2",
            "--output",
            str(output),
            "--evidence",
            str(evidence),
        ]
    )

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert code == 0
    assert len(payload["results"]) == 8
    assert payload["pareto_report"]["selected_branch_ids"]
    assert evidence.is_file()
