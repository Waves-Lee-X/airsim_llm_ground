"""Command-line entry point for reproducible L0 parallel deduction demos."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Sequence
from uuid import NAMESPACE_URL, uuid5

from aeromind_apm_lite.common.contracts import CoordinateFrame, Vector3
from aeromind_apm_lite.common.contracts.twin import (
    FidelityLevel,
    HealthStatus,
    TwinSource,
    TwinState,
    WorldEvent,
)
from aeromind_apm_lite.common.evidence import (
    EvidencePackageBuilder,
    ExperimentLock,
    deduction_markdown_report,
)

from .branch import BaselineSnapshot, BranchManager, DeductionRun, canonical_hash
from .pareto import pareto_front
from .strategies import default_strategies


REFERENCE_TIME = datetime(2026, 8, 10, 8, 0, tzinfo=timezone.utc)
ROOT = Path(__file__).resolve().parents[5]


def build_demo_snapshot(vehicle_count: int, duration_s: int) -> BaselineSnapshot:
    twins = tuple(
        TwinState(
            vehicle_id=index + 1,
            platform_type="multirotor",
            frame=CoordinateFrame.MAP,
            frame_calibration_id="m1-demo-map-v1",
            position_m=Vector3(
                x=float((index % 16) * 5),
                y=float((index // 16) * 5),
                z=5.0,
            ),
            energy_remaining=0.9,
            communication_quality=0.95,
            mission_phase="ready",
            health_status=HealthStatus.NOMINAL,
            source=TwinSource.SIM,
            confidence=1.0,
            fidelity=FidelityLevel.L0,
            sampled_at_utc=REFERENCE_TIME,
            simulated_at_utc=REFERENCE_TIME,
        )
        for index in range(vehicle_count)
    )
    return BaselineSnapshot(
        snapshot_id=f"m1-demo-{vehicle_count}",
        captured_at_utc=REFERENCE_TIME,
        duration_s=duration_s,
        target_count=vehicle_count * 2,
        minimum_separation_m=2.0,
        reserve_energy=0.1,
        twins=twins,
    )


def run_demo(
    *,
    vehicle_count: int,
    branch_count: int,
    seed: int,
    workers: int,
    duration_s: int,
) -> DeductionRun:
    snapshot = build_demo_snapshot(vehicle_count, duration_s)
    manager = BranchManager()
    strategies = default_strategies(seed)
    branches = []
    for index in range(branch_count):
        scenario_index = index // len(strategies)
        strategy = strategies[index % len(strategies)].model_copy(
            update={"strategy_id": f"{strategies[index % 4].strategy_id}-s{scenario_index}"}
        )
        events = _scenario_events(snapshot, scenario_index)
        branches.append(manager.create_branch(snapshot, strategy, events=events))

    started = time.perf_counter()
    results = manager.run(branches, workers=workers)
    wall_time = time.perf_counter() - started
    simulated_time = float(duration_s * branch_count)
    report = pareto_front(results)
    run_identity = canonical_hash(
        {
            "baseline_hash": snapshot.baseline_hash,
            "results": [item.reproducibility_hash for item in results],
            "pareto": report.model_dump(mode="json"),
        }
    )
    return DeductionRun(
        run_id=f"run-{run_identity[:20]}",
        baseline_hash=snapshot.baseline_hash,
        results=results,
        pareto_report=report,
        wall_time_s=wall_time,
        simulated_time_s=simulated_time,
        realtime_factor=simulated_time / max(wall_time, 1e-12),
    )


def _scenario_events(
    snapshot: BaselineSnapshot,
    scenario_index: int,
) -> tuple[WorldEvent, ...]:
    if scenario_index == 0:
        return ()
    vehicle_id = (scenario_index - 1) % len(snapshot.twins) + 1
    event_mode = (scenario_index - 1) % 4
    if event_mode == 0:
        event_type = "link_degradation"
        parameters = {"quality": 0.3}
        affected = (vehicle_id,)
    elif event_mode == 1:
        event_type = "vehicle_failure"
        parameters = {}
        affected = (vehicle_id,)
    elif event_mode == 2:
        event_type = "target_change"
        parameters = {"delta": max(1, len(snapshot.twins) // 4)}
        affected = ()
    else:
        event_type = "low_battery"
        parameters = {"energy_penalty": 0.2}
        affected = (vehicle_id,)
    identity = f"{snapshot.snapshot_id}:{scenario_index}:{event_type}:{vehicle_id}"
    return (
        WorldEvent(
            event_id=uuid5(NAMESPACE_URL, identity),
            event_type=event_type,
            occurs_at_utc=snapshot.captured_at_utc
            + timedelta(seconds=snapshot.duration_s // 2),
            affected_vehicle_ids=affected,
            parameters=parameters,
            source=TwinSource.SIM,
        ),
    )


def _code_commit() -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        return result.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def export_run_evidence(
    run: DeductionRun,
    output: Path,
    *,
    vehicle_count: int,
    branch_count: int,
    seed: int,
    workers: int,
    duration_s: int,
) -> str:
    configuration = {
        "vehicle_count": vehicle_count,
        "branch_count": branch_count,
        "seed": seed,
        "workers": workers,
        "duration_s": duration_s,
    }
    lock = ExperimentLock.from_inputs(
        code_commit=_code_commit(),
        software_versions={"aeromind-apm-lite": "0.1.0"},
        model_versions={"l0": "l0-event-v1"},
        algorithm_versions={"pareto": "pareto-v1"},
        scenario={"baseline_hash": run.baseline_hash},
        configuration=configuration,
        random_seeds=(seed,),
    )
    builder = EvidencePackageBuilder(
        experiment_id=run.run_id,
        created_at_utc=REFERENCE_TIME,
        lock=lock,
        requirements=(
            "M1-WP2-DETERMINISTIC-BRANCHES",
            "M1-WP2-PARETO",
            "M1-WP6-HASH-VERIFICATION",
        ),
        applicability=("L0 event-level simulation",),
        limitations=(
            "No L2 SITL or L3 physical ranking claim is included.",
            "Wall-clock throughput depends on the executing host.",
        ),
        conclusion="Eligible L0 strategies were compared with a shared metric contract.",
    )
    builder.add_json("deduction-run", run.model_dump(mode="json"), role="result")
    builder.add_bytes(
        "deduction-report",
        deduction_markdown_report(run).encode("utf-8"),
        role="report",
        media_type="text/markdown",
        suffix=".md",
    )
    package = builder.export(output)
    return package.package_hash


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    demo = subparsers.add_parser("demo")
    demo.add_argument("--vehicles", type=int, default=256)
    demo.add_argument("--branches", type=int, default=32)
    demo.add_argument("--seed", type=int, default=20260810)
    demo.add_argument("--workers", type=int, default=1)
    demo.add_argument("--duration", type=int, default=120)
    demo.add_argument("--output", type=Path, required=True)
    demo.add_argument("--evidence", type=Path)
    args = parser.parse_args(argv)

    if not 2 <= args.vehicles <= 10_000:
        parser.error("--vehicles must be in [2, 10000]")
    if not 1 <= args.branches <= 1_024:
        parser.error("--branches must be in [1, 1024]")
    if args.workers < 1:
        parser.error("--workers must be at least one")
    run = run_demo(
        vehicle_count=args.vehicles,
        branch_count=args.branches,
        seed=args.seed,
        workers=args.workers,
        duration_s=args.duration,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(run.model_dump(mode="json"), ensure_ascii=False, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )
    package_hash = None
    if args.evidence is not None:
        package_hash = export_run_evidence(
            run,
            args.evidence,
            vehicle_count=args.vehicles,
            branch_count=args.branches,
            seed=args.seed,
            workers=args.workers,
            duration_s=args.duration,
        )
    print(
        json.dumps(
            {
                "run_id": run.run_id,
                "branches": len(run.results),
                "pareto": list(run.pareto_report.selected_branch_ids),
                "realtime_factor": run.realtime_factor,
                "evidence_package_hash": package_hash,
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
