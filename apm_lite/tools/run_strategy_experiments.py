#!/usr/bin/env python3
"""Run and report a reproducible L0 strategy/scenario experiment matrix."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import statistics
from collections import defaultdict
from datetime import timedelta
from pathlib import Path

from aeromind_apm_lite.common.contracts.twin import TwinSource, WorldEvent
from aeromind_apm_lite.ground.deduction import BranchManager, default_strategies
from aeromind_apm_lite.ground.deduction.cli import build_demo_snapshot


SCENARIOS = ("nominal", "link_degradation", "vehicle_failure", "target_change", "low_battery")


def scenario_events(snapshot, scenario: str) -> tuple[WorldEvent, ...]:
    if scenario == "nominal":
        return ()
    parameters = {
        "link_degradation": {"quality": 0.3},
        "vehicle_failure": {},
        "target_change": {"delta": max(1, len(snapshot.twins) // 4)},
        "low_battery": {"energy_penalty": 0.2},
    }[scenario]
    affected = () if scenario == "target_change" else (1,)
    return (
        WorldEvent(
            event_type=scenario,
            occurs_at_utc=snapshot.captured_at_utc
            + timedelta(seconds=snapshot.duration_s // 2),
            affected_vehicle_ids=affected,
            parameters=parameters,
            source=TwinSource.SIM,
        ),
    )


def run_matrix(*, vehicles: int, duration: int, seeds: list[int], workers: int) -> dict:
    snapshot = build_demo_snapshot(vehicles, duration)
    manager = BranchManager()
    records = []
    reproducible = True
    for seed in seeds:
        for scenario in SCENARIOS:
            branches = tuple(
                manager.create_branch(snapshot, strategy, events=scenario_events(snapshot, scenario))
                for strategy in default_strategies(seed)
            )
            first = manager.run(branches, workers=workers)
            second = manager.run(branches, workers=workers)
            reproducible &= [r.reproducibility_hash for r in first] == [
                r.reproducibility_hash for r in second
            ]
            for result in first:
                row = {
                    "seed": seed,
                    "scenario": scenario,
                    "strategy": result.strategy_kind.value,
                    "eligible": not result.hard_constraint_violations,
                    "reproducibility_hash": result.reproducibility_hash,
                }
                row.update({metric.metric_name: metric.value for metric in result.metrics})
                records.append(row)

    grouped = defaultdict(list)
    for row in records:
        grouped[(row["scenario"], row["strategy"])].append(row)
    metric_names = (
        "task_completion_rate",
        "elapsed_time_s",
        "energy_consumption_ratio",
        "communication_load_ratio",
        "minimum_separation_m",
        "recovery_rate",
    )
    summaries = []
    for (scenario, strategy), rows in sorted(grouped.items()):
        summary = {
            "scenario": scenario,
            "strategy": strategy,
            "runs": len(rows),
            "eligible_runs": sum(bool(row["eligible"]) for row in rows),
        }
        for metric in metric_names:
            values = [float(row[metric]) for row in rows]
            summary[f"{metric}_mean"] = statistics.fmean(values)
            summary[f"{metric}_stdev"] = statistics.pstdev(values)
            summary[f"{metric}_worst"] = min(values) if metric in {
                "task_completion_rate", "minimum_separation_m", "recovery_rate"
            } else max(values)
        summaries.append(summary)
    payload = {
        "experiment": "parallel-strategy-multiseed-l0-v1",
        "fidelity": "L0 event-level model",
        "vehicles": vehicles,
        "duration_s": duration,
        "seeds": seeds,
        "scenarios": list(SCENARIOS),
        "strategies": [item.kind.value for item in default_strategies(seeds[0])],
        "matrix_runs": len(records),
        "duplicate_verification_runs": len(records),
        "all_duplicate_hashes_match": reproducible,
        "limitations": [
            "Results are L0 model evidence and do not establish L2 SITL or L3 flight performance.",
            "Strategy parameters are declared model inputs; physical calibration is not claimed.",
        ],
        "summary": summaries,
        "records": records,
    }
    canonical = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    payload["content_sha256"] = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    return payload


def write_outputs(payload: dict, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "strategy-experiment.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    summaries = payload["summary"]
    with (output_dir / "strategy-experiment.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)

    lines = [
        "# L0四策略多种子实验报告",
        "",
        f"- 实验矩阵：4种策略 × 5类场景 × {len(payload['seeds'])}个种子 = {payload['matrix_runs']}次有效运行",
        f"- 重复性复核：另执行{payload['duplicate_verification_runs']}次；哈希全部一致：{payload['all_duplicate_hashes_match']}",
        f"- 对象数量：{payload['vehicles']}；单次仿真时长：{payload['duration_s']} s",
        f"- 内容SHA-256：`{payload['content_sha256']}`",
        "- 证据边界：仅代表L0事件级模型，不代表SITL轨迹验收或真实飞行性能。",
        "",
        "| 场景 | 策略 | 完成率（均值±标准差） | 用时s | 能耗 | 通信负载 | 恢复率 | 硬约束通过 |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summaries:
        lines.append(
            "| {scenario} | {strategy} | {task_completion_rate_mean:.4f}±{task_completion_rate_stdev:.4f} "
            "| {elapsed_time_s_mean:.2f} | {energy_consumption_ratio_mean:.4f} "
            "| {communication_load_ratio_mean:.4f} | {recovery_rate_mean:.4f} "
            "| {eligible_runs}/{runs} |".format(**row)
        )
    (output_dir / "strategy-experiment.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--vehicles", type=int, default=32)
    parser.add_argument("--duration", type=int, default=120)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(20260820, 20260830)))
    args = parser.parse_args()
    payload = run_matrix(
        vehicles=args.vehicles, duration=args.duration, seeds=args.seeds, workers=args.workers
    )
    write_outputs(payload, args.output_dir)
    print(json.dumps({key: payload[key] for key in (
        "matrix_runs", "duplicate_verification_runs", "all_duplicate_hashes_match", "content_sha256"
    )}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
