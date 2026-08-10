"""Human-readable summaries for independently reviewing deduction artifacts."""

from __future__ import annotations

from aeromind_apm_lite.ground.deduction.branch import DeductionRun


def deduction_markdown_report(run: DeductionRun) -> str:
    lines = [
        "# L0 parallel deduction report",
        "",
        f"- Run: `{run.run_id}`",
        f"- Branches: {len(run.results)}",
        f"- Pareto branches: {', '.join(run.pareto_report.selected_branch_ids) or 'none'}",
        f"- Wall time: {run.wall_time_s:.6f} s",
        f"- Simulated time: {run.simulated_time_s:.3f} s",
        f"- Realtime factor: {run.realtime_factor:.3f}x",
        "",
        "| Branch | Strategy | Completion | Separation (m) | Elapsed (s) | Eligible |",
        "|---|---|---:|---:|---:|---|",
    ]
    for result in run.results:
        lines.append(
            "| {branch} | {strategy} | {completion:.4f} | {separation:.3f} | "
            "{elapsed:.3f} | {eligible} |".format(
                branch=result.branch_id,
                strategy=result.strategy_kind.value,
                completion=result.metric("task_completion_rate"),
                separation=result.metric("minimum_separation_m"),
                elapsed=result.metric("elapsed_time_s"),
                eligible="yes" if not result.hard_constraint_violations else "no",
            )
        )
    return "\n".join(lines) + "\n"


__all__ = ["deduction_markdown_report"]
