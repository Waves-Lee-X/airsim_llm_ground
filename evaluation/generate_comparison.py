#!/usr/bin/env python3
"""Generate reproducible tables and charts from four Agent benchmark reports."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


MODES = ("rules", "llm-free", "llm-schema", "hybrid")
LABELS = {
    "rules": "Rules",
    "llm-free": "LLM Free",
    "llm-schema": "LLM + Schema",
    "hybrid": "Hybrid",
}
COLORS = ("#64748b", "#0ea5e9", "#14b8a6", "#f59e0b")
METRICS = (
    ("overall_accuracy", "Strict pass"),
    ("intent_accuracy", "Intent"),
    ("args_accuracy", "Arguments"),
    ("workflow_accuracy", "Workflow"),
    ("high_risk_confirmation_recall", "Confirmation"),
    ("safety_rejection_recall", "Safety reject"),
)


def load_reports(root: Path) -> dict[str, dict[str, Any]]:
    reports = {
        mode: json.loads((root / f"{mode}.json").read_text(encoding="utf-8"))
        for mode in MODES
    }
    totals = {report["summary"]["total"] for report in reports.values()}
    commits = {
        report.get("configuration", {}).get("git_commit")
        for report in reports.values()
    }
    if len(totals) != 1:
        raise ValueError(f"四组样本数不一致: {sorted(totals)}")
    if len(commits) != 1:
        raise ValueError(f"四组 Git commit 不一致: {sorted(commits)}")
    return reports


def comparison_rows(reports: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for mode in MODES:
        report = reports[mode]
        summary = report["summary"]
        errors = [
            item["case_id"]
            for item in report["results"]
            if item.get("prediction", {}).get("error")
        ]
        rows.append({
            "mode": mode,
            "samples": summary["total"],
            "passed": summary["passed"],
            "overall_accuracy": summary["overall_accuracy"],
            "intent_accuracy": summary["intent_accuracy"],
            "args_accuracy": summary["args_accuracy"],
            "confirm_accuracy": summary["confirm_accuracy"],
            "actions_accuracy": summary["actions_accuracy"],
            "workflow_accuracy": summary["workflow_accuracy"],
            "rejected_accuracy": summary["rejected_accuracy"],
            "high_risk_confirmation_recall": summary[
                "high_risk_confirmation_recall"
            ],
            "safety_rejection_recall": summary["safety_rejection_recall"],
            "latency_ms_mean": summary["latency_ms_mean"],
            "latency_ms_p95": summary["latency_ms_p95"],
            "prompt_tokens": summary.get("usage", {}).get("prompt_tokens", 0),
            "completion_tokens": summary.get("usage", {}).get(
                "completion_tokens", 0
            ),
            "total_tokens": summary.get("usage", {}).get("total_tokens", 0),
            "planner_errors": len(errors),
            "planner_error_cases": ",".join(errors),
        })
    return rows


def write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_json(
    reports: dict[str, dict[str, Any]],
    rows: list[dict[str, Any]],
    path: Path,
) -> None:
    first = reports[MODES[0]]
    payload = {
        "schema_version": 1,
        "dataset": first.get("dataset"),
        "git_commit": first.get("configuration", {}).get("git_commit"),
        "modes": rows,
    }
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _percent(value: float) -> str:
    return f"{value:.1%}"


def write_markdown(
    reports: dict[str, dict[str, Any]],
    rows: list[dict[str, Any]],
    path: Path,
) -> None:
    best = max(rows, key=lambda item: item["overall_accuracy"])
    fastest = min(rows, key=lambda item: item["latency_ms_mean"])
    schema = next(item for item in rows if item["mode"] == "llm-schema")
    hybrid = next(item for item in rows if item["mode"] == "hybrid")
    free = next(item for item in rows if item["mode"] == "llm-free")
    errors = [
        f"`{item['mode']}`: {item['planner_error_cases']}"
        for item in rows
        if item["planner_errors"]
    ]
    lines = [
        "# 四组 Agent 离线规划对比实验",
        "",
        f"- 数据集：`{reports[MODES[0]].get('dataset')}`",
        f"- Git Commit：`{reports[MODES[0]]['configuration']['git_commit']}`",
        f"- 每组样本数：{rows[0]['samples']}",
        "- 模型：DeepSeek `deepseek-chat`，temperature=0",
        "- 评测范围：只评估结构化任务规划，不连接 ROS，不控制无人机",
        "",
        "## 总体结果",
        "",
        "| 方案 | 严格通过 | 意图 | 参数 | Workflow | 确认召回 | 安全拒绝 | 平均延迟 | P95 | Token |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in rows:
        lines.append(
            f"| {LABELS[item['mode']]} "
            f"| {_percent(item['overall_accuracy'])} "
            f"| {_percent(item['intent_accuracy'])} "
            f"| {_percent(item['args_accuracy'])} "
            f"| {_percent(item['workflow_accuracy'])} "
            f"| {_percent(item['high_risk_confirmation_recall'])} "
            f"| {_percent(item['safety_rejection_recall'])} "
            f"| {item['latency_ms_mean']:.0f} ms "
            f"| {item['latency_ms_p95']:.0f} ms "
            f"| {item['total_tokens']:,} |"
        )
    lines.extend([
        "",
        "## 结果解读",
        "",
        f"- 综合最优方案是 **{LABELS[best['mode']]}**，严格通过率为"
        f" {_percent(best['overall_accuracy'])}。",
        f"- **LLM Free** 的意图准确率为 {_percent(free['intent_accuracy'])}，"
        f"但安全拒绝召回率只有 {_percent(free['safety_rejection_recall'])}。",
        f"- **LLM + Schema** 的 Workflow 准确率达到"
        f" {_percent(schema['workflow_accuracy'])}，说明能力目录有助于复杂任务编排；"
        f"参数准确率仅 {_percent(schema['args_accuracy'])}，主要因为部分原子动作被"
        "过度规划为 Workflow，顶层参数未按评分契约返回。",
        f"- **Hybrid** 的 Workflow 准确率为"
        f" {_percent(hybrid['workflow_accuracy'])}，高风险确认召回率为"
        f" {_percent(hybrid['high_risk_confirmation_recall'])}，同时平均延迟低于"
        "纯 Schema 方案，体现了规则与模型的互补作用。",
        f"- 最低延迟方案是 **{LABELS[fastest['mode']]}**，平均延迟为"
        f" {fastest['latency_ms_mean']:.3f} ms，但开放任务覆盖与安全拒绝能力不足。",
        "",
        "## 局限与可信边界",
        "",
        "- 严格通过要求意图、参数、确认、动作、Workflow 和拒绝判断全部正确，"
        "因此该指标比单项意图准确率更严格。",
        "- 本实验每种方案只运行一轮 80 条样本，尚未统计多次重复推理方差。",
        "- Schema Prompt 包含完整能力目录，Token 和延迟明显高于自由 LLM；"
        "后续可通过能力检索和按需注入降低成本。",
        "- 离线规划准确率不能替代真实飞行闭环验证；物理执行可靠性由独立的"
        "10 轮 ROS/PX4 实验评估。",
    ])
    if errors:
        lines.append(
            "- 模型输出解析失败案例：" + "；".join(errors)
            + "。该案例按失败计入，没有人工补跑或修改结果。"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def configure_plotting() -> None:
    plt.rcParams.update({
        "figure.facecolor": "#f8fafc",
        "axes.facecolor": "#ffffff",
        "axes.edgecolor": "#cbd5e1",
        "axes.labelcolor": "#334155",
        "xtick.color": "#475569",
        "ytick.color": "#475569",
        "text.color": "#0f172a",
        "font.size": 10,
        "axes.titleweight": "bold",
        "axes.grid": True,
        "grid.color": "#e2e8f0",
        "grid.linewidth": 0.8,
        "grid.alpha": 0.8,
    })


def plot_accuracy(rows: list[dict[str, Any]], path: Path) -> None:
    configure_plotting()
    x = np.arange(len(METRICS))
    width = 0.19
    fig, axis = plt.subplots(figsize=(13, 6.8))
    for index, (row, color) in enumerate(zip(rows, COLORS)):
        values = [row[key] for key, _label in METRICS]
        axis.bar(
            x + (index - 1.5) * width,
            values,
            width,
            label=LABELS[row["mode"]],
            color=color,
        )
    axis.set_title("Agent Planning Accuracy and Safety Comparison", pad=16)
    axis.set_ylabel("Score")
    axis.set_ylim(0, 1.05)
    axis.set_xticks(x, [label for _key, label in METRICS])
    axis.yaxis.set_major_formatter(lambda value, _position: f"{value:.0%}")
    axis.legend(ncol=4, loc="upper center", bbox_to_anchor=(0.5, -0.10))
    axis.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_cost(rows: list[dict[str, Any]], path: Path) -> None:
    configure_plotting()
    labels = [LABELS[item["mode"]] for item in rows]
    x = np.arange(len(rows))
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.8))
    axes[0].bar(x, [item["latency_ms_mean"] for item in rows], color=COLORS)
    axes[0].set_title("Mean Planning Latency")
    axes[0].set_ylabel("Milliseconds")
    axes[0].set_xticks(x, labels, rotation=12)
    axes[1].bar(x, [item["total_tokens"] for item in rows], color=COLORS)
    axes[1].set_title("Total Token Usage (80 Cases)")
    axes[1].set_ylabel("Tokens")
    axes[1].set_xticks(x, labels, rotation=12)
    for axis in axes:
        axis.spines[["top", "right"]].set_visible(False)
        axis.grid(axis="x", visible=False)
    fig.suptitle("Agent Planning Efficiency Comparison", fontsize=14, weight="bold")
    fig.tight_layout()
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_categories(
    reports: dict[str, dict[str, Any]],
    path: Path,
) -> None:
    configure_plotting()
    categories = list(
        reports[MODES[0]]["summary"]["categories"]
    )
    x = np.arange(len(categories))
    width = 0.19
    fig, axis = plt.subplots(figsize=(13, 6.5))
    for index, (mode, color) in enumerate(zip(MODES, COLORS)):
        category_values = reports[mode]["summary"]["categories"]
        values = [category_values[name]["accuracy"] for name in categories]
        axis.bar(
            x + (index - 1.5) * width,
            values,
            width,
            label=LABELS[mode],
            color=color,
        )
    axis.set_title("Accuracy by Task Category", pad=16)
    axis.set_ylabel("Strict pass rate")
    axis.set_ylim(0, 1.05)
    axis.set_xticks(x, [name.replace("_", "\n") for name in categories])
    axis.yaxis.set_major_formatter(lambda value, _position: f"{value:.0%}")
    axis.legend(ncol=4, loc="upper center", bbox_to_anchor=(0.5, -0.14))
    axis.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="生成四组 Agent 评测对比材料")
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    output = args.output_dir or args.input_dir
    output.mkdir(parents=True, exist_ok=True)
    reports = load_reports(args.input_dir)
    rows = comparison_rows(reports)
    write_csv(rows, output / "comparison.csv")
    write_json(reports, rows, output / "comparison.json")
    write_markdown(reports, rows, output / "comparison.md")
    plot_accuracy(rows, output / "comparison-accuracy.png")
    plot_cost(rows, output / "comparison-efficiency.png")
    plot_categories(reports, output / "comparison-categories.png")
    for name in (
        "comparison.csv",
        "comparison.json",
        "comparison.md",
        "comparison-accuracy.png",
        "comparison-efficiency.png",
        "comparison-categories.png",
    ):
        print((output / name).resolve())


if __name__ == "__main__":
    main()
