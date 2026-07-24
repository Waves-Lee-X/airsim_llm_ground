"""Aggregate repeatable mission execution metrics for contest evidence."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sqlite3
import statistics
import time
from typing import Any, Iterable
from urllib.parse import quote


SUCCESS_STATES = {"completed", "done", "success"}
TERMINAL_STATES = SUCCESS_STATES | {"failed", "cancelled", "expired", "interrupted"}


def load_gateway_missions(db_path: Path) -> list[dict[str, Any]]:
    if not db_path.is_file():
        return []
    uri = f"file:{quote(str(db_path.resolve()))}?mode=ro"
    try:
        connection = sqlite3.connect(uri, uri=True)
    except sqlite3.Error:
        return []
    connection.row_factory = sqlite3.Row
    try:
        try:
            table = connection.execute(
                "SELECT name FROM sqlite_master WHERE type='table' AND name='gateway_missions'"
            ).fetchone()
            if table is None:
                return []
            rows = connection.execute("SELECT * FROM gateway_missions ORDER BY created_at").fetchall()
            timelines = _load_event_timelines(connection)
            confirmations = _load_confirmations(connection)
        except sqlite3.Error:
            return []
    finally:
        connection.close()
    records = []
    for row in rows:
        value = dict(row)
        confirmation_id = str(value.get("confirmation_id") or "")
        records.append(
            _gateway_record(
                value,
                timelines.get(confirmation_id, {}),
                confirmations.get(confirmation_id, {}),
            )
        )
    return records


def load_file_missions(root: Path) -> list[dict[str, Any]]:
    records = []
    if not root.is_dir():
        return records
    for path in root.glob("*/mission.json"):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if isinstance(payload, dict):
            records.append(_file_record(payload, path))
    return records


def _gateway_record(
    row: dict[str, Any],
    timeline: dict[str, Any] | None = None,
    confirmation: dict[str, Any] | None = None,
) -> dict[str, Any]:
    result = _json_object(row.get("result_json"))
    timeline = timeline or {}
    confirmation = confirmation or {}
    created = _number(row.get("created_at"))
    updated = _number(row.get("updated_at"))
    workflow = _workflow_metrics(result)
    required_at = timeline.get("confirmation_required_at") or created
    resolved_at = timeline.get("confirmation_resolved_at") or _number(confirmation.get("resolved_at"))
    started_at = timeline.get("control_started_at")
    completed_at = timeline.get("control_completed_at")
    verifying_at = timeline.get("verification_started_at")
    terminal_event_count = int(timeline.get("terminal_event_count") or 0)
    status = _status(row.get("status"))
    event_trace_available = bool(timeline)
    terminal_consistent = True
    if event_trace_available:
        terminal_consistent = (
            terminal_event_count == 1
            if status in SUCCESS_STATES | {"failed"}
            else terminal_event_count <= 1
        )
    return {
        "id": row.get("id"),
        "source": "gateway_db",
        "confirmation_id": row.get("confirmation_id"),
        "action": row.get("action") or "unknown",
        "title": row.get("title") or "",
        "status": status,
        "phase": row.get("phase") or "",
        "revision": int(row.get("revision") or 0),
        "physical_complete": bool(row.get("physical_complete")),
        "created_at": created,
        "updated_at": updated,
        "duration_sec": _duration(created, updated),
        "confirmation_latency_sec": _duration(required_at, resolved_at),
        "execution_duration_sec": _duration(started_at, completed_at),
        "verification_duration_sec": _duration(verifying_at, completed_at),
        "event_trace_available": event_trace_available,
        "terminal_event_count": terminal_event_count,
        "terminal_consistent": terminal_consistent,
        "failure_reason": _failure_reason(result, row.get("phase")),
        **workflow,
        "result": result,
    }


def _file_record(payload: dict[str, Any], path: Path) -> dict[str, Any]:
    parsed = payload.get("parsed") or {}
    created = _number(payload.get("created_at"))
    updated = _number(payload.get("updated_at") or payload.get("written_at"))
    status = _status(payload.get("status"))
    steps = payload.get("steps") or []
    failed_step = next((item for item in steps if _status(item.get("status")) == "failed"), None)
    return {
        "id": payload.get("id") or path.parent.name,
        "source": "mission_json",
        "action": parsed.get("intent") or "workflow",
        "title": parsed.get("reason") or payload.get("message") or "",
        "status": status,
        "phase": payload.get("phase") or "",
        "revision": int(payload.get("revision") or 0),
        "physical_complete": status in SUCCESS_STATES,
        "created_at": created,
        "updated_at": updated,
        "duration_sec": _duration(created, updated),
        "confirmation_latency_sec": None,
        "execution_duration_sec": None,
        "verification_duration_sec": None,
        "event_trace_available": False,
        "terminal_event_count": 0,
        "terminal_consistent": True,
        "failure_reason": _failure_reason(failed_step or payload, payload.get("message")),
        **_workflow_metrics({"workflow": {"steps": steps}}),
        "result": {"steps": steps, "message": payload.get("message")},
    }


def filter_records(records: Iterable[dict[str, Any]], since_hours: float | None) -> list[dict[str, Any]]:
    values = list(records)
    if since_hours is None:
        return values
    threshold = time.time() - max(0.0, since_hours) * 3600.0
    return [item for item in values if (item.get("created_at") or 0.0) >= threshold]


def select_campaign(
    records: Iterable[dict[str, Any]],
    action: str | None = None,
    title_contains: str = "",
    latest: int | None = None,
) -> list[dict[str, Any]]:
    selected = list(records)
    if action:
        selected = [item for item in selected if item.get("action") == action]
    title_term = title_contains.strip().casefold()
    if title_term:
        selected = [
            item for item in selected
            if title_term in str(item.get("title") or "").casefold()
        ]
    selected.sort(key=lambda item: item.get("created_at") or 0.0)
    if latest is not None and latest > 0:
        selected = selected[-latest:]
    return selected


def summarize(
    records: list[dict[str, Any]],
    expected_runs: int = 0,
    target_success_rate: float = 0.9,
    require_takeoff_completed: bool = False,
    require_return_to_start: bool = False,
) -> dict[str, Any]:
    terminal = [item for item in records if item["status"] in TERMINAL_STATES]
    completed = [item for item in terminal if item["status"] in SUCCESS_STATES]
    physically_complete = [item for item in terminal if item.get("physical_complete")]
    durations = [float(item["duration_sec"]) for item in terminal if item.get("duration_sec") is not None]
    confirmation_durations = _metric_values(terminal, "confirmation_latency_sec")
    execution_durations = _metric_values(terminal, "execution_duration_sec")
    verification_durations = _metric_values(terminal, "verification_duration_sec")
    status_counts = _counts(item["status"] for item in records)
    action_counts = _counts(item["action"] for item in records)
    failure_counts = _counts(
        item["failure_reason"] or "未记录原因"
        for item in terminal
        if item["status"] not in SUCCESS_STATES
    )
    denominator = len(terminal)
    success_rate = round(len(completed) / denominator, 4) if denominator else None
    physical_rate = round(len(physically_complete) / denominator, 4) if denominator else None
    duplicate_terminal_events = sum(
        max(0, int(item.get("terminal_event_count") or 0) - 1) for item in records
    )
    consistent = sum(bool(item.get("terminal_consistent", True)) for item in terminal)
    event_traced = sum(bool(item.get("event_trace_available")) for item in terminal)
    takeoff_completed = sum(
        item.get("takeoff_status") == "completed" for item in terminal
    )
    returned_to_start = sum(
        item.get("return_to_start_within_tolerance") is True for item in terminal
    )
    expected_runs = max(0, int(expected_runs))
    target_success_rate = min(1.0, max(0.0, float(target_success_rate)))
    campaign_ready = None
    if expected_runs:
        campaign_ready = bool(
            denominator == expected_runs
            and success_rate is not None
            and success_rate >= target_success_rate
            and physical_rate is not None
            and physical_rate >= target_success_rate
            and duplicate_terminal_events == 0
            and consistent == denominator
            and event_traced == denominator
            and (
                not require_takeoff_completed
                or takeoff_completed == denominator
            )
            and (
                not require_return_to_start
                or returned_to_start == denominator
            )
        )
    return {
        "total": len(records),
        "terminal": denominator,
        "nonterminal": len(records) - denominator,
        "completed": len(completed),
        "success_rate": success_rate,
        "physical_complete_rate": physical_rate,
        "duration_mean_sec": round(statistics.fmean(durations), 3) if durations else None,
        "duration_p95_sec": round(_percentile(durations, 0.95), 3) if durations else None,
        **_timing_summary("confirmation", confirmation_durations),
        **_timing_summary("execution", execution_durations),
        **_timing_summary("verification", verification_durations),
        "terminal_consistency_rate": round(consistent / denominator, 4) if denominator else None,
        "event_trace_rate": round(event_traced / denominator, 4) if denominator else None,
        "takeoff_completed": takeoff_completed,
        "takeoff_completion_rate": (
            round(takeoff_completed / denominator, 4) if denominator else None
        ),
        "return_to_start_completed": returned_to_start,
        "return_to_start_rate": (
            round(returned_to_start / denominator, 4) if denominator else None
        ),
        "require_takeoff_completed": bool(require_takeoff_completed),
        "require_return_to_start": bool(require_return_to_start),
        "duplicate_terminal_events": duplicate_terminal_events,
        "failed_steps": _counts(
            str(item.get("failed_step") or "未记录步骤")
            for item in terminal
            if item["status"] not in SUCCESS_STATES
        ),
        "expected_runs": expected_runs or None,
        "target_success_rate": target_success_rate,
        "campaign_ready": campaign_ready,
        "status_counts": status_counts,
        "action_counts": action_counts,
        "failure_reasons": failure_counts,
    }


def write_outputs(report: dict[str, Any], output_dir: Path, prefix: str) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        "json": output_dir / f"{prefix}.json",
        "csv": output_dir / f"{prefix}.csv",
        "markdown": output_dir / f"{prefix}.md",
    }
    paths["json"].write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    with paths["csv"].open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=[
            "id", "source", "action", "status", "physical_complete",
            "revision", "phase", "created_at", "updated_at", "duration_sec",
            "confirmation_latency_sec", "execution_duration_sec", "verification_duration_sec",
            "event_trace_available", "terminal_event_count", "terminal_consistent",
            "takeoff_status", "return_to_start_status",
            "return_to_start_error_m", "return_to_start_tolerance_m",
            "return_to_start_within_tolerance",
            "failed_step", "failure_reason", "title",
        ])
        writer.writeheader()
        for item in report["records"]:
            writer.writerow({key: item.get(key) for key in writer.fieldnames})
    paths["markdown"].write_text(_markdown(report), encoding="utf-8")
    return paths


def _markdown(report: dict[str, Any]) -> str:
    summary = report["summary"]
    rate = lambda value: "--" if value is None else f"{value * 100:.1f}%"
    duration = lambda value: "--" if value is None else f"{value:.2f}s"
    lines = [
        "# 无人机任务稳定性实验报告", "",
        f"- 生成时间：{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(report['generated_at']))}",
        f"- 样本总数：{summary['total']}",
        f"- 终态任务：{summary['terminal']}",
        f"- 成功率：{rate(summary['success_rate'])}",
        f"- 物理完成率：{rate(summary['physical_complete_rate'])}",
        f"- 平均耗时：{duration(summary['duration_mean_sec'])}",
        f"- P95 耗时：{duration(summary['duration_p95_sec'])}", "",
        "## 验收门禁", "",
        f"- 判定：{_campaign_label(summary)}",
        f"- 样本覆盖：{summary['terminal']}/{summary.get('expected_runs') or '--'}",
        f"- 目标成功率：{rate(summary.get('target_success_rate'))}",
        f"- 终态一致率：{rate(summary.get('terminal_consistency_rate'))}",
        f"- 事件链覆盖率：{rate(summary.get('event_trace_rate'))}",
        f"- 完整起飞率：{rate(summary.get('takeoff_completion_rate'))}",
        f"- 返回任务起点率：{rate(summary.get('return_to_start_rate'))}",
        f"- 重复终态事件：{summary.get('duplicate_terminal_events', 0)}", "",
        "## 链路耗时", "",
        "| 阶段 | 平均 | P95 |", "|---|---:|---:|",
        f"| 人工确认 | {duration(summary.get('confirmation_mean_sec'))} | {duration(summary.get('confirmation_p95_sec'))} |",
        f"| 物理执行 | {duration(summary.get('execution_mean_sec'))} | {duration(summary.get('execution_p95_sec'))} |",
        f"| 状态验证 | {duration(summary.get('verification_mean_sec'))} | {duration(summary.get('verification_p95_sec'))} |", "",
        "## 状态分布", "", "| 状态 | 数量 |", "|---|---:|",
    ]
    lines.extend(f"| {key} | {value} |" for key, value in summary["status_counts"].items())
    lines.extend(["", "## 失败原因", "", "| 原因 | 数量 |", "|---|---:|"])
    if summary["failure_reasons"]:
        lines.extend(f"| {key.replace('|', '/')} | {value} |" for key, value in summary["failure_reasons"].items())
    else:
        lines.append("| 暂无失败样本 | 0 |")
    lines.extend(["", "## 失败步骤", "", "| 步骤 | 数量 |", "|---|---:|"])
    if summary.get("failed_steps"):
        lines.extend(f"| {key.replace('|', '/')} | {value} |" for key, value in summary["failed_steps"].items())
    else:
        lines.append("| 暂无失败步骤 | 0 |")
    lines.extend(["", "## 明细", "", "| ID | 动作 | 状态 | 物理完成 | 回程误差 | 总耗时 | 执行耗时 | 失败步骤 |", "|---|---|---|---:|---:|---:|---:|---|"])
    for item in report["records"]:
        return_error = item.get("return_to_start_error_m")
        return_error_label = (
            "--" if return_error is None else f"{float(return_error):.2f}m"
        )
        lines.append(
            f"| {item['id']} | {item['action']} | {item['status']} | "
            f"{'是' if item.get('physical_complete') else '否'} | "
            f"{return_error_label} | {duration(item.get('duration_sec'))} | "
            f"{duration(item.get('execution_duration_sec'))} | {str(item.get('failed_step') or '').replace('|', '/')} |"
        )
    return "\n".join(lines) + "\n"


def _json_object(value: Any) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    if not value:
        return {}
    try:
        parsed = json.loads(value)
    except (TypeError, json.JSONDecodeError):
        return {"message": str(value)}
    return parsed if isinstance(parsed, dict) else {"value": parsed}


def _load_confirmations(connection: sqlite3.Connection) -> dict[str, dict[str, Any]]:
    table = connection.execute(
        "SELECT name FROM sqlite_master WHERE type='table' AND name='confirmations'"
    ).fetchone()
    if table is None:
        return {}
    return {
        str(row["id"]): dict(row)
        for row in connection.execute("SELECT * FROM confirmations").fetchall()
    }


def _load_event_timelines(connection: sqlite3.Connection) -> dict[str, dict[str, Any]]:
    table = connection.execute(
        "SELECT name FROM sqlite_master WHERE type='table' AND name='events'"
    ).fetchone()
    if table is None:
        return {}
    timelines: dict[str, dict[str, Any]] = {}
    rows = connection.execute("SELECT type, payload_json, created_at FROM events ORDER BY created_at").fetchall()
    for row in rows:
        payload = _json_object(row["payload_json"])
        confirmation_id = _confirmation_id(payload)
        if not confirmation_id:
            continue
        item = timelines.setdefault(confirmation_id, {"terminal_event_count": 0})
        event_type = str(row["type"])
        created_at = _number(row["created_at"])
        if event_type == "confirmation.required":
            item.setdefault("confirmation_required_at", created_at)
        elif event_type == "confirmation.resolved":
            item.setdefault("confirmation_resolved_at", created_at)
        elif event_type == "control.started":
            item.setdefault("control_started_at", created_at)
        elif event_type == "control.progress":
            phase = str(payload.get("phase") or "")
            if phase in {"verifying", "verification", "waiting_telemetry"}:
                item.setdefault("verification_started_at", created_at)
        elif event_type == "control.completed":
            item["terminal_event_count"] += 1
            item.setdefault("control_completed_at", created_at)
    return timelines


def _confirmation_id(payload: dict[str, Any]) -> str:
    direct = payload.get("confirmation_id")
    if direct:
        return str(direct)
    for key in ("confirmation", "mission"):
        value = payload.get(key)
        if not isinstance(value, dict):
            continue
        candidate = value.get("confirmation_id") if key == "mission" else value.get("id")
        if candidate:
            return str(candidate)
    return ""


def _workflow_metrics(result: dict[str, Any]) -> dict[str, Any]:
    workflow = result.get("workflow")
    if not isinstance(workflow, dict):
        workflow = {}
    steps = workflow.get("steps")
    if not isinstance(steps, list):
        steps = []
    statuses = [_status(step.get("status")) for step in steps if isinstance(step, dict)]
    failed = next(
        (step for step in steps if isinstance(step, dict) and _status(step.get("status")) == "failed"),
        None,
    )
    takeoff = next(
        (
            step for step in steps
            if isinstance(step, dict) and step.get("action") == "takeoff"
        ),
        None,
    )
    return_step = next(
        (
            step for step in steps
            if isinstance(step, dict) and step.get("action") == "return_to_start"
        ),
        None,
    )
    step_results = result.get("step_results")
    if not isinstance(step_results, dict):
        step_results = {}
    return_result = (
        step_results.get(str(return_step.get("id")))
        if isinstance(return_step, dict)
        else None
    )
    if not isinstance(return_result, dict):
        return_result = {}
    return_error = _number(return_result.get("horizontal_error_m"))
    return_tolerance = _number(return_result.get("tolerance_m"))
    if return_tolerance is None and isinstance(return_step, dict):
        return_tolerance = _number(
            (return_step.get("args") or {}).get("horizontal_tolerance_m")
        )
    return_status = _status((return_step or {}).get("status")) if return_step else ""
    return_within_tolerance = None
    if return_step:
        return_within_tolerance = bool(
            return_status == "completed"
            and return_error is not None
            and return_tolerance is not None
            and return_error <= return_tolerance
        )
    return {
        "workflow_step_total": len(steps),
        "workflow_step_completed": statuses.count("completed"),
        "workflow_step_failed": statuses.count("failed"),
        "workflow_step_skipped": statuses.count("skipped"),
        "failed_step": str((failed or {}).get("label") or (failed or {}).get("id") or ""),
        "takeoff_status": _status((takeoff or {}).get("status")) if takeoff else "",
        "return_to_start_status": return_status,
        "return_to_start_error_m": return_error,
        "return_to_start_tolerance_m": return_tolerance,
        "return_to_start_within_tolerance": return_within_tolerance,
    }


def _metric_values(records: Iterable[dict[str, Any]], key: str) -> list[float]:
    return [float(item[key]) for item in records if item.get(key) is not None]


def _timing_summary(prefix: str, values: list[float]) -> dict[str, float | None]:
    return {
        f"{prefix}_mean_sec": round(statistics.fmean(values), 3) if values else None,
        f"{prefix}_p95_sec": round(_percentile(values, 0.95), 3) if values else None,
    }


def _campaign_label(summary: dict[str, Any]) -> str:
    ready = summary.get("campaign_ready")
    if ready is None:
        return "未设置固定样本门禁"
    return "通过" if ready else "未通过"


def _failure_reason(result: Any, fallback: Any = "") -> str:
    if isinstance(result, dict):
        for key in ("message", "error", "reason", "detail"):
            if result.get(key):
                return str(result[key])[:240]
        nested = result.get("result")
        if nested is not None:
            return _failure_reason(nested, fallback)
    return str(fallback or "")[:240]


def _number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _duration(start: float | None, end: float | None) -> float | None:
    if start is None or end is None or end < start:
        return None
    return round(end - start, 3)


def _status(value: Any) -> str:
    return str(value or "unknown").strip().lower()


def _counts(values: Iterable[str]) -> dict[str, int]:
    result: dict[str, int] = {}
    for value in values:
        result[value] = result.get(value, 0) + 1
    return dict(sorted(result.items(), key=lambda item: (-item[1], item[0])))


def _percentile(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * quantile
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="汇总 Gateway 与 ROS Mission 稳定性指标")
    parser.add_argument("--db", type=Path, default=Path("~/.aeromind/agent_gateway.db").expanduser())
    parser.add_argument("--mission-root", type=Path, default=Path("missions"))
    parser.add_argument("--since-hours", type=float)
    parser.add_argument("--action", help="仅统计指定 Gateway 动作，例如 workflow")
    parser.add_argument("--title-contains", default="", help="仅统计标题包含该文本的任务")
    parser.add_argument("--latest", type=int, help="仅保留筛选后的最近 N 次任务")
    parser.add_argument("--expected-runs", type=int, default=0, help="固定验收样本数")
    parser.add_argument("--target-success-rate", type=float, default=0.9)
    parser.add_argument(
        "--require-takeoff-completed",
        action="store_true",
        help="要求每个样本的 takeoff 步骤均完成，跳过起飞不计通过",
    )
    parser.add_argument(
        "--require-return-to-start",
        action="store_true",
        help="要求每个样本在配置容差内完成 return_to_start",
    )
    parser.add_argument("--strict", action="store_true", help="门禁未通过时返回退出码 2")
    parser.add_argument("--output-dir", type=Path, default=Path("missions/contest"))
    parser.add_argument("--prefix", default="mission-metrics")
    args = parser.parse_args(argv)

    records = load_gateway_missions(args.db)
    known = {item["id"] for item in records}
    records.extend(item for item in load_file_missions(args.mission_root) if item["id"] not in known)
    records = filter_records(records, args.since_hours)
    records = select_campaign(records, args.action, args.title_contains, args.latest)
    summary = summarize(
        records,
        args.expected_runs,
        args.target_success_rate,
        require_takeoff_completed=args.require_takeoff_completed,
        require_return_to_start=args.require_return_to_start,
    )
    report = {
        "schema_version": 2,
        "generated_at": time.time(),
        "filters": {
            "since_hours": args.since_hours,
            "action": args.action,
            "title_contains": args.title_contains,
            "latest": args.latest,
            "require_takeoff_completed": args.require_takeoff_completed,
            "require_return_to_start": args.require_return_to_start,
        },
        "summary": summary,
        "records": records,
    }
    paths = write_outputs(report, args.output_dir, args.prefix)
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))
    for label, path in paths.items():
        print(f"{label}: {path.resolve()}")
    if args.strict and summary["campaign_ready"] is False:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
