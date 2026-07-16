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
        except sqlite3.Error:
            return []
    finally:
        connection.close()
    return [_gateway_record(dict(row)) for row in rows]


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


def _gateway_record(row: dict[str, Any]) -> dict[str, Any]:
    result = _json_object(row.get("result_json"))
    created = _number(row.get("created_at"))
    updated = _number(row.get("updated_at"))
    return {
        "id": row.get("id"),
        "source": "gateway_db",
        "action": row.get("action") or "unknown",
        "title": row.get("title") or "",
        "status": _status(row.get("status")),
        "physical_complete": bool(row.get("physical_complete")),
        "created_at": created,
        "updated_at": updated,
        "duration_sec": _duration(created, updated),
        "failure_reason": _failure_reason(result, row.get("phase")),
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
        "physical_complete": status in SUCCESS_STATES,
        "created_at": created,
        "updated_at": updated,
        "duration_sec": _duration(created, updated),
        "failure_reason": _failure_reason(failed_step or payload, payload.get("message")),
        "result": {"steps": steps, "message": payload.get("message")},
    }


def filter_records(records: Iterable[dict[str, Any]], since_hours: float | None) -> list[dict[str, Any]]:
    values = list(records)
    if since_hours is None:
        return values
    threshold = time.time() - max(0.0, since_hours) * 3600.0
    return [item for item in values if (item.get("created_at") or 0.0) >= threshold]


def summarize(records: list[dict[str, Any]]) -> dict[str, Any]:
    terminal = [item for item in records if item["status"] in TERMINAL_STATES]
    completed = [item for item in terminal if item["status"] in SUCCESS_STATES]
    physically_complete = [item for item in terminal if item.get("physical_complete")]
    durations = [float(item["duration_sec"]) for item in terminal if item.get("duration_sec") is not None]
    status_counts = _counts(item["status"] for item in records)
    action_counts = _counts(item["action"] for item in records)
    failure_counts = _counts(
        item["failure_reason"] or "未记录原因"
        for item in terminal
        if item["status"] not in SUCCESS_STATES
    )
    denominator = len(terminal)
    return {
        "total": len(records),
        "terminal": denominator,
        "completed": len(completed),
        "success_rate": round(len(completed) / denominator, 4) if denominator else None,
        "physical_complete_rate": round(len(physically_complete) / denominator, 4) if denominator else None,
        "duration_mean_sec": round(statistics.fmean(durations), 3) if durations else None,
        "duration_p95_sec": round(_percentile(durations, 0.95), 3) if durations else None,
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
            "created_at", "updated_at", "duration_sec", "failure_reason", "title",
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
        "## 状态分布", "", "| 状态 | 数量 |", "|---|---:|",
    ]
    lines.extend(f"| {key} | {value} |" for key, value in summary["status_counts"].items())
    lines.extend(["", "## 失败原因", "", "| 原因 | 数量 |", "|---|---:|"])
    if summary["failure_reasons"]:
        lines.extend(f"| {key.replace('|', '/')} | {value} |" for key, value in summary["failure_reasons"].items())
    else:
        lines.append("| 暂无失败样本 | 0 |")
    lines.extend(["", "## 明细", "", "| ID | 动作 | 状态 | 物理完成 | 耗时 |", "|---|---|---|---:|---:|"])
    for item in report["records"]:
        lines.append(
            f"| {item['id']} | {item['action']} | {item['status']} | "
            f"{'是' if item.get('physical_complete') else '否'} | {duration(item.get('duration_sec'))} |"
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
    parser.add_argument("--output-dir", type=Path, default=Path("missions/contest"))
    parser.add_argument("--prefix", default="mission-metrics")
    args = parser.parse_args(argv)

    records = load_gateway_missions(args.db)
    known = {item["id"] for item in records}
    records.extend(item for item in load_file_missions(args.mission_root) if item["id"] not in known)
    records = filter_records(records, args.since_hours)
    report = {"schema_version": 1, "generated_at": time.time(), "summary": summarize(records), "records": records}
    paths = write_outputs(report, args.output_dir, args.prefix)
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))
    for label, path in paths.items():
        print(f"{label}: {path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
