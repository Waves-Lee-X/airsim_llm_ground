"""Validated workflow plans composed from existing confirmed flight actions."""

from __future__ import annotations

import uuid
from typing import Any


WORKFLOW_ACTIONS = {
    "safety_check",
    "perception_check",
    "arm",
    "disarm",
    "takeoff",
    "land",
    "move",
    "return_home",
    "hover",
}
WORKFLOW_CONDITIONS = {
    "always",
    "if_airborne",
    "if_not_airborne",
    "if_safe",
}
WORKFLOW_FAILURE_POLICIES = {"stop", "continue"}


def validate_workflow(value: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError("workflow 必须是对象")
    name = " ".join(str(value.get("name") or "组合飞行任务").split())[:80]
    raw_steps = value.get("steps")
    if not isinstance(raw_steps, list) or not 1 <= len(raw_steps) <= 20:
        raise ValueError("workflow.steps 必须包含 1 到 20 个步骤")
    steps = []
    seen_ids = set()
    for index, raw in enumerate(raw_steps):
        if not isinstance(raw, dict):
            raise ValueError(f"workflow 步骤 {index + 1} 必须是对象")
        step_id = str(raw.get("id") or f"step-{index + 1}").strip()
        if not step_id or step_id in seen_ids:
            raise ValueError(f"workflow 步骤 ID 无效或重复: {step_id}")
        action = str(raw.get("action", "")).strip()
        if action not in WORKFLOW_ACTIONS:
            raise ValueError(f"workflow 不允许动作: {action}")
        depends_on = [str(item).strip() for item in raw.get("depends_on", [])]
        if any(item not in seen_ids for item in depends_on):
            raise ValueError(f"步骤 {step_id} 只能依赖前面已经定义的步骤")
        condition = str(raw.get("condition") or "always")
        if condition not in WORKFLOW_CONDITIONS:
            raise ValueError(f"步骤 {step_id} 条件无效: {condition}")
        retries = int(raw.get("retries", 0))
        if not 0 <= retries <= 2:
            raise ValueError(f"步骤 {step_id} 重试次数必须在 0 到 2 之间")
        on_failure = str(raw.get("on_failure") or "stop")
        if on_failure not in WORKFLOW_FAILURE_POLICIES:
            raise ValueError(f"步骤 {step_id} 失败策略无效")
        args = _validate_step_args(action, raw.get("args") or {})
        steps.append(
            {
                "id": step_id,
                "label": " ".join(
                    str(raw.get("label") or _step_label(action, args)).split()
                )[:120],
                "action": action,
                "args": args,
                "depends_on": depends_on,
                "condition": condition,
                "retries": retries,
                "on_failure": on_failure,
                "status": "pending",
            }
        )
        seen_ids.add(step_id)
    return {
        "workflow_id": str(value.get("workflow_id") or f"workflow-{uuid.uuid4().hex}"),
        "name": name,
        "steps": steps,
    }


def build_square_workflow(
    side_length: float,
    altitude: float = 10.0,
    takeoff_if_needed: bool = True,
    land_after: bool = False,
) -> dict[str, Any]:
    side = float(side_length)
    height = float(altitude)
    if not 1.0 <= side <= 50.0:
        raise ValueError("正方形边长必须在 1 到 50 米之间")
    if not 1.0 <= height <= 30.0:
        raise ValueError("任务高度必须在 1 到 30 米之间")
    steps = []
    previous = []
    if takeoff_if_needed:
        steps.append(
            {
                "id": "takeoff",
                "label": f"必要时起飞到 {height:.1f} 米",
                "action": "takeoff",
                "args": {"altitude": height},
                "condition": "if_not_airborne",
                "retries": 1,
            }
        )
        previous = ["takeoff"]
    for index, direction in enumerate(("forward", "right", "backward", "left"), 1):
        step_id = f"edge-{index}"
        steps.append(
            {
                "id": step_id,
                "label": f"正方形第 {index} 条边：{direction} {side:.1f} 米",
                "action": "move",
                "args": {"direction": direction, "distance": side},
                "depends_on": previous,
                "retries": 1,
            }
        )
        previous = [step_id]
    if land_after:
        steps.append(
            {
                "id": "land",
                "label": "正方形轨迹完成后降落",
                "action": "land",
                "depends_on": previous,
                "condition": "if_airborne",
            }
        )
    return validate_workflow(
        {
            "name": f"{side:.1f} 米正方形轨迹",
            "steps": steps,
        }
    )


def _validate_step_args(action: str, args: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(args, dict):
        raise ValueError(f"动作 {action} 的 args 必须是对象")
    if action == "takeoff":
        altitude = float(args.get("altitude", 10.0))
        if not 1.0 <= altitude <= 30.0:
            raise ValueError("起飞高度必须在 1 到 30 米之间")
        return {"altitude": altitude}
    if action == "move":
        direction = str(args.get("direction", "")).lower()
        if direction not in {"forward", "backward", "left", "right", "up", "down"}:
            raise ValueError(f"移动方向无效: {direction}")
        distance = float(args.get("distance", 0.0))
        if not 0.5 <= distance <= 100.0:
            raise ValueError("移动距离必须在 0.5 到 100 米之间")
        return {"direction": direction, "distance": distance}
    if action == "safety_check":
        minimum = float(args.get("minimum_obstacle_distance", 2.0))
        if not 0.5 <= minimum <= 20.0:
            raise ValueError("安全检查最小障碍物距离必须在 0.5 到 20 米之间")
        return {
            "require_gps": bool(args.get("require_gps", True)),
            "minimum_obstacle_distance": minimum,
        }
    if action == "perception_check":
        target = " ".join(str(args.get("target") or "").split())[:80]
        minimum_confidence = float(args.get("minimum_confidence", 0.35))
        if not 0.0 <= minimum_confidence <= 1.0:
            raise ValueError("感知检查置信度必须在 0 到 1 之间")
        return {
            "target": target,
            "minimum_confidence": minimum_confidence,
        }
    return {}


def _step_label(action: str, args: dict[str, Any]) -> str:
    if action == "takeoff":
        return f"起飞到 {args['altitude']:.1f} 米"
    if action == "move":
        return f"向 {args['direction']} 移动 {args['distance']:.1f} 米"
    labels = {
        "safety_check": "检查飞控与障碍物安全状态",
        "perception_check": "读取当前目标检测与避障摘要",
        "arm": "解锁无人机",
        "disarm": "加锁无人机",
        "land": "降落",
        "return_home": "RTL 返航",
        "hover": "悬停",
    }
    return labels[action]
