"""Validated workflow plans composed from existing confirmed flight actions."""

from __future__ import annotations

import math
import uuid
from copy import deepcopy
from typing import Any

from .capability_registry import workflow_action_specs

WORKFLOW_ACTIONS = set(workflow_action_specs())
WORKFLOW_CONDITIONS = {
    "always",
    "if_airborne",
    "if_not_airborne",
    "if_safe",
}
WORKFLOW_FAILURE_POLICIES = {"stop", "continue"}
WORKFLOW_BRANCH_CONDITIONS = {
    "step_succeeded",
    "step_failed",
    "target_detected",
    "target_not_detected",
    "semantic_target_detected",
    "semantic_target_not_detected",
    "risk_level_is",
}


def validate_workflow(value: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError("workflow 必须是对象")
    name = " ".join(str(value.get("name") or "组合飞行任务").split())[:80]
    raw_steps = value.get("steps")
    if not isinstance(raw_steps, list) or not 1 <= len(raw_steps) <= 20:
        raise ValueError("workflow.steps 必须包含 1 到 20 个步骤或循环块")
    raw_steps = _expand_repeat_blocks(raw_steps)
    if len(raw_steps) > 60:
        raise ValueError("workflow 展开后不能超过 60 个步骤")
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
        condition = _validate_condition(
            raw.get("condition") or "always", seen_ids, step_id
        )
        retries = int(raw.get("retries", 0))
        if not 0 <= retries <= 2:
            raise ValueError(f"步骤 {step_id} 重试次数必须在 0 到 2 之间")
        if action in {"move", "follow_waypoints"} and retries:
            raise ValueError(
                f"步骤 {step_id} 包含相对移动，不允许自动重试；请将 retries 设为 0"
            )
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


def workflow_json_schema() -> dict[str, Any]:
    """Return the model-facing schema generated from the executable catalog."""
    simple_condition = {"type": "string", "enum": sorted(WORKFLOW_CONDITIONS)}
    branch_variants = []
    for condition_type in sorted(WORKFLOW_BRANCH_CONDITIONS):
        properties = {
            "type": {"const": condition_type},
            "step_id": {"type": "string"},
        }
        required = ["type", "step_id"]
        if condition_type in {
            "semantic_target_detected",
            "semantic_target_not_detected",
        }:
            properties["target"] = {"type": "string", "minLength": 1, "maxLength": 80}
            required.append("target")
        if condition_type == "risk_level_is":
            properties["value"] = {
                "type": "string",
                "enum": ["low", "medium", "high"],
            }
            required.append("value")
        branch_variants.append(
            {
                "type": "object",
                "properties": properties,
                "required": required,
                "additionalProperties": False,
            }
        )
    condition_schema = {
        "oneOf": [simple_condition, {"oneOf": branch_variants}]
    }
    variants = []
    for action, spec in workflow_action_specs().items():
        variants.append(
            {
                "type": "object",
                "properties": {
                    "id": {"type": "string"},
                    "label": {"type": "string"},
                    "action": {"const": action},
                    "args": spec["parameters"],
                    "depends_on": {"type": "array", "items": {"type": "string"}},
                    "condition": condition_schema,
                    "retries": {"type": "integer", "minimum": 0, "maximum": 2},
                    "on_failure": {"type": "string", "enum": ["stop", "continue"]},
                },
                "required": ["action"],
                "additionalProperties": False,
            }
        )
    action_step = {"oneOf": variants}
    repeat_block = {
        "type": "object",
        "properties": {
            "id": {"type": "string"},
            "repeat": {
                "type": "object",
                "properties": {
                    "count": {"type": "integer", "minimum": 1, "maximum": 10},
                    "steps": {
                        "type": "array",
                        "minItems": 1,
                        "maxItems": 10,
                        "items": action_step,
                    },
                },
                "required": ["count", "steps"],
                "additionalProperties": False,
            },
        },
        "required": ["id", "repeat"],
        "additionalProperties": False,
    }
    return {
        "type": "object",
        "properties": {
            "name": {"type": "string", "minLength": 1, "maxLength": 80},
            "steps": {
                "type": "array",
                "minItems": 1,
                "maxItems": 20,
                "items": {"oneOf": [action_step, repeat_block]},
            },
        },
        "required": ["name", "steps"],
        "additionalProperties": False,
    }


def _expand_repeat_blocks(raw_steps: list[dict[str, Any]]) -> list[dict[str, Any]]:
    expanded: list[dict[str, Any]] = []
    aliases: dict[str, str] = {}
    for source_index, raw in enumerate(raw_steps):
        if not isinstance(raw, dict) or "repeat" not in raw:
            step = deepcopy(raw)
            if isinstance(step, dict):
                _remap_step_references(step, aliases)
                source_id = str(step.get("id") or f"step-{source_index + 1}").strip()
                aliases[source_id] = source_id
            expanded.append(step)
            continue

        block_id = str(raw.get("id") or f"repeat-{source_index + 1}").strip()
        if not block_id or block_id in aliases:
            raise ValueError(f"workflow 循环块 ID 无效或重复: {block_id}")
        repeat = raw.get("repeat")
        if not isinstance(repeat, dict):
            raise ValueError(f"循环块 {block_id}.repeat 必须是对象")
        count = int(repeat.get("count", 0))
        template = repeat.get("steps")
        if not 1 <= count <= 10:
            raise ValueError(f"循环块 {block_id} 次数必须在 1 到 10 之间")
        if not isinstance(template, list) or not 1 <= len(template) <= 10:
            raise ValueError(f"循环块 {block_id} 必须包含 1 到 10 个动作步骤")
        previous_iteration_last = None
        for iteration in range(1, count + 1):
            local_ids = {}
            for inner_index, inner in enumerate(template):
                if not isinstance(inner, dict) or "repeat" in inner:
                    raise ValueError(f"循环块 {block_id} 不允许嵌套循环")
                inner_id = str(inner.get("id") or f"step-{inner_index + 1}").strip()
                local_ids[inner_id] = f"{block_id}-{iteration}-{inner_id}"
            previous_step = previous_iteration_last
            for inner_index, inner in enumerate(template):
                step = deepcopy(inner)
                inner_id = str(step.get("id") or f"step-{inner_index + 1}").strip()
                step["id"] = local_ids[inner_id]
                dependencies = list(step.get("depends_on") or [])
                if dependencies:
                    step["depends_on"] = [
                        local_ids.get(str(item), aliases.get(str(item), str(item)))
                        for item in dependencies
                    ]
                elif previous_step:
                    step["depends_on"] = [previous_step]
                condition = step.get("condition")
                if isinstance(condition, dict) and condition.get("step_id"):
                    source = str(condition["step_id"])
                    condition["step_id"] = local_ids.get(
                        source, aliases.get(source, source)
                    )
                expanded.append(step)
                previous_step = step["id"]
            previous_iteration_last = previous_step
        aliases[block_id] = str(previous_iteration_last)
    return expanded


def _remap_step_references(step: dict[str, Any], aliases: dict[str, str]):
    if step.get("depends_on"):
        step["depends_on"] = [
            aliases.get(str(item), str(item)) for item in step["depends_on"]
        ]
    condition = step.get("condition")
    if isinstance(condition, dict) and condition.get("step_id"):
        source = str(condition["step_id"])
        condition["step_id"] = aliases.get(source, source)


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


def build_v_shape_workflow(
    width: float = 10.0,
    depth: float = 10.0,
    altitude: float = 10.0,
    takeoff_if_needed: bool = True,
    capture_at_vertex: bool = False,
    land_after: bool = False,
) -> dict[str, Any]:
    width_value = float(width)
    depth_value = float(depth)
    height = float(altitude)
    if not math.isfinite(width_value) or not 1.0 <= width_value <= 50.0:
        raise ValueError("V 字轨迹宽度必须在 1 到 50 米之间")
    if not math.isfinite(depth_value) or not 1.0 <= depth_value <= 50.0:
        raise ValueError("V 字轨迹深度必须在 1 到 50 米之间")
    if not math.isfinite(height) or not 1.0 <= height <= 30.0:
        raise ValueError("任务高度必须在 1 到 30 米之间")
    steps = []
    previous = []
    if takeoff_if_needed:
        steps.append({
            "id": "takeoff",
            "action": "takeoff",
            "args": {"altitude": height},
            "condition": "if_not_airborne",
            "retries": 1,
        })
        previous = ["takeoff"]
    steps.append({
        "id": "left-leg",
        "label": "飞向 V 字顶点",
        "action": "move",
        "args": {"forward_m": depth_value, "right_m": width_value / 2.0},
        "depends_on": previous,
    })
    previous = ["left-leg"]
    if capture_at_vertex:
        steps.append({
            "id": "vertex-capture",
            "label": "在 V 字顶点拍照",
            "action": "capture_image",
            "depends_on": previous,
            "retries": 0,
        })
        previous = ["vertex-capture"]
    steps.append({
        "id": "right-leg",
        "label": "从顶点飞向 V 字右端",
        "action": "move",
        "args": {"forward_m": -depth_value, "right_m": width_value / 2.0},
        "depends_on": previous,
    })
    previous = ["right-leg"]
    if land_after:
        steps.append({
            "id": "land",
            "action": "land",
            "depends_on": previous,
            "condition": "if_airborne",
        })
    return validate_workflow({
        "name": f"宽 {width_value:.1f} 米、深 {depth_value:.1f} 米 V 字轨迹",
        "steps": steps,
    })


def build_person_inspection_workflow(
    distance: float = 20.0,
    minimum_confidence: float = 0.5,
) -> dict[str, Any]:
    distance_value = float(distance)
    if not math.isfinite(distance_value) or not 0.5 <= distance_value <= 100.0:
        raise ValueError("巡检前进距离必须在 0.5 到 100 米之间")
    return validate_workflow({
        "name": "人员条件巡检",
        "steps": [
            {
                "id": "detect-person",
                "label": "检查前方是否有人",
                "action": "perception_check",
                "args": {
                    "target": "person",
                    "minimum_confidence": minimum_confidence,
                },
                "on_failure": "continue",
            },
            {
                "id": "hover-on-person",
                "label": "发现人员后悬停",
                "action": "hover",
                "condition": {
                    "type": "target_detected",
                    "step_id": "detect-person",
                },
            },
            {
                "id": "capture-on-person",
                "label": "发现人员后保存图像",
                "action": "capture_image",
                "depends_on": ["hover-on-person"],
                "condition": {
                    "type": "target_detected",
                    "step_id": "detect-person",
                },
            },
            {
                "id": "continue-if-clear",
                "label": f"未发现人员，继续前进 {distance_value:.1f} 米",
                "action": "move",
                "args": {"direction": "forward", "distance": distance_value},
                "condition": {
                    "type": "target_not_detected",
                    "step_id": "detect-person",
                },
            },
            {
                "id": "hover-if-sensor-unavailable",
                "label": "感知数据不可用，保持悬停",
                "action": "hover",
                "condition": {
                    "type": "step_failed",
                    "step_id": "detect-person",
                },
            },
        ],
    })


def _validate_step_args(action: str, args: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(args, dict):
        raise ValueError(f"动作 {action} 的 args 必须是对象")
    spec = workflow_action_specs()[action]["parameters"]
    allowed = set(spec.get("properties") or {})
    unknown = set(args) - allowed
    if unknown:
        raise ValueError(
            f"动作 {action} 包含未知参数: {', '.join(sorted(unknown))}"
        )
    if action == "takeoff":
        altitude = float(args.get("altitude", 10.0))
        if not 1.0 <= altitude <= 30.0:
            raise ValueError("起飞高度必须在 1 到 30 米之间")
        return {"altitude": altitude}
    if action == "move":
        vector_keys = ("forward_m", "right_m", "up_m")
        if any(key in args for key in vector_keys):
            vector = {key: float(args.get(key, 0.0)) for key in vector_keys}
            if not all(math.isfinite(value) for value in vector.values()):
                raise ValueError("移动向量必须是有限数值")
            if not any(abs(value) >= 0.1 for value in vector.values()):
                raise ValueError("移动向量不能全部为 0")
            if any(abs(value) > 100.0 for value in vector.values()):
                raise ValueError("移动向量各分量不能超过 100 米")
            return vector
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
            "check_takeoff_zone": bool(args.get("check_takeoff_zone", False)),
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
    if action == "analyze_image":
        prompt = " ".join(str(args.get("prompt") or "").split())
        if not prompt:
            raise ValueError("VLM 图像分析 prompt 不能为空")
        if len(prompt) > 1000:
            raise ValueError("VLM 图像分析 prompt 不能超过 1000 字符")
        return {"prompt": prompt}
    if action == "wait":
        duration = float(args.get("duration_sec", 0.0))
        if not math.isfinite(duration) or not 0.1 <= duration <= 60.0:
            raise ValueError("等待时长必须在 0.1 到 60 秒之间")
        return {"duration_sec": duration}
    if action == "mission_report":
        title = " ".join(str(args.get("title") or "任务报告").split())[:120]
        return {"title": title or "任务报告"}
    if action == "follow_waypoints":
        raw_points = args.get("points")
        if not isinstance(raw_points, list) or not 1 <= len(raw_points) <= 20:
            raise ValueError("航点序列必须包含 1 到 20 个相对航点")
        points = []
        total_distance = 0.0
        for index, raw_point in enumerate(raw_points):
            if not isinstance(raw_point, dict):
                raise ValueError(f"航点 {index + 1} 必须是对象")
            unknown = set(raw_point) - {"forward_m", "right_m", "up_m"}
            if unknown:
                raise ValueError(f"航点 {index + 1} 包含未知参数")
            point = {
                key: float(raw_point.get(key, 0.0))
                for key in ("forward_m", "right_m", "up_m")
            }
            if not all(math.isfinite(value) for value in point.values()):
                raise ValueError(f"航点 {index + 1} 必须是有限数值")
            distance = math.sqrt(sum(value * value for value in point.values()))
            if distance < 0.5 or any(abs(value) > 100.0 for value in point.values()):
                raise ValueError(f"航点 {index + 1} 距离无效或分量超过 100 米")
            total_distance += distance
            points.append(point)
        if total_distance > 300.0:
            raise ValueError("航点序列累计距离不能超过 300 米")
        cruise_speed = float(args.get("cruise_speed", 1.5))
        if not math.isfinite(cruise_speed) or not 0.2 <= cruise_speed <= 4.0:
            raise ValueError("航点巡航速度必须在 0.2 到 4.0 m/s 之间")
        return {
            "points": points,
            "cruise_speed": cruise_speed,
            "capture_on_semantic_hold": bool(
                args.get("capture_on_semantic_hold", False)
            ),
        }
    if action == "capture_image":
        return {}
    return {}


def _validate_condition(value: Any, seen_ids: set[str], step_id: str) -> Any:
    if isinstance(value, str):
        condition = value.strip()
        if condition not in WORKFLOW_CONDITIONS:
            raise ValueError(f"步骤 {step_id} 条件无效: {condition}")
        return condition
    if not isinstance(value, dict):
        raise ValueError(f"步骤 {step_id} 条件必须是字符串或对象")
    condition_type = str(value.get("type") or "").strip()
    if condition_type not in WORKFLOW_BRANCH_CONDITIONS:
        raise ValueError(f"步骤 {step_id} 分支条件无效: {condition_type}")
    source_step = str(value.get("step_id") or "").strip()
    if source_step not in seen_ids:
        raise ValueError(f"步骤 {step_id} 的分支条件只能引用前面的步骤")
    condition = {"type": condition_type, "step_id": source_step}
    if condition_type in {"semantic_target_detected", "semantic_target_not_detected"}:
        target = " ".join(str(value.get("target") or "").split())[:80]
        if not target:
            raise ValueError(f"步骤 {step_id} 的语义目标条件缺少 target")
        condition["target"] = target
    if condition_type == "risk_level_is":
        risk_level = str(value.get("value") or "").strip().lower()
        if risk_level not in {"low", "medium", "high"}:
            raise ValueError(f"步骤 {step_id} 的风险条件必须为 low/medium/high")
        condition["value"] = risk_level
    return condition


def _step_label(action: str, args: dict[str, Any]) -> str:
    if action == "takeoff":
        return f"起飞到 {args['altitude']:.1f} 米"
    if action == "move":
        if "direction" in args:
            return f"向 {args['direction']} 移动 {args['distance']:.1f} 米"
        return (
            f"相对移动：前 {args.get('forward_m', 0.0):.1f} 米，"
            f"右 {args.get('right_m', 0.0):.1f} 米，"
            f"上 {args.get('up_m', 0.0):.1f} 米"
        )
    labels = {
        "safety_check": "检查飞控与障碍物安全状态",
        "perception_check": "读取当前目标检测与避障摘要",
        "arm": "解锁无人机",
        "disarm": "加锁无人机",
        "land": "降落",
        "return_home": "RTL 返航",
        "hover": "悬停",
        "capture_image": "保存当前相机图像",
        "analyze_image": "使用 VLM 分析当前画面",
        "wait": f"等待 {args.get('duration_sec', 0.0):.1f} 秒",
        "mission_report": str(args.get("title") or "生成任务报告快照"),
        "follow_waypoints": f"依次飞行 {len(args.get('points') or [])} 个相对航点",
    }
    return labels[action]
