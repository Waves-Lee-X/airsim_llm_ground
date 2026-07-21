"""Deterministic atomic capabilities exposed by the ROS execution layer."""

from __future__ import annotations

from copy import deepcopy
from typing import Any


EMPTY_ARGS = {"type": "object", "properties": {}, "additionalProperties": False}

MOVE_ARGS = {
    "type": "object",
    "properties": {
        "direction": {
            "type": "string",
            "enum": ["forward", "backward", "left", "right", "up", "down"],
        },
        "distance": {"type": "number", "minimum": 0.5, "maximum": 100.0},
        "forward_m": {"type": "number", "minimum": -100.0, "maximum": 100.0},
        "right_m": {"type": "number", "minimum": -100.0, "maximum": 100.0},
        "up_m": {"type": "number", "minimum": -100.0, "maximum": 100.0},
    },
    "additionalProperties": False,
}

WAYPOINT_ARGS = {
    "type": "object",
    "properties": {
        "points": {
            "type": "array",
            "minItems": 1,
            "maxItems": 20,
            "items": {
                "type": "object",
                "properties": {
                    "forward_m": {"type": "number", "minimum": -100.0, "maximum": 100.0},
                    "right_m": {"type": "number", "minimum": -100.0, "maximum": 100.0},
                    "up_m": {"type": "number", "minimum": -100.0, "maximum": 100.0},
                },
                "additionalProperties": False,
            },
        },
        "cruise_speed": {"type": "number", "minimum": 0.2, "maximum": 4.0, "default": 1.5},
        "capture_on_semantic_hold": {
            "type": "boolean",
            "default": False,
            "description": "人员进入剩余航迹触发悬停时保存一次现场图像。",
        },
    },
    "required": ["points"],
    "additionalProperties": False,
}


WORKFLOW_ACTION_SPECS: dict[str, dict[str, Any]] = {
    "safety_check": {
        "label": "安全检查",
        "risk_level": "low",
        "mode": "local",
        "description": "检查飞控状态、定位、EKF 与最近障碍物。",
        "parameters": {
            "type": "object",
            "properties": {
                "require_gps": {"type": "boolean", "default": True},
                "check_takeoff_zone": {"type": "boolean", "default": False},
                "minimum_obstacle_distance": {
                    "type": "number",
                    "minimum": 0.5,
                    "maximum": 20.0,
                    "default": 2.0,
                },
            },
            "additionalProperties": False,
        },
        "preconditions": ["telemetry_available"],
        "effects": ["safety_result_available"],
    },
    "perception_check": {
        "label": "目标检测检查",
        "risk_level": "low",
        "mode": "local",
        "description": "读取当前 YOLO 检测并判断指定目标是否存在。",
        "parameters": {
            "type": "object",
            "properties": {
                "target": {"type": "string", "maxLength": 80},
                "minimum_confidence": {
                    "type": "number",
                    "minimum": 0.0,
                    "maximum": 1.0,
                    "default": 0.35,
                },
            },
            "additionalProperties": False,
        },
        "preconditions": ["perception_available"],
        "effects": ["target_detection_result_available"],
    },
    "analyze_image": {
        "label": "VLM 图像分析",
        "risk_level": "low",
        "mode": "local",
        "description": "让视觉语言模型分析最新 RGB 图像并返回场景、目标和风险。",
        "parameters": {
            "type": "object",
            "properties": {"prompt": {"type": "string", "minLength": 1, "maxLength": 1000}},
            "required": ["prompt"],
            "additionalProperties": False,
        },
        "preconditions": ["rgb_frame_available", "vlm_optional"],
        "effects": ["semantic_image_result_available"],
    },
    "wait": {
        "label": "等待",
        "risk_level": "low",
        "mode": "local",
        "description": "在任务流程中等待指定秒数，可被暂停或取消。",
        "parameters": {
            "type": "object",
            "properties": {
                "duration_sec": {"type": "number", "minimum": 0.1, "maximum": 60.0}
            },
            "required": ["duration_sec"],
            "additionalProperties": False,
        },
        "preconditions": [],
        "effects": ["wait_elapsed"],
    },
    "mission_report": {
        "label": "任务报告快照",
        "risk_level": "low",
        "mode": "local",
        "description": "记录当前飞控、里程计、感知和自主状态作为报告数据。",
        "parameters": {
            "type": "object",
            "properties": {"title": {"type": "string", "maxLength": 120}},
            "additionalProperties": False,
        },
        "preconditions": [],
        "effects": ["mission_report_available"],
    },
    "arm": {
        "label": "解锁",
        "risk_level": "high",
        "mode": "action",
        "description": "请求 PX4 解锁。",
        "parameters": EMPTY_ARGS,
        "preconditions": ["safety_check_passed"],
        "effects": ["armed"],
    },
    "disarm": {
        "label": "加锁",
        "risk_level": "high",
        "mode": "action",
        "description": "请求 PX4 加锁。",
        "parameters": EMPTY_ARGS,
        "preconditions": ["landed"],
        "effects": ["disarmed"],
    },
    "takeoff": {
        "label": "起飞",
        "risk_level": "high",
        "mode": "action",
        "description": "按目标高度执行 PX4 起飞。",
        "parameters": {
            "type": "object",
            "properties": {"altitude": {"type": "number", "minimum": 1.0, "maximum": 30.0}},
            "required": ["altitude"],
            "additionalProperties": False,
        },
        "preconditions": ["safety_check_passed"],
        "effects": ["airborne"],
    },
    "land": {
        "label": "降落",
        "risk_level": "high",
        "mode": "action",
        "description": "执行 PX4 原生降落。",
        "parameters": EMPTY_ARGS,
        "preconditions": ["airborne"],
        "effects": ["landed", "disarmed"],
    },
    "move": {
        "label": "相对移动",
        "risk_level": "high",
        "mode": "action",
        "description": "按机头坐标系发布相对三维目标并等待自主规划完成。",
        "parameters": MOVE_ARGS,
        "preconditions": ["airborne", "odometry_healthy", "autonomy_available"],
        "effects": ["relative_goal_reached"],
    },
    "follow_waypoints": {
        "label": "航点序列",
        "risk_level": "high",
        "mode": "composite",
        "description": "通过 ROS 2 Action 连续执行相对航点并生成 Minimum Snap 轨迹；人员进入航迹时底层确定性悬停，风险解除后续接剩余轨迹。",
        "parameters": WAYPOINT_ARGS,
        "preconditions": ["airborne", "odometry_healthy", "autonomy_available"],
        "effects": ["waypoint_sequence_completed"],
    },
    "return_home": {
        "label": "返航",
        "risk_level": "high",
        "mode": "action",
        "description": "触发 PX4 原生 RTL。",
        "parameters": EMPTY_ARGS,
        "preconditions": ["airborne"],
        "effects": ["returned_home", "landed"],
    },
    "hover": {
        "label": "悬停",
        "risk_level": "medium",
        "mode": "action",
        "description": "取消自主目标并进入悬停保持。",
        "parameters": EMPTY_ARGS,
        "preconditions": ["airborne"],
        "effects": ["hovering"],
    },
    "capture_image": {
        "label": "拍照保存",
        "risk_level": "low",
        "mode": "action",
        "description": "保存当前前视 RGB 图像并返回路径。",
        "parameters": EMPTY_ARGS,
        "preconditions": ["rgb_frame_available"],
        "effects": ["image_captured"],
    },
}


CAPABILITIES: tuple[dict[str, Any], ...] = (
    {"name": "system.status", "label": "飞控状态", "category": "system", "risk_level": "low", "mode": "read", "interface": "/control/drone_state", "description": "读取解锁、模式、电池、GPS、EKF、位置和速度。", "example_task": "查询无人机状态"},
    {"name": "system.safety_check", "label": "安全检查", "category": "system", "risk_level": "low", "mode": "local", "action": "safety_check", "interface": "Gateway snapshot", "description": "确定性检查状态新鲜度、EKF、GPS 和最近障碍物。", "example_task": "检查当前是否适合起飞"},
    {"name": "flight.arm", "label": "解锁", "category": "flight", "risk_level": "high", "mode": "action", "action": "arm", "interface": "/control/arm", "description": "请求 PX4 解锁。", "example_task": "解锁无人机"},
    {"name": "flight.disarm", "label": "加锁", "category": "flight", "risk_level": "high", "mode": "action", "action": "disarm", "interface": "/control/arm", "description": "请求 PX4 加锁。", "example_task": "加锁无人机"},
    {"name": "flight.takeoff", "label": "起飞", "category": "flight", "risk_level": "high", "mode": "action", "action": "takeoff", "interface": "/control/takeoff", "description": "按目标高度执行 PX4 起飞。", "example_task": "起飞到10米"},
    {"name": "flight.land", "label": "降落", "category": "flight", "risk_level": "high", "mode": "action", "action": "land", "interface": "/control/land", "description": "执行 PX4 原生降落。", "example_task": "降落"},
    {"name": "flight.move_relative", "label": "相对移动", "category": "flight", "risk_level": "high", "mode": "action", "action": "move", "interface": "/autonomy/goal", "description": "按机头坐标系发布相对三维目标并由自主规划器执行。", "example_task": "向前飞10米"},
    {"name": "flight.follow_waypoints", "label": "航点序列", "category": "flight", "risk_level": "high", "mode": "composite", "action": "follow_waypoints", "interface": "/autonomy/follow_waypoints (ROS 2 Action)", "description": "连续执行多个相对航点，使用 Minimum Snap 轨迹经过中间点而不逐点刹停。", "example_task": "飞一个三角形并在顶点拍照"},
    {"name": "flight.hover", "label": "悬停", "category": "flight", "risk_level": "medium", "mode": "action", "action": "hover", "interface": "/autonomy/cancel + /control/cmd_vel", "description": "取消自主目标并进入悬停保持。", "example_task": "原地悬停"},
    {"name": "flight.return_home", "label": "返航", "category": "flight", "risk_level": "high", "mode": "action", "action": "return_home", "interface": "/control/return_home", "description": "触发 PX4 原生 RTL。", "example_task": "返航"},
    {"name": "perception.capture_image", "label": "拍照保存", "category": "perception", "risk_level": "low", "mode": "action", "action": "capture_image", "interface": "/perception/capture_image", "description": "保存当前前视 RGB 图像并返回路径。", "example_task": "拍一张前方照片"},
    {"name": "perception.detect_target", "label": "目标检测", "category": "perception", "risk_level": "low", "mode": "local", "action": "perception_check", "interface": "/perception/detections", "description": "读取 YOLO 目标类别、置信度和图像位置。", "example_task": "检测前方是否有人"},
    {"name": "perception.world_query", "label": "世界对象查询", "category": "perception", "risk_level": "low", "mode": "read", "interface": "/world_model/query", "description": "按类别、距离、时效和对象 ID 查询当前或历史三维语义对象。", "example_task": "最近的人在哪里，过去十秒出现过几次"},
    {"name": "perception.semantic_image", "label": "图像语义", "category": "perception", "risk_level": "low", "mode": "local", "action": "analyze_image", "interface": "/perception/analyze_image", "description": "使用 VLM 或规则摘要分析当前画面。", "example_task": "分析当前画面"},
    {"name": "workflow.wait", "label": "等待", "category": "workflow", "risk_level": "low", "mode": "local", "action": "wait", "interface": "Gateway workflow", "description": "在组合任务中等待指定时间。", "example_task": "每次移动后等待2秒"},
    {"name": "workflow.mission_report", "label": "报告快照", "category": "workflow", "risk_level": "low", "mode": "local", "action": "mission_report", "interface": "Gateway snapshot", "description": "保存任务结束时的状态、感知和自主信息。", "example_task": "完成后生成任务报告"},
)


def workflow_action_specs() -> dict[str, dict[str, Any]]:
    return deepcopy(WORKFLOW_ACTION_SPECS)


def capability_catalog() -> list[dict[str, Any]]:
    catalog = []
    for item in CAPABILITIES:
        value = {**item, "enabled": True, "type": "capability"}
        action = value.get("action")
        if action in WORKFLOW_ACTION_SPECS:
            spec = WORKFLOW_ACTION_SPECS[action]
            value["parameters"] = deepcopy(spec["parameters"])
            value["preconditions"] = list(spec["preconditions"])
            value["effects"] = list(spec["effects"])
        catalog.append(value)
    return catalog
