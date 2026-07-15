"""Deterministic atomic capabilities exposed by the ROS execution layer."""

from __future__ import annotations

from typing import Any


CAPABILITIES: tuple[dict[str, Any], ...] = (
    {"name": "system.status", "label": "飞控状态", "category": "system", "risk_level": "low", "mode": "read", "interface": "/control/drone_state", "description": "读取解锁、模式、电池、GPS、EKF、位置和速度。", "example_task": "查询无人机状态"},
    {"name": "system.safety_check", "label": "安全检查", "category": "system", "risk_level": "low", "mode": "local", "interface": "Gateway snapshot", "description": "确定性检查状态新鲜度、EKF、GPS 和最近障碍物。", "example_task": "检查当前是否适合起飞"},
    {"name": "flight.arm", "label": "解锁", "category": "flight", "risk_level": "high", "mode": "action", "action": "arm", "interface": "/control/arm", "description": "请求 PX4 解锁。", "example_task": "解锁无人机"},
    {"name": "flight.disarm", "label": "加锁", "category": "flight", "risk_level": "high", "mode": "action", "action": "disarm", "interface": "/control/arm", "description": "请求 PX4 加锁。", "example_task": "加锁无人机"},
    {"name": "flight.takeoff", "label": "起飞", "category": "flight", "risk_level": "high", "mode": "action", "action": "takeoff", "interface": "/control/takeoff", "description": "按目标高度执行 PX4 起飞。", "example_task": "起飞到10米"},
    {"name": "flight.land", "label": "降落", "category": "flight", "risk_level": "high", "mode": "action", "action": "land", "interface": "/control/land", "description": "执行 PX4 原生降落。", "example_task": "降落"},
    {"name": "flight.move_relative", "label": "相对移动", "category": "flight", "risk_level": "high", "mode": "action", "action": "move", "interface": "/autonomy/goal", "description": "按机头坐标系发布相对三维目标并由自主规划器执行。", "example_task": "向前飞10米"},
    {"name": "flight.hover", "label": "悬停", "category": "flight", "risk_level": "medium", "mode": "action", "action": "hover", "interface": "/autonomy/cancel + /control/cmd_vel", "description": "取消自主目标并进入悬停保持。", "example_task": "原地悬停"},
    {"name": "flight.return_home", "label": "返航", "category": "flight", "risk_level": "high", "mode": "action", "action": "return_home", "interface": "/control/return_home", "description": "触发 PX4 原生 RTL。", "example_task": "返航"},
    {"name": "perception.capture_image", "label": "拍照保存", "category": "perception", "risk_level": "low", "mode": "action", "action": "capture_image", "interface": "/perception/capture_image", "description": "保存当前前视 RGB 图像并返回路径。", "example_task": "拍一张前方照片"},
    {"name": "perception.detect_target", "label": "目标检测", "category": "perception", "risk_level": "low", "mode": "read", "interface": "/perception/detections", "description": "读取 YOLO 目标类别、置信度和图像位置。", "example_task": "检测前方是否有人"},
    {"name": "perception.semantic_image", "label": "图像语义", "category": "perception", "risk_level": "low", "mode": "read", "interface": "/perception/analyze_image", "description": "使用 VLM 或规则摘要分析当前画面。", "example_task": "分析当前画面"},
)


def capability_catalog() -> list[dict[str, Any]]:
    return [{**item, "enabled": True, "type": "capability"} for item in CAPABILITIES]
