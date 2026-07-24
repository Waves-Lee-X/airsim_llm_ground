#!/usr/bin/env python3
"""AeroMind Web 控制台节点。

用一个轻量 HTTP 服务把常用 ROS topic/service 暴露给浏览器：
  - 订阅飞控状态、里程计、RGB 图像、深度图、点云、检测结果
  - 调用 arm/takeoff/land/execute_task 服务
  - 发布 /control/cmd_vel 供 Web 虚拟摇杆使用
"""

import base64
import hashlib
import json
import math
import mimetypes
import os
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Empty, String

from aeromind_interfaces.msg import (
    AutonomyStatus,
    Detection,
    DetectionArray,
    DroneState,
    PerceptionHealth,
    SemanticObjectArray,
    SemanticRelationArray,
    WorldModelEvent,
    WorldModelHealth,
    Trajectory,
)
from aeromind_interfaces.srv import ArmDrone, ExecuteTask, Land, ReturnHome, Takeoff
from .json_support import json_compatible

try:
    from aeromind_agent.skill_catalog import get_skill_catalog as get_agent_skill_catalog
except Exception:
    get_agent_skill_catalog = None


SKILL_CATALOG = [
    {
        "name": "StatusSkill",
        "label": "状态检查",
        "type": "hard",
        "risk_level": "low",
        "description": "读取飞控状态，汇总解锁、模式、电池、GPS、EKF。",
        "example_task": "查询状态",
        "enabled": True,
    },
    {
        "name": "ArmSkill",
        "label": "解锁/加锁",
        "type": "hard",
        "risk_level": "medium",
        "description": "调用 /control/arm 执行解锁或加锁。",
        "example_task": "解锁",
        "enabled": True,
    },
    {
        "name": "TakeoffSkill",
        "label": "安全起飞",
        "type": "hard",
        "risk_level": "medium",
        "description": "解析目标高度，检查状态并调用 /control/takeoff。",
        "example_task": "起飞到10米",
        "enabled": True,
    },
    {
        "name": "LandSkill",
        "label": "降落",
        "type": "hard",
        "risk_level": "medium",
        "description": "读取当前状态并调用 /control/land。",
        "example_task": "降落",
        "enabled": True,
    },
    {
        "name": "PlanningSkill",
        "label": "路径规划",
        "type": "soft",
        "risk_level": "high",
        "description": "自然语言目标转成 /autonomy/goal，由底层自主避障执行。",
        "example_task": "飞到前方20米",
        "enabled": True,
    },
    {
        "name": "HoverSkill",
        "label": "悬停保持",
        "type": "hard",
        "risk_level": "medium",
        "description": "取消自主目标并发布零速度，进入悬停保持。",
        "example_task": "原地悬停",
        "enabled": True,
    },
    {
        "name": "ReturnHomeSkill",
        "label": "返航",
        "type": "hard",
        "risk_level": "high",
        "description": "调用 /control/return_home 触发 PX4 原生 RTL。",
        "example_task": "返航并降落",
        "enabled": True,
    },
    {
        "name": "EmergencyStopSkill",
        "label": "急停",
        "type": "hard",
        "risk_level": "high",
        "description": "取消自主目标、发布零速度，并尝试调用 /control/arm 加锁。",
        "example_task": "立即急停",
        "enabled": True,
    },
    {
        "name": "PerceptionSkill",
        "label": "环境感知",
        "type": "perception",
        "risk_level": "low",
        "description": "读取相机、深度、LiDAR 或 detection 分析环境。",
        "example_task": "检查前方有没有障碍物",
        "enabled": True,
    },
    {
        "name": "CaptureImageSkill",
        "label": "拍照取证",
        "type": "perception",
        "risk_level": "low",
        "description": "调用 /perception/capture_image 保存当前 RGB 图像。",
        "example_task": "拍一张前方照片",
        "enabled": True,
    },
    {
        "name": "ScanAreaSkill",
        "label": "区域扫描",
        "type": "soft",
        "risk_level": "medium",
        "description": "基于当前传感器缓存生成区域扫描摘要。",
        "example_task": "扫描前方区域并报告障碍物",
        "enabled": True,
    },
    {
        "name": "TargetSearchSkill",
        "label": "目标搜索",
        "type": "soft",
        "risk_level": "medium",
        "description": "根据自然语言目标匹配当前 detection，例如中文“人”会映射到 YOLO 的 person。",
        "example_task": "检测人",
        "enabled": True,
    },
    {
        "name": "SemanticImageSkill",
        "label": "图像语义分析",
        "type": "perception",
        "risk_level": "low",
        "description": "调用 /perception/analyze_image，让视觉语言模型或规则摘要分析当前画面。",
        "example_task": "分析当前画面中有什么",
        "enabled": True,
    },
    {
        "name": "MissionSequenceSkill",
        "label": "复合任务",
        "type": "soft",
        "risk_level": "high",
        "description": "拆解起飞、移动、目标检测等多步自然语言任务。",
        "example_task": "起飞，向左飞20米，并检测人",
        "enabled": True,
    },
    {
        "name": "MissionReportSkill",
        "label": "任务报告",
        "type": "soft",
        "risk_level": "low",
        "description": "汇总飞行状态、工具调用、检测结果和任务结论。",
        "example_task": "生成当前任务报告",
        "enabled": True,
    },
]


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _finite_json_number(value):
    number = float(value)
    return number if math.isfinite(number) else None


def _web_health_signal(status, age_sec, rate_hz=None):
    result = {
        "status": str(status or "MISSING").upper(),
        "age_s": _finite_json_number(age_sec),
    }
    if rate_hz is not None:
        result["rate_hz"] = _finite_json_number(rate_hz)
    return result


def _position_standard_deviation(covariance):
    values = list(covariance)
    if len(values) != 9:
        return None
    diagonal = [float(values[index]) for index in (0, 4, 8)]
    if any(not math.isfinite(value) or value < 0.0 for value in diagonal):
        return None
    return math.sqrt(sum(diagonal) / 3.0)


class WebConsoleNode(Node):
    """ROS node + embedded HTTP server for the browser console."""

    def __init__(self):
        super().__init__("web_console_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter(
            "mission_log_dir",
            os.environ.get("AEROMIND_MISSION_DIR", os.path.expanduser("~/aeromind_ws/missions")),
        )
        self._host = str(self.get_parameter("host").value)
        self._port = int(self.get_parameter("port").value)
        self._mission_log_dir = os.path.expanduser(str(self.get_parameter("mission_log_dir").value))

        self._lock = threading.Lock()
        self._state = None
        self._state_received_at = 0.0
        self._odom = None
        self._odom_received_at = 0.0
        self._image = None
        self._depth = None
        self._depthcloud = None
        self._pointcloud = None
        self._detection = None
        self._detections = []
        self._detections_received_at = 0.0
        self._detections_frame_id = ""
        self._perception_health = None
        self._world_objects = []
        self._world_objects_received_at = 0.0
        self._world_objects_frame_id = ""
        self._world_health = None
        self._world_relations = []
        self._world_relations_received_at = 0.0
        self._world_events = []
        self._autonomy = None
        self._autonomy_goal = None
        self._autonomy_trajectory = None
        self._mission = None
        self._image_analysis = None
        self._events = []

        self.create_subscription(DroneState, "/control/drone_state", self._state_cb, 10)
        self.create_subscription(Odometry, "/sensor/odometry", self._odom_cb, 10)
        self.create_subscription(Image, "/sensor/camera/rgb/front_center", self._image_cb, 10)
        self.create_subscription(Image, "/sensor/camera/depth/front_center", self._depth_cb, 10)
        self.create_subscription(PointCloud2, "/sensor/lidar/points", self._pointcloud_cb, 10)
        self.create_subscription(Detection, "/perception/detection", self._detection_cb, 10)
        self.create_subscription(DetectionArray, "/perception/detections", self._detections_cb, 10)
        self.create_subscription(PerceptionHealth, "/perception/health", self._perception_health_cb, 10)
        self.create_subscription(
            SemanticObjectArray,
            "/world_model/objects",
            self._world_objects_cb,
            10,
        )
        self.create_subscription(
            WorldModelHealth,
            "/world_model/health",
            self._world_health_cb,
            10,
        )
        self.create_subscription(
            SemanticRelationArray,
            "/world_model/relations",
            self._world_relations_cb,
            10,
        )
        self.create_subscription(
            WorldModelEvent,
            "/world_model/events",
            self._world_event_cb,
            10,
        )
        self.create_subscription(AutonomyStatus, "/autonomy/status", self._autonomy_cb, 10)
        self.create_subscription(PoseStamped, "/autonomy/goal", self._autonomy_goal_cb, 10)
        self.create_subscription(Trajectory, "/autonomy/trajectory", self._autonomy_trajectory_cb, 10)
        self.create_subscription(String, "/agent/mission_status", self._mission_cb, 10)
        self.create_subscription(String, "/perception/image_analysis", self._image_analysis_cb, 10)

        self._cmd_vel_pub = self.create_publisher(Twist, "/control/cmd_vel", 10)
        self._autonomy_cancel_pub = self.create_publisher(Empty, "/autonomy/cancel", 10)

        self._arm_client = self.create_client(ArmDrone, "/control/arm")
        self._takeoff_client = self.create_client(Takeoff, "/control/takeoff")
        self._land_client = self.create_client(Land, "/control/land")
        self._return_home_client = self.create_client(ReturnHome, "/control/return_home")
        self._task_client = self.create_client(ExecuteTask, "/agent/execute_task")

        self._static_dir = os.path.join(
            get_package_share_directory("aeromind_web"), "static"
        )
        self._server = ThreadingHTTPServer((self._host, self._port), ConsoleRequestHandler)
        self._server.node = self
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="aeromind-web-http",
            daemon=True,
        )
        self._server_thread.start()

        self._add_event("system", f"Web 控制台监听中: http://{self._host}:{self._port}")
        self.get_logger().info(
            f"Web 控制台已启动: http://localhost:{self._port}"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _state_cb(self, msg: DroneState):
        received_at = time.time()
        with self._lock:
            self._state = {
                "armed": bool(msg.armed),
                "mode": msg.mode,
                "battery": float(msg.battery),
                "gps_fix": int(msg.gps_fix),
                "ekf_healthy": bool(msg.ekf_healthy),
                "preflight_ok": bool(msg.preflight_ok),
                "preflight_valid": bool(msg.preflight_valid),
                "landed": bool(msg.landed),
                "landed_valid": bool(msg.landed_valid),
            }
            self._state_received_at = received_at

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        received_at = time.time()
        with self._lock:
            self._odom = {
                "stamp": _stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "position": {"x": pos.x, "y": pos.y, "z": pos.z},
                "velocity": {"x": vel.x, "y": vel.y, "z": vel.z},
            }
            self._odom_received_at = received_at

    def _image_cb(self, msg: Image):
        data = bytes(msg.data)
        with self._lock:
            self._image = {
                "stamp": _stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "height": int(msg.height),
                "width": int(msg.width),
                "encoding": msg.encoding,
                "step": int(msg.step),
                "data": base64.b64encode(data).decode("ascii"),
            }

    def _depth_cb(self, msg: Image):
        summary, depthcloud = self._summarize_depth(msg)
        with self._lock:
            self._depth = summary
            self._depthcloud = depthcloud

    def _pointcloud_cb(self, msg: PointCloud2):
        summary = self._summarize_pointcloud(msg)
        with self._lock:
            self._pointcloud = summary

    def _detection_cb(self, msg: Detection):
        with self._lock:
            self._detection = {
                "class_name": msg.class_name,
                "confidence": float(msg.confidence),
                "x": float(msg.x),
                "y": float(msg.y),
                "width": float(msg.width),
                "height": float(msg.height),
            }
            self._detections_received_at = time.time()

    def _detections_cb(self, msg: DetectionArray):
        detections = []
        for item in msg.detections:
            detections.append({
                "class_name": item.class_name,
                "confidence": float(item.confidence),
                "x": float(item.x),
                "y": float(item.y),
                "width": float(item.width),
                "height": float(item.height),
            })
        with self._lock:
            self._detections = detections
            self._detections_received_at = time.time()
            self._detections_frame_id = msg.header.frame_id
            if detections:
                self._detection = max(detections, key=lambda item: item["confidence"])
            else:
                self._detection = None

    def _perception_health_cb(self, msg: PerceptionHealth):
        with self._lock:
            self._perception_health = {
                "received_at": time.time(),
                "overall_status": msg.overall_status,
                "yolo_enabled": bool(msg.yolo_enabled),
                "vlm_enabled": bool(msg.vlm_enabled),
                "rgb": _web_health_signal(msg.rgb_status, msg.rgb_age_sec, msg.rgb_rate_hz),
                "depth": _web_health_signal(msg.depth_status, msg.depth_age_sec, msg.depth_rate_hz),
                "camera_info": _web_health_signal(msg.camera_info_status, msg.camera_info_age_sec),
                "pointcloud": _web_health_signal(msg.pointcloud_status, msg.pointcloud_age_sec, msg.pointcloud_rate_hz),
                "detections": _web_health_signal(msg.detections_status, msg.detections_age_sec, msg.detections_rate_hz),
                "vlm": {
                    "status": msg.vlm_status,
                    "age_s": _finite_json_number(msg.vlm_age_sec),
                    "message": msg.vlm_message,
                },
                "message": msg.message,
            }

    def _world_objects_cb(self, msg: SemanticObjectArray):
        objects = []
        for item in msg.objects:
            objects.append({
                "id": item.id,
                "class_name": item.class_name,
                "confidence": float(item.confidence),
                "position_valid": bool(item.position_valid),
                "position": {
                    "x": float(item.position.x),
                    "y": float(item.position.y),
                    "z": float(item.position.z),
                } if item.position_valid else None,
                "velocity": {
                    "x": float(item.velocity.x),
                    "y": float(item.velocity.y),
                    "z": float(item.velocity.z),
                },
                "dynamic": bool(item.dynamic),
                "position_covariance": [
                    float(value) for value in item.position_covariance
                ],
                "position_std_m": _position_standard_deviation(
                    item.position_covariance
                ),
                "depth_m": _finite_json_number(item.depth_m),
                "age_s": float(item.age_sec),
                "state": item.state,
                "evidence_type": item.evidence_type,
                "sources": list(item.sources),
                "hit_count": int(item.hit_count),
                "miss_count": int(item.miss_count),
            })
        with self._lock:
            self._world_objects = objects
            self._world_objects_received_at = time.time()
            self._world_objects_frame_id = msg.header.frame_id

    def _world_health_cb(self, msg: WorldModelHealth):
        with self._lock:
            self._world_health = {
                "received_at": time.time(),
                "frame_id": msg.header.frame_id,
                "healthy": bool(msg.healthy),
                "state": msg.state,
                "detections_fresh": bool(msg.detections_fresh),
                "depth_fresh": bool(msg.depth_fresh),
                "camera_info_valid": bool(msg.camera_info_valid),
                "tf_available": bool(msg.tf_available),
                "sync_delta_s": _finite_json_number(msg.sync_delta_sec),
                "observation_count": int(msg.observation_count),
                "track_count": int(msg.track_count),
                "confirmed_track_count": int(msg.confirmed_track_count),
                "message": msg.message,
            }

    def _world_relations_cb(self, msg: SemanticRelationArray):
        values = [
            {
                "subject_id": item.subject_id,
                "predicate": item.predicate,
                "object_id": item.object_id,
                "frame_id": msg.header.frame_id,
                "confidence": float(item.confidence),
                "distance_m": _finite_json_number(item.distance_m),
                "evidence_type": item.evidence_type,
                "sources": list(item.sources),
            }
            for item in msg.relations
        ]
        with self._lock:
            self._world_relations = values
            self._world_relations_received_at = time.time()

    def _world_event_cb(self, msg: WorldModelEvent):
        try:
            evidence = json.loads(msg.evidence_json) if msg.evidence_json else {}
        except json.JSONDecodeError:
            evidence = {"raw": msg.evidence_json}
        value = {
            "id": msg.id,
            "received_at": time.time(),
            "frame_id": msg.header.frame_id,
            "event_type": msg.event_type,
            "severity": msg.severity,
            "active": bool(msg.active),
            "object_ids": list(msg.object_ids),
            "confidence": float(msg.confidence),
            "evidence": evidence,
        }
        with self._lock:
            self._world_events.append(value)
            self._world_events = self._world_events[-100:]

    def _autonomy_cb(self, msg: AutonomyStatus):
        with self._lock:
            self._autonomy = {
                "stamp": _stamp_to_float(msg.header.stamp),
                "enabled": bool(msg.enabled),
                "state": msg.state,
                "replanning": bool(msg.replanning),
                "nearest_obstacle_m": float(msg.nearest_obstacle_m),
                "takeoff_clearance_valid": bool(msg.takeoff_clearance_valid),
                "takeoff_clearance_m": float(msg.takeoff_clearance_m),
                "takeoff_clearance_source": msg.takeoff_clearance_source,
                "target_distance_m": float(msg.target_distance_m),
                "active_strategy": msg.active_strategy,
                "message": msg.message,
            }

    def _autonomy_goal_cb(self, msg: PoseStamped):
        position = msg.pose.position
        with self._lock:
            self._autonomy_goal = {
                "stamp": _stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "position": {"x": position.x, "y": position.y, "z": position.z},
            }

    def _autonomy_trajectory_cb(self, msg: Trajectory):
        with self._lock:
            self._autonomy_trajectory = {
                "stamp": _stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "collision_free": bool(msg.collision_free),
                "planner_mode": msg.planner_mode,
                "message": msg.message,
                "points": [
                    {
                        "x": point.position.x,
                        "y": point.position.y,
                        "z": point.position.z,
                        "yaw": float(point.yaw),
                    }
                    for point in msg.points[:200]
                ],
            }

    def _mission_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("收到无法解析的 /agent/mission_status")
            return
        with self._lock:
            self._mission = payload.get("active_mission")

    def _image_analysis_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("收到无法解析的 /perception/image_analysis")
            return
        with self._lock:
            self._image_analysis = payload

    # ------------------------------------------------------------------
    # API methods called by HTTP handler
    # ------------------------------------------------------------------

    def snapshot(self):
        with self._lock:
            now = time.time()
            state_age = (
                max(0.0, now - self._state_received_at)
                if self._state_received_at > 0.0
                else None
            )
            odom_age = (
                max(0.0, now - self._odom_received_at)
                if self._odom_received_at > 0.0
                else None
            )
            state_fresh = state_age is not None and state_age <= 2.5
            odom_fresh = odom_age is not None and odom_age <= 2.5
            detection_age = (
                max(0.0, now - self._detections_received_at)
                if self._detections_received_at > 0.0
                else None
            )
            detections_active = detection_age is not None and detection_age <= 3.0
            perception_health = (
                dict(self._perception_health) if self._perception_health else None
            )
            if perception_health:
                health_age = max(0.0, now - perception_health["received_at"])
                perception_health["age_s"] = health_age
                perception_health["fresh"] = health_age <= 2.0
                if not perception_health["fresh"]:
                    perception_health["overall_status"] = "MISSING"
            world_objects_age = (
                max(0.0, now - self._world_objects_received_at)
                if self._world_objects_received_at > 0.0
                else None
            )
            world_objects_active = (
                world_objects_age is not None and world_objects_age <= 3.0
            )
            world_relations_age = (
                max(0.0, now - self._world_relations_received_at)
                if self._world_relations_received_at > 0.0 else None
            )
            world_relations_active = (
                world_relations_age is not None and world_relations_age <= 3.0
            )
            return {
                "state": self._state,
                "odom": self._odom,
                "telemetry_meta": {
                    "state_received_at": self._state_received_at,
                    "state_age_s": state_age,
                    "state_fresh": state_fresh,
                    "odom_received_at": self._odom_received_at,
                    "odom_age_s": odom_age,
                    "odom_fresh": odom_fresh,
                    "active": state_fresh and odom_fresh,
                },
                "depth": self._depth,
                "pointcloud": self._pointcloud_meta(),
                "detection": self._detection if detections_active else None,
                "detections": list(self._detections) if detections_active else [],
                "detection_meta": {
                    "source_topic": "/perception/detections",
                    "frame_id": self._detections_frame_id,
                    "received_at": self._detections_received_at,
                    "age_s": detection_age,
                    "active": detections_active,
                    "health_status": (
                        self._perception_health.get("detections", {}).get("status")
                        if self._perception_health else None
                    ),
                },
                "perception_health": perception_health,
                "world_objects": (
                    list(self._world_objects) if world_objects_active else []
                ),
                "world_model_meta": {
                    "source_topic": "/world_model/objects",
                    "frame_id": self._world_objects_frame_id,
                    "received_at": self._world_objects_received_at,
                    "age_s": world_objects_age,
                    "active": world_objects_active,
                    "health": dict(self._world_health) if self._world_health else None,
                    "relations": list(self._world_relations) if world_relations_active else [],
                    "relations_age_s": world_relations_age,
                    "events": list(self._world_events[-20:]),
                },
                "autonomy": self._autonomy,
                "autonomy_goal": self._autonomy_goal,
                "autonomy_trajectory": self._autonomy_trajectory,
                "mission": self._mission,
                "image_analysis": self._image_analysis,
                "events": list(self._events[-80:]),
                "services": {
                    "arm": self._arm_client.service_is_ready(),
                    "takeoff": self._takeoff_client.service_is_ready(),
                    "land": self._land_client.service_is_ready(),
                    "return_home": self._return_home_client.service_is_ready(),
                    "agent": self._task_client.service_is_ready(),
                },
            }

    def latest_image(self):
        with self._lock:
            return dict(self._image) if self._image is not None else None

    def latest_pointcloud(self):
        with self._lock:
            return dict(self._pointcloud) if self._pointcloud is not None else None

    def latest_depthcloud(self):
        with self._lock:
            return dict(self._depthcloud) if self._depthcloud is not None else None

    def arm(self, arm: bool):
        req = ArmDrone.Request()
        req.arm = bool(arm)
        return self._call_service(self._arm_client, req, "解锁" if arm else "加锁", timeout_sec=15.0)

    def takeoff(self, altitude: float):
        req = Takeoff.Request()
        req.altitude = float(altitude)
        return self._call_service(self._takeoff_client, req, f"起飞到 {altitude:.1f}m", timeout_sec=30.0)

    def land(self):
        req = Land.Request()
        return self._call_service(self._land_client, req, "降落", timeout_sec=30.0)

    def return_home(self):
        req = ReturnHome.Request()
        return self._call_service(self._return_home_client, req, "PX4 RTL 返航", timeout_sec=15.0)

    def execute_task(self, task: str):
        req = ExecuteTask.Request()
        req.task_description = task.strip()
        if not req.task_description:
            return {"success": False, "message": "任务不能为空"}
        # Agent may call downstream Control/PX4/AirSim services after confirmation.
        label = "确认执行任务" if req.task_description.startswith("__aeromind_confirm__:") else "自然语言任务"
        return self._call_service(self._task_client, req, label, timeout_sec=90.0)

    def skill_catalog(self):
        if get_agent_skill_catalog is not None:
            return {"success": True, "skills": get_agent_skill_catalog()}
        return {"success": True, "skills": [dict(skill) for skill in SKILL_CATALOG]}

    def mission_reports(self, limit: int = 20):
        root = os.path.abspath(self._mission_log_dir)
        if not os.path.isdir(root):
            return {"success": True, "reports": [], "mission_log_dir": root}
        reports = []
        for name in os.listdir(root):
            if not name.startswith("mission-"):
                continue
            mission_dir = os.path.abspath(os.path.join(root, name))
            if not mission_dir.startswith(root) or not os.path.isdir(mission_dir):
                continue
            report_path = os.path.join(mission_dir, "report.md")
            mission_path = os.path.join(mission_dir, "mission.json")
            if not os.path.exists(report_path) and not os.path.exists(mission_path):
                continue
            stat_path = report_path if os.path.exists(report_path) else mission_path
            item = {
                "id": name,
                "mission_dir": mission_dir,
                "report_md": report_path if os.path.exists(report_path) else "",
                "mission_json": mission_path if os.path.exists(mission_path) else "",
                "captures": self._mission_captures(mission_dir),
                "updated_at": os.path.getmtime(stat_path),
                "status": "--",
                "message": "",
            }
            if os.path.exists(mission_path):
                try:
                    with open(mission_path, "r", encoding="utf-8") as file:
                        mission = json.load(file)
                    item["status"] = mission.get("status", "--")
                    item["message"] = mission.get("message", "")
                    item["updated_at"] = float(mission.get("updated_at") or item["updated_at"])
                except Exception:
                    pass
            reports.append(item)
        reports.sort(key=lambda item: item["updated_at"], reverse=True)
        return {"success": True, "reports": reports[: max(1, int(limit))], "mission_log_dir": root}

    def mission_report(self, mission_id: str):
        safe_id = os.path.basename(mission_id.strip())
        root = os.path.abspath(self._mission_log_dir)
        mission_dir = os.path.abspath(os.path.join(root, safe_id))
        if not mission_dir.startswith(root) or not os.path.isdir(mission_dir):
            return {"success": False, "message": "任务报告不存在"}
        report_path = os.path.join(mission_dir, "report.md")
        mission_path = os.path.join(mission_dir, "mission.json")
        if not os.path.exists(report_path):
            return {"success": False, "message": "report.md 尚未生成"}
        with open(report_path, "r", encoding="utf-8") as file:
            content = file.read()
        mission = None
        if os.path.exists(mission_path):
            try:
                with open(mission_path, "r", encoding="utf-8") as file:
                    mission = json.load(file)
            except Exception:
                mission = None
        return {
            "success": True,
            "id": safe_id,
            "mission_dir": mission_dir,
            "report_md": report_path,
            "mission_json": mission_path if os.path.exists(mission_path) else "",
            "captures": self._mission_captures(mission_dir),
            "content": content,
            "mission": mission,
        }

    def mission_report_path(self, mission_id: str):
        safe_id = os.path.basename(mission_id.strip())
        root = os.path.abspath(self._mission_log_dir)
        mission_dir = os.path.abspath(os.path.join(root, safe_id))
        report_path = os.path.abspath(os.path.join(mission_dir, "report.md"))
        if not mission_dir.startswith(root) or not report_path.startswith(mission_dir):
            return None
        return report_path if os.path.exists(report_path) else None

    def mission_capture_path(self, mission_id: str, filename: str):
        safe_id = os.path.basename(mission_id.strip())
        safe_name = os.path.basename(filename.strip())
        root = os.path.abspath(self._mission_log_dir)
        captures_dir = os.path.abspath(os.path.join(root, safe_id, "captures"))
        path = os.path.abspath(os.path.join(captures_dir, safe_name))
        if not captures_dir.startswith(root) or not path.startswith(captures_dir):
            return None
        return path if os.path.exists(path) else None

    def _mission_captures(self, mission_dir: str):
        captures_dir = os.path.join(mission_dir, "captures")
        if not os.path.isdir(captures_dir):
            return []
        captures = []
        for name in sorted(os.listdir(captures_dir)):
            if not name.lower().endswith(
                (".png", ".jpg", ".jpeg", ".webp", ".ppm", ".pgm")
            ):
                continue
            path = os.path.join(captures_dir, name)
            if os.path.isfile(path):
                captures.append({"filename": name, "path": path, "mtime": os.path.getmtime(path)})
        return captures

    def publish_cmd_vel(self, payload):
        msg = Twist()
        linear = payload.get("linear", {})
        angular = payload.get("angular", {})
        msg.linear.x = float(linear.get("x", 0.0))
        msg.linear.y = float(linear.get("y", 0.0))
        msg.linear.z = float(linear.get("z", 0.0))
        msg.angular.z = float(angular.get("z", 0.0))
        self._cmd_vel_pub.publish(msg)
        return {"success": True, "message": "cmd_vel 已发布"}

    def cancel_autonomy(self):
        self._autonomy_cancel_pub.publish(Empty())
        self._cmd_vel_pub.publish(Twist())
        self._add_event("service", "自主任务已取消，已发布悬停零速度")
        return {"success": True, "message": "已取消当前自主任务并发布零速度"}

    def shutdown_server(self):
        if hasattr(self, "_server"):
            self._server.shutdown()
            self._server.server_close()

    def _call_service(self, client, request, label: str, timeout_sec: float = 8.0):
        if not client.service_is_ready():
            msg = f"{label}: 服务未就绪"
            self._add_event("error", msg)
            return {"success": False, "message": msg}

        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout_sec):
            msg = f"{label}: 服务调用超时"
            self._add_event("error", msg)
            return {"success": False, "message": msg}

        try:
            result = future.result()
        except Exception as exc:
            msg = f"{label}: {exc}"
            self._add_event("error", msg)
            return {"success": False, "message": msg}

        data = {}
        for field in ("success", "message", "result"):
            if hasattr(result, field):
                data[field] = getattr(result, field)
        if "success" not in data:
            data["success"] = True
        if "message" not in data:
            data["message"] = f"{label}: 已完成"

        self._add_event("service", f"{label}: {data.get('message', '')}")
        return data

    def _add_event(self, kind: str, message: str):
        with self._lock:
            self._events.append({"kind": kind, "message": message})
            self._events = self._events[-100:]

    def _pointcloud_meta(self):
        if self._pointcloud is None:
            return None
        return {
            key: value
            for key, value in self._pointcloud.items()
            if key != "points"
        }

    def _summarize_depth(self, msg: Image):
        data = bytes(msg.data)
        width = int(msg.width)
        height = int(msg.height)
        value_step = None
        read_value = None
        to_meters = None
        encoding = msg.encoding.lower()
        summary = {
            "stamp": _stamp_to_float(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "width": width,
            "height": height,
            "encoding": msg.encoding,
            "step": int(msg.step),
            "center_m": None,
            "min_m": None,
            "max_m": None,
            "valid_samples": 0,
        }
        depthcloud = {
            "stamp": summary["stamp"],
            "frame_id": msg.header.frame_id or "camera_front_center_body",
            "source_topic": "/sensor/camera/depth/front_center",
            "fixed_frame": "camera_front_center_body",
            "style": "rviz_depthcloud",
            "color_transformer": "AxisColor",
            "axis": "Y",
            "width": width,
            "height": height,
            "total_points": width * height,
            "sampled_points": 0,
            "points": [],
        }

        if encoding in ("32fc1", "32fc"):
            value_step = 4
            read_value = lambda offset: struct.unpack_from("<f", data, offset)[0]
            to_meters = lambda value: value
        elif encoding in ("16uc1", "mono16"):
            value_step = 2
            read_value = lambda offset: struct.unpack_from("<H", data, offset)[0]
            to_meters = lambda value: float(value) / 1000.0

        if width <= 0 or height <= 0 or value_step is None or not data:
            return summary, depthcloud

        center_offset = (height // 2) * int(msg.step) + (width // 2) * value_step
        if center_offset + value_step <= len(data):
            center = to_meters(read_value(center_offset))
            if math.isfinite(center) and center > 0:
                summary["center_m"] = center

        total = width * height
        stride = max(1, total // 6000)
        min_depth = None
        max_depth = None
        valid = 0
        for idx in range(0, total, stride):
            row = idx // width
            col = idx % width
            offset = row * int(msg.step) + col * value_step
            if offset + value_step > len(data):
                continue
            value = to_meters(read_value(offset))
            if not math.isfinite(value) or value <= 0:
                continue
            min_depth = value if min_depth is None else min(min_depth, value)
            max_depth = value if max_depth is None else max(max_depth, value)
            valid += 1

        summary["min_m"] = min_depth
        summary["max_m"] = max_depth
        summary["valid_samples"] = valid
        depthcloud["points"] = self._sample_depthcloud_points(
            data, width, height, int(msg.step), value_step, read_value, to_meters
        )
        depthcloud["sampled_points"] = len(depthcloud["points"])
        return summary, depthcloud

    def _sample_depthcloud_points(
        self, data, width, height, row_step, value_step, read_value, to_meters
    ):
        # Matches the camera config used by CameraBridge for front_center depth.
        fov_rad = math.radians(95.0)
        fx = width / (2.0 * math.tan(fov_rad / 2.0))
        fy = fx
        cx = (width - 1) / 2.0
        cy = (height - 1) / 2.0
        target_points = 2400
        pixel_stride = max(2, int(math.sqrt(max(1, (width * height) / target_points))))
        points = []

        for v in range(0, height, pixel_stride):
            for u in range(0, width, pixel_stride):
                offset = v * row_step + u * value_step
                if offset + value_step > len(data):
                    continue
                depth = to_meters(read_value(offset))
                if not math.isfinite(depth) or depth <= 0.05 or depth > 80.0:
                    continue

                lateral = (u - cx) * depth / fx
                vertical = -(v - cy) * depth / fy
                # RViz DepthCloud is viewed in the camera body frame here:
                # X forward, Y left/right, Z up.
                points.append([
                    round(float(depth), 3),
                    round(float(-lateral), 3),
                    round(float(vertical), 3),
                ])

        return points

    def _summarize_pointcloud(self, msg: PointCloud2):
        width = int(msg.width)
        height = int(msg.height)
        point_step = int(msg.point_step)
        total_points = width * height
        summary = {
            "stamp": _stamp_to_float(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "width": width,
            "height": height,
            "point_step": point_step,
            "row_step": int(msg.row_step),
            "total_points": total_points,
            "sampled_points": 0,
            "points": [],
        }

        fields = {field.name: field for field in msg.fields}
        if point_step <= 0 or not all(name in fields for name in ("x", "y", "z")):
            return summary

        x_field = fields["x"]
        y_field = fields["y"]
        z_field = fields["z"]
        if not all(field.datatype == 7 for field in (x_field, y_field, z_field)):
            return summary

        endian = ">" if msg.is_bigendian else "<"
        stride = max(1, total_points // 1800)
        data = bytes(msg.data)
        points = []

        for idx in range(0, total_points, stride):
            row = idx // width if width else 0
            col = idx % width if width else 0
            base = row * int(msg.row_step) + col * point_step
            max_offset = base + max(x_field.offset, y_field.offset, z_field.offset) + 4
            if max_offset > len(data):
                continue
            x = struct.unpack_from(endian + "f", data, base + x_field.offset)[0]
            y = struct.unpack_from(endian + "f", data, base + y_field.offset)[0]
            z = struct.unpack_from(endian + "f", data, base + z_field.offset)[0]
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            points.append([round(float(x), 3), round(float(y), 3), round(float(z), 3)])

        summary["points"] = points
        summary["sampled_points"] = len(points)
        return summary


class ConsoleRequestHandler(BaseHTTPRequestHandler):
    """HTTP routes for static files and JSON APIs."""

    server_version = "AeroMindWeb/0.1"

    def do_OPTIONS(self):
        self.send_response(204)
        self._send_cors_headers()
        self.end_headers()

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/ws":
            self._serve_websocket()
        elif path == "/":
            self._serve_static("index.html")
        elif path == "/api/status":
            self._json(self.server.node.snapshot())
        elif path == "/api/skills":
            self._json(self.server.node.skill_catalog())
        elif path == "/api/reports":
            self._json(self.server.node.mission_reports())
        elif path.startswith("/api/reports/") and path.endswith("/download"):
            mission_id = unquote(path.removeprefix("/api/reports/").removesuffix("/download").strip("/"))
            self._serve_report_download(mission_id)
        elif path.startswith("/api/reports/") and "/captures/" in path:
            rest = path.removeprefix("/api/reports/")
            mission_id, filename = rest.split("/captures/", 1)
            self._serve_report_capture(unquote(mission_id), unquote(filename))
        elif path.startswith("/api/reports/"):
            mission_id = unquote(path.removeprefix("/api/reports/"))
            result = self.server.node.mission_report(mission_id)
            self._json(result, status=200 if result.get("success") else 404)
        elif path == "/api/camera":
            image = self.server.node.latest_image()
            if image is None:
                self._json({"success": False, "message": "暂无相机图像"}, status=404)
            else:
                image["success"] = True
                self._json(image)
        elif path == "/api/depthcloud":
            depthcloud = self.server.node.latest_depthcloud()
            if depthcloud is None:
                self._json({"success": False, "message": "暂无深度点云"}, status=404)
            else:
                depthcloud["success"] = True
                self._json(depthcloud)
        elif path == "/api/pointcloud":
            pointcloud = self.server.node.latest_pointcloud()
            if pointcloud is None:
                self._json({"success": False, "message": "暂无点云数据"}, status=404)
            else:
                pointcloud["success"] = True
                self._json(pointcloud)
        elif path.startswith("/static/"):
            self._serve_static(path.removeprefix("/static/"))
        elif path in {"/styles.css", "/enterprise.css", "/app.js"}:
            self._serve_static(path.removeprefix("/"))
        else:
            self._json({"success": False, "message": "not found"}, status=404)

    def do_POST(self):
        path = urlparse(self.path).path
        payload = self._read_json()
        node = self.server.node

        if path == "/api/control/arm":
            result = node.arm(bool(payload.get("arm", True)))
        elif path == "/api/control/takeoff":
            result = node.takeoff(float(payload.get("altitude", 10.0)))
        elif path == "/api/control/land":
            result = node.land()
        elif path == "/api/control/return_home":
            result = node.return_home()
        elif path == "/api/agent/task":
            result = node.execute_task(str(payload.get("task", "")))
        elif path == "/api/agent/chat":
            result = node.execute_task(str(payload.get("message", "")))
        elif path == "/api/cmd_vel":
            result = node.publish_cmd_vel(payload)
        elif path == "/api/autonomy/cancel":
            result = node.cancel_autonomy()
        else:
            result = {"success": False, "message": "not found"}
            self._json(result, status=404)
            return

        self._json(result, status=200 if result.get("success", False) else 503)

    def log_message(self, fmt, *args):
        # Keep HTTP access logs out of the ROS console unless there is a real error.
        return

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            return {}

    def _json(self, data, status=200):
        body = json.dumps(
            json_compatible(data), ensure_ascii=False, allow_nan=False
        ).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self._send_cors_headers()
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_cors_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _serve_static(self, rel_path: str):
        rel_path = rel_path.strip("/") or "index.html"
        static_dir = self.server.node._static_dir
        full_path = os.path.abspath(os.path.join(static_dir, rel_path))
        if not full_path.startswith(os.path.abspath(static_dir)) or not os.path.exists(full_path):
            self._json({"success": False, "message": "not found"}, status=404)
            return

        content_type = mimetypes.guess_type(full_path)[0] or "application/octet-stream"
        with open(full_path, "rb") as f:
            body = f.read()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_report_download(self, mission_id: str):
        path = self.server.node.mission_report_path(mission_id)
        if not path:
            self._json({"success": False, "message": "report.md 不存在"}, status=404)
            return
        with open(path, "rb") as file:
            body = file.read()
        self.send_response(200)
        self.send_header("Content-Type", "text/markdown; charset=utf-8")
        self.send_header("Content-Disposition", f'attachment; filename="{os.path.basename(os.path.dirname(path))}_report.md"')
        self._send_cors_headers()
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_report_capture(self, mission_id: str, filename: str):
        path = self.server.node.mission_capture_path(mission_id, filename)
        if not path:
            self._json({"success": False, "message": "截图不存在"}, status=404)
            return
        mime, _ = mimetypes.guess_type(path)
        with open(path, "rb") as file:
            body = file.read()
        self.send_response(200)
        self.send_header("Content-Type", mime or "application/octet-stream")
        self.send_header("Cache-Control", "no-store")
        self._send_cors_headers()
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_websocket(self):
        key = self.headers.get("Sec-WebSocket-Key")
        if not key:
            self._json({"success": False, "message": "missing websocket key"}, status=400)
            return

        accept = base64.b64encode(
            hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()
        ).decode("ascii")
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()

        try:
            while True:
                payload = {
                    "type": "telemetry",
                    "stamp": time.time(),
                    "data": self.server.node.snapshot(),
                }
                self._send_ws_json(payload)
                time.sleep(1.0)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def _send_ws_json(self, payload):
        data = json.dumps(
            json_compatible(payload), ensure_ascii=False, allow_nan=False
        ).encode("utf-8")
        header = bytearray([0x81])
        length = len(data)
        if length < 126:
            header.append(length)
        elif length < 65536:
            header.extend([126, (length >> 8) & 0xFF, length & 0xFF])
        else:
            header.extend([
                127,
                (length >> 56) & 0xFF,
                (length >> 48) & 0xFF,
                (length >> 40) & 0xFF,
                (length >> 32) & 0xFF,
                (length >> 24) & 0xFF,
                (length >> 16) & 0xFF,
                (length >> 8) & 0xFF,
                length & 0xFF,
            ])
        self.wfile.write(header)
        self.wfile.write(data)
        self.wfile.flush()


def main(args=None):
    rclpy.init(args=args)
    node = WebConsoleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_server()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
