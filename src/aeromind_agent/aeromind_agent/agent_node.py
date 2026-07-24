#!/usr/bin/env python3
"""
agent_node.py - LLM Agent 任务调度节点

功能：
  - 提供 /agent/execute_task 服务：接收自然语言任务
  - 初级自然语言规则解析：解锁、加锁、起飞、降落、查询状态
  - 预留 LLM 接口参数（后续可升级为 LLM + 工具调用链）

调用链路：
  execute_task → parse intent → call control/status
"""

import json
import math
import os
import re
import shutil
import struct
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Empty, String

from aeromind_interfaces.msg import AutonomyStatus, Detection, DetectionArray, DroneState
from aeromind_interfaces.srv import AnalyzeImage, ArmDrone, CaptureImage, ExecuteAction, ExecuteTask, Land, ReturnHome, Takeoff

from .skill_catalog import get_skill_catalog, skill_by_name


class AgentNode(Node):
    """LLM Agent 任务调度节点"""

    def __init__(self):
        super().__init__("agent_node")

        # LLM uses an OpenAI-compatible chat completions endpoint.
        self.declare_parameter("llm_api_url", "http://localhost:11434/v1")
        self.declare_parameter("llm_model", "llama3")
        self.declare_parameter("llm_enabled", True)
        self.declare_parameter("llm_timeout_sec", 3.0)
        self.declare_parameter("llm_api_key", "")
        self.declare_parameter(
            "mission_log_dir",
            os.environ.get("AEROMIND_MISSION_DIR", os.path.expanduser("~/aeromind_ws/missions")),
        )
        self._llm_api_url = self.get_parameter("llm_api_url").value
        self._llm_model = self.get_parameter("llm_model").value
        self._llm_enabled = bool(self.get_parameter("llm_enabled").value)
        self._llm_timeout_sec = float(self.get_parameter("llm_timeout_sec").value)
        self._mission_log_dir = os.path.expanduser(str(self.get_parameter("mission_log_dir").value))
        self._llm_api_key = (
            str(self.get_parameter("llm_api_key").value).strip()
            or os.environ.get("AEROMIND_LLM_API_KEY", "").strip()
            or os.environ.get("DEEPSEEK_API_KEY", "").strip()
            or os.environ.get("OPENAI_API_KEY", "").strip()
        )
        self._latest_state = None
        self._latest_odom = None
        self._home_odom = None
        self._latest_image = None
        self._latest_depth = None
        self._latest_pointcloud = None
        self._latest_detection = None
        self._latest_detections = []
        self._latest_image_analysis = None
        self._last_task_summary = None
        self._pending_tasks = {}
        self._deferred_move = None
        self._latest_autonomy_status = None
        self._active_mission = None

        # 提供任务执行服务
        self._task_srv = self.create_service(
            ExecuteTask, "/agent/execute_task", self._execute_task_callback
        )
        self._action_srv = self.create_service(
            ExecuteAction, "/agent/execute_action", self._execute_action_callback
        )

        # 创建服务客户端（用于调用其他模块的服务）
        self._service_client_group = ReentrantCallbackGroup()
        self._arm_client = self.create_client(
            ArmDrone, "/control/arm", callback_group=self._service_client_group
        )
        self._takeoff_client = self.create_client(
            Takeoff, "/control/takeoff", callback_group=self._service_client_group
        )
        self._land_client = self.create_client(
            Land, "/control/land", callback_group=self._service_client_group
        )
        self._return_home_client = self.create_client(
            ReturnHome,
            "/control/return_home",
            callback_group=self._service_client_group,
        )
        self._capture_image_client = self.create_client(
            CaptureImage,
            "/perception/capture_image",
            callback_group=self._service_client_group,
        )
        self._analyze_image_client = self.create_client(
            AnalyzeImage,
            "/perception/analyze_image",
            callback_group=self._service_client_group,
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/control/cmd_vel", 10)
        self._autonomy_goal_pub = self.create_publisher(PoseStamped, "/autonomy/goal", 10)
        self._autonomy_cancel_pub = self.create_publisher(Empty, "/autonomy/cancel", 10)
        self._mission_status_pub = self.create_publisher(String, "/agent/mission_status", 10)

        self.create_subscription(
            DroneState, "/control/drone_state", self._drone_state_callback, 10
        )
        self.create_subscription(Odometry, "/sensor/odometry", self._odom_callback, 10)
        self.create_subscription(Image, "/sensor/camera/rgb/front_center", self._image_callback, 10)
        self.create_subscription(Image, "/sensor/camera/depth/front_center", self._depth_callback, 10)
        self.create_subscription(PointCloud2, "/sensor/lidar/points", self._pointcloud_callback, 10)
        self.create_subscription(Detection, "/perception/detection", self._detection_callback, 10)
        self.create_subscription(DetectionArray, "/perception/detections", self._detections_callback, 10)
        self.create_subscription(AutonomyStatus, "/autonomy/status", self._autonomy_status_callback, 10)
        self.create_subscription(String, "/perception/image_analysis", self._image_analysis_callback, 10)
        self.create_timer(0.5, self._deferred_move_timer)
        self.create_timer(0.5, self._mission_timer_callback)
        self.create_timer(0.5, self._publish_mission_status)

        # 等待下游服务就绪
        self.get_logger().info("等待下游服务就绪...")
        self._wait_for_optional_service(self._arm_client, "/control/arm")
        self._wait_for_optional_service(self._takeoff_client, "/control/takeoff")
        self._wait_for_optional_service(self._land_client, "/control/land")
        self._wait_for_optional_service(self._return_home_client, "/control/return_home")
        self._wait_for_optional_service(self._capture_image_client, "/perception/capture_image")
        self._wait_for_optional_service(self._analyze_image_client, "/perception/analyze_image")

        mode = "LLM + 规则兜底" if self._llm_enabled else "规则解析"
        self.get_logger().info(f"Agent 节点已启动（{mode}）")
        self.get_logger().info(
            f"LLM 配置: enabled={self._llm_enabled}, url={self._llm_api_url}, "
            f"model={self._llm_model}, api_key={'已配置' if self._llm_api_key else '未配置'}"
        )

    def _execute_task_callback(self, request, response):
        """任务执行服务回调：解析自然语言并调用对应能力。"""
        task = request.task_description.strip()
        self.get_logger().info(f"收到任务: {task}")

        if not task:
            response.success = False
            response.result = self._json_result("unknown", {}, "任务为空")
            response.message = "请输入任务，例如：起飞到10米、降落、解锁、查询状态"
            return response

        if task.startswith("__aeromind_confirm__:"):
            token = task.removeprefix("__aeromind_confirm__:").strip()
            return self._execute_pending_task(token, response)

        if task.startswith("__aeromind_cancel__:"):
            token = task.removeprefix("__aeromind_cancel__:").strip()
            return self._cancel_pending_task(token, response)

        if task.startswith("__aeromind_move_vector__:"):
            try:
                args = json.loads(
                    task.removeprefix("__aeromind_move_vector__:").strip()
                )
                if not isinstance(args, dict):
                    raise ValueError("移动向量必须是对象")
                args = self._normalize_move_args(args, "")
                components = (
                    args["forward_m"], args["right_m"], args["up_m"]
                )
                if not all(math.isfinite(value) for value in components) or not any(
                    abs(value) >= 0.1 for value in components
                ) or any(
                    abs(value) > 100.0 for value in components
                ):
                    raise ValueError("移动向量超出允许范围")
            except (TypeError, ValueError, json.JSONDecodeError):
                response.success = False
                response.result = self._json_result("unknown", {}, "移动向量无效")
                response.message = "移动向量无效"
                return response
            parsed = self._parsed(
                "move_to",
                args,
                self._move_reason(args),
                risk_level="high",
                need_confirm=True,
                skill="PlanningSkill",
            )
            self.get_logger().info(f"解析结果: {json.dumps(parsed, ensure_ascii=False)}")
            return self._request_confirmation(parsed, response)

        parsed = self._parse_task(task)
        self.get_logger().info(f"解析结果: {json.dumps(parsed, ensure_ascii=False)}")

        if parsed["intent"] == "status":
            return self._execute_status(parsed, response)

        if self._requires_confirmation(parsed):
            return self._request_confirmation(parsed, response)

        return self._execute_parsed_task(parsed, response)

    def _execute_action_callback(self, request, response):
        """执行已经由 Gateway 确认的结构化基础动作。"""
        action = request.action.strip()
        try:
            args = json.loads(request.args_json or "{}")
            if not isinstance(args, dict):
                raise ValueError("args_json 必须是对象")
            parsed = self._structured_action(action, args)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            response.success = False
            response.message = f"结构化动作无效: {exc}"
            response.result = self._json_result("unknown", {}, response.message)
            return response

        self.get_logger().info(
            f"执行结构化动作[{request.request_id or '--'}]: "
            f"{json.dumps(parsed, ensure_ascii=False)}"
        )
        if action in {"arm", "disarm", "takeoff", "land", "move", "return_home"}:
            safety = self._safety_check(parsed)
            if not safety["ok"]:
                parsed["safety_check"] = safety
                response.success = False
                response.message = safety["message"]
                response.result = self._agent_result(
                    parsed,
                    reply=f"结构化动作安全检查未通过：{safety['message']}",
                    tool_calls=safety["tool_calls"],
                    final_status="安全检查未通过，未执行",
                    progress=safety["progress"],
                )
                return response
        return self._execute_parsed_task(parsed, response)

    def _structured_action(self, action: str, args: dict):
        simple = {
            "arm": ("arm", "ArmSkill", "执行解锁"),
            "disarm": ("disarm", "ArmSkill", "执行加锁"),
            "land": ("land", "LandSkill", "执行降落"),
            "return_home": ("return_home", "ReturnHomeSkill", "触发 PX4 原生 RTL 返航"),
            "hover": ("hover", "HoverSkill", "进入悬停保持"),
            "capture_image": ("capture_image", "CaptureImageSkill", "保存当前相机图像"),
        }
        if action in simple:
            intent, skill, reason = simple[action]
            return self._parsed(intent, {}, reason, skill=skill, need_confirm=False)

        if action == "takeoff":
            altitude = self._coerce_float(args.get("altitude"), 0.0)
            if not math.isfinite(altitude) or not 1.0 <= altitude <= 30.0:
                raise ValueError("起飞高度必须在 1 到 30 米之间")
            return self._parsed(
                "takeoff",
                {"altitude": altitude},
                f"起飞到 {altitude:.1f} 米",
                risk_level="high",
                need_confirm=False,
                skill="TakeoffSkill",
            )

        if action == "move":
            if "direction" in args:
                distance = self._coerce_float(args.get("distance"), 0.0)
                direction = str(args.get("direction", "")).lower()
                direction = {
                    "前方": "forward", "后方": "backward", "左侧": "left",
                    "右侧": "right", "上方": "up", "下方": "down",
                }.get(direction, direction)
                vectors = {
                    "forward": (distance, 0.0, 0.0),
                    "backward": (-distance, 0.0, 0.0),
                    "left": (0.0, -distance, 0.0),
                    "right": (0.0, distance, 0.0),
                    "up": (0.0, 0.0, distance),
                    "down": (0.0, 0.0, -distance),
                }
                if direction not in vectors or not 0.5 <= distance <= 100.0:
                    raise ValueError("移动方向或距离无效")
                forward_m, right_m, up_m = vectors[direction]
            else:
                forward_m = self._coerce_float(args.get("forward_m"), 0.0)
                right_m = self._coerce_float(args.get("right_m"), 0.0)
                up_m = self._coerce_float(args.get("up_m"), 0.0)
            components = (forward_m, right_m, up_m)
            if not all(math.isfinite(value) for value in components):
                raise ValueError("移动向量必须是有限数值")
            if not any(abs(value) >= 0.1 for value in components):
                raise ValueError("移动向量不能全部为 0")
            if any(abs(value) > 100.0 for value in components):
                raise ValueError("移动向量各分量不能超过 100 米")
            move_args = {
                "forward_m": forward_m,
                "right_m": right_m,
                "up_m": up_m,
                "takeoff_altitude": self._coerce_float(
                    args.get("takeoff_altitude"), 10.0
                ),
            }
            return self._parsed(
                "move_to",
                move_args,
                self._move_reason(move_args),
                risk_level="high",
                need_confirm=False,
                skill="PlanningSkill",
            )
        raise ValueError(f"不支持的基础动作: {action}")

    def _drone_state_callback(self, msg: DroneState):
        self._latest_state = {
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

    def _odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._latest_odom = {
            "frame_id": msg.header.frame_id,
            "position": {
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "z": float(msg.pose.pose.position.z),
            },
            "orientation": {
                "x": float(q.x),
                "y": float(q.y),
                "z": float(q.z),
                "w": float(q.w),
                "yaw": yaw,
            },
            "velocity": {
                "x": float(msg.twist.twist.linear.x),
                "y": float(msg.twist.twist.linear.y),
                "z": float(msg.twist.twist.linear.z),
            },
        }
        if self._home_odom is None:
            self._home_odom = json.loads(json.dumps(self._latest_odom))

    def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _image_callback(self, msg: Image):
        self._latest_image = {
            "topic": "/sensor/camera/rgb/front_center",
            "frame_id": msg.header.frame_id,
            "width": int(msg.width),
            "height": int(msg.height),
            "encoding": msg.encoding,
            "step": int(msg.step),
            "has_frame": bool(msg.data),
        }

    def _depth_callback(self, msg: Image):
        self._latest_depth = self._summarize_depth_image(msg)

    def _pointcloud_callback(self, msg: PointCloud2):
        self._latest_pointcloud = {
            "topic": "/sensor/lidar/points",
            "frame_id": msg.header.frame_id,
            "width": int(msg.width),
            "height": int(msg.height),
            "point_step": int(msg.point_step),
            "total_points": int(msg.width) * int(msg.height),
            "has_points": bool(msg.data) and int(msg.point_step) > 0,
        }

    def _detection_callback(self, msg: Detection):
        self._latest_detection = {
            "class_name": msg.class_name,
            "confidence": float(msg.confidence),
            "bbox": {
                "x": float(msg.x),
                "y": float(msg.y),
                "width": float(msg.width),
                "height": float(msg.height),
            },
        }

    def _detections_callback(self, msg: DetectionArray):
        detections = []
        for item in msg.detections:
            detections.append({
                "class_name": item.class_name,
                "confidence": float(item.confidence),
                "bbox": {
                    "x": float(item.x),
                    "y": float(item.y),
                    "width": float(item.width),
                    "height": float(item.height),
                },
            })
        self._latest_detections = detections
        if detections:
            self._latest_detection = max(
                detections,
                key=lambda item: item["confidence"],
            )

    def _image_analysis_callback(self, msg: String):
        try:
            self._latest_image_analysis = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("收到无法解析的 /perception/image_analysis")

    def _autonomy_status_callback(self, msg: AutonomyStatus):
        self._latest_autonomy_status = {
            "stamp": time.time(),
            "enabled": bool(msg.enabled),
            "state": msg.state,
            "replanning": bool(msg.replanning),
            "nearest_obstacle_m": float(msg.nearest_obstacle_m),
            "target_distance_m": float(msg.target_distance_m),
            "active_strategy": msg.active_strategy,
            "message": msg.message,
        }

    def _is_airborne(self):
        return self._current_altitude() >= 0.8 and bool(self._latest_state and self._latest_state.get("armed"))

    def _current_altitude(self):
        if self._latest_odom is None:
            return 0.0
        return float(self._latest_odom.get("position", {}).get("z", 0.0))

    def _ready_for_deferred_move(self, target_altitude: float):
        if self._latest_state is None:
            return False
        armed = bool(self._latest_state.get("armed"))
        altitude = self._current_altitude()
        required = max(0.8, float(target_altitude) * 0.75)
        return armed and altitude >= required

    def _deferred_move_timer(self):
        if not self._deferred_move:
            return
        if time.time() - self._deferred_move["created_at"] > 90.0:
            self.get_logger().warn("延迟移动任务超时，已取消")
            self._deferred_move = None
            return
        target_altitude = float(self._deferred_move.get("takeoff_altitude", 10.0))
        if not self._ready_for_deferred_move(target_altitude) or self._latest_odom is None:
            return
        item = self._deferred_move
        self._deferred_move = None
        result = self._publish_autonomy_goal(item["args"])
        self.get_logger().info(f"起飞完成，已自动发布自主避障目标: {json.dumps(result, ensure_ascii=False)}")

    def _wait_for_optional_service(self, client, name: str):
        if client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f"  {name} 就绪")
        else:
            self.get_logger().warn(f"  {name} 未就绪（超时）")

    def _requires_confirmation(self, parsed: dict):
        return parsed.get("intent") in {
            "arm",
            "disarm",
            "takeoff",
            "land",
            "move_to",
            "mission_sequence",
            "return_home",
            "emergency_stop",
        }

    def _request_confirmation(self, parsed: dict, response):
        progress = [
            self._progress_step("parse_task", "解析自然语言任务", "done"),
            self._progress_step("select_skill", f"选择 {parsed.get('skill', '--')}", "done"),
        ]
        safety = self._safety_check(parsed)
        progress.extend(safety["progress"])
        if not safety["ok"]:
            parsed["safety_check"] = safety
            response.success = False
            response.message = safety["message"]
            response.result = self._agent_result(
                parsed,
                reply=f"安全检查未通过：{safety['message']}",
                tool_calls=safety["tool_calls"],
                final_status="安全检查未通过，未生成执行确认",
                progress=progress,
            )
            return response

        token = f"{int(time.time() * 1000)}-{len(self._pending_tasks) + 1}"
        parsed["need_confirm"] = True
        parsed["safety_check"] = safety
        self._pending_tasks[token] = {
            "parsed": parsed,
            "created_at": time.time(),
        }
        pending = {
            "token": token,
            "intent": parsed.get("intent"),
            "skill": parsed.get("skill"),
            "args": parsed.get("args", {}),
            "summary": self._confirmation_summary(parsed),
        }
        response.success = True
        response.message = "任务已解析，等待人工确认后执行"
        response.result = self._agent_result(
            parsed,
            reply=f"我已解析任务：{pending['summary']}。请确认后再执行。",
            tool_calls=safety["tool_calls"],
            final_status="等待用户确认",
            pending_confirmation=pending,
            progress=[
                *progress,
                self._progress_step("wait_confirm", "等待用户确认", "active"),
                self._progress_step("execute", "调用 ROS 控制服务", "pending"),
                self._progress_step("verify", "验证执行结果", "pending"),
            ],
        )
        return response

    def _execute_pending_task(self, token: str, response):
        item = self._pending_tasks.pop(token, None)
        if item is None:
            parsed = self._parsed("unknown", {}, "确认令牌不存在或已过期", skill="UnknownSkill")
            response.success = False
            response.message = "确认令牌不存在或已过期"
            response.result = self._agent_result(
                parsed,
                reply=response.message,
                tool_calls=[],
                final_status="未执行",
            )
            return response

        parsed = item["parsed"]
        parsed["confirmed"] = True
        parsed["need_confirm"] = True
        self.get_logger().info(f"用户已确认执行: {json.dumps(parsed, ensure_ascii=False)}")
        return self._execute_parsed_task(parsed, response)

    def _cancel_pending_task(self, token: str, response):
        item = self._pending_tasks.pop(token, None)
        parsed = item["parsed"] if item else self._parsed(
            "unknown", {}, "任务已取消或不存在", skill="UnknownSkill"
        )
        response.success = True
        response.message = "已取消待执行任务"
        response.result = self._agent_result(
            parsed,
            reply="已取消该任务，没有调用任何 ROS 控制服务。",
            tool_calls=[],
            final_status="已取消",
        )
        return response

    def _confirmation_summary(self, parsed: dict):
        intent = parsed.get("intent")
        args = parsed.get("args", {})
        if intent == "arm":
            return "解锁无人机"
        if intent == "disarm":
            return "加锁无人机"
        if intent == "takeoff":
            return f"起飞到 {float(args.get('altitude', 10.0)):.1f} 米"
        if intent == "move_to":
            return self._move_reason(args)
        if intent == "mission_sequence":
            return self._sequence_reason(args)
        if intent == "land":
            return "降落"
        if intent == "return_home":
            return "触发 PX4 原生 RTL 返航"
        if intent == "emergency_stop":
            return "立即取消自主任务、发布零速度并尝试加锁"
        return parsed.get("reason", "执行任务")

    def _safety_check(self, parsed: dict):
        intent = parsed.get("intent")
        args = parsed.get("args", {})
        checks = []
        tool_calls = [
            self._tool_call(
                "get_drone_state",
                "/control/drone_state",
                self._latest_state is not None,
                result=self._latest_state,
            )
        ]

        def add(name, label, ok, detail):
            checks.append({"name": name, "label": label, "ok": bool(ok), "detail": detail})

        add("state_available", "飞控状态可用", self._latest_state is not None, "已收到 /control/drone_state" if self._latest_state else "尚未收到 /control/drone_state")

        if intent == "takeoff":
            altitude = float(args.get("altitude", 10.0))
            add("altitude_range", "目标高度范围", 1.0 <= altitude <= 50.0, f"{altitude:.1f} m，允许范围 1-50 m")
            add("takeoff_service", "起飞服务就绪", self._takeoff_client.service_is_ready(), "/control/takeoff")
            if self._latest_state is not None:
                add("ekf_healthy", "EKF 健康", bool(self._latest_state.get("ekf_healthy")), str(self._latest_state.get("ekf_healthy")))
                add(
                    "px4_preflight",
                    "PX4 解锁预检",
                    bool(self._latest_state.get("preflight_valid"))
                    and bool(self._latest_state.get("preflight_ok")),
                    (
                        "已通过"
                        if self._latest_state.get("preflight_ok")
                        else "未通过或状态不可用"
                    ),
                )
                add("gps_fix", "GPS/定位可用", int(self._latest_state.get("gps_fix", 0)) >= 2, f"gps_fix={self._latest_state.get('gps_fix')}")
        elif intent == "arm":
            add("arm_service", "解锁服务就绪", self._arm_client.service_is_ready(), "/control/arm")
            if self._latest_state is not None:
                add("ekf_healthy", "EKF 健康", bool(self._latest_state.get("ekf_healthy")), str(self._latest_state.get("ekf_healthy")))
                add(
                    "px4_preflight",
                    "PX4 解锁预检",
                    bool(self._latest_state.get("preflight_valid"))
                    and bool(self._latest_state.get("preflight_ok")),
                    (
                        "已通过"
                        if self._latest_state.get("preflight_ok")
                        else "未通过或状态不可用"
                    ),
                )
        elif intent == "disarm":
            add("arm_service", "加锁服务就绪", self._arm_client.service_is_ready(), "/control/arm")
        elif intent == "land":
            add("land_service", "降落服务就绪", self._land_client.service_is_ready(), "/control/land")
        elif intent == "move_to":
            add("odometry", "里程计可用", self._latest_odom is not None, "已收到 /sensor/odometry" if self._latest_odom else "尚未收到 /sensor/odometry")
            add(
                "airborne_or_takeoff_ready",
                "空中状态或起飞服务可用",
                self._is_airborne() or self._takeoff_client.service_is_ready(),
                "已在空中" if self._is_airborne() else "/control/takeoff",
            )
        elif intent == "mission_sequence":
            if args.get("takeoff"):
                altitude = self._coerce_float(args.get("takeoff_altitude"), 10.0)
                add("altitude_range", "目标高度范围", 1.0 <= altitude <= 50.0, f"{altitude:.1f} m，允许范围 1-50 m")
                add("takeoff_service", "起飞服务就绪", self._takeoff_client.service_is_ready(), "/control/takeoff")
            if args.get("move"):
                add("odometry", "里程计可用", self._latest_odom is not None, "已收到 /sensor/odometry" if self._latest_odom else "尚未收到 /sensor/odometry")
            if args.get("target"):
                add("camera", "RGB 相机可用", bool(self._latest_image and self._latest_image.get("has_frame")), "已有 RGB 画面" if self._latest_image else "尚未收到 RGB 画面")
        elif intent == "return_home":
            add("return_home_service", "返航服务就绪", self._return_home_client.service_is_ready(), "/control/return_home")
            add("airborne", "当前处于空中", self._is_airborne(), f"当前高度 {self._current_altitude():.2f} m")
        elif intent == "emergency_stop":
            add("arm_service", "加锁服务就绪", self._arm_client.service_is_ready(), "/control/arm")

        failed = [check for check in checks if not check["ok"]]
        message = "安全检查通过" if not failed else "；".join(
            f"{check['label']}失败：{check['detail']}" for check in failed
        )
        progress = [
            self._progress_step(
                f"safety_{check['name']}",
                f"{check['label']}: {check['detail']}",
                "done" if check["ok"] else "failed",
            )
            for check in checks
        ]
        return {
            "ok": not failed,
            "message": message,
            "checks": checks,
            "tool_calls": tool_calls,
            "progress": progress,
        }

    def _execute_parsed_task(self, parsed: dict, response):
        intent = parsed["intent"]
        args = parsed.get("args", {})

        if intent == "arm":
            return self._execute_arm(True, parsed, response)
        if intent == "disarm":
            return self._execute_arm(False, parsed, response)
        if intent == "takeoff":
            return self._execute_takeoff(args["altitude"], parsed, response)
        if intent == "land":
            return self._execute_land(parsed, response)
        if intent == "status":
            return self._execute_status(parsed, response)
        if intent == "hover":
            return self._execute_hover(parsed, response)
        if intent == "return_home":
            return self._execute_return_home(parsed, response)
        if intent == "emergency_stop":
            return self._execute_emergency_stop(parsed, response)
        if intent == "perception":
            return self._execute_perception(parsed, response)
        if intent == "capture_image":
            return self._execute_capture_image(parsed, response)
        if intent == "semantic_image":
            return self._execute_semantic_image(parsed, response)
        if intent == "scan_area":
            return self._execute_scan_area(parsed, response)
        if intent == "search_target":
            return self._execute_search_target(parsed, response)
        if intent == "mission_report":
            return self._execute_mission_report(parsed, response)
        if intent == "mission_sequence":
            return self._execute_mission_sequence(parsed, response)
        if intent == "move_to":
            return self._execute_move_to(parsed, response)

        response.success = False
        response.result = self._agent_result(
            parsed,
            reply=parsed["reason"],
            tool_calls=[],
            final_status="任务未执行",
        )
        response.message = parsed["reason"]
        return response

    def _parse_task(self, task: str):
        rule_first = self._parse_task_with_rules(task)
        deterministic_intents = {
            "search_target",
            "mission_sequence",
            "move_to",
            "arm",
            "disarm",
            "takeoff",
            "land",
            "hover",
            "return_home",
            "emergency_stop",
        }
        if rule_first.get("intent") in deterministic_intents:
            rule_first["parser"] = "rules"
            return rule_first

        if self._llm_enabled:
            llm_parsed = self._parse_task_with_llm(task)
            if llm_parsed is not None:
                return llm_parsed

        parsed = self._parse_task_with_rules(task)
        parsed["parser"] = "rules"
        return parsed

    def _parse_task_with_rules(self, task: str):
        text = task.strip().lower()
        compact = re.sub(r"\s+", "", text)
        wants_status_check = any(word in compact for word in ("状态", "情况", "电量", "模式", "查询", "检查", "安全"))
        sequence = self._extract_sequence_args(task)
        if sequence is not None:
            return self._parsed(
                "mission_sequence",
                sequence,
                self._sequence_reason(sequence),
                risk_level="high",
                need_confirm=True,
                skill="MissionSequenceSkill",
            )

        if any(word in compact for word in ("任务报告", "报告", "总结", "汇总", "missionreport")):
            return self._parsed(
                "mission_report",
                {},
                "生成当前任务报告",
                skill="MissionReportSkill",
            )

        if any(word in compact for word in ("急停", "紧急停止", "立刻停止", "立即停止", "emergencystop", "estop")):
            return self._parsed(
                "emergency_stop",
                {},
                "立即取消自主任务、发布零速度并尝试加锁",
                risk_level="high",
                need_confirm=True,
                skill="EmergencyStopSkill",
            )

        if any(word in compact for word in ("返航", "返回起点", "回家", "returnhome", "rtl")):
            return self._parsed(
                "return_home",
                {},
                "触发 PX4 原生 RTL 返航",
                risk_level="high",
                need_confirm=True,
                skill="ReturnHomeSkill",
            )

        if any(word in compact for word in ("拍照", "截图", "保存图像", "取证", "capture", "photo")):
            return self._parsed(
                "capture_image",
                {},
                "读取当前前视 RGB 相机帧并返回图像摘要",
                skill="CaptureImageSkill",
            )

        semantic_markers = (
            "分析画面",
            "分析图像",
            "分析图片",
            "描述画面",
            "描述图像",
            "描述图片",
            "看看画面",
            "看一下画面",
            "画面中有什么",
            "图像中有什么",
            "图片中有什么",
            "当前画面",
            "视觉语义",
            "语义分析",
            "scene",
            "vlm",
        )
        if any(word in compact for word in semantic_markers):
            return self._parsed(
                "semantic_image",
                {"prompt": task},
                "调用图像语义分析服务理解当前 RGB 画面",
                risk_level="low",
                skill="SemanticImageSkill",
            )

        if any(word in compact for word in ("扫描", "巡检", "scanarea", "区域扫描")):
            return self._parsed(
                "scan_area",
                {"mode": "perception_only"},
                "基于当前传感器数据执行一次区域扫描摘要",
                risk_level="medium",
                skill="ScanAreaSkill",
            )

        target_query_markers = ("搜索", "查找", "寻找", "目标", "searchtarget")
        detection_query_markers = ("检测", "识别", "发现", "是否有", "有没有", "有无")
        extracted_target = self._extract_search_target(task)
        if any(word in compact for word in target_query_markers) or (
            any(word in compact for word in detection_query_markers)
            and extracted_target not in ("未知目标", "障碍物", "障碍", "避障")
        ):
            target = extracted_target
            return self._parsed(
                "search_target",
                {"target": target},
                f"基于当前感知结果搜索目标：{target}",
                risk_level="medium",
                skill="TargetSearchSkill",
            )

        if any(word in compact for word in ("起飞", "升空", "takeoff")):
            altitude = self._extract_altitude(compact)
            if altitude is None:
                altitude = 10.0
            if altitude < 1.0 or altitude > 50.0:
                return self._parsed(
                    "unknown",
                    {"altitude": altitude},
                    "起飞高度必须在 1 到 50 米之间",
                    risk_level="high",
                    need_confirm=True,
                    skill="TakeoffSkill",
                )
            args = {"altitude": altitude}
            if wants_status_check:
                args["conditional_status_check"] = True
            reason = (
                f"先检查飞控状态，安全检查通过后起飞到 {altitude:.1f} 米"
                if wants_status_check
                else f"起飞到 {altitude:.1f} 米"
            )
            return self._parsed(
                "takeoff",
                args,
                reason,
                skill="TakeoffSkill",
            )

        if any(word in compact for word in ("前进", "向前", "后退", "向后", "左移", "右移", "向左", "向右", "左飞", "右飞", "向上", "向下", "飞到", "飞向", "航点", "飞行", "移动")):
            args = self._extract_move_args(compact)
            return self._parsed(
                "move_to",
                args,
                self._move_reason(args),
                risk_level="high",
                need_confirm=True,
                skill="PlanningSkill",
            )

        if any(word in compact for word in ("障碍物", "避障", "前方", "相机", "深度", "点云", "感知", "perception")):
            return self._parsed(
                "perception",
                {},
                "检查前方环境与障碍物风险",
                skill="PerceptionSkill",
            )

        if any(word in compact for word in ("悬停", "停止", "刹停", "hover", "停住")):
            return self._parsed(
                "hover",
                {},
                "发布零速度进入悬停保持",
                risk_level="medium",
                skill="HoverSkill",
            )

        if any(word in compact for word in ("状态", "情况", "电量", "模式", "查询", "检查")):
            return self._parsed("status", {}, "查询当前无人机状态", skill="StatusSkill")

        if any(word in compact for word in ("加锁", "上锁", "锁定", "disarm")):
            return self._parsed("disarm", {}, "执行加锁", skill="ArmSkill")

        if any(word in compact for word in ("解锁", "启动电机", "arm")):
            return self._parsed("arm", {}, "执行解锁", skill="ArmSkill")

        if any(word in compact for word in ("降落", "着陆", "落地", "land")):
            return self._parsed("land", {}, "执行降落", skill="LandSkill")

        return self._parsed(
            "unknown",
            {},
            "暂时无法理解该任务。当前支持：解锁、加锁、起飞到N米、降落、查询状态",
            skill="UnknownSkill",
        )

    def _parse_task_with_llm(self, task: str):
        try:
            content = self._call_llm(task)
            payload = self._extract_json_object(content)
            parsed = self._normalize_llm_payload(payload)
            parsed["parser"] = "llm"
            parsed["llm_model"] = self._llm_model
            return parsed
        except Exception as exc:
            self.get_logger().warn(f"LLM 解析失败，回退规则解析: {exc}")
            return None

    def _call_llm(self, task: str):
        url = self._llm_api_url.rstrip("/") + "/chat/completions"
        prompt = self._llm_prompt(task)
        body = json.dumps(
            {
                "model": self._llm_model,
                "temperature": 0.1,
                "messages": [
                    {
                        "role": "system",
                        "content": (
                            "你是 AeroMind 无人机 ROS2 Agent 的任务解析器。"
                            "你只能输出一个 JSON 对象，不要输出 Markdown。"
                        ),
                    },
                    {"role": "user", "content": prompt},
                ],
            },
            ensure_ascii=False,
        ).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if self._llm_api_key:
            headers["Authorization"] = f"Bearer {self._llm_api_key}"
        request = urllib.request.Request(
            url,
            data=body,
            headers=headers,
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self._llm_timeout_sec) as response:
            data = json.loads(response.read().decode("utf-8"))
        return data["choices"][0]["message"]["content"]

    def _llm_prompt(self, task: str):
        catalog = [
            {
                "name": skill["name"],
                "intent": skill["intent"],
                "enabled": skill["enabled"],
                "risk_level": skill["risk_level"],
                "description": skill["description"],
                "tools": skill["tools"],
            }
            for skill in get_skill_catalog()
        ]
        state = self._latest_state or {}
        schema = {
            "intent": "status|arm|disarm|takeoff|land|move_to|hover|return_home|emergency_stop|perception|capture_image|semantic_image|scan_area|search_target|mission_sequence|mission_report|unknown",
            "skill": "StatusSkill|ArmSkill|TakeoffSkill|LandSkill|PlanningSkill|HoverSkill|ReturnHomeSkill|EmergencyStopSkill|PerceptionSkill|CaptureImageSkill|SemanticImageSkill|ScanAreaSkill|TargetSearchSkill|MissionSequenceSkill|MissionReportSkill|UnknownSkill",
            "args": {
                "altitude": 10.0,
                "target": "汽车",
                "prompt": "分析当前画面中有什么",
                "forward_m": 10.0,
                "right_m": 0.0,
                "up_m": 0.0,
            },
            "risk_level": "low|medium|high",
            "need_confirm": False,
            "reason": "一句中文解释",
        }
        return (
            f"用户任务：{task}\n\n"
            f"当前无人机状态：{json.dumps(state, ensure_ascii=False)}\n\n"
            f"可用技能目录：{json.dumps(catalog, ensure_ascii=False)}\n\n"
            "请把用户任务解析成下面 JSON schema。"
            "不要直接编造 ROS topic，不要执行任务，只选择技能和参数。"
            "如果任务包含起飞高度，请提取 altitude，单位米。"
            "如果 intent=search_target，args.target 必须只填写要搜索的目标短词，例如 人/person/汽车，"
            "不要填写“用户要求...”这类解释句。"
            "如果用户说“搜索前方是否有汽车”，target 应为“汽车”。"
            "如果用户说“检测人”或“检测前方有人吗”，intent=search_target, skill=TargetSearchSkill, target 应为“人”。"
            "如果用户说“分析画面/描述图像/当前画面中有什么”，intent=semantic_image, skill=SemanticImageSkill, args.prompt 保留用户问题。"
            "如果一句话同时包含起飞/移动/检测等多个动作，intent=mission_sequence, skill=MissionSequenceSkill。"
            "如果任务需要未启用技能，也照样选择对应 skill，但 intent 保持对应类型。"
            "如果无法理解，intent=unknown, skill=UnknownSkill。\n"
            f"JSON schema 示例：{json.dumps(schema, ensure_ascii=False)}"
        )

    def _extract_json_object(self, content: str):
        text = content.strip()
        if text.startswith("```"):
            text = re.sub(r"^```(?:json)?", "", text).strip()
            text = re.sub(r"```$", "", text).strip()
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            start = text.find("{")
            end = text.rfind("}")
            if start < 0 or end <= start:
                raise ValueError("LLM 未返回 JSON 对象")
            return json.loads(text[start : end + 1])

    def _normalize_llm_payload(self, payload: dict):
        if not isinstance(payload, dict):
            raise ValueError("LLM JSON 不是对象")

        intent = str(payload.get("intent", "unknown")).strip() or "unknown"
        skill = str(payload.get("skill", "UnknownSkill")).strip() or "UnknownSkill"
        args = payload.get("args") if isinstance(payload.get("args"), dict) else {}
        risk_level = str(payload.get("risk_level", "low")).strip() or "low"
        reason = str(payload.get("reason", "LLM 已解析任务")).strip() or "LLM 已解析任务"
        need_confirm = bool(payload.get("need_confirm", risk_level == "high"))

        allowed_intents = {
            "status",
            "arm",
            "disarm",
            "takeoff",
            "land",
            "move_to",
            "hover",
            "return_home",
            "emergency_stop",
            "perception",
            "capture_image",
            "semantic_image",
            "scan_area",
            "search_target",
            "mission_sequence",
            "mission_report",
            "unknown",
        }
        if intent not in allowed_intents:
            intent = "unknown"

        if intent == "disarm":
            skill = "ArmSkill"
            need_confirm = True
        elif intent == "arm":
            skill = "ArmSkill"
            need_confirm = True
        elif intent == "status":
            skill = "StatusSkill"
        elif intent == "takeoff":
            skill = "TakeoffSkill"
            need_confirm = True
            altitude = self._coerce_altitude(args.get("altitude"))
            if altitude is None:
                altitude = self._extract_altitude(reason) or 10.0
            args["altitude"] = altitude
            if any(word in reason for word in ("检查", "安全", "状态")):
                args["conditional_status_check"] = True
            if altitude < 1.0 or altitude > 50.0:
                return self._parsed(
                    "unknown",
                    {"altitude": altitude},
                    "起飞高度必须在 1 到 50 米之间",
                    risk_level="high",
                    need_confirm=True,
                    skill="TakeoffSkill",
                )
        elif intent == "land":
            skill = "LandSkill"
            need_confirm = True
        elif intent == "hover":
            skill = "HoverSkill"
            risk_level = "medium"
        elif intent == "perception":
            skill = "PerceptionSkill"
            risk_level = "low"
        elif intent == "mission_report":
            skill = "MissionReportSkill"
            risk_level = "low"
        elif intent == "capture_image":
            skill = "CaptureImageSkill"
            risk_level = "low"
        elif intent == "semantic_image":
            skill = "SemanticImageSkill"
            risk_level = "low"
            args["prompt"] = str(args.get("prompt") or reason or "分析当前画面中有什么")
        elif intent == "scan_area":
            skill = "ScanAreaSkill"
            risk_level = "medium"
            args.setdefault("mode", "perception_only")
        elif intent == "search_target":
            skill = "TargetSearchSkill"
            risk_level = "medium"
            args["target"] = self._clean_search_target(args.get("target"), reason)
        elif intent == "mission_sequence":
            skill = "MissionSequenceSkill"
            risk_level = "high"
            need_confirm = True
            args = self._normalize_sequence_args(args, reason)
        elif intent == "return_home":
            skill = "ReturnHomeSkill"
            risk_level = "high"
            need_confirm = True
        elif intent == "emergency_stop":
            skill = "EmergencyStopSkill"
            risk_level = "high"
            need_confirm = True
        elif intent == "move_to":
            skill = "PlanningSkill"
            risk_level = "high"
            need_confirm = True
            args = self._normalize_move_args(args, reason)

        if skill_by_name(skill) is None:
            skill = "UnknownSkill"

        if risk_level not in ("low", "medium", "high"):
            risk_level = "medium"

        return self._parsed(
            intent,
            args,
            reason,
            risk_level=risk_level,
            need_confirm=need_confirm,
            skill=skill,
        )

    def _coerce_altitude(self, value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _normalize_move_args(self, args: dict, reason: str):
        normalized = {
            "forward_m": self._coerce_float(args.get("forward_m"), 0.0),
            "right_m": self._coerce_float(args.get("right_m"), 0.0),
            "up_m": self._coerce_float(args.get("up_m"), 0.0),
            "takeoff_altitude": self._coerce_float(args.get("takeoff_altitude"), 10.0),
        }
        if not any(abs(normalized[key]) > 1e-6 for key in ("forward_m", "right_m", "up_m")):
            normalized.update(self._extract_move_args(reason))
        return normalized

    def _normalize_sequence_args(self, args: dict, reason: str):
        text = json.dumps(args, ensure_ascii=False) + " " + str(reason)
        sequence = self._extract_sequence_args(text) or {}
        if "takeoff" in args:
            sequence["takeoff"] = bool(args.get("takeoff"))
        if "altitude" in args:
            sequence["takeoff_altitude"] = self._coerce_float(args.get("altitude"), sequence.get("takeoff_altitude", 10.0))
        if "takeoff_altitude" in args:
            sequence["takeoff_altitude"] = self._coerce_float(args.get("takeoff_altitude"), sequence.get("takeoff_altitude", 10.0))
        if any(key in args for key in ("forward_m", "right_m", "up_m")):
            sequence["move"] = {
                "forward_m": self._coerce_float(args.get("forward_m"), 0.0),
                "right_m": self._coerce_float(args.get("right_m"), 0.0),
                "up_m": self._coerce_float(args.get("up_m"), 0.0),
                "takeoff_altitude": self._coerce_float(args.get("takeoff_altitude"), sequence.get("takeoff_altitude", 10.0)),
            }
        if args.get("target"):
            sequence["target"] = self._clean_search_target(args.get("target"), reason)
        return self._complete_sequence_args(sequence)

    def _coerce_float(self, value, default=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _extract_move_args(self, text: str):
        distance = self._extract_altitude(text)
        if distance is None:
            distance = 10.0
        distance = max(0.1, min(float(distance), 100.0))
        args = {"forward_m": 0.0, "right_m": 0.0, "up_m": 0.0, "takeoff_altitude": 10.0}
        if any(word in text for word in ("后退", "向后")):
            args["forward_m"] = -distance
        elif any(word in text for word in ("右移", "向右", "右侧")):
            args["right_m"] = distance
        elif any(word in text for word in ("左移", "向左", "左侧")):
            args["right_m"] = -distance
        elif any(word in text for word in ("上升", "向上", "升高")):
            args["up_m"] = distance
        elif any(word in text for word in ("下降", "向下", "降低")):
            args["up_m"] = -distance
        else:
            args["forward_m"] = distance
        return args

    def _extract_sequence_args(self, text: str):
        compact = re.sub(r"\s+", "", str(text).lower())
        has_takeoff = any(word in compact for word in ("起飞", "升空", "takeoff"))
        # “起飞到10米”中的“飞到”描述的是起飞高度，不是平移目标。
        # 去掉完整起飞高度片段后，再提取后续移动方向和距离。
        number = r"(?:-?\d+(?:\.\d+)?|[零一二两三四五六七八九十]+)"
        movement_text = re.sub(
            rf"(?:起飞|升空)(?:到|至|高度)?{number}(?:米|m)?",
            "",
            compact,
        )
        movement_text = re.sub(
            rf"takeoff(?:to)?{number}(?:m)?",
            "",
            movement_text,
        )
        movement_text = re.sub(r"(?:起飞|升空|takeoff)", "", movement_text)
        has_move = any(word in movement_text for word in (
            "前进", "向前", "后退", "向后", "左移", "右移", "向左", "向右", "飞到", "飞向", "移动",
        ))
        has_target = any(word in compact for word in ("检测", "识别", "搜索", "查找", "寻找", "是否有", "有没有", "目标", "person", "car"))
        action_count = sum(1 for value in (has_takeoff, has_move, has_target) if value)
        if action_count < 2:
            return None

        sequence = {
            "takeoff": has_takeoff,
            "takeoff_altitude": self._extract_altitude(compact) if has_takeoff else 10.0,
            "move": self._extract_move_args(movement_text) if has_move else None,
            "target": self._extract_search_target(text) if has_target else None,
        }
        return self._complete_sequence_args(sequence)

    def _complete_sequence_args(self, sequence: dict):
        takeoff_altitude = self._coerce_float(sequence.get("takeoff_altitude"), 10.0)
        if takeoff_altitude < 1.0 or takeoff_altitude > 50.0:
            takeoff_altitude = 10.0
        sequence["takeoff_altitude"] = takeoff_altitude
        if sequence.get("move"):
            move = self._normalize_move_args(sequence["move"], "")
            move["takeoff_altitude"] = takeoff_altitude
            sequence["move"] = move
        if sequence.get("target"):
            sequence["target"] = self._clean_search_target(sequence.get("target"), "")
        steps = []
        if sequence.get("takeoff"):
            steps.append({"intent": "takeoff", "label": f"起飞到 {takeoff_altitude:.1f} 米"})
        if sequence.get("move"):
            steps.append({"intent": "move_to", "label": self._move_reason(sequence["move"])})
        if sequence.get("target"):
            steps.append({"intent": "search_target", "label": f"检测目标：{sequence['target']}"})
        sequence["steps"] = steps
        return sequence

    def _sequence_reason(self, args: dict):
        steps = args.get("steps") or []
        if steps:
            return "复合任务：" + " → ".join(step.get("label", "--") for step in steps)
        return "复合任务：拆解并按顺序执行"

    def _clean_search_target(self, value, fallback: str = ""):
        text = str(value or "").strip()
        compact = re.sub(r"\s+", "", text)
        explanation_markers = (
            "用户",
            "任务",
            "对应",
            "意图",
            "无需",
            "要求",
            "属于",
            "使用",
            "进行",
            "匹配",
            "检测",
            "报告",
            "是否有",
            "有没有",
            "前方",
        )
        if not compact or len(compact) > 20 or any(marker in compact for marker in explanation_markers):
            return self._extract_search_target(f"{text} {fallback}")
        return compact

    def _extract_search_target(self, text: str):
        compact = re.sub(r"\s+", "", str(text))
        lower_compact = compact.lower()
        english_targets = ("person", "car", "truck", "bus", "bicycle", "motorcycle", "chair", "bottle")
        for target in english_targets:
            if target in lower_compact:
                return target
        known_targets = (
            "红色汽车",
            "蓝色汽车",
            "红色目标",
            "蓝色目标",
            "公交车",
            "摩托车",
            "自行车",
            "汽车",
            "车辆",
            "小车",
            "卡车",
            "行人",
            "人员",
            "瓶子",
            "椅子",
            "箱子",
            "背包",
            "飞机",
            "人",
            "车",
        )
        for target in known_targets:
            if target in compact:
                return target
        for color in ("红色", "蓝色", "绿色", "黄色", "白色", "黑色"):
            if color in compact:
                return f"{color}目标"

        patterns = (
            r"(?:是否有|有没有|有无)([^，。,.！？?]+)",
            r"(?:搜索|查找|寻找)(?:前方|附近|当前画面中|画面里)?(?:是否有|有没有)?([^，。,.！？?]+)",
            r"(?:目标为|目标是|目标)([^，。,.！？?]+)",
        )
        for pattern in patterns:
            match = re.search(pattern, compact, flags=re.IGNORECASE)
            if not match:
                continue
            target = match.group(1)
            target = re.sub(
                r"(并报告位置|报告位置|并报告|的位置|对应.*|属于.*|使用.*|进行.*|无需.*)$",
                "",
                target,
            )
            target = re.sub(r"(用户要求|用户任务为|任务为|前方|当前|画面中|画面里)", "", target)
            target = target.strip()
            if target:
                return target[:20]

        cleaned = re.sub(
            r"(搜索|查找|寻找|检测|识别|发现|目标|请|帮我|一下|searchtarget|报告位置|并报告|前方|是否有|有没有)",
            "",
            compact,
            flags=re.IGNORECASE,
        )
        return cleaned[:20] if cleaned else "未知目标"

    def _move_reason(self, args: dict):
        parts = []
        if abs(args.get("forward_m", 0.0)) > 1e-6:
            direction = "前方" if args["forward_m"] > 0 else "后方"
            parts.append(f"{direction} {abs(args['forward_m']):.1f} 米")
        if abs(args.get("right_m", 0.0)) > 1e-6:
            direction = "右侧" if args["right_m"] > 0 else "左侧"
            parts.append(f"{direction} {abs(args['right_m']):.1f} 米")
        if abs(args.get("up_m", 0.0)) > 1e-6:
            direction = "上方" if args["up_m"] > 0 else "下方"
            parts.append(f"{direction} {abs(args['up_m']):.1f} 米")
        return "发布自主避障目标：" + ("，".join(parts) if parts else "当前位置")

    def _extract_altitude(self, text: str):
        patterns = (
            r"(?:到|至|高度|飞到)?(-?\d+(?:\.\d+)?)(?:米|m)",
            r"(-?\d+(?:\.\d+)?)",
        )
        for pattern in patterns:
            match = re.search(pattern, text)
            if match:
                return float(match.group(1))

        match = re.search(r"([零一二两三四五六七八九十]+)(?:米|m)?", text)
        if match:
            return self._parse_chinese_number(match.group(1))
        return None

    def _parse_chinese_number(self, text: str):
        digits = {
            "零": 0,
            "一": 1,
            "二": 2,
            "两": 2,
            "三": 3,
            "四": 4,
            "五": 5,
            "六": 6,
            "七": 7,
            "八": 8,
            "九": 9,
        }
        if not text:
            return None
        if text == "十":
            return 10.0
        if "十" in text:
            left, _, right = text.partition("十")
            tens = digits.get(left, 1) if left else 1
            ones = digits.get(right, 0) if right else 0
            return float(tens * 10 + ones)
        total = 0
        for char in text:
            if char not in digits:
                return None
            total = total * 10 + digits[char]
        return float(total)

    def _parsed(
        self,
        intent: str,
        args: dict,
        reason: str,
        risk_level: str = "low",
        need_confirm: bool = False,
        skill: str = "UnknownSkill",
    ):
        return {
            "intent": intent,
            "skill": skill,
            "skill_info": skill_by_name(skill),
            "args": args,
            "risk_level": risk_level,
            "need_confirm": need_confirm,
            "reason": reason,
        }

    def _execute_arm(self, arm: bool, parsed: dict, response):
        req = ArmDrone.Request()
        req.arm = arm
        tool_calls = [
            self._tool_call(
                "get_drone_state",
                "读取飞控状态",
                True,
                result=self._latest_state,
            )
        ]
        result = self._call_service(self._arm_client, req, "/control/arm")
        tool_calls.append(
            self._tool_call(
                "arm_drone" if arm else "disarm_drone",
                "/control/arm",
                result["success"],
                args={"arm": arm},
                result=result,
            )
        )
        return self._fill_response(response, parsed, result, "解锁" if arm else "加锁", tool_calls)

    def _execute_takeoff(self, altitude: float, parsed: dict, response):
        req = Takeoff.Request()
        req.altitude = float(altitude)
        tool_calls = [
            self._tool_call(
                "get_drone_state",
                "读取飞控状态",
                self._latest_state is not None,
                result=self._latest_state,
            )
        ]
        result = self._call_service(
            self._takeoff_client,
            req,
            "/control/takeoff",
            wait_result=False,
        )
        tool_calls.append(
            self._tool_call(
                "takeoff",
                "/control/takeoff",
                result["success"],
                args={"altitude": altitude},
                result=result,
            )
        )
        return self._fill_response(response, parsed, result, f"起飞到 {altitude:.1f} 米", tool_calls)

    def _execute_land(self, parsed: dict, response):
        req = Land.Request()
        tool_calls = [
            self._tool_call(
                "get_drone_state",
                "读取飞控状态",
                self._latest_state is not None,
                result=self._latest_state,
            )
        ]
        result = self._call_service(
            self._land_client,
            req,
            "/control/land",
            wait_result=False,
        )
        tool_calls.append(
            self._tool_call(
                "land",
                "/control/land",
                result["success"],
                result=result,
            )
        )
        return self._fill_response(response, parsed, result, "降落", tool_calls)

    def _execute_status(self, parsed: dict, response):
        response.success = True
        tool_calls = [
            self._tool_call(
                "get_drone_state",
                "/control/drone_state",
                self._latest_state is not None,
                result=self._latest_state,
            )
        ]
        if self._latest_state is None:
            parsed["state"] = None
            response.result = self._agent_result(
                parsed,
                reply="尚未收到飞控状态。",
                tool_calls=tool_calls,
                final_status="状态不可用",
                progress=[
                    self._progress_step("parse_task", "解析自然语言任务", "done"),
                    self._progress_step("get_drone_state", "读取飞控状态", "failed"),
                ],
            )
            response.message = "尚未收到 /control/drone_state，请确认 bridge/control 已启动"
            return response

        parsed["state"] = self._latest_state
        response.message = (
            f"当前状态：armed={self._latest_state['armed']}, "
            f"mode={self._latest_state['mode']}, "
            f"battery={self._latest_state['battery']:.1f}V, "
            f"gps_fix={self._latest_state['gps_fix']}, "
            f"ekf_healthy={self._latest_state['ekf_healthy']}, "
            f"preflight_ok={self._latest_state['preflight_ok']}"
        )
        response.result = self._agent_result(
            parsed,
            reply="我已读取当前无人机飞控状态。",
            tool_calls=tool_calls,
            final_status=response.message,
            progress=[
                self._progress_step("parse_task", "解析自然语言任务", "done"),
                self._progress_step("get_drone_state", "读取飞控状态", "done"),
                self._progress_step("reply", "生成状态回复", "done"),
            ],
        )
        return response

    def _execute_hover(self, parsed: dict, response):
        msg = Twist()
        self._autonomy_cancel_pub.publish(Empty())
        self._cmd_vel_pub.publish(msg)
        result = {
            "success": True,
            "message": "已取消自主目标并向 /control/cmd_vel 发布零速度指令",
        }
        parsed["service_result"] = result
        response.success = True
        response.message = "已进入悬停保持"
        response.result = self._agent_result(
            parsed,
            reply="我已发布零速度控制指令，让无人机停止平移/转向并保持当前状态。",
            tool_calls=[
                self._tool_call(
                    "get_drone_state",
                    "/control/drone_state",
                    self._latest_state is not None,
                    result=self._latest_state,
                ),
                self._tool_call(
                    "cancel_autonomy_goal",
                    "/autonomy/cancel",
                    True,
                    result={"success": True, "message": "自主目标已清空"},
                ),
                self._tool_call(
                    "publish_cmd_vel_zero",
                    "/control/cmd_vel",
                    True,
                    args={
                        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "angular": {"z": 0.0},
                    },
                    result=result,
                ),
            ],
            final_status=response.message,
            progress=[
                self._progress_step("parse_task", "解析自然语言任务", "done"),
                self._progress_step("select_skill", "选择 HoverSkill", "done"),
                self._progress_step("cancel_autonomy", "取消 /autonomy/goal 当前任务", "done"),
                self._progress_step("publish_cmd_vel", "发布零速度到 /control/cmd_vel", "done"),
                self._progress_step("reply", "显示悬停状态", "done"),
            ],
        )
        return response

    def _execute_move_to(self, parsed: dict, response):
        args = parsed.get("args", {})
        tool_calls = [
            self._tool_call(
                "get_odometry",
                "/sensor/odometry",
                self._latest_odom is not None,
                result=self._latest_odom,
            )
        ]
        if self._latest_odom is None:
            response.success = False
            response.message = "无法发布自主目标：尚未收到 /sensor/odometry"
            response.result = self._agent_result(
                parsed,
                reply=response.message,
                tool_calls=tool_calls,
                final_status="移动任务未执行",
                progress=[
                    self._progress_step("parse_task", "解析自然语言任务", "done"),
                    self._progress_step("select_skill", "选择 PlanningSkill", "done"),
                    self._progress_step("get_odometry", "读取当前位置", "failed"),
                ],
            )
            return response

        if not self._is_airborne():
            takeoff_altitude = max(1.0, self._coerce_float(args.get("takeoff_altitude"), 10.0))
            req = Takeoff.Request()
            req.altitude = takeoff_altitude
            takeoff_result = self._call_service(
                self._takeoff_client,
                req,
                "/control/takeoff",
                wait_result=False,
            )
            self._deferred_move = {
                "args": args,
                "created_at": time.time(),
                "takeoff_altitude": takeoff_altitude,
            }
            parsed["service_result"] = takeoff_result
            response.success = bool(takeoff_result["success"])
            response.message = (
                f"已先下发起飞到 {takeoff_altitude:.1f} 米；起飞完成后将自动发布自主避障移动目标"
                if response.success
                else f"起飞请求失败：{takeoff_result['message']}"
            )
            tool_calls.append(
                self._tool_call(
                    "takeoff_before_move",
                    "/control/takeoff",
                    takeoff_result["success"],
                    args={"altitude": takeoff_altitude},
                    result=takeoff_result,
                )
            )
            response.result = self._agent_result(
                parsed,
                reply=(
                    "当前还未确认处于空中状态。我会先执行起飞，"
                    "飞控进入空中/Offboard 状态后自动发布 /autonomy/goal。"
                ),
                tool_calls=tool_calls,
                final_status=response.message,
                progress=[
                    self._progress_step("parse_task", "解析自然语言任务", "done"),
                    self._progress_step("select_skill", "选择 PlanningSkill", "done"),
                    self._progress_step("get_odometry", "读取当前位置", "done"),
                    self._progress_step("takeoff", f"先起飞到 {takeoff_altitude:.1f} 米", "done" if response.success else "failed"),
                    self._progress_step("publish_goal", "起飞完成后发布 /autonomy/goal", "active" if response.success else "pending"),
                    self._progress_step("execute_trajectory", "等待底层自主避障生成并执行轨迹", "pending"),
                ],
            )
            return response

        result = self._publish_autonomy_goal(args)
        parsed["service_result"] = result
        response.success = True
        goal = result["goal"]
        response.message = f"已发布自主避障目标点: ({goal['x']:.2f}, {goal['y']:.2f}, {goal['z']:.2f})"
        tool_calls.append(
            self._tool_call(
                "publish_autonomy_goal",
                "/autonomy/goal",
                True,
                args={
                    "forward_m": self._coerce_float(args.get("forward_m"), 0.0),
                    "right_m": self._coerce_float(args.get("right_m"), 0.0),
                    "up_m": self._coerce_float(args.get("up_m"), 0.0),
                },
                result=result,
            )
        )
        response.result = self._agent_result(
            parsed,
            reply=(
                "我已把移动任务转换为自主避障目标点。"
                "接下来 autonomy_node 会实时重规划轨迹，control_node 会执行 /autonomy/trajectory。"
            ),
            tool_calls=tool_calls,
            final_status=response.message,
            progress=[
                self._progress_step("parse_task", "解析自然语言任务", "done"),
                self._progress_step("select_skill", "选择 PlanningSkill", "done"),
                self._progress_step("get_odometry", "读取当前位置", "done"),
                self._progress_step("publish_goal", "发布 /autonomy/goal", "done"),
                self._progress_step("execute_trajectory", "等待底层自主避障生成并执行轨迹", "active"),
            ],
        )
        return response

    def _publish_autonomy_goal(self, args: dict):
        pos = self._latest_odom["position"]
        yaw = float(self._latest_odom.get("orientation", {}).get("yaw", 0.0))
        forward_m = self._coerce_float(args.get("forward_m"), 0.0)
        right_m = self._coerce_float(args.get("right_m"), 0.0)
        up_m = self._coerce_float(args.get("up_m"), 0.0)
        # Match the user-facing Web/teleop convention:
        # forward uses +Y when yaw=0, right uses +X when yaw=0.
        # This keeps natural-language movement aligned with the camera/ground-station view.
        forward_x = -math.sin(yaw)
        forward_y = math.cos(yaw)
        right_x = math.cos(yaw)
        right_y = math.sin(yaw)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._latest_odom.get("frame_id") or "map"
        goal.pose.position.x = float(pos["x"]) + forward_x * forward_m + right_x * right_m
        goal.pose.position.y = float(pos["y"]) + forward_y * forward_m + right_y * right_m
        goal.pose.position.z = float(pos["z"]) + up_m
        goal.pose.orientation.w = 1.0
        self._autonomy_goal_pub.publish(goal)

        return {
            "success": True,
            "message": "已发布 /autonomy/goal，底层自主避障将生成 /autonomy/trajectory 并由 control_node 执行",
            "goal": {
                "frame_id": goal.header.frame_id,
                "x": goal.pose.position.x,
                "y": goal.pose.position.y,
                "z": goal.pose.position.z,
            },
        }

    def _publish_absolute_autonomy_goal(self, x: float, y: float, z: float, frame_id: str = "map"):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = frame_id or "map"
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = float(z)
        goal.pose.orientation.w = 1.0
        self._autonomy_goal_pub.publish(goal)
        return {
            "success": True,
            "message": "已发布绝对 /autonomy/goal",
            "goal": {
                "frame_id": goal.header.frame_id,
                "x": goal.pose.position.x,
                "y": goal.pose.position.y,
                "z": goal.pose.position.z,
            },
        }

    def _execute_return_home(self, parsed: dict, response):
        tool_calls = [
            self._tool_call("get_drone_state", "/control/drone_state", self._latest_state is not None, result=self._latest_state),
        ]
        req = ReturnHome.Request()
        result = self._call_service(
            self._return_home_client,
            req,
            "/control/return_home",
            wait_result=False,
        )
        tool_calls.append(
            self._tool_call(
                "return_home",
                "/control/return_home",
                result["success"],
                result=result,
            )
        )
        parsed["service_result"] = result
        response.success = bool(result["success"])
        response.message = (
            f"PX4 原生 RTL 返航请求已下发：{result['message']}"
            if response.success
            else f"PX4 原生 RTL 返航请求失败：{result['message']}"
        )
        response.result = self._agent_result(
            parsed,
            reply=response.message,
            tool_calls=tool_calls,
            final_status=response.message,
            progress=[
                self._progress_step("parse_task", "解析自然语言任务", "done"),
                self._progress_step("select_skill", "选择 ReturnHomeSkill", "done"),
                self._progress_step("call_rtl", "调用 /control/return_home 触发 PX4 原生 RTL", "done" if result["success"] else "failed"),
                self._progress_step("verify", "等待飞控进入 AUTO_RTL/返航状态", "active" if result["success"] else "failed"),
            ],
        )
        return response

    def _execute_emergency_stop(self, parsed: dict, response):
        self._autonomy_cancel_pub.publish(Empty())
        self._cmd_vel_pub.publish(Twist())
        req = ArmDrone.Request()
        req.arm = False
        disarm_result = self._call_service(self._arm_client, req, "/control/arm", wait_result=False)
        result = {
            "success": bool(disarm_result.get("success", False)),
            "accepted": bool(disarm_result.get("accepted", False)),
            "message": "已取消自主任务、发布零速度，并尝试加锁",
            "disarm": disarm_result,
        }
        parsed["service_result"] = result
        response.success = True
        response.message = result["message"]
        response.result = self._agent_result(
            parsed,
            reply=response.message,
            tool_calls=[
                self._tool_call("cancel_autonomy_goal", "/autonomy/cancel", True),
                self._tool_call("publish_cmd_vel_zero", "/control/cmd_vel", True),
                self._tool_call("disarm_drone", "/control/arm", disarm_result["success"], args={"arm": False}, result=disarm_result),
            ],
            final_status=response.message,
            progress=[
                self._progress_step("parse_task", "解析自然语言任务", "done"),
                self._progress_step("select_skill", "选择 EmergencyStopSkill", "done"),
                self._progress_step("cancel_autonomy", "取消当前自主目标", "done"),
                self._progress_step("publish_cmd_vel", "发布零速度", "done"),
                self._progress_step("disarm", "尝试加锁", "active" if disarm_result.get("accepted") else ("done" if disarm_result["success"] else "failed")),
            ],
        )
        return response

    def _execute_perception(self, parsed: dict, response):
        analysis = self._build_perception_analysis()
        parsed["perception"] = analysis
        response.success = True
        response.message = analysis["summary"]
        response.result = self._agent_result(
            parsed,
            reply=analysis["reply"],
            tool_calls=[
                self._tool_call(
                    "get_camera_summary",
                    "/sensor/camera/rgb/front_center",
                    self._latest_image is not None,
                    result=self._latest_image,
                ),
                self._tool_call(
                    "get_depth_summary",
                    "/sensor/camera/depth/front_center",
                    self._latest_depth is not None,
                    result=self._latest_depth,
                ),
                self._tool_call(
                    "get_lidar_summary",
                    "/sensor/lidar/points",
                    self._latest_pointcloud is not None,
                    result=self._latest_pointcloud,
                ),
                self._tool_call(
                    "get_detections",
                    "/perception/detections",
                    bool(self._latest_detections),
                    result=self._latest_detections,
                ),
            ],
            final_status=response.message,
            progress=[
                self._progress_step("parse_task", "解析自然语言任务", "done"),
                self._progress_step("select_skill", "选择 PerceptionSkill", "done"),
                self._progress_step("read_camera", "读取 RGB 相机摘要", "done" if self._latest_image else "failed"),
                self._progress_step("read_depth", "读取深度相机摘要", "done" if self._latest_depth else "failed"),
                self._progress_step("read_pointcloud", "读取点云摘要", "done" if self._latest_pointcloud else "failed"),
                self._progress_step("analyze", analysis["summary"], "done"),
            ],
        )
        return response

    def _execute_capture_image(self, parsed: dict, response):
        image = self._latest_image
        req = CaptureImage.Request()
        req.label = "front_rgb"
        result = self._call_service(
            self._capture_image_client,
            req,
            "/perception/capture_image",
            timeout_sec=30.0,
        )
        has_frame = bool(image and image.get("has_frame"))
        capture = {
            "has_frame": has_frame or result.get("success", False),
            "image": image,
            "summary": (
                f"图像已保存：{result.get('file_path')}"
                if result.get("success")
                else result.get("message", "当前没有可用 RGB 图像帧")
            ),
            "saved_file": result.get("file_path"),
            "width": result.get("width"),
            "height": result.get("height"),
            "encoding": result.get("encoding"),
            "timestamp": result.get("timestamp"),
        }
        parsed["capture"] = capture
        parsed["service_result"] = result
        response.success = bool(result.get("success", False))
        response.message = capture["summary"]
        response.result = self._agent_result(
            parsed,
            reply=capture["summary"],
            tool_calls=[
                self._tool_call(
                    "capture_image",
                    "/perception/capture_image",
                    response.success,
                    args={"label": req.label},
                    result=result,
                )
            ],
            final_status=response.message,
            progress=[
                self._progress_step("parse_task", "解析自然语言任务", "done"),
                self._progress_step("select_skill", "选择 CaptureImageSkill", "done"),
                self._progress_step("capture_image", "调用 /perception/capture_image 保存图片", "done" if response.success else "failed"),
            ],
        )
        return response

    def _execute_semantic_image(self, parsed: dict, response):
        prompt = str(parsed.get("args", {}).get("prompt") or "分析当前画面中有什么")
        req = AnalyzeImage.Request()
        req.prompt = prompt
        req.use_vlm = True
        result = self._call_service(
            self._analyze_image_client,
            req,
            "/perception/analyze_image",
            timeout_sec=45.0,
        )
        objects = []
        if result.get("objects_json"):
            try:
                objects = json.loads(result["objects_json"])
            except json.JSONDecodeError:
                objects = []
        analysis = {
            "prompt": prompt,
            "success": bool(result.get("success")),
            "scene": result.get("scene", "--"),
            "risk_level": result.get("risk_level", "unknown"),
            "suggestion": result.get("suggestion", "--"),
            "objects": objects,
            "image": {
                "width": result.get("width"),
                "height": result.get("height"),
                "encoding": result.get("encoding"),
            },
            "raw_response": result.get("raw_response", ""),
        }
        parsed["semantic_image"] = analysis
        parsed["service_result"] = result
        response.success = bool(result.get("success"))
        response.message = result.get("message", "图像语义分析完成" if response.success else "图像语义分析失败")
        reply = "\n".join([
            "图像语义分析结果：",
            f"- 场景：{analysis['scene']}",
            f"- 风险等级：{analysis['risk_level']}",
            f"- 检测/语义目标：{self._format_semantic_objects(objects)}",
            f"- 建议：{analysis['suggestion']}",
        ])
        response.result = self._agent_result(
            parsed,
            reply=reply,
            tool_calls=[
                self._tool_call("analyze_image", "/perception/analyze_image", response.success, args={"prompt": prompt, "use_vlm": True}, result=result),
                self._tool_call("get_camera_summary", "/sensor/camera/rgb/front_center", self._latest_image is not None, result=self._latest_image),
                self._tool_call("get_detections", "/perception/detections", bool(self._valid_detections()), result=self._valid_detections()),
            ],
            final_status=response.message,
            progress=[
                self._progress_step("parse_task", "解析自然语言任务", "done"),
                self._progress_step("select_skill", "选择 SemanticImageSkill", "done"),
                self._progress_step("analyze_image", "调用 /perception/analyze_image 分析当前画面", "done" if response.success else "failed"),
                self._progress_step("summarize", analysis["suggestion"], "done" if response.success else "failed"),
            ],
        )
        return response

    def _execute_scan_area(self, parsed: dict, response):
        scan = self._build_scan_area_summary()
        detections = self._valid_detections()
        parsed["scan"] = scan
        response.success = True
        response.message = scan["summary"]
        response.result = self._agent_result(
            parsed,
            reply=scan["reply"],
            tool_calls=[
                self._tool_call("get_depth_summary", "/sensor/camera/depth/front_center", self._latest_depth is not None, result=self._latest_depth),
                self._tool_call("get_lidar_summary", "/sensor/lidar/points", self._latest_pointcloud is not None, result=self._latest_pointcloud),
                self._tool_call("get_detections", "/perception/detections", bool(detections), result=detections),
            ],
            final_status=response.message,
            progress=[
                self._progress_step("parse_task", "解析自然语言任务", "done"),
                self._progress_step("select_skill", "选择 ScanAreaSkill", "done"),
                self._progress_step("collect_sensors", "读取深度、点云和检测摘要", "done"),
                self._progress_step("analyze_area", scan["summary"], "done"),
            ],
        )
        return response

    def _execute_search_target(self, parsed: dict, response):
        target = str(parsed.get("args", {}).get("target", "未知目标"))
        search = self._build_target_search_summary(target)
        detections = search.get("detections", [])
        parsed["target_search"] = search
        response.success = True
        response.message = search["summary"]
        response.result = self._agent_result(
            parsed,
            reply=search["reply"],
            tool_calls=[
                self._tool_call("get_detections", "/perception/detections", bool(detections), result=detections),
                self._tool_call("get_camera_summary", "/sensor/camera/rgb/front_center", self._latest_image is not None, result=self._latest_image),
                self._tool_call("match_target", "Agent rule matcher", True, args={"target": target}, result=search),
            ],
            final_status=response.message,
            progress=[
                self._progress_step("parse_task", "解析自然语言任务", "done"),
                self._progress_step("select_skill", "选择 TargetSearchSkill", "done"),
                self._progress_step("read_detection", "读取检测结果", "done" if detections else "failed"),
                self._progress_step("match_target", search["summary"], "done"),
            ],
        )
        return response

    def _execute_mission_sequence(self, parsed: dict, response):
        args = self._complete_sequence_args(parsed.get("args", {}))
        parsed["args"] = args
        mission = self._create_mission(parsed)
        self._active_mission = mission
        response.success = True
        response.message = f"复合任务已创建: {mission['id']}"
        response.result = self._agent_result(
            parsed,
            reply=(
                "我已创建复合任务状态机，将按顺序执行：\n"
                + "\n".join(f"- {step['label']}" for step in mission["steps"])
            ),
            tool_calls=[
                self._tool_call("create_mission", "Agent MissionManager", True, result=self._mission_snapshot(mission)),
            ],
            final_status="MissionManager 执行中",
            progress=self._mission_progress(mission),
        )
        return response

    def _execute_mission_sequence_legacy(self, parsed: dict, response):
        args = self._complete_sequence_args(parsed.get("args", {}))
        parsed["args"] = args
        tool_calls = [
            self._tool_call("get_drone_state", "/control/drone_state", self._latest_state is not None, result=self._latest_state),
            self._tool_call("get_odometry", "/sensor/odometry", self._latest_odom is not None, result=self._latest_odom),
        ]
        progress = [
            self._progress_step("parse_task", "解析复合自然语言任务", "done"),
            self._progress_step("select_skill", "选择 MissionSequenceSkill", "done"),
        ]
        replies = ["我已将复合任务拆解为多步执行链："]
        for step in args.get("steps", []):
            replies.append(f"- {step.get('label', '--')}")

        success = True
        final_messages = []
        takeoff_result = None
        if args.get("takeoff") and not self._is_airborne():
            req = Takeoff.Request()
            req.altitude = self._coerce_float(args.get("takeoff_altitude"), 10.0)
            takeoff_result = self._call_service(
                self._takeoff_client,
                req,
                "/control/takeoff",
                wait_result=False,
            )
            success = success and bool(takeoff_result.get("success"))
            final_messages.append(f"起飞请求{'已下发' if takeoff_result.get('success') else '失败'}")
            tool_calls.append(
                self._tool_call(
                    "takeoff_drone",
                    "/control/takeoff",
                    takeoff_result.get("success", False),
                    args={"altitude": req.altitude},
                    result=takeoff_result,
                )
            )
            progress.append(
                self._progress_step(
                    "takeoff",
                    f"起飞到 {req.altitude:.1f} 米",
                    "done" if takeoff_result.get("success") else "failed",
                )
            )
        elif args.get("takeoff"):
            final_messages.append("已在空中，跳过起飞")
            progress.append(self._progress_step("takeoff", "已在空中，跳过起飞", "done"))

        move_result = None
        move_args = args.get("move")
        if move_args:
            if self._latest_odom is None:
                success = False
                move_result = {"success": False, "message": "尚未收到 /sensor/odometry，无法发布移动目标"}
                progress.append(self._progress_step("publish_goal", move_result["message"], "failed"))
            elif self._is_airborne():
                move_result = self._publish_autonomy_goal(move_args)
                progress.append(self._progress_step("publish_goal", "发布 /autonomy/goal", "done"))
            elif takeoff_result and takeoff_result.get("success"):
                self._deferred_move = {
                    "args": move_args,
                    "created_at": time.time(),
                    "takeoff_altitude": self._coerce_float(args.get("takeoff_altitude"), 10.0),
                }
                move_result = {
                    "success": True,
                    "message": "已登记延迟移动任务，起飞完成后自动发布 /autonomy/goal",
                    "move": move_args,
                }
                progress.append(self._progress_step("publish_goal", "起飞完成后自动发布 /autonomy/goal", "active"))
            else:
                success = False
                move_result = {"success": False, "message": "无人机尚未起飞，移动目标未发布"}
                progress.append(self._progress_step("publish_goal", move_result["message"], "failed"))
            success = success and bool(move_result.get("success"))
            final_messages.append(move_result["message"])
            tool_calls.append(
                self._tool_call(
                    "publish_autonomy_goal",
                    "/autonomy/goal",
                    move_result.get("success", False),
                    args=move_args,
                    result=move_result,
                )
            )

        search = None
        target = args.get("target")
        if target:
            search = self._build_target_search_summary(target)
            detections = search.get("detections", [])
            parsed["target_search"] = search
            final_messages.append(search["summary"])
            replies.extend(["", "当前画面目标检测结果：", search["reply"]])
            tool_calls.extend([
                self._tool_call("get_detections", "/perception/detections", bool(detections), result=detections),
                self._tool_call("match_target", "Agent rule matcher", True, args={"target": target}, result=search),
            ])
            progress.append(self._progress_step("match_target", search["summary"], "done"))

        parsed["sequence_result"] = {
            "takeoff": takeoff_result,
            "move": move_result,
            "search": search,
        }
        response.success = bool(success)
        response.message = "；".join(final_messages) if final_messages else "复合任务已处理"
        response.result = self._agent_result(
            parsed,
            reply="\n".join(replies),
            tool_calls=tool_calls,
            final_status=response.message,
            progress=progress,
        )
        return response

    def _create_mission(self, parsed: dict):
        args = parsed.get("args", {})
        mission_id = f"mission-{int(time.time() * 1000)}"
        steps = []
        if args.get("takeoff"):
            altitude = self._coerce_float(args.get("takeoff_altitude"), 10.0)
            steps.extend([
                {
                    "name": "takeoff",
                    "label": f"起飞到 {altitude:.1f} 米",
                    "status": "pending",
                    "timeout_sec": 15.0,
                    "args": {"altitude": altitude},
                },
                {
                    "name": "wait_takeoff_done",
                    "label": f"等待高度到达 {altitude:.1f} 米",
                    "status": "pending",
                    "timeout_sec": 90.0,
                    "args": {"altitude": altitude},
                },
            ])
        if args.get("move"):
            steps.extend([
                {
                    "name": "move_to_goal",
                    "label": self._move_reason(args["move"]),
                    "status": "pending",
                    "timeout_sec": 10.0,
                    "args": args["move"],
                },
                {
                    "name": "wait_goal_done",
                    "label": "等待自主目标到达或阻塞",
                    "status": "pending",
                    "timeout_sec": 120.0,
                    "args": {},
                },
            ])
        if args.get("target"):
            steps.append({
                "name": "search_target",
                "label": f"检测目标：{args['target']}",
                "status": "pending",
                "timeout_sec": 5.0,
                "args": {"target": args["target"]},
            })
        steps.append({
            "name": "mission_report",
            "label": "生成任务报告",
            "status": "pending",
            "timeout_sec": 5.0,
            "args": {},
        })
        mission_dir = os.path.join(self._mission_log_dir, mission_id)
        mission = {
            "id": mission_id,
            "status": "active",
            "created_at": time.time(),
            "updated_at": time.time(),
            "mission_dir": mission_dir,
            "mission_json": os.path.join(mission_dir, "mission.json"),
            "report_md": os.path.join(mission_dir, "report.md"),
            "current_index": 0,
            "parsed": parsed,
            "steps": steps,
            "results": {},
            "captures": [],
            "message": "任务已创建，等待 MissionManager 执行",
        }
        self._persist_mission(mission)
        self._capture_mission_image(mission, "mission_start")
        return mission

    def _mission_timer_callback(self):
        mission = self._active_mission
        if not mission or mission.get("status") not in ("active", "running"):
            return
        if mission["current_index"] >= len(mission["steps"]):
            self._finish_mission(mission, "done", "复合任务完成")
            return

        step = mission["steps"][mission["current_index"]]
        now = time.time()
        if step["status"] == "pending":
            step["status"] = "active"
            step["started_at"] = now
            mission["status"] = "running"
            mission["updated_at"] = now
            self.get_logger().info(f"Mission {mission['id']} 执行步骤: {step['label']}")
            self._persist_mission(mission)

        if now - step.get("started_at", now) > step.get("timeout_sec", 30.0):
            self._fail_current_mission(f"步骤超时：{step['label']}")
            return

        handler = getattr(self, f"_mission_step_{step['name']}", None)
        if handler is None:
            self._fail_current_mission(f"未知任务步骤：{step['name']}")
            return
        handler(mission, step)

    def _mission_step_takeoff(self, mission: dict, step: dict):
        altitude = self._coerce_float(step.get("args", {}).get("altitude"), 10.0)
        if self._is_airborne() and self._current_altitude() >= max(0.8, altitude * 0.75):
            self._complete_mission_step(mission, step, {"success": True, "message": "已在空中，跳过起飞"})
            return
        req = Takeoff.Request()
        req.altitude = altitude
        result = self._call_service(self._takeoff_client, req, "/control/takeoff", wait_result=False)
        mission["results"]["takeoff"] = result
        if not result.get("success"):
            self._fail_current_mission(f"起飞请求失败：{result.get('message')}")
            return
        self._complete_mission_step(mission, step, result)

    def _mission_step_wait_takeoff_done(self, mission: dict, step: dict):
        altitude = self._coerce_float(step.get("args", {}).get("altitude"), 10.0)
        if self._ready_for_deferred_move(altitude):
            self._complete_mission_step(
                mission,
                step,
                {"success": True, "altitude": self._current_altitude(), "message": "起飞高度已满足"},
            )

    def _mission_step_move_to_goal(self, mission: dict, step: dict):
        if self._latest_odom is None:
            self._fail_current_mission("无法发布自主目标：尚未收到 /sensor/odometry")
            return
        if not self._is_airborne():
            self._fail_current_mission("无法发布自主目标：无人机尚未处于空中")
            return
        result = self._publish_autonomy_goal(step.get("args", {}))
        mission["results"]["move"] = result
        self._complete_mission_step(mission, step, result)

    def _mission_step_wait_goal_done(self, mission: dict, step: dict):
        status = self._latest_autonomy_status or {}
        state = str(status.get("state", ""))
        strategy = str(status.get("active_strategy", ""))
        message = str(status.get("message", ""))
        if strategy == "goal_reached" or state in ("ARRIVED",):
            self._complete_mission_step(mission, step, {"success": True, "autonomy": status})
            return
        if state == "BLOCKED_HOLD" or "阻塞" in message:
            self._complete_mission_step(
                mission,
                step,
                {"success": False, "blocked": True, "autonomy": status, "message": message or "自主任务阻塞"},
            )

    def _mission_step_search_target(self, mission: dict, step: dict):
        target = str(step.get("args", {}).get("target", "未知目标"))
        search = self._build_target_search_summary(target)
        mission["results"]["search"] = search
        self._complete_mission_step(mission, step, search)

    def _mission_step_mission_report(self, mission: dict, step: dict):
        report = self._build_mission_report()
        mission["results"]["report"] = report
        self._complete_mission_step(mission, step, report)
        self._finish_mission(mission, "done", "复合任务完成")

    def _complete_mission_step(self, mission: dict, step: dict, result):
        step["status"] = "done"
        step["result"] = result
        step["finished_at"] = time.time()
        mission["current_index"] += 1
        mission["updated_at"] = time.time()
        mission["message"] = f"完成步骤：{step['label']}"
        self.get_logger().info(f"Mission {mission['id']} {mission['message']}")
        self._persist_mission(mission)

    def _fail_current_mission(self, message: str):
        mission = self._active_mission
        if not mission:
            return
        index = mission.get("current_index", 0)
        if index < len(mission.get("steps", [])):
            step = mission["steps"][index]
            step["status"] = "failed"
            step["finished_at"] = time.time()
            step["result"] = {"success": False, "message": message}
            mission["updated_at"] = time.time()
        self._finish_mission(mission, "failed", message)

    def _finish_mission(self, mission: dict, status: str, message: str):
        mission["status"] = status
        mission["message"] = message
        mission["updated_at"] = time.time()
        self._capture_mission_image(mission, "mission_end")
        self._write_mission_report(mission)
        self._persist_mission(mission)
        self._last_task_summary = {
            "intent": "mission_sequence",
            "skill": "MissionSequenceSkill",
            "final_status": message,
            "mission": self._mission_snapshot(mission),
            "mission_dir": mission.get("mission_dir"),
            "report_md": mission.get("report_md"),
            "timestamp": time.time(),
        }
        self.get_logger().info(f"Mission {mission['id']} 结束: {status} - {message}")

    def _mission_progress(self, mission: dict):
        return [
            self._progress_step(step["name"], step["label"], step["status"])
            for step in mission.get("steps", [])
        ]

    def _publish_mission_status(self):
        payload = {
            "stamp": time.time(),
            "active_mission": self._mission_snapshot(self._active_mission) if self._active_mission else None,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._mission_status_pub.publish(msg)

    def _persist_mission(self, mission: dict):
        try:
            mission_dir = mission.get("mission_dir")
            if not mission_dir:
                return
            os.makedirs(mission_dir, exist_ok=True)
            path = mission.get("mission_json") or os.path.join(mission_dir, "mission.json")
            payload = self._mission_snapshot(mission)
            payload["written_at"] = time.time()
            with open(path, "w", encoding="utf-8") as file:
                json.dump(payload, file, ensure_ascii=False, indent=2)
        except Exception as exc:
            self.get_logger().warn(f"保存 mission.json 失败: {exc}")

    def _write_mission_report(self, mission: dict):
        try:
            mission_dir = mission.get("mission_dir")
            if not mission_dir:
                return
            os.makedirs(mission_dir, exist_ok=True)
            path = mission.get("report_md") or os.path.join(mission_dir, "report.md")
            with open(path, "w", encoding="utf-8") as file:
                file.write(self._mission_report_markdown(mission))
        except Exception as exc:
            self.get_logger().warn(f"保存 report.md 失败: {exc}")

    def _capture_mission_image(self, mission: dict, label: str):
        if not mission or not self._capture_image_client.service_is_ready():
            return None
        mission_dir = mission.get("mission_dir")
        if not mission_dir:
            return None

        capture = {
            "label": label,
            "timestamp": time.time(),
            "success": None,
            "status": "pending",
            "message": "截图请求已下发",
            "source_file": "",
            "mission_file": "",
        }
        mission.setdefault("captures", []).append(capture)
        mission.setdefault("results", {}).setdefault("captures", []).append(capture)
        self._persist_mission(mission)

        try:
            req = CaptureImage.Request()
            req.label = f"{mission.get('id', 'mission')}_{label}"
            future = self._capture_image_client.call_async(req)
            future.add_done_callback(
                lambda done: self._archive_mission_capture(mission, capture, done)
            )
            return capture
        except Exception as exc:
            capture.update({
                "success": False,
                "status": "failed",
                "message": f"任务截图请求失败: {exc}",
            })
            self._persist_mission(mission)
            self.get_logger().warn(f"任务截图归档失败: {exc}")
            return capture

    def _archive_mission_capture(self, mission: dict, capture: dict, future):
        """Archive a completed screenshot without blocking mission execution."""
        try:
            result = future.result()
            success = bool(result is not None and getattr(result, "success", False))
            source_file = getattr(result, "file_path", "") if result is not None else ""
            capture.update({
                "success": success,
                "status": "done" if success else "failed",
                "message": getattr(result, "message", "截图服务无返回") if result is not None else "截图服务无返回",
                "source_file": source_file,
            })

            if success and source_file and os.path.exists(source_file):
                mission_dir = mission.get("mission_dir", "")
                captures_dir = os.path.join(mission_dir, "captures")
                os.makedirs(captures_dir, exist_ok=True)
                ext = os.path.splitext(source_file)[1] or ".png"
                dest = os.path.join(captures_dir, f"{capture['label']}{ext}")
                shutil.copy2(source_file, dest)
                capture["mission_file"] = dest
        except Exception as exc:
            capture.update({
                "success": False,
                "status": "failed",
                "message": f"任务截图归档失败: {exc}",
            })
            self.get_logger().warn(capture["message"])

        self._persist_mission(mission)
        if mission.get("status") in ("done", "failed", "cancelled"):
            self._write_mission_report(mission)

    def _mission_report_markdown(self, mission: dict):
        parsed = mission.get("parsed", {})
        report = mission.get("results", {}).get("report") or self._build_mission_report()
        perception = report.get("perception", {})
        semantic = report.get("semantic_image", {}) or self._latest_image_analysis or {}
        lines = [
            f"# 任务报告：{mission.get('id', '--')}",
            "",
            "## 基本信息",
            "",
            f"- 任务状态：{mission.get('status', '--')}",
            f"- 结果说明：{mission.get('message', '--')}",
            f"- 创建时间：{self._format_timestamp(mission.get('created_at'))}",
            f"- 更新时间：{self._format_timestamp(mission.get('updated_at'))}",
            f"- 用户意图：{parsed.get('intent', '--')}",
            f"- 选择技能：{parsed.get('skill', '--')}",
            f"- 风险等级：{parsed.get('risk_level', '--')}",
            f"- 解析原因：{parsed.get('reason', '--')}",
            "",
            "## 执行步骤",
            "",
        ]
        for index, step in enumerate(mission.get("steps", []), start=1):
            result = step.get("result") or {}
            result_message = result.get("message") if isinstance(result, dict) else str(result)
            lines.extend([
                f"{index}. {step.get('label', step.get('name', '--'))}",
                f"   - 状态：{step.get('status', '--')}",
                f"   - 开始：{self._format_timestamp(step.get('started_at'))}",
                f"   - 结束：{self._format_timestamp(step.get('finished_at'))}",
                f"   - 结果：{result_message or '--'}",
            ])
        lines.extend([
            "",
            "## 飞行状态",
            "",
            f"- 飞行模式：{report.get('state', {}).get('mode', '--')}",
            f"- 是否解锁：{'是' if report.get('state', {}).get('armed') else '否'}",
            f"- 当前高度：{self._format_meters(report.get('altitude_m'))}",
            f"- 下一步建议：{report.get('next_step', '--')}",
            "",
            "## 感知与语义",
            "",
            f"- 感知摘要：{perception.get('summary', '--')}",
            f"- 障碍风险：{'有' if perception.get('blocked') else '无明显近距离障碍'}",
            f"- 图像场景：{semantic.get('scene', '暂无')}",
            f"- 语义风险：{semantic.get('risk_level', '暂无')}",
            f"- 语义建议：{semantic.get('suggestion', '暂无')}",
            f"- 语义来源：{semantic.get('source', '暂无')}",
            "",
            "## 截图归档",
            "",
        ])
        captures = mission.get("captures", [])
        if captures:
            for capture in captures:
                lines.append(
                    f"- {capture.get('label', '--')}：{capture.get('mission_file') or capture.get('source_file') or capture.get('message', '--')}"
                )
        else:
            lines.append("- 暂无截图")
        lines.extend([
            "",
            "## 原始数据位置",
            "",
            f"- mission.json：{mission.get('mission_json', '--')}",
            f"- report.md：{mission.get('report_md', '--')}",
            "",
        ])
        return "\n".join(lines)

    def _format_timestamp(self, value):
        if value is None:
            return "--"
        try:
            return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(float(value)))
        except (TypeError, ValueError):
            return "--"

    def _mission_snapshot(self, mission: dict):
        if not mission:
            return None
        return {
            "id": mission.get("id"),
            "status": mission.get("status"),
            "message": mission.get("message"),
            "created_at": mission.get("created_at"),
            "updated_at": mission.get("updated_at"),
            "mission_dir": mission.get("mission_dir"),
            "mission_json": mission.get("mission_json"),
            "report_md": mission.get("report_md"),
            "current_index": mission.get("current_index", 0),
            "parsed": mission.get("parsed", {}),
            "steps": [
                {
                    "name": step.get("name"),
                    "label": step.get("label"),
                    "status": step.get("status"),
                    "result": step.get("result"),
                }
                for step in mission.get("steps", [])
            ],
            "results": mission.get("results", {}),
            "captures": mission.get("captures", []),
        }

    def _execute_mission_report(self, parsed: dict, response):
        report = self._build_mission_report()
        if self._active_mission is not None:
            report["active_mission"] = self._mission_snapshot(self._active_mission)
        parsed["report"] = report
        response.success = True
        response.message = report["summary"]
        response.result = self._agent_result(
            parsed,
            reply=report["reply"],
            tool_calls=[
                self._tool_call("get_drone_state", "/control/drone_state", self._latest_state is not None, result=self._latest_state),
                self._tool_call("get_odometry", "/sensor/odometry", self._latest_odom is not None, result=self._latest_odom),
                self._tool_call("get_perception_summary", "Agent cache", True, result=self._build_perception_analysis()),
                self._tool_call("summarize_logs", "Agent recent task", self._last_task_summary is not None, result=self._last_task_summary),
            ],
            final_status=response.message,
            progress=[
                self._progress_step("parse_task", "解析自然语言任务", "done"),
                self._progress_step("select_skill", "选择 MissionReportSkill", "done"),
                self._progress_step("collect_state", "汇总飞控、里程计和感知摘要", "done"),
                self._progress_step("summarize", "生成任务报告和下一步建议", "done"),
            ],
        )
        return response

    def _summarize_depth_image(self, msg: Image):
        width = int(msg.width)
        height = int(msg.height)
        encoding = msg.encoding.lower()
        if not msg.data or width <= 0 or height <= 0:
            return {
                "topic": "/sensor/camera/depth/front_center",
                "frame_id": msg.header.frame_id,
                "width": width,
                "height": height,
                "encoding": msg.encoding,
                "has_frame": False,
            }

        if encoding in ("32fc1", "32fc"):
            fmt = "<f"
            byte_width = 4
            convert = lambda value: float(value)
        elif encoding in ("16uc1", "mono16"):
            fmt = "<H"
            byte_width = 2
            convert = lambda value: float(value) / 1000.0
        else:
            return {
                "topic": "/sensor/camera/depth/front_center",
                "frame_id": msg.header.frame_id,
                "width": width,
                "height": height,
                "encoding": msg.encoding,
                "has_frame": True,
                "valid_samples": 0,
                "message": "暂不支持该深度编码",
            }

        step = int(msg.step) or width * byte_width
        center = self._read_depth_value(msg.data, step, width // 2, height // 2, fmt, byte_width, convert)
        valid = []
        stride_x = max(1, width // 32)
        stride_y = max(1, height // 18)
        for y in range(0, height, stride_y):
            for x in range(0, width, stride_x):
                value = self._read_depth_value(msg.data, step, x, y, fmt, byte_width, convert)
                if value is not None and math.isfinite(value) and 0.05 < value < 100.0:
                    valid.append(value)

        return {
            "topic": "/sensor/camera/depth/front_center",
            "frame_id": msg.header.frame_id,
            "width": width,
            "height": height,
            "encoding": msg.encoding,
            "has_frame": True,
            "center_m": center,
            "min_m": min(valid) if valid else None,
            "max_m": max(valid) if valid else None,
            "valid_samples": len(valid),
        }

    def _read_depth_value(self, data, step, x, y, fmt, byte_width, convert):
        offset = y * step + x * byte_width
        if offset < 0 or offset + byte_width > len(data):
            return None
        try:
            return convert(struct.unpack_from(fmt, data, offset)[0])
        except (struct.error, ValueError):
            return None

    def _build_perception_analysis(self):
        rgb_ok = bool(self._latest_image and self._latest_image.get("has_frame"))
        depth = self._latest_depth or {}
        pointcloud = self._latest_pointcloud or {}
        detections = self._valid_detections()
        detection = max(detections, key=lambda item: item.get("confidence", 0.0)) if detections else None
        center = depth.get("center_m")
        nearest = depth.get("min_m")
        sampled = pointcloud.get("total_points", 0) if pointcloud.get("has_points") else 0

        blocked = False
        reasons = []
        if nearest is not None and nearest < 2.0:
            blocked = True
            reasons.append(f"最近深度 {nearest:.2f} m，小于 2.0 m")
        if center is not None and center < 3.0:
            blocked = True
            reasons.append(f"中心深度 {center:.2f} m，小于 3.0 m")
        if not rgb_ok:
            reasons.append("RGB 相机暂无有效画面")
        if not depth:
            reasons.append("深度相机暂无有效数据")

        recommendation = "不建议前进，建议悬停或重新规划" if blocked else "未发现明显近距离障碍，可继续低速前进/起飞前观察"
        summary = "前方存在近距离障碍风险" if blocked else "前方未发现明显近距离障碍"
        reply_lines = [
            "我已读取前方感知摘要：",
            f"- RGB 画面：{'有' if rgb_ok else '无'}",
            f"- 深度中心距离：{self._format_meters(center)}",
            f"- 最近障碍物距离：{self._format_meters(nearest)}",
            f"- 点云数量：{sampled}",
            f"- 检测目标：{self._format_detections()}",
            f"- 建议：{recommendation}",
        ]
        return {
            "rgb_has_frame": rgb_ok,
            "depth_center_m": center,
            "nearest_obstacle_m": nearest,
            "pointcloud_points": sampled,
            "detection": detection,
            "detections": detections,
            "blocked": blocked,
            "reasons": reasons,
            "recommendation": recommendation,
            "summary": summary,
            "reply": "\n".join(reply_lines),
        }

    def _build_scan_area_summary(self):
        perception = self._build_perception_analysis()
        depth = self._latest_depth or {}
        pointcloud = self._latest_pointcloud or {}
        coverage = []
        if self._latest_image and self._latest_image.get("has_frame"):
            coverage.append("RGB")
        if depth:
            coverage.append("Depth")
        if pointcloud.get("has_points"):
            coverage.append("LiDAR")
        if self._latest_detection or self._latest_detections:
            coverage.append("Detection")
        coverage_text = " / ".join(coverage) if coverage else "暂无有效传感器"
        risk = "blocked" if perception["blocked"] else "clear"
        reply_lines = [
            "区域扫描摘要：",
            f"- 覆盖数据：{coverage_text}",
            f"- 深度最近距离：{self._format_meters(depth.get('min_m'))}",
            f"- 深度中心距离：{self._format_meters(depth.get('center_m'))}",
            f"- 点云数量：{pointcloud.get('total_points', 0) if pointcloud.get('has_points') else 0}",
            f"- 风险状态：{risk}",
            f"- 建议：{perception['recommendation']}",
        ]
        return {
            "coverage": coverage,
            "risk": risk,
            "perception": perception,
            "summary": "区域扫描完成：" + ("存在障碍风险" if perception["blocked"] else "未发现明显近距离障碍"),
            "reply": "\n".join(reply_lines),
        }

    def _build_target_search_summary(self, target: str):
        detections = self._valid_detections()
        image_ok = bool(self._latest_image and self._latest_image.get("has_frame"))
        target_norm = self._normalize_target_name(target)
        matched = False
        reason = "暂无 detection 结果"
        best_match = None
        if detections:
            for item in detections:
                class_name = str(item.get("class_name", "")).strip()
                class_norm = self._normalize_target_name(class_name)
                if target_norm and (target_norm in class_norm or class_norm in target_norm):
                    if best_match is None or item.get("confidence", 0.0) > best_match.get("confidence", 0.0):
                        best_match = item
            matched = best_match is not None
            if best_match:
                class_name = str(best_match.get("class_name", ""))
                reason = f"检测到 {class_name}，置信度 {best_match.get('confidence', 0.0):.2f}"
            else:
                names = ", ".join(str(item.get("class_name", "--")) for item in detections[:6])
                reason = f"当前检测到 {names}，未匹配目标 {target}"
        elif image_ok:
            reason = "RGB 有画面，但当前没有目标检测结果"

        semantic = None
        if not matched and image_ok:
            semantic = self._semantic_target_search(target, target_norm)
            if semantic.get("matched"):
                matched = True
                reason = semantic.get("reason", reason)

        summary = f"目标搜索{'命中' if matched else '未命中'}：{target}"
        reply_lines = [
            "目标搜索结果：",
            f"- 搜索目标：{target}",
            f"- RGB 画面：{'有' if image_ok else '无'}",
            f"- 匹配结果：{'命中' if matched else '未命中'}",
            f"- 依据：{reason}",
            f"- 语义分析：{semantic.get('scene', '未调用') if semantic else '未调用'}",
            "- 建议：可悬停并继续观察目标位置" if matched else "- 建议：未发现目标，可调整视角或继续低速搜索",
        ]
        return {
            "target": target,
            "matched": matched,
            "detection": best_match,
            "detections": detections,
            "semantic": semantic,
            "image_has_frame": image_ok,
            "reason": reason,
            "summary": summary,
            "reply": "\n".join(reply_lines),
        }

    def _semantic_target_search(self, target: str, target_norm: str):
        if not self._analyze_image_client.service_is_ready():
            return {"matched": False, "reason": "/perception/analyze_image 未就绪"}
        req = AnalyzeImage.Request()
        req.prompt = (
            f"请判断当前无人机前视画面中是否存在目标“{target}”。"
            "如果存在，请在 objects 中给出目标名称、方位、置信度或依据；"
            "如果不存在，请说明原因。请重点用于无人机目标搜索。"
        )
        req.use_vlm = True
        result = self._call_service(
            self._analyze_image_client,
            req,
            "/perception/analyze_image",
            timeout_sec=45.0,
        )
        objects = []
        if result.get("objects_json"):
            try:
                objects = json.loads(result["objects_json"])
            except json.JSONDecodeError:
                objects = []
        haystack_parts = [
            result.get("scene", ""),
            result.get("suggestion", ""),
            result.get("message", ""),
            result.get("raw_response", ""),
        ]
        for item in objects:
            if isinstance(item, dict):
                haystack_parts.extend(str(item.get(key, "")) for key in ("name", "class_name", "label", "description", "position"))
            else:
                haystack_parts.append(str(item))
        haystack = self._normalize_target_name(" ".join(haystack_parts))
        matched = bool(target_norm and target_norm in haystack)
        reason = (
            f"语义分析命中目标 {target}：{result.get('scene', '')}"
            if matched
            else f"语义分析未确认目标 {target}：{result.get('scene', result.get('message', ''))}"
        )
        return {
            "matched": matched,
            "reason": reason,
            "scene": result.get("scene", ""),
            "risk_level": result.get("risk_level", ""),
            "suggestion": result.get("suggestion", ""),
            "objects": objects,
            "raw": result,
        }

    def _normalize_target_name(self, value: str):
        text = re.sub(r"\s+", "", str(value)).lower()
        aliases = {
            "人": "person",
            "行人": "person",
            "人员": "person",
            "车": "car",
            "汽车": "car",
            "车辆": "car",
            "小车": "car",
            "卡车": "truck",
            "公交车": "bus",
            "巴士": "bus",
            "自行车": "bicycle",
            "摩托车": "motorcycle",
            "飞机": "airplane",
            "椅子": "chair",
            "瓶子": "bottle",
            "箱子": "suitcase",
            "背包": "backpack",
        }
        if text in aliases:
            return aliases[text]
        for key, value in sorted(aliases.items(), key=lambda item: len(item[0]), reverse=True):
            if key in text:
                return value
        return text

    def _build_mission_report(self):
        state = self._latest_state or {}
        odom = self._latest_odom or {}
        perception = self._build_perception_analysis()
        semantic = self._latest_image_analysis or {}
        position = odom.get("position", {})
        altitude = position.get("z")
        last_task = self._last_task_summary
        next_step = self._next_step_advice(state, perception)
        reply_lines = [
            "当前任务报告：",
            f"- 飞行模式：{state.get('mode', '--')}",
            f"- 是否解锁：{'是' if state.get('armed') else '否'}",
            f"- 当前高度：{self._format_meters(altitude)}",
            f"- EKF/GPS：{'正常' if state.get('ekf_healthy') else '未知/异常'} / gps_fix={state.get('gps_fix', '--')}",
            (
                "- PX4 解锁预检："
                + (
                    "通过"
                    if state.get("preflight_valid") and state.get("preflight_ok")
                    else "未通过或状态不可用"
                )
            ),
            f"- 最近任务：{last_task.get('final_status') if last_task else '暂无'}",
            f"- 感知结论：{perception['summary']}",
            f"- 图像语义：{semantic.get('scene', '暂无最近图像语义分析')}",
            f"- 语义建议：{semantic.get('suggestion', '暂无')}",
            f"- 下一步建议：{next_step}",
        ]
        return {
            "state": state,
            "odom": odom,
            "altitude_m": altitude,
            "last_task": last_task,
            "perception": perception,
            "semantic_image": semantic,
            "next_step": next_step,
            "summary": "任务报告已生成",
            "reply": "\n".join(reply_lines),
        }

    def _next_step_advice(self, state: dict, perception: dict):
        if perception.get("blocked"):
            return "先悬停，检查障碍物或重新规划路径"
        if not state:
            return "先启动 bridge/control，确认飞控状态可用"
        if not state.get("ekf_healthy"):
            return "先等待 EKF 正常后再执行飞行任务"
        if not state.get("armed"):
            return "如需飞行，可先执行安全检查后解锁/起飞"
        return "可继续执行低速移动、感知检查或降落任务"

    def _format_meters(self, value):
        if value is None:
            return "--"
        try:
            number = float(value)
        except (TypeError, ValueError):
            return "--"
        if not math.isfinite(number):
            return "--"
        return f"{number:.2f} m"

    def _format_detections(self):
        detections = self._valid_detections()
        if not detections:
            return "暂无"
        labels = []
        for item in detections[:5]:
            labels.append(f"{item.get('class_name', '--')}({item.get('confidence', 0.0):.2f})")
        if len(detections) > 5:
            labels.append(f"等 {len(detections)} 个")
        return "、".join(labels)

    def _format_semantic_objects(self, objects):
        if not objects:
            return "暂无明确目标"
        labels = []
        for item in objects[:6]:
            if isinstance(item, dict):
                name = str(item.get("name") or item.get("class_name") or item.get("label") or "--")
                confidence = item.get("confidence")
                if confidence is not None:
                    labels.append(f"{name}({self._coerce_float(confidence, 0.0):.2f})")
                else:
                    labels.append(name)
            else:
                labels.append(str(item))
        if len(objects) > 6:
            labels.append(f"等 {len(objects)} 个")
        return "、".join(labels)

    def _valid_detections(self):
        raw = self._latest_detections or ([self._latest_detection] if self._latest_detection else [])
        detections = []
        for item in raw:
            if not item:
                continue
            class_name = str(item.get("class_name", "")).strip()
            confidence = self._coerce_float(item.get("confidence"), 0.0)
            if not class_name or confidence <= 0.01:
                continue
            normalized = dict(item)
            normalized["class_name"] = class_name
            normalized["confidence"] = confidence
            detections.append(normalized)
        return detections

    def _call_service(
        self,
        client,
        request,
        service_name: str,
        timeout_sec: float = 60.0,
        wait_result: bool = True,
    ):
        if not client.service_is_ready():
            return {
                "success": False,
                "message": f"{service_name} 服务未就绪，请确认对应节点已启动",
            }

        future = client.call_async(request)
        if not wait_result:
            future.add_done_callback(lambda done: self._log_async_service_result(service_name, done))
            return {
                "success": True,
                "accepted": True,
                "message": f"{service_name} 请求已下发，执行状态请查看飞控状态/WebSocket 实时反馈",
            }

        completed = threading.Event()
        future.add_done_callback(lambda _done: completed.set())
        if not completed.wait(timeout=timeout_sec):
            return {"success": False, "message": f"{service_name} 无响应或调用超时"}
        try:
            result = future.result()
        except Exception as exc:
            return {"success": False, "message": f"{service_name} 调用异常: {exc}"}
        if result is None:
            return {"success": False, "message": f"{service_name} 无响应或调用超时"}

        data = {
            "success": bool(getattr(result, "success", True)),
            "message": getattr(result, "message", "服务调用完成"),
        }
        for field in (
            "file_path",
            "width",
            "height",
            "encoding",
            "timestamp",
            "scene",
            "risk_level",
            "suggestion",
            "objects_json",
            "raw_response",
        ):
            if hasattr(result, field):
                data[field] = getattr(result, field)
        return data

    def _log_async_service_result(self, service_name: str, future):
        try:
            result = future.result()
            message = getattr(result, "message", "服务已返回") if result is not None else "无返回"
            self.get_logger().info(f"{service_name} 异步结果: {message}")
        except Exception as exc:
            self.get_logger().warn(f"{service_name} 异步调用异常: {exc}")

    def _fill_response(self, response, parsed: dict, service_result: dict, action_name: str, tool_calls=None):
        parsed["service_result"] = service_result
        response.success = service_result["success"]
        if service_result.get("accepted"):
            response.message = f"{action_name}请求已下发：{service_result['message']}"
        else:
            response.message = (
                f"{action_name}成功：{service_result['message']}"
                if service_result["success"]
                else f"{action_name}失败：{service_result['message']}"
            )
        response.result = self._agent_result(
            parsed,
            reply=f"我已解析任务为 {parsed['skill']}，并{'下发' if service_result.get('accepted') else '尝试执行'}：{action_name}。",
            tool_calls=tool_calls or [],
            final_status=response.message,
            progress=[
                self._progress_step("parse_task", "解析自然语言任务", "done"),
                self._progress_step("select_skill", f"选择 {parsed.get('skill', '--')}", "done"),
                self._progress_step("wait_confirm", "用户确认执行", "done" if parsed.get("confirmed") else "skipped"),
                self._progress_step("execute", f"调用控制服务：{action_name}", "done" if service_result["success"] else "failed"),
                self._progress_step(
                    "verify",
                    response.message if not service_result.get("accepted") else "等待飞控状态实时验证",
                    "active" if service_result.get("accepted") else ("done" if service_result["success"] else "failed"),
                ),
            ],
        )
        return response

    def _json_result(self, intent: str, args: dict, reason: str):
        parsed = self._parsed(intent, args, reason)
        return self._agent_result(parsed, reply=reason, tool_calls=[], final_status="未执行")

    def _tool_call(self, name: str, target: str, success: bool, args=None, result=None):
        return {
            "name": name,
            "target": target,
            "status": "success" if success else "failed",
            "args": args or {},
            "result": result,
        }

    def _agent_result(
        self,
        parsed: dict,
        reply: str,
        tool_calls: list,
        final_status: str,
        pending_confirmation=None,
        progress=None,
    ):
        self._last_task_summary = {
            "intent": parsed.get("intent"),
            "skill": parsed.get("skill"),
            "final_status": final_status,
            "timestamp": time.time(),
        }
        return json.dumps(
            {
                "reply": reply,
                "parsed_task": parsed,
                "tool_calls": tool_calls,
                "final_status": final_status,
                "pending_confirmation": pending_confirmation,
                "progress": progress or [],
            },
            ensure_ascii=False,
        )

    def _progress_step(self, name: str, label: str, status: str):
        return {"name": name, "label": label, "status": status}


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
