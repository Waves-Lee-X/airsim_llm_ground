"""Thread-safe, read-only ROS state cache exposed to Agent tools."""

from __future__ import annotations

import asyncio
import json
import math
import threading
import time
from typing import Any, Awaitable, Callable

from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String

from aeromind_interfaces.action import FollowWaypoints
from aeromind_interfaces.msg import AutonomyStatus, DetectionArray, DroneState
from aeromind_interfaces.srv import AnalyzeImage, ExecuteAction
from .workflow import validate_workflow


ProgressCallback = Callable[[str, dict[str, Any]], Awaitable[None]]

ACTION_VERIFICATION_TIMEOUTS = {
    "arm": 20.0,
    "disarm": 20.0,
    "takeoff": 120.0,
    "land": 180.0,
    "move": 180.0,
    "return_home": 300.0,
    "hover": 20.0,
    "capture_image": 30.0,
}


class RosStateBridge(Node):
    def __init__(self):
        super().__init__("agent_gateway_ros_state")
        self._lock = threading.RLock()
        self._state: dict[str, Any] | None = None
        self._odom: dict[str, Any] | None = None
        self._autonomy: dict[str, Any] | None = None
        self._autonomy_terminal: dict[str, Any] | None = None
        self._detections: list[dict[str, Any]] = []
        self._detections_stamp: float | None = None
        self._mission: dict[str, Any] | None = None
        self._workflow_controls: dict[str, str] = {}
        self._action_client = self.create_client(
            ExecuteAction, "/agent/execute_action"
        )
        self._analyze_image_client = self.create_client(
            AnalyzeImage, "/perception/analyze_image"
        )
        self._waypoint_action_client = ActionClient(
            self, FollowWaypoints, "/autonomy/follow_waypoints"
        )
        self._autonomy_cancel_pub = self.create_publisher(
            Empty, "/autonomy/cancel", 10
        )

        self.create_subscription(
            DroneState, "/control/drone_state", self._state_cb, 10
        )
        self.create_subscription(Odometry, "/sensor/odometry", self._odom_cb, 10)
        self.create_subscription(
            AutonomyStatus, "/autonomy/status", self._autonomy_cb, 10
        )
        self.create_subscription(
            DetectionArray, "/perception/detections", self._detections_cb, 10
        )
        self.create_subscription(String, "/agent/mission_status", self._mission_cb, 10)

    def _state_cb(self, msg: DroneState):
        battery = float(msg.battery)
        with self._lock:
            self._state = {
                "stamp": time.time(),
                "armed": bool(msg.armed),
                "mode": msg.mode,
                "battery": battery if battery > 0.0 else None,
                "battery_available": battery > 0.0,
                "gps_fix": int(msg.gps_fix),
                "gps_usable": int(msg.gps_fix) >= 2,
                "ekf_healthy": bool(msg.ekf_healthy),
            }

    def _odom_cb(self, msg: Odometry):
        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear
        with self._lock:
            self._odom = {
                "stamp": time.time(),
                "frame_id": msg.header.frame_id,
                "position_m": {
                    "x": float(position.x),
                    "y": float(position.y),
                    "z": float(position.z),
                },
                "velocity_mps": {
                    "x": float(velocity.x),
                    "y": float(velocity.y),
                    "z": float(velocity.z),
                },
            }

    def _autonomy_cb(self, msg: AutonomyStatus):
        received_at = time.time()
        record = {
            "stamp": received_at,
            "enabled": bool(msg.enabled),
            "state": msg.state,
            "replanning": bool(msg.replanning),
            "nearest_obstacle_m": float(msg.nearest_obstacle_m),
            "target_distance_m": float(msg.target_distance_m),
            "active_strategy": msg.active_strategy,
            "message": msg.message,
        }
        with self._lock:
            self._autonomy = record
            if (
                msg.state in {"ARRIVED", "BLOCKED_HOLD"}
                or msg.active_strategy in {"goal_reached", "blocked_hold"}
            ):
                self._autonomy_terminal = dict(record)

    def _detections_cb(self, msg: DetectionArray):
        values = [
            {
                "class_name": item.class_name,
                "confidence": float(item.confidence),
                "center_px": {"x": float(item.x), "y": float(item.y)},
                "size_px": {"width": float(item.width), "height": float(item.height)},
            }
            for item in msg.detections
        ]
        with self._lock:
            self._detections = values
            self._detections_stamp = time.time()

    def _mission_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("收到无法解析的 /agent/mission_status")
            return
        mission = payload.get("active_mission")
        with self._lock:
            self._mission = mission if isinstance(mission, dict) else None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            state = _record_with_freshness(self._state, 3.0)
            odometry = _record_with_freshness(self._odom, 2.0)
            autonomy = _record_with_freshness(self._autonomy, 2.0)
            detections_age = _age_seconds(self._detections_stamp)
            return {
                "available": bool(state and state["fresh"]),
                "state": state,
                "odometry": odometry,
                "autonomy": autonomy,
                "autonomy_terminal": (
                    dict(self._autonomy_terminal)
                    if self._autonomy_terminal
                    else None
                ),
                "detections": [dict(item) for item in self._detections]
                if detections_age is not None and detections_age <= 2.0
                else [],
                "detections_fresh": detections_age is not None and detections_age <= 2.0,
                "detections_age_sec": detections_age,
                "mission": dict(self._mission) if self._mission else None,
            }

    def drone_state(self) -> dict[str, Any]:
        snapshot = self.snapshot()
        return {
            "available": snapshot["available"],
            "state": snapshot["state"],
            "odometry": snapshot["odometry"],
        }

    def perception_summary(self) -> dict[str, Any]:
        snapshot = self.snapshot()
        return {
            "available": bool(snapshot["detections"]),
            "detections": snapshot["detections"],
            "detections_fresh": snapshot["detections_fresh"],
            "detections_age_sec": snapshot["detections_age_sec"],
            "autonomy": snapshot["autonomy"],
        }

    async def analyze_current_image(self, prompt: str = "") -> dict[str, Any]:
        """Ask perception_node to analyze the latest RGB frame with its VLM."""
        if (
            not self._analyze_image_client.service_is_ready()
            and not self._analyze_image_client.wait_for_service(timeout_sec=1.0)
        ):
            return {
                "success": False,
                "message": "/perception/analyze_image 服务未就绪",
                "source": "unavailable",
            }

        request = AnalyzeImage.Request()
        request.prompt = prompt.strip() or "请分析当前前视相机画面。"
        request.use_vlm = True
        ros_future = self._analyze_image_client.call_async(request)
        loop = asyncio.get_running_loop()
        result_future = loop.create_future()

        def on_done(done_future):
            try:
                response = done_future.result()
                raw = _parse_json_object(response.raw_response)
                value = {
                    "success": bool(response.success),
                    "message": response.message,
                    "scene": response.scene,
                    "risk_level": response.risk_level,
                    "suggestion": response.suggestion,
                    "objects": _parse_json_list(response.objects_json),
                    "source": raw.get("source", "unknown"),
                    "image": {
                        "width": int(response.width),
                        "height": int(response.height),
                        "encoding": response.encoding,
                    },
                }
                if raw.get("vlm_error"):
                    value["vlm_error"] = str(raw["vlm_error"])
                loop.call_soon_threadsafe(_set_result_if_pending, result_future, value)
            except Exception as exc:
                loop.call_soon_threadsafe(
                    _set_exception_if_pending, result_future, exc
                )

        ros_future.add_done_callback(on_done)
        try:
            return await asyncio.wait_for(result_future, timeout=35.0)
        except asyncio.TimeoutError:
            return {
                "success": False,
                "message": "/perception/analyze_image 调用超时（35s）",
                "source": "timeout",
            }

    async def execute_confirmed_action(
        self,
        action: str,
        args: dict[str, Any],
        progress: ProgressCallback,
    ) -> dict[str, Any]:
        if action == "workflow":
            return await self._execute_workflow(args, progress)
        started_at = time.time()
        accepted = await self._execute_action_service(
            action,
            args,
            request_id=f"gateway-{int(started_at * 1000)}",
            timeout_sec=45.0,
        )
        if not accepted.get("success"):
            return accepted
        payload = _parse_agent_payload(accepted.get("result"))
        accepted["agent_payload"] = payload
        accepted["gateway_confirmed_action"] = {"action": action, "args": args}

        # CaptureImage is completed synchronously by perception_node. Unlike a
        # flight action, it has no later telemetry state to wait for.
        if action == "capture_image":
            await progress(
                "completed",
                {
                    "message": accepted.get("message") or "当前相机图像已保存",
                    "physical_complete": True,
                },
            )
            return {
                "success": True,
                "message": accepted.get("message") or "当前相机图像已保存",
                "physical_complete": True,
                "command_result": accepted,
            }

        agent_payload = accepted.get("agent_payload") or payload
        ros_mission_id = _extract_ros_mission_id(agent_payload)
        await progress(
            "accepted",
            {
                "message": accepted.get("message") or "ROS Agent 已受理控制请求",
                "ros_mission_id": ros_mission_id,
                "physical_complete": False,
            },
        )
        await progress(
            "verifying",
            {
                "message": "等待 ROS Mission 或飞控遥测确认物理动作完成",
                "ros_mission_id": ros_mission_id,
                "physical_complete": False,
            },
        )
        verification = await self._wait_for_action_completion(
            action,
            args,
            started_at,
            ros_mission_id,
            ACTION_VERIFICATION_TIMEOUTS[action],
        )
        if action == "move" and not verification["success"]:
            self._autonomy_cancel_pub.publish(Empty())
            await progress(
                "stopped",
                {
                    "message": "移动验证失败，已清空自主飞行目标并保持悬停",
                    "physical_complete": False,
                },
            )
        return {
            "success": bool(verification["success"]),
            "message": verification["message"],
            "physical_complete": bool(verification["success"]),
            "ros_mission_id": ros_mission_id,
            "verification": verification,
            "command_result": accepted,
        }

    def control_workflow(self, workflow_id: str, command: str) -> dict[str, Any]:
        normalized = command.strip().lower()
        if normalized not in {"pause", "resume", "cancel"}:
            raise ValueError("workflow command 必须为 pause、resume 或 cancel")
        with self._lock:
            if workflow_id not in self._workflow_controls:
                raise ValueError("workflow 不存在或已经结束")
            current = self._workflow_controls[workflow_id]
            transitions = {
                ("running", "pause"): "paused",
                ("paused", "resume"): "running",
                ("running", "cancel"): "cancelled",
                ("paused", "cancel"): "cancelled",
            }
            target = transitions.get((current, normalized))
            if target is None:
                raise ValueError(f"workflow 当前为 {current}，不能执行 {normalized}")
            self._workflow_controls[workflow_id] = target
        if normalized in {"pause", "cancel"}:
            self._autonomy_cancel_pub.publish(Empty())
        status_label = {
            "paused": "暂停",
            "running": "恢复",
            "cancelled": "取消",
        }[target]
        return {
            "success": True,
            "workflow_id": workflow_id,
            "status": target,
            "message": f"workflow 已{status_label}",
        }

    async def _execute_workflow(
        self, value: dict[str, Any], progress: ProgressCallback
    ) -> dict[str, Any]:
        workflow = validate_workflow(value)
        workflow_id = workflow["workflow_id"]
        results: dict[str, dict[str, Any]] = {}
        with self._lock:
            self._workflow_controls[workflow_id] = "running"
        try:
            for index, step in enumerate(workflow["steps"]):
                dependency_failed = any(
                    results.get(item, {}).get("success") is not True
                    for item in step["depends_on"]
                )
                if dependency_failed or not self._workflow_condition(
                    step["condition"], results
                ):
                    step["status"] = "skipped"
                    results[step["id"]] = {
                        "success": True,
                        "skipped": True,
                        "message": "依赖或执行条件不满足，已跳过",
                    }
                    await self._emit_workflow_progress(
                        progress, workflow, index, results, "workflow_step"
                    )
                    continue
                attempt = 0
                while attempt <= step["retries"]:
                    control = await self._wait_workflow_control(
                        workflow, index, results, progress
                    )
                    if control == "cancelled":
                        return self._cancelled_workflow_result(workflow, results)
                    step["status"] = "running"
                    step["attempt"] = attempt + 1
                    await self._emit_workflow_progress(
                        progress, workflow, index, results, "workflow_step"
                    )

                    async def child_progress(
                        phase: str, details: dict[str, Any]
                    ):
                        step["phase"] = phase
                        step["message"] = details.get("message", "")
                        await self._emit_workflow_progress(
                            progress, workflow, index, results, "workflow_step"
                        )

                    child = asyncio.create_task(
                        self._execute_workflow_step(step, child_progress)
                    )
                    interrupted = False
                    while not child.done():
                        await asyncio.sleep(0.2)
                        with self._lock:
                            control = self._workflow_controls.get(workflow_id)
                        if control in {"paused", "cancelled"}:
                            child.cancel()
                            await asyncio.gather(child, return_exceptions=True)
                            self._autonomy_cancel_pub.publish(Empty())
                            interrupted = True
                            break
                    if interrupted:
                        if control == "cancelled":
                            return self._cancelled_workflow_result(workflow, results)
                        step["status"] = "paused"
                        continue
                    result = await child
                    results[step["id"]] = result
                    if result.get("success"):
                        step["status"] = "completed"
                        break
                    attempt += 1
                    step["status"] = "retrying" if attempt <= step["retries"] else "failed"
                await self._emit_workflow_progress(
                    progress, workflow, index, results, "workflow_step"
                )
                if step["status"] == "failed" and step["on_failure"] == "stop":
                    return {
                        "success": False,
                        "status": "failed",
                        "physical_complete": False,
                        "workflow_id": workflow_id,
                        "message": f"组合任务失败：{step['label']}",
                        "workflow": workflow,
                        "step_results": results,
                    }
            return {
                "success": True,
                "status": "completed",
                "physical_complete": True,
                "workflow_id": workflow_id,
                "message": f"组合任务完成：{workflow['name']}",
                "workflow": workflow,
                "step_results": results,
            }
        finally:
            with self._lock:
                self._workflow_controls.pop(workflow_id, None)

    async def _wait_workflow_control(
        self,
        workflow: dict[str, Any],
        index: int,
        results: dict[str, dict[str, Any]],
        progress: ProgressCallback,
    ) -> str:
        workflow_id = workflow["workflow_id"]
        while True:
            with self._lock:
                control = self._workflow_controls.get(workflow_id, "cancelled")
            if control != "paused":
                return control
            await self._emit_workflow_progress(
                progress, workflow, index, results, "paused"
            )
            await asyncio.sleep(0.25)

    def _workflow_condition(
        self, condition: str | dict[str, Any], results: dict[str, dict[str, Any]]
    ) -> bool:
        if isinstance(condition, dict):
            source = results.get(str(condition.get("step_id"))) or {}
            condition_type = str(condition.get("type"))
            if condition_type == "step_succeeded":
                return source.get("success") is True
            if condition_type == "step_failed":
                return source.get("success") is False
            matched = bool(source.get("matched"))
            if condition_type == "target_detected":
                return matched
            if condition_type == "target_not_detected":
                return bool(source.get("observation_available")) and not matched
            if condition_type == "risk_level_is":
                return str(source.get("risk_level") or "").lower() == str(
                    condition.get("value") or ""
                ).lower()
            if condition_type in {
                "semantic_target_detected",
                "semantic_target_not_detected",
            }:
                semantic_matched = _semantic_result_contains(
                    source, str(condition.get("target") or "")
                )
                return (
                    semantic_matched
                    if condition_type == "semantic_target_detected"
                    else bool(source.get("success")) and not semantic_matched
                )
            return False
        snapshot = self.snapshot()
        state = snapshot.get("state") or {}
        odom = snapshot.get("odometry") or {}
        altitude = float((odom.get("position_m") or {}).get("z", 0.0))
        airborne = bool(state.get("armed")) and altitude > 0.5
        safe = self._safety_check({}).get("success") is True
        return (
            condition == "always"
            or (condition == "if_airborne" and airborne)
            or (condition == "if_not_airborne" and not airborne)
            or (condition == "if_safe" and safe)
        )

    async def _execute_workflow_step(
        self,
        step: dict[str, Any],
        progress: ProgressCallback,
    ) -> dict[str, Any]:
        action = step["action"]
        if action == "safety_check":
            result = self._safety_check(step["args"])
            await progress("checked", {"message": result["message"]})
            return result
        if action == "perception_check":
            result = self._perception_check(step["args"])
            await progress("checked", {"message": result["message"]})
            return result
        if action == "analyze_image":
            result = await self.analyze_current_image(step["args"]["prompt"])
            result = {
                **result,
                "physical_complete": bool(result.get("success")),
            }
            await progress(
                "analyzed",
                {"message": result.get("message", "图像语义分析完成")},
            )
            return result
        if action == "wait":
            duration = float(step["args"]["duration_sec"])
            await progress("waiting", {"message": f"等待 {duration:.1f} 秒"})
            await asyncio.sleep(duration)
            return {
                "success": True,
                "physical_complete": True,
                "message": f"已等待 {duration:.1f} 秒",
                "duration_sec": duration,
            }
        if action == "mission_report":
            snapshot = self.snapshot()
            title = str(step["args"].get("title") or "任务报告")
            result = {
                "success": True,
                "physical_complete": True,
                "message": f"已生成报告数据：{title}",
                "title": title,
                "snapshot": snapshot,
            }
            await progress("reported", {"message": result["message"]})
            return result
        if action == "follow_waypoints":
            return await self._execute_waypoint_action(step["args"], progress)
        return await self.execute_confirmed_action(
            action, step["args"], progress
        )

    async def _execute_waypoint_action(
        self, args: dict[str, Any], progress: ProgressCallback
    ) -> dict[str, Any]:
        if not self._waypoint_action_client.server_is_ready():
            if not self._waypoint_action_client.wait_for_server(timeout_sec=2.0):
                return {
                    "success": False,
                    "physical_complete": False,
                    "message": "/autonomy/follow_waypoints Action 服务未就绪",
                }
        request = FollowWaypoints.Goal()
        request.cruise_speed = float(args.get("cruise_speed", 1.5))
        for point in args["points"]:
            request.relative_waypoints.append(
                Vector3(
                    x=float(point.get("forward_m", 0.0)),
                    y=float(point.get("right_m", 0.0)),
                    z=float(point.get("up_m", 0.0)),
                )
            )
        loop = asyncio.get_running_loop()

        def feedback_callback(feedback_message):
            feedback = feedback_message.feedback
            asyncio.run_coroutine_threadsafe(
                progress(
                    "waypoint",
                    {
                        "message": (
                            f"连续轨迹航点 {feedback.current_waypoint}/"
                            f"{len(request.relative_waypoints)}，"
                            f"进度 {feedback.progress * 100:.0f}%"
                        ),
                        "waypoint_index": int(feedback.current_waypoint),
                        "waypoint_count": len(request.relative_waypoints),
                        "progress": float(feedback.progress),
                        "remaining_distance_m": float(feedback.remaining_distance),
                        "nearest_obstacle_m": float(feedback.nearest_obstacle),
                        "state": feedback.state,
                    },
                ),
                loop,
            )

        goal_handle = await self._await_ros_future(
            self._waypoint_action_client.send_goal_async(
                request, feedback_callback=feedback_callback
            ),
            timeout_sec=10.0,
        )
        if goal_handle is None or not goal_handle.accepted:
            return {
                "success": False,
                "physical_complete": False,
                "message": "连续多航点 Action 目标被拒绝",
            }
        await progress(
            "accepted",
            {
                "message": f"ROS 2 Action 已受理 {len(request.relative_waypoints)} 个航点",
                "physical_complete": False,
            },
        )
        try:
            wrapped_result = await self._await_ros_future(
                goal_handle.get_result_async(), timeout_sec=360.0
            )
        except asyncio.CancelledError:
            goal_handle.cancel_goal_async()
            raise
        if wrapped_result is None:
            goal_handle.cancel_goal_async()
            return {
                "success": False,
                "physical_complete": False,
                "message": "连续多航点 Action 等待结果超时",
            }
        action_result = wrapped_result.result
        return {
            "success": bool(action_result.success),
            "physical_complete": bool(action_result.success),
            "message": action_result.message,
            "completed_waypoints": int(action_result.completed_waypoints),
            "final_position": {
                "x": float(action_result.final_position.x),
                "y": float(action_result.final_position.y),
                "z": float(action_result.final_position.z),
            },
        }

    async def _await_ros_future(self, ros_future, timeout_sec: float):
        loop = asyncio.get_running_loop()
        result_future = loop.create_future()

        def on_done(done_future):
            try:
                loop.call_soon_threadsafe(
                    _set_result_if_pending, result_future, done_future.result()
                )
            except Exception as exc:
                loop.call_soon_threadsafe(
                    _set_exception_if_pending, result_future, exc
                )

        ros_future.add_done_callback(on_done)
        try:
            return await asyncio.wait_for(result_future, timeout=timeout_sec)
        except asyncio.TimeoutError:
            return None

    def _safety_check(self, args: dict[str, Any]) -> dict[str, Any]:
        snapshot = self.snapshot()
        state = snapshot.get("state") or {}
        autonomy = snapshot.get("autonomy") or {}
        issues = []
        state_fresh = _record_is_current(state, 3.0)
        autonomy_fresh = _record_is_current(autonomy, 2.0)
        if not snapshot.get("available"):
            issues.append("飞控状态不可用")
        elif not state_fresh:
            issues.append("飞控状态已过期")
        if state and not state.get("ekf_healthy", False):
            issues.append("EKF 状态异常")
        if args.get("require_gps", True) and int(state.get("gps_fix", 0)) < 2:
            issues.append("GPS 定位质量不足")
        minimum = float(args.get("minimum_obstacle_distance", 2.0))
        obstacle = autonomy.get("nearest_obstacle_m") if autonomy_fresh else None
        if not autonomy_fresh:
            issues.append("自主避障状态不可用或已过期")
        elif obstacle is not None and math.isfinite(float(obstacle)):
            if float(obstacle) < minimum:
                issues.append(f"最近障碍物仅 {float(obstacle):.2f} 米")
        success = not issues
        return {
            "success": success,
            "physical_complete": success,
            "message": "安全检查通过" if success else "安全检查未通过：" + "；".join(issues),
            "checks": {
                "ekf_healthy": state.get("ekf_healthy"),
                "gps_fix": state.get("gps_fix"),
                "gps_usable": state.get("gps_usable"),
                "nearest_obstacle_m": obstacle,
                "minimum_obstacle_distance": minimum,
                "state_age_sec": state.get("age_sec"),
                "autonomy_age_sec": autonomy.get("age_sec"),
            },
        }

    def _perception_check(self, args: dict[str, Any]) -> dict[str, Any]:
        snapshot = self.snapshot()
        detections = snapshot.get("detections") or []
        observation_available = bool(
            snapshot.get("detections_fresh", bool(detections))
        )
        target = str(args.get("target") or "").strip().lower()
        confidence = float(args.get("minimum_confidence", 0.35))
        matched = [
            item for item in detections
            if float(item.get("confidence", 0.0)) >= confidence
            and (not target or target in str(item.get("class_name", "")).lower())
        ]
        success = observation_available if target else bool(
            observation_available or snapshot.get("autonomy")
        )
        if target:
            message = (
                f"感知检查命中 {target}，共 {len(matched)} 个目标"
                if success else f"感知检查未发现 {target}"
            )
        else:
            message = f"感知摘要已读取，共 {len(detections)} 个检测目标"
        return {
            "success": success,
            "physical_complete": success,
            "message": message,
            "matched": bool(matched),
            "observation_available": observation_available,
            "target": target,
            "detections": matched if target else detections,
            "autonomy": snapshot.get("autonomy"),
        }

    async def _emit_workflow_progress(
        self,
        progress: ProgressCallback,
        workflow: dict[str, Any],
        index: int,
        results: dict[str, dict[str, Any]],
        phase: str,
    ):
        await progress(
            phase,
            {
                "message": workflow["steps"][index].get("label", workflow["name"]),
                "workflow_id": workflow["workflow_id"],
                "workflow": {
                    **workflow,
                    "current_index": index,
                    "status": phase,
                    "results": results,
                },
                "physical_complete": False,
            },
        )

    @staticmethod
    def _cancelled_workflow_result(
        workflow: dict[str, Any], results: dict[str, dict[str, Any]]
    ) -> dict[str, Any]:
        return {
            "success": False,
            "status": "cancelled",
            "physical_complete": False,
            "workflow_id": workflow["workflow_id"],
            "message": f"组合任务已取消：{workflow['name']}",
            "workflow": workflow,
            "step_results": results,
        }

    async def _wait_for_action_completion(
        self,
        action: str,
        args: dict[str, Any],
        started_at: float,
        ros_mission_id: str | None,
        timeout_sec: float,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            result = _evaluate_action_completion(
                action, args, snapshot, started_at, ros_mission_id
            )
            if result["terminal"]:
                return result
            await asyncio.sleep(0.25)
        snapshot = self.snapshot()
        return {
            "terminal": True,
            "success": False,
            "phase": "timeout",
            "message": f"动作已受理，但在 {timeout_sec:.0f}s 内未通过物理状态验证",
            "evidence": _verification_evidence(snapshot),
        }

    async def _execute_action_service(
        self,
        action: str,
        args: dict[str, Any],
        request_id: str,
        timeout_sec: float,
    ) -> dict[str, Any]:
        if not self._action_client.service_is_ready() and not self._action_client.wait_for_service(
            timeout_sec=1.0
        ):
            return {
                "success": False,
                "message": "/agent/execute_action 结构化动作服务未就绪",
            }
        request = ExecuteAction.Request()
        request.action = action
        request.args_json = json.dumps(args, ensure_ascii=False)
        request.request_id = request_id
        ros_future = self._action_client.call_async(request)
        loop = asyncio.get_running_loop()
        result_future = loop.create_future()

        def on_done(done_future):
            try:
                response = done_future.result()
                value = {
                    "success": bool(response.success),
                    "result": response.result,
                    "message": response.message,
                }
                loop.call_soon_threadsafe(_set_result_if_pending, result_future, value)
            except Exception as exc:
                loop.call_soon_threadsafe(
                    _set_exception_if_pending, result_future, exc
                )

        ros_future.add_done_callback(on_done)
        try:
            return await asyncio.wait_for(result_future, timeout=timeout_sec)
        except asyncio.TimeoutError:
            return {
                "success": False,
                "message": f"/agent/execute_action 调用超时（{timeout_sec:.0f}s）",
            }


def _set_result_if_pending(future: asyncio.Future, value: Any):
    if not future.done():
        future.set_result(value)


def _set_exception_if_pending(future: asyncio.Future, error: Exception):
    if not future.done():
        future.set_exception(error)


def _parse_json_object(value: str) -> dict[str, Any]:
    try:
        parsed = json.loads(value or "{}")
    except (json.JSONDecodeError, TypeError):
        return {}
    return parsed if isinstance(parsed, dict) else {}


def _parse_json_list(value: str) -> list[Any]:
    try:
        parsed = json.loads(value or "[]")
    except (json.JSONDecodeError, TypeError):
        return []
    return parsed if isinstance(parsed, list) else []


def _parse_agent_payload(value: Any) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    if not value:
        return {}
    try:
        payload = json.loads(str(value))
        return payload if isinstance(payload, dict) else {}
    except json.JSONDecodeError:
        return {}


def _extract_ros_mission_id(payload: dict[str, Any]) -> str | None:
    candidates = [payload.get("mission"), payload.get("active_mission")]
    parsed = payload.get("parsed_task") or {}
    if isinstance(parsed, dict):
        candidates.extend([parsed.get("mission"), parsed.get("active_mission")])
    for call in payload.get("tool_calls") or []:
        if isinstance(call, dict) and call.get("name") == "create_mission":
            candidates.append(call.get("result"))
    for candidate in candidates:
        if isinstance(candidate, dict) and candidate.get("id"):
            return str(candidate["id"])
    return None


def _evaluate_action_completion(
    action: str,
    args: dict[str, Any],
    snapshot: dict[str, Any],
    started_at: float,
    ros_mission_id: str | None = None,
) -> dict[str, Any]:
    mission = snapshot.get("mission") or {}
    if ros_mission_id and mission.get("id") == ros_mission_id:
        mission_status = str(mission.get("status", ""))
        if mission_status == "done":
            return _verification_result(True, mission.get("message") or "ROS Mission 完成", snapshot)
        if mission_status in {"failed", "cancelled"}:
            return _verification_result(
                False,
                mission.get("message") or f"ROS Mission {mission_status}",
                snapshot,
            )

    state = snapshot.get("state") or {}
    odom = snapshot.get("odometry") or {}
    autonomy = snapshot.get("autonomy") or {}
    state_fresh = _record_is_fresh(state, started_at)
    odom_fresh = _record_is_fresh(odom, started_at)
    autonomy_fresh = _record_is_fresh(autonomy, started_at)
    armed = bool(state.get("armed"))
    position = odom.get("position_m") or {}
    velocity = odom.get("velocity_mps") or {}
    altitude = float(position.get("z", 0.0))
    speed = math.sqrt(
        sum(float(velocity.get(axis, 0.0)) ** 2 for axis in ("x", "y", "z"))
    )

    if action == "arm" and state_fresh and armed:
        return _verification_result(True, "飞控遥测确认无人机已解锁", snapshot)
    if action == "disarm" and state_fresh and not armed:
        return _verification_result(True, "飞控遥测确认无人机已加锁", snapshot)
    if action == "takeoff" and state_fresh and odom_fresh and armed:
        target = float(args.get("altitude", 10.0))
        tolerance = max(0.5, min(1.5, target * 0.15))
        if altitude >= target - tolerance:
            return _verification_result(
                True,
                f"里程计确认起飞高度已达到 {altitude:.2f} 米",
                snapshot,
            )
    if action == "land" and state_fresh and odom_fresh:
        landed = altitude <= 0.35 and abs(float(velocity.get("z", 0.0))) <= 0.3
        if not armed or landed:
            return _verification_result(True, "飞控遥测确认无人机已落地", snapshot)
    if action == "move" and autonomy_fresh:
        autonomy_state = str(autonomy.get("state", ""))
        strategy = str(autonomy.get("active_strategy", ""))
        if strategy == "goal_reached" or autonomy_state == "ARRIVED":
            return _verification_result(True, "自主规划器确认已到达目标", snapshot)
        if autonomy_state == "BLOCKED_HOLD" and strategy == "blocked_hold":
            return _verification_result(
                False,
                str(autonomy.get("message", "")) or "自主规划器报告目标不可达",
                snapshot,
            )
    if action == "move":
        terminal = snapshot.get("autonomy_terminal") or {}
        if float(terminal.get("stamp", 0.0)) >= started_at:
            terminal_state = str(terminal.get("state", ""))
            terminal_strategy = str(terminal.get("active_strategy", ""))
            terminal_message = str(terminal.get("message", ""))
            if terminal_strategy == "goal_reached" or terminal_state == "ARRIVED":
                return _verification_result(
                    True,
                    terminal_message or "自主规划器确认已到达目标",
                    snapshot,
                )
            if terminal_state == "BLOCKED_HOLD" and terminal_strategy == "blocked_hold":
                return _verification_result(
                    False,
                    terminal_message or "自主规划器报告目标不可达",
                    snapshot,
                )
    if action == "return_home" and state_fresh and not armed:
        return _verification_result(True, "飞控遥测确认 RTL 已完成并加锁", snapshot)
    if action == "hover" and odom_fresh and speed <= 0.25:
        strategy = str(autonomy.get("active_strategy", ""))
        if not autonomy_fresh or strategy in {
            "hover_no_goal",
            "safe_hover",
            "blocked_hold",
        }:
            return _verification_result(True, "里程计确认无人机已悬停", snapshot)

    return {
        "terminal": False,
        "success": False,
        "phase": "verifying",
        "message": "等待物理状态验证",
        "evidence": _verification_evidence(snapshot),
    }


def _record_is_fresh(record: dict[str, Any], started_at: float) -> bool:
    return float(record.get("stamp", 0.0)) >= started_at - 0.5


def _age_seconds(stamp: float | None) -> float | None:
    if stamp is None:
        return None
    return round(max(0.0, time.time() - float(stamp)), 3)


def _record_with_freshness(
    record: dict[str, Any] | None, timeout_sec: float
) -> dict[str, Any] | None:
    if record is None:
        return None
    value = dict(record)
    age = _age_seconds(value.get("stamp"))
    value["age_sec"] = age
    value["fresh"] = age is not None and age <= timeout_sec
    return value


def _record_is_current(record: dict[str, Any], timeout_sec: float) -> bool:
    if "fresh" in record:
        return bool(record["fresh"])
    age = _age_seconds(record.get("stamp"))
    return age is not None and age <= timeout_sec


def _verification_result(
    success: bool, message: str, snapshot: dict[str, Any]
) -> dict[str, Any]:
    return {
        "terminal": True,
        "success": success,
        "phase": "completed" if success else "failed",
        "message": message,
        "evidence": _verification_evidence(snapshot),
    }


def _verification_evidence(snapshot: dict[str, Any]) -> dict[str, Any]:
    state = snapshot.get("state") or {}
    odom = snapshot.get("odometry") or {}
    autonomy = snapshot.get("autonomy") or {}
    autonomy_terminal = snapshot.get("autonomy_terminal") or {}
    return {
        "armed": state.get("armed"),
        "mode": state.get("mode"),
        "position_m": odom.get("position_m"),
        "velocity_mps": odom.get("velocity_mps"),
        "autonomy_state": autonomy.get("state"),
        "autonomy_strategy": autonomy.get("active_strategy"),
        "target_distance_m": autonomy.get("target_distance_m"),
        "autonomy_terminal": {
            "stamp": autonomy_terminal.get("stamp"),
            "state": autonomy_terminal.get("state"),
            "strategy": autonomy_terminal.get("active_strategy"),
            "message": autonomy_terminal.get("message"),
        } if autonomy_terminal else None,
    }


def _semantic_result_contains(result: dict[str, Any], target: str) -> bool:
    aliases = {
        "人": "person",
        "行人": "person",
        "人员": "person",
        "汽车": "car",
        "车辆": "car",
        "车": "car",
    }

    def normalize(value: Any) -> str:
        text = "".join(str(value or "").lower().split())
        return aliases.get(text, text)

    expected = normalize(target)
    if not expected:
        return False
    values = []
    for item in result.get("objects") or []:
        if isinstance(item, dict):
            values.extend(
                item.get(key, "")
                for key in ("name", "class_name", "label", "description")
            )
        else:
            values.append(item)
    normalized = [normalize(value) for value in values]
    return any(expected in value for value in normalized)


def _action_task(action: str, args: dict[str, Any]) -> tuple[str, str]:
    if action == "arm":
        return "解锁无人机", "arm"
    if action == "disarm":
        return "加锁无人机", "disarm"
    if action == "takeoff":
        return f"起飞到 {float(args['altitude']):.1f} 米", "takeoff"
    if action == "land":
        return "降落", "land"
    if action == "return_home":
        return "返航", "return_home"
    if action == "hover":
        return "悬停", "hover"
    if action == "move":
        if "direction" not in args:
            vector = {
                key: float(args.get(key, 0.0))
                for key in ("forward_m", "right_m", "up_m")
            }
            return (
                "__aeromind_move_vector__:" + json.dumps(vector),
                "move_to",
            )
        return (
            f"向{args['direction']}飞行 {float(args['distance']):.1f} 米",
            "move_to",
        )
    if action == "capture_image":
        return "拍照并保存当前相机图像", "capture_image"
    raise ValueError(f"不支持的控制动作: {action}")
