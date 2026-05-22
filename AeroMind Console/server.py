import argparse
import json
import mimetypes
import sys
import time
import traceback
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parent
STATIC_DIR = ROOT / "static"
LOG_DIR = ROOT / "logs"
LOG_FILE = LOG_DIR / "aeromind.log"
sys.path.insert(0, str(ROOT))

from core.task_planner import TaskPlanner  # noqa: E402
from core.task_schema import MissionArea  # noqa: E402
from core.airsim_adapter import AirSimAdapter, CollisionStatus, DistanceSensorsStatus, ObstacleStatus  # noqa: E402
from core.video_stream import VideoStream  # noqa: E402
from core.path_planner import Waypoint, lawnmower_path  # noqa: E402
from core.task_executor import TaskExecutor  # noqa: E402
from core.agent_tools import AgentToolRuntime  # noqa: E402
from core.safety_gate import SafetyGate, SafetyState  # noqa: E402
from core.vision_detector import VisionDetector  # noqa: E402
from core.auth import AuthManager, AuthConfig  # noqa: E402
from core.exceptions import ErrorHandler  # noqa: E402


class ConsoleState:
    def __init__(self) -> None:
        self.started_at = time.time()
        self.task_status = "idle"
        self.current_task = "等待任务"
        self.plan: list[dict[str, str]] = []
        self.events: list[dict[str, str]] = []
        self.trail: list[dict[str, float]] = []
        self.route: list[dict[str, float]] = []
        self.search_area: dict[str, float] | None = None
        self.targets: list[dict[str, float | str]] = []
        self.planner = TaskPlanner()
        self.airsim = AirSimAdapter()
        self.executor = TaskExecutor(self.airsim)
        self.safety_gate = SafetyGate()
        self.safety_gate.set_state_checker(self._safety_state_checker)
        self.detector = VisionDetector()
        self.agent_tools = AgentToolRuntime(self.airsim, self.executor, self.detector)
        self.auth = AuthManager(AuthConfig(enabled=False))
        self.video_stream = VideoStream(self.airsim)
        self.latest_detection: dict[str, Any] = {
            "ok": False,
            "detections": [],
            "message": "尚未运行目标检测。",
            "target": "",
            "camera": "",
        }
        self._last_progress_status = ""
        self._last_progress_message = ""
        self._last_progress_waypoint = (-1, -1)
        self._last_progress_heartbeat_ts = 0.0
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.log("AeroMind Console 已启动，等待 AirSim/UE 视频输入。", "INFO")

    def log(self, message: str, level: str = "INFO") -> None:
        now = datetime.now()
        item = {
            "time": now.strftime("%H:%M:%S"),
            "ts": now.timestamp(),
            "level": level,
            "message": message,
        }
        self.events.insert(0, item)
        self.events = self.events[:80]
        try:
            with LOG_FILE.open("a", encoding="utf-8") as handle:
                handle.write(f"{now.isoformat(timespec='seconds')} {level.upper():<7} {message}\n")
        except Exception:
            pass

    def snapshot(self) -> dict[str, Any]:
        telemetry = self.airsim.telemetry()
        connected = self.airsim.is_connected()
        if connected:
            self.video_stream.start()
        if connected:
            collision = self.airsim.collision_status()
            obstacle = self.airsim.obstacle_status()
            distances = self.airsim.distance_sensors_status()
            obstacle_points = self._sample_obstacle_points(
                self.airsim.lidar_obstacle_points_world(
                    range_m=35.0,
                    vertical_window_m=2.5,
                    vertical_down_m=1.4,
                    denoise_cell_m=2.0,
                    min_points_per_cell=3,
                )
            )
            uavs = self._get_all_uav_telemetry()
        else:
            collision = CollisionStatus()
            obstacle = ObstacleStatus(error=self.airsim.last_error)
            distances = DistanceSensorsStatus(error=self.airsim.last_error)
            obstacle_points = []
            uavs = []
        video_ready = self.video_stream.is_ready()
        video_mode = "airsim_front" if connected else "placeholder"
        agent_progress = self.agent_tools.mission_progress()
        self._sync_agent_progress_events(agent_progress)
        self._sync_terminal_agent_state(agent_progress)
        if connected:
            self._update_trail(float(telemetry.x), float(telemetry.y), float(telemetry.z))
        return {
            "connected": connected,
            "server": {
                "started_at": self.started_at,
                "started_at_iso": datetime.fromtimestamp(self.started_at).isoformat(timespec="seconds"),
            },
            "video": {
                "mode": video_mode,
                "front_camera_url": "/video/front",
                "message": "AirSim 前视相机在线。" if video_ready else "相机视频流尚未连接。",
                "camera_name": self.airsim.last_camera_name,
                "selected_camera_name": self.airsim.active_camera_name,
                "camera_candidates": list(self.airsim.camera_candidates),
                "image_type": self.airsim.last_image_type,
            },
            "vehicle": self.airsim.vehicle_options(),
            "uav": telemetry.to_api(),
            "uavs": uavs,
            "task": {
                "status": self.task_status,
                "title": self.current_task,
                "plan": self.plan,
                "agent_progress": agent_progress,
            },
            "events": self.events,
            "map": {
                "home": {"x": 0.0, "y": 0.0},
                "uav": {"x": telemetry.x, "y": telemetry.y, "z": telemetry.z},
                "uavs": uavs,
                "trail": self.trail[-240:],
                "route": self._progress_route(agent_progress) or self.route,
                "search_area": self.search_area,
                "targets": self.targets,
                "obstacles": obstacle_points,
                "blocked_zones": self._assessment_blocked_zones(agent_progress),
            },
            "uptime_s": round(time.time() - self.started_at, 1),
            "airsim_error": self.airsim.last_error,
            "safety": {
                "collision": collision.to_api(),
                "obstacle": obstacle.to_api(),
                "distance_sensors": distances.to_api(),
                "lidar": {
                    "world_points": len(obstacle_points),
                    "sample_limit": 220,
                },
                "limits": {
                    "max_speed_mps": self.safety_gate.limits.max_speed_mps,
                    "min_altitude_m": self.safety_gate.limits.min_altitude_m,
                    "max_altitude_m": self.safety_gate.limits.max_altitude_m,
                },
            },
            "vision": {
                "detector": self.detector.status(),
                "latest_detection": self.latest_detection,
                },
            }

    def telemetry_snapshot(self) -> dict[str, Any]:
        telemetry = self.airsim.telemetry()
        connected = self.airsim.is_connected()
        uavs = self._get_all_uav_telemetry() if connected else []
        if connected:
            self._update_trail(float(telemetry.x), float(telemetry.y), float(telemetry.z))
        return {
            "connected": connected,
            "vehicle": self.airsim.vehicle_options(),
            "uav": telemetry.to_api(),
            "uavs": uavs,
            "map": {
                "uav": {"x": telemetry.x, "y": telemetry.y, "z": telemetry.z, "name": telemetry.name},
                "uavs": uavs,
                "trail": self.trail[-240:],
            },
            "airsim_error": self.airsim.last_error,
        }

    @staticmethod
    def _sample_obstacle_points(points: list[tuple[float, float]], limit: int = 220) -> list[dict[str, float]]:
        if not points:
            return []
        step = max(1, len(points) // max(1, limit))
        sampled = points[::step][:limit]
        return [{"x": float(x), "y": float(y)} for x, y in sampled]

    @staticmethod
    def _assessment_blocked_zones(agent_progress: dict[str, Any]) -> list[dict[str, Any]]:
        assessment = agent_progress.get("area_assessment")
        if not isinstance(assessment, dict):
            return []
        zones = assessment.get("blocked_zones")
        return zones if isinstance(zones, list) else []

    @staticmethod
    def _progress_route(agent_progress: dict[str, Any]) -> list[dict[str, float]]:
        route = agent_progress.get("replanned_route")
        return route if isinstance(route, list) else []

    def _sync_terminal_agent_state(self, agent_progress: dict[str, Any]) -> None:
        if self.task_status != "running":
            return
        status = str(agent_progress.get("status", ""))
        terminal_statuses = {
            "completed",
            "recovered",
            "stuck_recovered",
            "collision",
            "blocked",
            "timeout",
            "failed",
        }
        if status in terminal_statuses:
            self.task_status = "stopped" if status != "completed" else "completed"
            message = agent_progress.get("message")
            if isinstance(message, str) and message:
                self.current_task = message

    def _sync_agent_progress_events(self, agent_progress: dict[str, Any]) -> None:
        status = str(agent_progress.get("status", "") or "")
        message = str(agent_progress.get("message", "") or "")
        current = int(agent_progress.get("current_waypoint_index") or 0)
        total = int(agent_progress.get("total_waypoints") or 0)
        waypoint = (current, total)
        telemetry = self.airsim.telemetry()
        distance_to_wp = agent_progress.get("distance_to_waypoint_m")
        obstacle = agent_progress.get("obstacle") if isinstance(agent_progress.get("obstacle"), dict) else {}
        distances = agent_progress.get("distance_sensors") if isinstance(agent_progress.get("distance_sensors"), dict) else {}
        planner = agent_progress.get("planner") if isinstance(agent_progress.get("planner"), dict) else {}
        risk = str(obstacle.get("risk_level", "-") or "-")
        nearest = obstacle.get("nearest_distance_m")
        front = distances.get("front_m")
        planner_ok = planner.get("ok")
        planner_phase = str(planner.get("phase", "") or "")
        now_ts = time.time()
        should_log = False
        if status and status != self._last_progress_status:
            should_log = True
        elif message and message != self._last_progress_message:
            should_log = True
        elif total > 0 and waypoint != self._last_progress_waypoint and (current <= 1 or current == total or current % 2 == 0):
            should_log = True
        elif status in {"taking_off", "pre_scanning", "running", "avoiding"} and (now_ts - self._last_progress_heartbeat_ts) >= 3.0:
            should_log = True
        if should_log and (status or message):
            suffix = f" | waypoint {current}/{total}" if total > 0 else ""
            dist_text = f"{float(distance_to_wp):.1f}m" if isinstance(distance_to_wp, (int, float)) else "-"
            nearest_text = f"{float(nearest):.1f}m" if isinstance(nearest, (int, float)) else "-"
            front_text = f"{float(front):.1f}m" if isinstance(front, (int, float)) else "-"
            planner_text = "ok" if planner_ok is True else "blocked" if planner_ok is False else "wait"
            detail = (
                f" alt={float(telemetry.altitude_m):.1f}m"
                f" speed={float(telemetry.speed_mps):.1f}m/s"
                f" d2wp={dist_text}"
                f" risk={risk}"
                f" obs={nearest_text}"
                f" front={front_text}"
                f" planner={planner_text}"
            )
            if planner_phase:
                detail += f"/{planner_phase}"
            self.log(f"Mission progress: {status or '-'} | {message or '-'}{suffix} |{detail}", "MISSION")
            self._last_progress_heartbeat_ts = now_ts
        self._last_progress_status = status
        self._last_progress_message = message
        self._last_progress_waypoint = waypoint

    def reconnect_airsim(self) -> dict[str, Any]:
        if self.airsim.connect():
            self.video_stream.start()
            self.log("AirSim connection established.", "AIRSIM")
        else:
            self.log(f"AirSim connection failed: {self.airsim.last_error}", "WARN")
        return self.snapshot()

    def probe_cameras(self) -> dict[str, Any]:
        results = self.airsim.probe_cameras()
        self.log(f"Camera probe completed: {results}", "AIRSIM")
        return {
            "connected": self.airsim.connected,
            "vehicle_name": self.airsim.vehicle_name,
            "selected_camera_name": self.airsim.active_camera_name,
            "results": results,
            "last_error": self.airsim.last_error,
        }

    def camera_options(self) -> dict[str, Any]:
        return self.airsim.camera_options()

    def vehicle_options(self) -> dict[str, Any]:
        return self.airsim.vehicle_options()

    def select_vehicle(self, payload: dict[str, Any]) -> dict[str, Any]:
        vehicle = str(payload.get("vehicle", "")).strip()
        try:
            self.airsim.switch_vehicle(vehicle)
            self.video_stream.reset()
            self.trail = []
            self.log(f"Vehicle selected: {vehicle}", "AIRSIM")
            return {
                "ok": True,
                "vehicle": self.airsim.vehicle_options(),
                "state": self.snapshot(),
            }
        except Exception as exc:
            self.log(f"Vehicle switch failed: {exc}", "ERROR")
            return {
                "ok": False,
                "detail": str(exc),
                "vehicle": self.airsim.vehicle_options(),
                "state": self.snapshot(),
            }

    def select_camera(self, payload: dict[str, Any]) -> dict[str, Any]:
        camera = str(payload.get("camera", "")).strip()
        try:
            result = self.airsim.select_camera(camera)
            self.video_stream.reset()
            self.log(f"Camera selected: {camera}", "AIRSIM")
            return {
                "ok": True,
                "camera": result,
                "state": self.snapshot(),
            }
        except Exception as exc:
            self.log(f"Camera switch failed: {exc}", "ERROR")
            return {
                "ok": False,
                "detail": str(exc),
                "camera": self.airsim.camera_options(),
                "state": self.snapshot(),
            }

    def logs(self) -> dict[str, Any]:
        file_lines: list[str] = []
        if LOG_FILE.exists():
            try:
                file_lines = LOG_FILE.read_text(encoding="utf-8", errors="replace").splitlines()[-300:]
            except Exception as exc:
                file_lines = [f"Failed to read log file: {exc}"]
        return {
            "events": self.events,
            "file": str(LOG_FILE),
            "lines": file_lines,
        }

    def clear_logs(self) -> dict[str, Any]:
        self.events.clear()
        try:
            LOG_FILE.write_text("", encoding="utf-8")
        except Exception:
            pass
        self.log("Logs cleared.", "INFO")
        return self.logs()

    def list_tools(self) -> dict[str, Any]:
        return {
            "tools": self.agent_tools.list_tools(),
            "safety": {
                "limits": {
                    "min_altitude_m": self.safety_gate.limits.min_altitude_m,
                    "max_altitude_m": self.safety_gate.limits.max_altitude_m,
                    "max_speed_mps": self.safety_gate.limits.max_speed_mps,
                    "boundary": {
                        "x_min": self.safety_gate.limits.boundary_x_min,
                        "x_max": self.safety_gate.limits.boundary_x_max,
                        "y_min": self.safety_gate.limits.boundary_y_min,
                        "y_max": self.safety_gate.limits.boundary_y_max,
                    },
                    "critical_tools": list(self.safety_gate.limits.critical_tools),
                }
            },
        }

    def call_tool(self, payload: dict[str, Any]) -> dict[str, Any]:
        tool = str(payload.get("tool", ""))
        args = payload.get("args", {})
        if not isinstance(args, dict):
            args = {}
        decision = self.safety_gate.validate(tool, args, connected=self.airsim.is_connected())
        accepted = decision.accepted
        if not accepted:
            self.log(f"Tool rejected: {tool} | {decision.reason}", "SAFE")
            return {
                "accepted": False,
                "safety": decision.to_api(),
                "result": None,
                "state": self.snapshot(),
            }

        self.log(f"Tool accepted: {decision.tool} {decision.args}", "TOOL")
        result = self.agent_tools.call(decision.tool, decision.args)
        self.log(f"Tool result: {result.tool} | ok={result.ok} | {result.message}", "TOOL" if result.ok else "ERROR")
        self._reflect_tool_on_state(result.tool, decision.args, result.ok, result.data)
        if result.tool == "detect_objects":
            self._store_detection_result(result.data, result.ok, result.message)
        return {
            "accepted": True,
            "safety": decision.to_api(),
            "result": result.to_api(),
            "state": self.snapshot(),
        }

    def detect_latest(self, payload: dict[str, Any]) -> dict[str, Any]:
        target = str(payload.get("target", "object") or "object")
        camera = str(payload.get("camera", self.airsim.active_camera_name) or self.airsim.active_camera_name)
        result = self.call_tool({"tool": "detect_objects", "args": {"target": target, "camera": camera}})
        return result

    def preview_task(self, text: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        draft = self._mission_draft(text, params)
        return draft

    def run_task(self, text: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        draft = self._mission_draft(text, params)
        mission = self.planner.plan(text)
        self.executor.start(mission)
        self.current_task = mission.task
        self.plan = [step.to_api() for step in mission.plan]
        draft_map = draft.get("map")
        if isinstance(draft_map, dict):
            self.route = draft_map.get("route", []) if isinstance(draft_map.get("route"), list) else []
            self.search_area = draft_map.get("search_area") if isinstance(draft_map.get("search_area"), dict) else None
            self.targets = []
        else:
            self._set_mission_map(mission)
        self.task_status = "planning"
        self.log(f"Mission received: {self.current_task}", "TASK")
        self.log(f"Intent resolved: {mission.intent}", "PLAN")
        try:
            nlp = mission.raw.get("nlp", {}) if isinstance(mission.raw, dict) else {}
            scores = nlp.get("intent_scores")
            if isinstance(scores, dict):
                self.log(f"Intent score: {scores}", "PLAN")
        except Exception:
            pass
        executable = draft.get("executable")
        if isinstance(executable, dict) and executable.get("accepted"):
            tool_call = executable.get("tool_call")
            tool_call = tool_call if isinstance(tool_call, dict) else {}
            tool = str(tool_call.get("tool", ""))
            args = tool_call.get("args", {})
            args = args if isinstance(args, dict) else {}
            result = self.agent_tools.call(tool, args)
            self.log(f"Natural-language tool dispatched: {tool} | ok={result.ok} | {result.message}", "TOOL" if result.ok else "ERROR")
            self._reflect_tool_on_state(tool, args, result.ok, result.data)
        else:
            reason = ""
            if isinstance(executable, dict):
                safety = executable.get("safety")
                if isinstance(safety, dict):
                    reason = str(safety.get("reason", ""))
            self.log(f"Structured plan generated; no executable tool dispatched. {reason}".strip(), "PLAN")
        return self.snapshot()

    def pause_task(self) -> dict[str, Any]:
        return self.flight_hover(message="Mission paused.")

    def stop_task(self) -> dict[str, Any]:
        return self.flight_stop(message="Mission stopped.")

    def rtl(self) -> dict[str, Any]:
        mission = self.planner.return_to_launch_plan()
        self.task_status = "returning"
        self.current_task = mission.task
        self.plan = [step.to_api() for step in mission.plan]
        self.route = [{"x": 0.0, "y": 0.0, "z": 0.0}]
        try:
            self.executor.return_to_launch()
            self.log("Return-to-launch command sent.", "SAFE")
        except Exception as exc:
            self.log(f"Return-to-launch failed: {exc}", "ERROR")
        return self.snapshot()

    def flight_takeoff(self, altitude_m: float = 8.0) -> dict[str, Any]:
        try:
            self.executor.takeoff(altitude_m=altitude_m)
            self.task_status = "holding"
            self.current_task = f"Holding at {altitude_m:.1f}m"
            self.log(f"Takeoff complete; holding altitude={altitude_m:.1f}m", "FLIGHT")
        except Exception as exc:
            self.log(f"Takeoff failed: {exc}", "ERROR")
            self.log(traceback.format_exc(), "TRACE")
        return self.snapshot()

    def flight_hover(self, message: str = "Hover command sent.") -> dict[str, Any]:
        try:
            self.executor.hover()
            self.task_status = "paused"
            self.log(message, "FLIGHT")
        except Exception as exc:
            self.log(f"Hover failed: {exc}", "ERROR")
            self.log(traceback.format_exc(), "TRACE")
        return self.snapshot()

    def flight_land(self) -> dict[str, Any]:
        try:
            self.executor.land()
            self.task_status = "stopped"
            self.current_task = "Land"
            self.log("Land command sent.", "FLIGHT")
        except Exception as exc:
            self.log(f"Land failed: {exc}", "ERROR")
            self.log(traceback.format_exc(), "TRACE")
        return self.snapshot()

    def flight_stop(self, message: str = "Stop command sent.") -> dict[str, Any]:
        try:
            self.executor.stop()
            self.task_status = "stopped"
            self.log(message, "FLIGHT")
        except Exception as exc:
            self.log(f"Stop failed: {exc}", "ERROR")
            self.log(traceback.format_exc(), "TRACE")
        return self.snapshot()

    def _mission_draft(self, text: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        mission = self.planner.plan(text)
        params = params if isinstance(params, dict) else {}
        tool_call = self._mission_tool_call(mission, params)
        route = self._preview_route(mission, tool_call)
        map_payload = self._preview_map(mission, tool_call, route)
        executable: dict[str, Any] | None = None
        if tool_call:
            decision = self.safety_gate.validate(
                str(tool_call.get("tool", "")),
                tool_call.get("args", {}),
                connected=self.airsim.is_connected(),
            )
            accepted = decision.accepted
            executable = {
                "accepted": accepted,
                "tool_call": {
                    "tool": decision.tool,
                    "args": decision.args if accepted else tool_call.get("args", {}),
                },
                "safety": decision.to_api(),
            }
        preflight = self._preflight_checks(mission, tool_call, executable, route)
        return {
            **mission.to_api(),
            "target": self._draft_target(mission, tool_call),
            "altitude_m": self._draft_altitude(mission, tool_call),
            "area": map_payload["search_area"],
            "executable": executable,
            "route": route,
            "map": map_payload,
            "preflight": preflight,
            "preview": {
                "waypoint_count": len(route),
                "estimated_distance_m": self._route_distance(route),
                "connected": self.airsim.is_connected(),
                "message": self._preview_message(mission.intent, executable),
            },
        }

    def _preflight_checks(
        self,
        mission: Any,
        tool_call: dict[str, Any] | None,
        executable: dict[str, Any] | None,
        route: list[dict[str, float]],
    ) -> dict[str, Any]:
        checks: list[dict[str, str]] = []
        connected = self.airsim.is_connected()
        checks.append({"key": "airsim_connection", "label": "AirSim 连接", "status": "ok" if connected else "warn", "detail": "AirSim 已连接" if connected else "AirSim 离线，启动后将自动重连。"})
        if tool_call is None:
            checks.append({"key": "tool_mapping", "label": "工具映射", "status": "error", "detail": f"意图 {getattr(mission, 'intent', 'unknown')} 暂未映射到可执行工具。"})
            return {"overall": "blocked", "checks": checks}
        checks.append({"key": "tool_mapping", "label": "工具映射", "status": "ok", "detail": f"将执行 {tool_call.get('tool')}。"})
        if executable is not None and not executable.get("accepted"):
            safety = executable.get("safety")
            reason = safety.get("reason") if isinstance(safety, dict) else ""
            checks.append({"key": "safety_gate", "label": "安全门", "status": "error", "detail": str(reason or "工具调用未通过安全门。")})
            return {"overall": "blocked", "checks": checks}

        overall = "ready" if connected else "caution"
        return {"overall": overall, "checks": checks}

    def _preflight_area_error(self, area: dict[str, Any]) -> str:
        try:
            x_min = float(area.get("x_min", 0.0))
            x_max = float(area.get("x_max", 60.0))
            y_min = float(area.get("y_min", -20.0))
            y_max = float(area.get("y_max", 20.0))
        except (TypeError, ValueError):
            return "Area contains non-numeric coordinates."
        limits = self.safety_gate.limits
        if x_min >= x_max:
            return "area.x_min 必须小于 area.x_max。"
        if y_min >= y_max:
            return "area.y_min 必须小于 area.y_max。"
        if x_min < limits.boundary_x_min or x_max > limits.boundary_x_max:
            return "任务区域 X 范围超出边界。"
        if y_min < limits.boundary_y_min or y_max > limits.boundary_y_max:
            return "任务区域 Y 范围超出边界。"
        return ""

    @staticmethod
    def _draft_target(mission: Any, tool_call: dict[str, Any] | None) -> str:
        if tool_call:
            args = tool_call.get("args", {})
            if isinstance(args, dict) and args.get("target"):
                return str(args["target"])
        return str(getattr(mission, "target", "unknown"))

    @staticmethod
    def _draft_altitude(mission: Any, tool_call: dict[str, Any] | None) -> float:
        if tool_call:
            args = tool_call.get("args", {})
            if isinstance(args, dict):
                try:
                    return float(args.get("altitude_m", getattr(mission, "altitude_m", 8.0)))
                except (TypeError, ValueError):
                    pass
        return float(getattr(mission, "altitude_m", 8.0))

    def _mission_tool_call(self, mission: Any, params: dict[str, Any]) -> dict[str, Any] | None:
        intent = str(getattr(mission, "intent", ""))
        mission_raw = getattr(mission, "raw", {}) if isinstance(getattr(mission, "raw", {}), dict) else {}
        nlp = mission_raw.get("nlp", {}) if isinstance(mission_raw.get("nlp", {}), dict) else {}
        goal = nlp.get("goal", {}) if isinstance(nlp.get("goal", {}), dict) else {}
        default_speed = float(nlp.get("speed_mps", 2.0) or 2.0)
        default_count = int(nlp.get("count", 3) or 3)
        default_shape = str(nlp.get("shape", "v") or "v")
        if intent == "takeoff":
            return {
                "tool": "takeoff",
                "args": {"altitude_m": self._float_param(params, "altitude_m", float(getattr(mission, "altitude_m", 8.0)))},
            }
        if intent == "image_collection":
            area = params.get("area") if isinstance(params.get("area"), dict) else None
            mission_area = getattr(mission, "area", None)
            area_payload = area or (mission_area.to_api() if mission_area is not None else MissionArea(x_max=40.0, y_min=-15.0, y_max=15.0).to_api())
            return {
                "tool": "collect_images",
                "args": {
                    "area": area_payload,
                    "altitude_m": self._float_param(params, "altitude_m", float(getattr(mission, "altitude_m", 8.0))),
                    "spacing_m": self._float_param(params, "spacing_m", 8.0),
                    "speed_mps": self._float_param(params, "speed_mps", max(0.8, default_speed)),
                    "camera": str(params.get("camera", "front_center")),
                    "dataset_name": str(params.get("dataset_name", f"collect_{int(time.time())}")),
                    "max_images": int(self._float_param(params, "max_images", 200.0)),
                    "capture_interval_s": self._float_param(params, "capture_interval_s", 1.0),
                    "avoidance": self._bool_param(params, "avoidance", True),
                    "planned_avoidance": self._bool_param(params, "planned_avoidance", True),
                    "obstacle_distance_m": self._float_param(params, "obstacle_distance_m", 8.0),
                    "avoidance_offset_m": self._float_param(params, "avoidance_offset_m", 6.0),
                    "scan_margin_m": self._float_param(params, "scan_margin_m", 4.0),
                    "pre_scan": self._bool_param(params, "pre_scan", True),
                    "pre_scan_stop_on_high_risk": self._bool_param(params, "pre_scan_stop_on_high_risk", False),
                },
            }
        if intent == "area_search":
            area = params.get("area") if isinstance(params.get("area"), dict) else None
            mission_area = getattr(mission, "area", None)
            area_payload = area or (mission_area.to_api() if mission_area is not None else MissionArea().to_api())
            altitude_m = self._float_param(params, "altitude_m", float(getattr(mission, "altitude_m", 8.0)))
            return {
                "tool": "search_area",
                "args": {
                    "target": str(params.get("target") or getattr(mission, "target", "object") or "object"),
                    "area": area_payload,
                    "altitude_m": altitude_m,
                    "spacing_m": self._float_param(params, "spacing_m", 10.0),
                    "speed_mps": self._float_param(params, "speed_mps", max(0.8, default_speed)),
                    "avoidance": self._bool_param(params, "avoidance", True),
                    "planned_avoidance": self._bool_param(params, "planned_avoidance", True),
                    "obstacle_distance_m": self._float_param(params, "obstacle_distance_m", 8.0),
                    "avoidance_offset_m": self._float_param(params, "avoidance_offset_m", 6.0),
                    "scan_margin_m": self._float_param(params, "scan_margin_m", 4.0),
                    "pre_scan": self._bool_param(params, "pre_scan", True),
                    "pre_scan_stop_on_high_risk": self._bool_param(params, "pre_scan_stop_on_high_risk", True),
                },
            }
        if intent in {"autonomous_navigation", "obstacle_aware_navigation"}:
            altitude_m = self._float_param(params, "altitude_m", float(getattr(mission, "altitude_m", 8.0)))
            area = params.get("area") if isinstance(params.get("area"), dict) else {}
            distance_m = self._float_param(area, "x_max", self._float_param(params, "distance_m", float(goal.get("x", 60.0))))
            distance_m = max(5.0, distance_m)
            return {
                "tool": "autonomous_nav",
                "args": {
                    "x": self._float_param(params, "x", distance_m),
                    "y": self._float_param(params, "y", float(goal.get("y", 0.0))),
                    "z": -abs(altitude_m),
                    "speed_mps": self._float_param(params, "speed_mps", max(0.8, default_speed)),
                    "replan_interval_s": self._float_param(params, "replan_interval_s", 1.0),
                    "control_dt_s": self._float_param(params, "control_dt_s", 0.2),
                    "lookahead_m": self._float_param(params, "lookahead_m", 3.0),
                    "timeout_s": self._float_param(params, "timeout_s", 120.0),
                },
            }
        if intent == "path_planning":
            altitude_m = self._float_param(params, "altitude_m", float(getattr(mission, "altitude_m", 8.0)))
            area = params.get("area") if isinstance(params.get("area"), dict) else {}
            distance_m = self._float_param(area, "x_max", self._float_param(params, "distance_m", float(goal.get("x", 60.0))))
            distance_m = max(5.0, distance_m)
            return {
                "tool": "waypoint_route",
                "args": {
                    "waypoints": [
                        {"x": distance_m / 2.0, "y": float(goal.get("y", 0.0)), "z": -abs(altitude_m)},
                        {"x": distance_m, "y": float(goal.get("y", 0.0)), "z": -abs(altitude_m)},
                    ],
                    "speed_mps": self._float_param(params, "speed_mps", max(0.8, default_speed)),
                    "avoidance": self._bool_param(params, "avoidance", True),
                    "hold_at_end": True,
                },
            }
        if intent == "formation_flight":
            altitude_m = self._float_param(params, "altitude_m", float(getattr(mission, "altitude_m", 8.0)))
            area = params.get("area") if isinstance(params.get("area"), dict) else {}
            distance_m = self._float_param(area, "x_max", self._float_param(params, "distance_m", 40.0))
            return {
                "tool": "formation_flight",
                "args": {
                    "count": int(self._float_param(params, "count", float(default_count))),
                    "shape": str(params.get("shape", default_shape)),
                    "distance_m": max(5.0, distance_m),
                    "altitude_m": altitude_m,
                    "spacing_m": self._float_param(params, "formation_spacing_m", 6.0),
                    "speed_mps": self._float_param(params, "speed_mps", max(0.8, default_speed)),
                    "avoidance": self._bool_param(params, "avoidance", True),
                },
            }
        if intent == "return_to_launch":
            return {"tool": "return_home", "args": {"safe_altitude_m": self._float_param(params, "altitude_m", 8.0)}}
        return None

    def _preview_route(self, mission: Any, tool_call: dict[str, Any] | None) -> list[dict[str, float]]:
        if tool_call and tool_call.get("tool") in {"search_area", "collect_images"}:
            args = tool_call.get("args", {})
            args = args if isinstance(args, dict) else {}
            area = args.get("area", {})
            area = area if isinstance(area, dict) else {}
            mission_area = MissionArea(
                x_min=float(area.get("x_min", 0.0)),
                x_max=float(area.get("x_max", 60.0)),
                y_min=float(area.get("y_min", -20.0)),
                y_max=float(area.get("y_max", 20.0)),
            )
            return [
                point.to_api()
                for point in lawnmower_path(
                    mission_area,
                    float(args.get("altitude_m", getattr(mission, "altitude_m", 8.0))),
                    spacing_m=float(args.get("spacing_m", 10.0)),
                )
            ]
        if tool_call and tool_call.get("tool") == "waypoint_route":
            args = tool_call.get("args", {})
            args = args if isinstance(args, dict) else {}
            waypoints = args.get("waypoints", [])
            return [
                {"x": float(point.get("x", 0.0)), "y": float(point.get("y", 0.0)), "z": float(point.get("z", -8.0))}
                for point in waypoints
                if isinstance(point, dict)
            ]
        if tool_call and tool_call.get("tool") == "autonomous_nav":
            args = tool_call.get("args", {})
            args = args if isinstance(args, dict) else {}
            return [
                {
                    "x": float(args.get("x", 0.0)),
                    "y": float(args.get("y", 0.0)),
                    "z": float(args.get("z", -8.0)),
                }
            ]
        if tool_call and tool_call.get("tool") == "formation_flight":
            args = tool_call.get("args", {})
            args = args if isinstance(args, dict) else {}
            z = -abs(float(args.get("altitude_m", getattr(mission, "altitude_m", 8.0))))
            distance_m = float(args.get("distance_m", 40.0))
            return [Waypoint(0.0, 0.0, z).to_api(), Waypoint(distance_m, 0.0, z).to_api()]
        if getattr(mission, "area", None) is not None:
            return [point.to_api() for point in lawnmower_path(mission.area, mission.altitude_m)]
        return []

    def _preview_map(self, mission: Any, tool_call: dict[str, Any] | None, route: list[dict[str, float]]) -> dict[str, Any]:
        area = None
        if tool_call and tool_call.get("tool") in {"search_area", "collect_images"}:
            args = tool_call.get("args", {})
            args = args if isinstance(args, dict) else {}
            area = args.get("area")
        if area is None and getattr(mission, "area", None) is not None:
            area = mission.area.to_api()
        uavs = self._get_all_uav_telemetry() if self.airsim.is_connected() else []
        return {
            "home": {"x": 0.0, "y": 0.0},
            "uav": {"x": 0.0, "y": 0.0, "z": 0.0},
            "uavs": uavs,
            "trail": [],
            "route": route,
            "search_area": area,
            "targets": [],
            "obstacles": [],
            "blocked_zones": [],
        }

    @staticmethod
    def _route_distance(route: list[dict[str, float]]) -> float:
        total = 0.0
        for previous, current in zip(route, route[1:]):
            dx = float(current.get("x", 0.0)) - float(previous.get("x", 0.0))
            dy = float(current.get("y", 0.0)) - float(previous.get("y", 0.0))
            dz = float(current.get("z", 0.0)) - float(previous.get("z", 0.0))
            total += (dx * dx + dy * dy + dz * dz) ** 0.5
        return round(total, 1)

    @staticmethod
    def _preview_message(intent: str, executable: dict[str, Any] | None) -> str:
        if executable is None:
            return f"意图 '{intent}' 暂未接入直接工具调用。"
        if executable.get("accepted"):
            tool_call = executable.get("tool_call")
            tool = tool_call.get("tool") if isinstance(tool_call, dict) else "tool"
            return f"已通过安全门，可下发 '{tool}'。"
        safety = executable.get("safety")
        reason = safety.get("reason") if isinstance(safety, dict) else ""
        return str(reason or "工具调用被安全门拒绝。")

    @staticmethod
    def _float_param(params: dict[str, Any], key: str, default: float) -> float:
        try:
            return float(params.get(key, default))
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _bool_param(params: dict[str, Any], key: str, default: bool) -> bool:
        value = params.get(key, default)
        if isinstance(value, str):
            return value.lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _reflect_tool_on_state(self, tool: str, args: dict[str, Any], ok: bool, result_data: dict[str, Any] | None = None) -> None:
        if not ok:
            return
        if tool == "takeoff":
            altitude = float(args.get("altitude_m", 8.0))
            self.task_status = "holding"
            self.current_task = f"Agent holding at {altitude:.1f}m"
        elif tool == "hover":
            self.task_status = "paused"
            self.current_task = "Agent hover"
        elif tool == "stop":
            self.task_status = "stopped"
            self.current_task = "Agent stop"
        elif tool == "land":
            self.task_status = "stopped"
            self.current_task = "Agent land"
        elif tool == "return_home":
            self.task_status = "returning"
            self.current_task = "Agent return home"
            safe_z = -abs(float(args.get("safe_altitude_m", 8.0)))
            self.route = [{"x": 0.0, "y": 0.0, "z": safe_z}]
        elif tool == "goto_local":
            self.task_status = "running"
            self.current_task = "Agent goto local waypoint"
            self.route = [
                {
                    "x": float(args.get("x", 0.0)),
                    "y": float(args.get("y", 0.0)),
                    "z": float(args.get("z", -8.0)),
                }
            ]
        elif tool == "autonomous_nav":
            self.task_status = "running"
            self.current_task = "自主导航"
            self.route = [
                {
                    "x": float(args.get("x", 0.0)),
                    "y": float(args.get("y", 0.0)),
                    "z": float(args.get("z", -8.0)),
                }
            ]
            self.search_area = None
        elif tool == "search_area":
            self.task_status = "running"
            self.current_task = f"Agent search area: {args.get('target', 'object')}"
            area = (result_data or {}).get("effective_area") or args.get("area")
            if isinstance(area, dict):
                self.search_area = {
                    "x_min": float(area.get("x_min", 0.0)),
                    "x_max": float(area.get("x_max", 60.0)),
                    "y_min": float(area.get("y_min", -20.0)),
                    "y_max": float(area.get("y_max", 20.0)),
                }
            route = (result_data or {}).get("route")
            if isinstance(route, list):
                self.route = route
        elif tool == "collect_images":
            self.task_status = "running"
            dataset_name = (result_data or {}).get("dataset_name") or args.get("dataset_name", "dataset")
            self.current_task = f"采集训练图像: {dataset_name}"
            area = (result_data or {}).get("effective_area") or args.get("area")
            if isinstance(area, dict):
                self.search_area = {
                    "x_min": float(area.get("x_min", 0.0)),
                    "x_max": float(area.get("x_max", 40.0)),
                    "y_min": float(area.get("y_min", -15.0)),
                    "y_max": float(area.get("y_max", 15.0)),
                }
            route = (result_data or {}).get("route")
            if isinstance(route, list):
                self.route = route
        elif tool == "waypoint_route":
            self.task_status = "running"
            self.current_task = "航点飞行"
            route = (result_data or {}).get("route") or args.get("waypoints")
            if isinstance(route, list):
                self.route = route
                self.search_area = None
        elif tool == "formation_flight":
            self.task_status = "running"
            self.current_task = "编队飞行"
            route = (result_data or {}).get("route")
            if isinstance(route, list):
                self.route = route
                self.search_area = None

    def _store_detection_result(self, data: dict[str, Any] | None, ok: bool, message: str) -> None:
        payload = dict(data or {})
        detections = payload.get("detections")
        detections = detections if isinstance(detections, list) else []
        self.latest_detection = {
            "ok": ok,
            "target": str(payload.get("target", "")),
            "camera": str(payload.get("camera", "")),
            "bytes": int(payload.get("bytes", 0) or 0),
            "detections": detections,
            "model_path": str(payload.get("model_path", "")),
            "backend": str(payload.get("backend", "")),
            "elapsed_ms": int(payload.get("elapsed_ms", 0) or 0),
            "message": message,
        }
        if detections:
            self.log(f"Detection completed: {len(detections)} object(s).", "VISION")
        else:
            self.log(f"Detection completed: {message}", "VISION" if ok else "WARN")

    def _get_all_uav_telemetry(self) -> list[dict[str, float | str]]:
        uavs = []
        for name in self.airsim.vehicle_names:
            try:
                telemetry = self.airsim.telemetry(vehicle_name=name)
                uavs.append(telemetry.to_api())
            except Exception:
                pass
        return uavs

    def _update_trail(self, x: float, y: float, z: float) -> None:
        point = {"x": x, "y": y, "z": z}
        if not self.trail:
            self.trail.append(point)
            return
        last = self.trail[-1]
        distance_sq = (last["x"] - x) ** 2 + (last["y"] - y) ** 2 + (last["z"] - z) ** 2
        if distance_sq >= 0.25:
            self.trail.append(point)
            self.trail = self.trail[-300:]

    def _set_mission_map(self, mission: Any) -> None:
        self.route = []
        self.search_area = mission.area.to_api() if getattr(mission, "area", None) else None
        self.targets = []
        if mission.area is not None:
            self.route = [point.to_api() for point in lawnmower_path(mission.area, mission.altitude_m)]
        elif mission.intent == "return_to_launch":
            self.route = [{"x": 0.0, "y": 0.0, "z": 0.0}]

    def _safety_state_checker(self) -> SafetyState:
        connected = self.airsim.is_connected()
        if not connected:
            return SafetyState()
        telemetry = self.airsim.telemetry()
        collision = self.airsim.collision_status()
        obstacle = self.airsim.obstacle_status()
        distances = self.airsim.distance_sensors_status()
        return SafetyState(
            has_collision=collision.has_collided,
            collision_object=collision.object_name,
            obstacle_risk_level=obstacle.risk_level,
            obstacle_blocked=obstacle.blocked,
            distance_emergency=distances.emergency,
            distance_direction=distances.emergency_direction,
            current_altitude_m=float(telemetry.altitude_m),
            current_speed_mps=float(telemetry.speed_mps),
            current_x=float(telemetry.x),
            current_y=float(telemetry.y),
            current_z=float(telemetry.z),
        )


STATE = ConsoleState()


class MissionConsoleServer(ThreadingHTTPServer):
    pass


class Handler(BaseHTTPRequestHandler):
    def _client_ip(self) -> str:
        return self.client_address[0] if self.client_address else "unknown"

    def _auth_headers(self) -> dict[str, str]:
        return {k: v for k, v in self.headers.items()}

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        public_paths = {"/", "/index.html", "/static/", "/video/front", "/camera/latest"}
        if any(path.startswith(p) for p in public_paths):
            self._handle_public_get(path)
            return
        
        protected_paths = {
            "/api/state": STATE.snapshot,
            "/api/telemetry": STATE.telemetry_snapshot,
            "/api/camera/probe": STATE.probe_cameras,
            "/api/camera/options": STATE.camera_options,
            "/api/vehicle/options": STATE.vehicle_options,
            "/api/logs": STATE.logs,
            "/api/tools": STATE.list_tools,
            "/api/detect/latest": lambda: {"vision": STATE.latest_detection, "detector": STATE.detector.status()},
        }
        
        handler = protected_paths.get(path)
        if handler is None:
            self.send_json({"detail": "Not found"}, HTTPStatus.NOT_FOUND)
            return
        
        try:
            status, data, extra_headers = STATE.auth.authenticate(handler, self._client_ip(), self._auth_headers())
            self.send_json(data, status, extra_headers)
        except Exception as exc:
            status, data = ErrorHandler.handle_exception(exc)
            self.send_json(data, status)

    def _handle_public_get(self, path: str) -> None:
        if path == "/video/front":
            self.send_mjpeg()
            return
        if path == "/camera/latest":
            self.send_latest_camera_frame()
            return
        if path == "/":
            path = "/index.html"
        self.send_static(path)

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        payload = self.read_json()
        
        routes: dict[str, Callable[[], dict[str, Any]]] = {
            "/api/airsim/reconnect": STATE.reconnect_airsim,
            "/api/camera/probe": STATE.probe_cameras,
            "/api/camera/select": lambda: STATE.select_camera(payload),
            "/api/vehicle/select": lambda: STATE.select_vehicle(payload),
            "/api/logs": STATE.logs,
            "/api/tools/call": lambda: STATE.call_tool(payload),
            "/api/detect/latest": lambda: STATE.detect_latest(payload),
            "/api/flight/takeoff": lambda: STATE.flight_takeoff(float(payload.get("altitude_m", 8.0))),
            "/api/flight/hover": STATE.flight_hover,
            "/api/flight/land": STATE.flight_land,
            "/api/flight/stop": STATE.flight_stop,
            "/api/flight/rtl": STATE.rtl,
            "/api/task/preview": lambda: STATE.preview_task(str(payload.get("text", "")), payload.get("params", {})),
            "/api/task/run": lambda: STATE.run_task(str(payload.get("text", "")), payload.get("params", {})),
            "/api/task/pause": STATE.pause_task,
            "/api/task/stop": STATE.stop_task,
            "/api/task/rtl": STATE.rtl,
        }
        handler = routes.get(path)
        if handler is None:
            self.send_json({"detail": "Not found"}, HTTPStatus.NOT_FOUND)
            return
        
        try:
            status, data, extra_headers = STATE.auth.authenticate(handler, self._client_ip(), self._auth_headers())
            self.send_json(data, status, extra_headers)
        except Exception as exc:
            status, data = ErrorHandler.handle_exception(exc)
            self.send_json(data, status)

    def do_DELETE(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/api/logs":
            try:
                status, data, extra_headers = STATE.auth.authenticate(STATE.clear_logs, self._client_ip(), self._auth_headers())
                self.send_json(data, status, extra_headers)
            except Exception as exc:
                status, data = ErrorHandler.handle_exception(exc)
                self.send_json(data, status)
            return
        self.send_json({"detail": "Not found"}, HTTPStatus.NOT_FOUND)

    def read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length") or "0")
        if length <= 0:
            return {}
        parsed = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
        return parsed if isinstance(parsed, dict) else {}

    def send_json(self, payload: dict[str, Any], status: int = HTTPStatus.OK, extra_headers: dict[str, str] | None = None) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        try:
            self.send_response(int(status))
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for key, value in (extra_headers or {}).items():
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return

    def send_mjpeg(self) -> None:
        if not STATE.airsim.connect():
            self.send_json(
                {"detail": f"AirSim camera stream is not connected yet: {STATE.airsim.last_error}"},
                HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={STATE.video_stream.boundary}")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        try:
            for frame in STATE.video_stream.frames():
                self.wfile.write(f"--{STATE.video_stream.boundary}\r\n".encode("ascii"))
                self.wfile.write(f"Content-Type: {frame.content_type}\r\n".encode("ascii"))
                self.wfile.write(f"Content-Length: {len(frame.data)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame.data)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return

    def send_latest_camera_frame(self) -> None:
        frame = STATE.video_stream.latest_frame() or STATE.airsim.camera_frame()
        if frame is None:
            self.send_json(
                {"detail": f"AirSim camera frame is not available: {STATE.airsim.last_error}"},
                HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", frame.content_type)
            self.send_header("Content-Length", str(len(frame.data)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.end_headers()
            self.wfile.write(frame.data)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return

    def send_static(self, request_path: str) -> None:
        relative = request_path.lstrip("/")
        if relative.startswith("static/"):
            relative = relative[len("static/") :]
        target = (STATIC_DIR / relative).resolve()
        if not str(target).startswith(str(STATIC_DIR.resolve())) or not target.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        data = target.read_bytes()
        try:
            self.send_response(HTTPStatus.OK)
            content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
            if content_type.startswith("text/") or content_type in {"application/javascript", "application/json"}:
                content_type = f"{content_type}; charset=utf-8"
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return

    def log_message(self, format: str, *args: Any) -> None:
        return


def run() -> None:
    parser = argparse.ArgumentParser(description="Run AeroMind Console.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8010)
    args = parser.parse_args()
    server = MissionConsoleServer((args.host, args.port), Handler)
    print(f"AeroMind Console: http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        STATE.video_stream.stop()
        STATE.airsim.stop_reconnect_worker()
        server.server_close()


if __name__ == "__main__":
    run()
