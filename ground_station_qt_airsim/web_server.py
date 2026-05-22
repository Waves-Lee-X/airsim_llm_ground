from __future__ import annotations

import argparse
import json
import mimetypes
import re
import threading
import time
from collections import deque
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from ground_station_qt_airsim.backend import AirSimBackend
from ground_station_qt_airsim.config import GroundStationAirSimConfig, load_config
from ground_station_qt_airsim.models import TelemetryData, Waypoint


COMMAND_LABELS = {
    "arm": "解锁",
    "disarm": "上锁",
    "takeoff": "起飞",
    "land": "降落",
    "rtl": "返航",
    "hover": "悬停",
    "expert_goto_local": "专家避障指点",
    "goto_local": "直接指点",
    "stop_motion": "停止",
}

CRITICAL_COMMANDS = {"arm", "disarm", "takeoff", "land", "rtl"}


class ApiError(RuntimeError):
    def __init__(self, message: str, status: int = 400) -> None:
        super().__init__(message)
        self.status = status


class WebGroundStation:
    def __init__(self, config: GroundStationAirSimConfig) -> None:
        self.config = config
        self.logs: deque[dict[str, str]] = deque(maxlen=300)
        self.lock = threading.RLock()
        self.backend = AirSimBackend(config=config, log=self.log)
        self.selected_uav = int(config.ui.default_selected_uav)
        self.target_altitude_m = abs(float(config.ui.default_altitude_m))
        self.waypoints: list[Waypoint] = []
        self.started_at = time.time()
        self.log("Web 地面站服务已启动。", "INFO")
        self.connect(initial=True)

    def log(self, message: str, level: str = "INFO") -> None:
        self.logs.appendleft(
            {
                "time": datetime.now().strftime("%H:%M:%S"),
                "level": level.upper(),
                "message": str(message),
            }
        )

    def connect(self, *, initial: bool = False) -> bool:
        with self.lock:
            try:
                self.backend.close()
            except Exception:
                pass
            try:
                self.backend.connect()
                ids = [binding.sysid for binding in self.backend.vehicle_bindings]
                if ids and self.selected_uav not in ids:
                    self.selected_uav = ids[0]
                self.log("AirSim 后端已连接。", "INFO")
                return True
            except Exception as exc:
                self.log(f"AirSim 连接失败: {exc}", "ERROR")
                if not initial:
                    raise ApiError(str(exc), 400) from exc
                return False

    def target_z(self, altitude_m: float | None = None, z: float | None = None) -> float:
        if z is not None:
            return float(z)
        value = self.target_altitude_m if altitude_m is None else float(altitude_m)
        return -abs(value)

    def state(self) -> dict[str, Any]:
        with self.lock:
            telemetry = self.backend.refresh()
            return self._serialize_state(telemetry)

    def _serialize_state(self, telemetry: dict[int, TelemetryData]) -> dict[str, Any]:
        now = time.time()
        vehicles = []
        for tele in telemetry.values():
            vehicles.append(
                {
                    "sysid": tele.sysid,
                    "vehicle_name": tele.vehicle_name,
                    "display_name": tele.display_name,
                    "online": tele.is_online(now, 3.0),
                    "lat": tele.lat,
                    "lon": tele.lon,
                    "alt": tele.alt,
                    "speed": tele.speed,
                    "yaw_deg": tele.yaw_deg,
                    "pitch_deg": tele.pitch_deg,
                    "roll_deg": tele.roll_deg,
                    "armed": tele.armed,
                    "mode": tele.mode,
                    "gps_valid": tele.gps_valid,
                    "local_valid": tele.local_valid,
                    "local_x": tele.local_x,
                    "local_y": tele.local_y,
                    "local_z": tele.local_z,
                    "leader_id": tele.leader_id,
                    "team_no": tele.team_no,
                    "follow_enabled": tele.follow_enabled,
                    "trail": [{"x": x, "y": y} for _lat, _lon, x, y, _ts in list(tele.trail)[-120:]],
                }
            )
        online = [item for item in vehicles if item["online"]]
        avg_speed = sum(item["speed"] for item in online) / len(online) if online else 0.0
        return {
            "connected": bool(self.backend.connected),
            "selected_uav": self.selected_uav,
            "target_altitude_m": self.target_altitude_m,
            "command_speed_mps": self.backend.command_speed_mps,
            "vehicles": vehicles,
            "waypoints": [
                {"x": wp.x, "y": wp.y, "z": wp.z, "idx": idx + 1}
                for idx, wp in enumerate(self.waypoints)
            ],
            "commands": [
                {
                    "index": row.index,
                    "target_id": row.target_id,
                    "command": row.command,
                    "label": COMMAND_LABELS.get(row.command, row.command),
                    "status": row.status,
                    "issued_at": row.issued_at.replace("T", " "),
                    "detail": row.detail,
                }
                for row in self.backend.command_history[:80]
            ],
            "logs": list(self.logs)[:120],
            "overview": {
                "vehicles": len(vehicles),
                "online": len(online),
                "armed": sum(1 for item in online if item["armed"]),
                "avg_speed": avg_speed,
                "waypoints": len(self.waypoints),
                "uptime_s": max(0.0, time.time() - self.started_at),
            },
            "config": {
                "airsim_host": self.config.airsim.host,
                "airsim_port": self.config.airsim.port,
                "poll_interval_s": self.config.airsim.poll_interval_s,
            },
        }

    def select(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            self.selected_uav = int(payload.get("target_id") or self.selected_uav)
            if payload.get("altitude_m") is not None:
                self.target_altitude_m = abs(float(payload["altitude_m"]))
            self.log(f"当前控制目标: UAV{self.selected_uav}, 高度 {self.target_altitude_m:.1f} m", "INFO")
            return self._serialize_state(self.backend.telemetry)

    def send_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        command = str(payload.get("command", "")).strip()
        if not command:
            raise ApiError("缺少命令")
        if command in CRITICAL_COMMANDS and not bool(payload.get("confirmed")):
            raise ApiError(f"命令 {command} 需要确认", 409)
        target_id = int(payload.get("target_id") or self.selected_uav)
        with self.lock:
            try:
                self.backend.send_basic_command(target_id, command)
                self.log(f"命令已发送 -> UAV{target_id}: {COMMAND_LABELS.get(command, command)}", "COMMAND")
            except Exception as exc:
                self.log(f"命令失败 -> UAV{target_id}: {exc}", "ERROR")
                raise ApiError(str(exc)) from exc
            return self._serialize_state(self.backend.telemetry)

    def goto(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not bool(payload.get("confirmed")):
            raise ApiError("指点飞行需要确认", 409)
        target_id = int(payload.get("target_id") or self.selected_uav)
        x = float(payload["x"])
        y = float(payload["y"])
        target_z = self.target_z(payload.get("altitude_m"), payload.get("z"))
        with self.lock:
            try:
                if payload.get("mode") == "direct":
                    self.backend.goto_local(target_id, x, y, target_z)
                    label = "直接指点"
                else:
                    self.backend.start_expert_goto_local(target_id, x, y, target_z)
                    label = "专家避障指点"
                self.waypoints = [Waypoint(x, y, target_z)]
                self.log(f"{label}已启动 -> UAV{target_id}: x={x:.1f}, y={y:.1f}, z={target_z:.1f}", "COMMAND")
            except Exception as exc:
                self.log(f"导航失败 -> UAV{target_id}: {exc}", "ERROR")
                raise ApiError(str(exc)) from exc
            return self._serialize_state(self.backend.telemetry)

    def set_waypoints(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            self.waypoints = [self._payload_to_waypoint(point) for point in payload.get("points", [])]
            self.log(f"航点列表已替换: {len(self.waypoints)} 个点", "INFO")
            return self._serialize_state(self.backend.telemetry)

    def add_waypoint(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            point = self._payload_to_waypoint(payload)
            self.waypoints.append(point)
            self.log(f"航点 #{len(self.waypoints)} 已添加: x={point.x:.1f}, y={point.y:.1f}, z={point.z:.1f}", "INFO")
            return self._serialize_state(self.backend.telemetry)

    def _payload_to_waypoint(self, payload: dict[str, Any]) -> Waypoint:
        return Waypoint(
            x=float(payload["x"]),
            y=float(payload["y"]),
            z=self.target_z(payload.get("altitude_m"), payload.get("z")),
        )

    def clear_waypoints(self) -> dict[str, Any]:
        with self.lock:
            self.waypoints.clear()
            self.log("航点已清空。", "INFO")
            return self._serialize_state(self.backend.telemetry)

    def run_waypoints(self, payload: dict[str, Any]) -> dict[str, Any]:
        target_id = int(payload.get("target_id") or self.selected_uav)
        with self.lock:
            points = payload.get("points") or []
            if points:
                self.waypoints = [self._payload_to_waypoint(point) for point in points]
            if not self.waypoints:
                raise ApiError("没有可执行的航点")
            try:
                self.backend.run_waypoints(target_id, list(self.waypoints))
                self.log(f"航点任务已启动 -> UAV{target_id}: {len(self.waypoints)} 个点", "COMMAND")
            except Exception as exc:
                self.log(f"航点任务失败 -> UAV{target_id}: {exc}", "ERROR")
                raise ApiError(str(exc)) from exc
            return self._serialize_state(self.backend.telemetry)

    def parse_llm_command(self, text: str) -> dict[str, Any]:
        raw = text.strip()
        if not raw:
            return {"steps": []}
        try:
            parsed = json.loads(raw)
            if isinstance(parsed, dict):
                return parsed
            if isinstance(parsed, list):
                return {"steps": parsed}
        except json.JSONDecodeError:
            pass
        lowered = raw.lower()
        steps: list[dict[str, Any]] = []
        z = self.extract_z(raw)
        x_match = re.search(r"\bX\s*=?\s*(-?\d+(?:\.\d+)?)", raw, re.IGNORECASE)
        y_match = re.search(r"\bY\s*=?\s*(-?\d+(?:\.\d+)?)", raw, re.IGNORECASE)
        if "takeoff" in lowered or "起飞" in raw:
            steps.append({"command": "takeoff", "args": {}})
        if x_match and y_match:
            steps.append(
                {
                    "command": "expert_goto_local",
                    "args": {"x": float(x_match.group(1)), "y": float(y_match.group(1)), "z": z},
                }
            )
        if "hover" in lowered or "stop" in lowered or "悬停" in raw or "停止" in raw:
            steps.append({"command": "hover", "args": {}})
        if "rtl" in lowered or "go home" in lowered or "return" in lowered or "返航" in raw:
            steps.append({"command": "rtl", "args": {}})
        if "land" in lowered or "降落" in raw:
            steps.append({"command": "land", "args": {}})
        return {"task": raw, "steps": steps}

    def extract_z(self, text: str) -> float:
        explicit = re.search(r"\bZ\s*=?\s*(-?\d+(?:\.\d+)?)", text, re.IGNORECASE)
        if explicit:
            return float(explicit.group(1))
        height = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|meter|meters|米)", text, re.IGNORECASE)
        if height:
            return -abs(float(height.group(1)))
        return self.target_z()

    def run_llm(self, payload: dict[str, Any]) -> dict[str, Any]:
        plan = self.parse_llm_command(str(payload.get("text", "")))
        steps = plan.get("steps", plan.get("plan", []))
        if not isinstance(steps, list) or not steps:
            raise ApiError("自然语言命令没有可执行步骤")
        target_id = int(payload.get("target_id") or self.selected_uav)
        with self.lock:
            for step in steps:
                if not isinstance(step, dict):
                    continue
                command = str(step.get("command", step.get("skill", ""))).strip()
                args = step.get("args", step.get("parameters", {}))
                if not isinstance(args, dict):
                    args = {}
                if command in CRITICAL_COMMANDS and not bool(payload.get("confirmed")):
                    raise ApiError(f"命令 {command} 需要确认", 409)
                try:
                    if command in {"navigate_with_expert", "expert_goto_local", "goto_local"}:
                        self.backend.start_expert_goto_local(
                            target_id,
                            float(args.get("target_x", args.get("x"))),
                            float(args.get("target_y", args.get("y"))),
                            float(args.get("target_z", args.get("z", self.target_z()))),
                        )
                        self.log(f"自然语言已启动专家避障指点 -> UAV{target_id}", "COMMAND")
                    elif command in COMMAND_LABELS or command in {"hover", "rtl", "land", "takeoff", "arm", "disarm"}:
                        self.backend.send_basic_command(target_id, command)
                        self.log(f"自然语言命令已发送 -> UAV{target_id}: {COMMAND_LABELS.get(command, command)}", "COMMAND")
                    else:
                        raise ValueError(f"不支持的自然语言命令: {command}")
                except Exception as exc:
                    self.log(f"自然语言命令失败 -> UAV{target_id}: {exc}", "ERROR")
                    raise ApiError(str(exc)) from exc
            return self._serialize_state(self.backend.telemetry)


class GroundStationHttpServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], station: WebGroundStation, static_dir: Path) -> None:
        super().__init__(server_address, GroundStationRequestHandler)
        self.station = station
        self.static_dir = static_dir


class GroundStationRequestHandler(BaseHTTPRequestHandler):
    server: GroundStationHttpServer

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/api/state":
            self._send_json(self.server.station.state())
            return
        if path == "/":
            path = "/index.html"
        self._send_static(path)

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        try:
            payload = self._read_json()
            station = self.server.station
            routes = {
                "/api/reconnect": lambda: station.connect() and station.state(),
                "/api/select": lambda: station.select(payload),
                "/api/command": lambda: station.send_command(payload),
                "/api/goto": lambda: station.goto(payload),
                "/api/waypoints": lambda: station.set_waypoints(payload),
                "/api/waypoints/add": lambda: station.add_waypoint(payload),
                "/api/waypoints/clear": station.clear_waypoints,
                "/api/waypoints/run": lambda: station.run_waypoints(payload),
                "/api/llm/preview": lambda: station.parse_llm_command(str(payload.get("text", ""))),
                "/api/llm/run": lambda: station.run_llm(payload),
            }
            handler = routes.get(path)
            if handler is None:
                self._send_json({"detail": "Not found"}, HTTPStatus.NOT_FOUND)
                return
            self._send_json(handler())
        except ApiError as exc:
            self._send_json({"detail": str(exc)}, exc.status)
        except Exception as exc:
            self._send_json({"detail": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length") or "0")
        if length <= 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        parsed = json.loads(raw or "{}")
        return parsed if isinstance(parsed, dict) else {}

    def _send_json(self, payload: dict[str, Any], status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(int(status))
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_static(self, request_path: str) -> None:
        relative = request_path.lstrip("/")
        if relative.startswith("static/"):
            relative = relative[len("static/") :]
        target = (self.server.static_dir / relative).resolve()
        if not str(target).startswith(str(self.server.static_dir.resolve())) or not target.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        data = target.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, format: str, *args: Any) -> None:
        return


def run(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Run the AirSim Web ground station.")
    parser.add_argument("--config", default="config_airsim.yaml")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args(argv)

    config = load_config(args.config)
    station = WebGroundStation(config)
    static_dir = Path(__file__).resolve().parent / "web_static"
    server = GroundStationHttpServer((args.host, args.port), station, static_dir)
    print(f"AirSim Web ground station: http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        station.backend.close()
        server.server_close()
