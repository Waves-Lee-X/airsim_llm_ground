from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Callable

from core.airsim_adapter import AirSimAdapter
from core.obstacle_avoidance import plan_local_path
from core.path_planner import lawnmower_path
from core.path_planner import Waypoint
from core.task_executor import TaskExecutor
from core.task_schema import MissionArea
from core.vision_detector import VisionDetector


DATASET_ROOT = Path(__file__).resolve().parents[1] / "datasets" / "yolo_collect"


@dataclass(frozen=True)
class ToolSpec:
    name: str
    description: str
    required: tuple[str, ...] = ()
    properties: dict[str, str] = field(default_factory=dict)

    def to_api(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "required": list(self.required),
            "properties": self.properties,
        }


@dataclass(frozen=True)
class ToolResult:
    ok: bool
    tool: str
    message: str
    data: dict[str, Any] = field(default_factory=dict)
    elapsed_ms: int = 0

    def to_api(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "tool": self.tool,
            "message": self.message,
            "data": self.data,
            "elapsed_ms": self.elapsed_ms,
        }


class AgentToolRuntime:
    """Mission-level tools that an LLM agent is allowed to call."""

    def __init__(self, adapter: AirSimAdapter, executor: TaskExecutor, detector: VisionDetector | None = None) -> None:
        self.adapter = adapter
        self.executor = executor
        self.detector = detector or VisionDetector()
        self._mission_lock = threading.RLock()
        self._mission_stop = threading.Event()
        self._mission_thread: threading.Thread | None = None
        self._post_mission_behavior = "keep_state"
        self._terminal_hold_guard = threading.Lock()
        self._terminal_hold_applied_at = 0.0
        self._mission_progress: dict[str, Any] = {
            "status": "idle",
            "tool": "",
            "target": "",
            "current_waypoint_index": 0,
            "total_waypoints": 0,
            "distance_to_waypoint_m": None,
            "collision": None,
            "obstacle": None,
            "avoidance_count": 0,
            "planner": None,
            "recovery": None,
            "formation": None,
            "formation_targets": [],
            "formation_vehicles": [],
            "message": "No active tool mission.",
            "started_at": None,
            "updated_at": None,
        }
        self._handlers: dict[str, Callable[[dict[str, Any]], ToolResult]] = {
            "takeoff": self._takeoff,
            "hover": self._hover,
            "stop": self._stop,
            "land": self._land,
            "return_home": self._return_home,
            "goto_local": self._goto_local,
            "waypoint_route": self._waypoint_route,
            "autonomous_nav": self._autonomous_nav,
            "formation_flight": self._formation_flight,
            "recover_from_collision": self._recover_from_collision,
            "search_area": self._search_area,
            "collect_images": self._collect_images,
            "detect_objects": self._detect_objects,
            "report_target": self._report_target,
        }

    def list_tools(self) -> list[dict[str, Any]]:
        return [spec.to_api() for spec in self.specs()]

    def mission_progress(self) -> dict[str, Any]:
        with self._mission_lock:
            return dict(self._mission_progress)

    def specs(self) -> list[ToolSpec]:
        return [
            ToolSpec(
                "takeoff",
                "Arm the UAV and take off to a safe mission altitude.",
                ("altitude_m",),
                {"altitude_m": "Target altitude above ground in meters."},
            ),
            ToolSpec("hover", "Hold current position.", (), {}),
            ToolSpec("stop", "Cancel the current AirSim task and hold current position.", (), {}),
            ToolSpec("land", "Land the UAV. Requires confirmed=true at the safety gate.", (), {}),
            ToolSpec(
                "return_home",
                "Return to local origin at a safe altitude. Requires confirmed=true.",
                ("safe_altitude_m",),
                {"safe_altitude_m": "Altitude to maintain during return-to-home."},
            ),
            ToolSpec(
                "goto_local",
                "Fly to a local NED position in AirSim coordinates.",
                ("x", "y", "z", "speed_mps"),
                {
                    "x": "Forward local position in meters.",
                    "y": "Right local position in meters.",
                    "z": "NED z value, negative means above ground.",
                    "speed_mps": "Transit speed in meters per second.",
                },
            ),
            ToolSpec(
                "waypoint_route",
                "Fly a sequence of local NED waypoints.",
                ("waypoints", "speed_mps"),
                {
                    "waypoints": "List of {x, y, z} local NED waypoints.",
                    "speed_mps": "Transit speed in meters per second.",
                    "avoidance": "Check obstacle state before each leg.",
                    "hold_at_end": "Hover at final waypoint.",
                },
            ),
            ToolSpec(
                "autonomous_nav",
                "Closed-loop local autonomous navigation using LiDAR A* replanning and velocity control.",
                ("x", "y", "z"),
                {
                    "x": "Goal forward local position in meters.",
                    "y": "Goal right local position in meters.",
                    "z": "Goal NED z, negative means above ground.",
                    "speed_mps": "Maximum horizontal speed.",
                    "replan_interval_s": "Seconds between LiDAR A* replans.",
                    "control_dt_s": "Velocity command duration.",
                    "lookahead_m": "Short target distance along planned path.",
                    "timeout_s": "Maximum navigation time.",
                },
            ),
            ToolSpec(
                "formation_flight",
                "Execute a leader route with formation slot preview for multiple UAVs.",
                ("count", "shape", "distance_m", "altitude_m"),
                {
                    "count": "Number of aircraft in formation.",
                    "shape": "Formation shape, currently v or line.",
                    "distance_m": "Leader forward travel distance.",
                    "spacing_m": "Distance between formation slots.",
                    "speed_mps": "Leader transit speed.",
                },
            ),
            ToolSpec(
                "recover_from_collision",
                "Cancel the active task, climb, back away, and hover when the UAV is stuck or colliding.",
                (),
                {
                    "climb_m": "Vertical escape distance.",
                    "backoff_m": "Backward escape distance.",
                    "force_relocate": "Use AirSim pose relocation if physical recovery does not move the UAV.",
                },
            ),
            ToolSpec(
                "search_area",
                "Execute a rectangular lawnmower search path in local AirSim coordinates.",
                ("target", "area", "altitude_m"),
                {
                    "target": "Object or region target to search for.",
                    "area": "Rectangle with x_min, x_max, y_min, y_max.",
                    "altitude_m": "Search altitude above ground in meters.",
                    "spacing_m": "Spacing between scan lanes.",
                    "speed_mps": "Waypoint transit speed.",
                    "avoidance": "Enable local LiDAR avoidance.",
                    "planned_avoidance": "Use LiDAR occupancy grid and A* for local path planning.",
                    "obstacle_distance_m": "Lookahead distance for front obstacle checks.",
                    "avoidance_offset_m": "Side-step distance used for temporary bypass waypoints.",
                    "scan_margin_m": "Inset scan lanes from area boundaries to avoid walls and edge obstacles.",
                    "pre_scan": "Assess the area with LiDAR before flying the coverage route.",
                    "pre_scan_stop_on_high_risk": "Stop before execution if pre-scan finds route-blocking obstacles.",
                },
            ),
            ToolSpec(
                "detect_objects",
                "Placeholder detector hook. Returns latest frame metadata for now.",
                ("target", "camera"),
                {"target": "Object label to detect.", "camera": "AirSim camera name."},
            ),
            ToolSpec(
                "collect_images",
                "Fly a coverage route with obstacle avoidance and save camera images for YOLO dataset labeling.",
                ("area", "altitude_m"),
                {
                    "area": "Rectangle with x_min, x_max, y_min, y_max.",
                    "altitude_m": "Collection altitude above ground in meters.",
                    "camera": "Camera name to capture.",
                    "dataset_name": "Folder name under datasets/yolo_collect.",
                    "max_images": "Maximum images to save.",
                    "capture_interval_s": "Minimum seconds between saved frames.",
                    "planned_avoidance": "Use LiDAR occupancy grid and A* for path planning.",
                    "pre_scan": "Assess obstacle map before collection route.",
                },
            ),
            ToolSpec(
                "report_target",
                "Create a structured target report placeholder.",
                ("target_id",),
                {"target_id": "Target identifier from detection history."},
            ),
        ]

    def call(self, tool: str, args: dict[str, Any] | None = None) -> ToolResult:
        normalized_tool = str(tool or "").strip()
        handler = self._handlers.get(normalized_tool)
        if handler is None:
            return ToolResult(False, normalized_tool, f"Unknown tool: {normalized_tool}")
        started_at = time.time()
        try:
            result = handler(dict(args or {}))
            return ToolResult(
                ok=result.ok,
                tool=result.tool,
                message=result.message,
                data=result.data,
                elapsed_ms=int((time.time() - started_at) * 1000),
            )
        except Exception as exc:
            return ToolResult(False, normalized_tool, f"{normalized_tool} failed: {exc}", elapsed_ms=int((time.time() - started_at) * 1000))

    def _takeoff(self, args: dict[str, Any]) -> ToolResult:
        altitude_m = float(args.get("altitude_m", 8.0))
        self.executor.takeoff(altitude_m=altitude_m)
        self._set_progress(status="holding", tool="takeoff", message=f"Takeoff complete; holding at {altitude_m:.1f}m.")
        return ToolResult(True, "takeoff", f"Takeoff command sent to {altitude_m:.1f}m", {"altitude_m": altitude_m})

    def _hover(self, args: dict[str, Any]) -> ToolResult:
        self._mission_stop.set()
        self.executor.hover()
        self._set_progress(status="paused", message="Hover command sent; active tool mission paused.")
        return ToolResult(True, "hover", "Hover command sent")

    def _stop(self, args: dict[str, Any]) -> ToolResult:
        self._mission_stop.set()
        self.executor.stop()
        self._set_progress(status="stopped", message="Stop command sent; active tool mission stopped.")
        return ToolResult(True, "stop", "Stop command sent")

    def _land(self, args: dict[str, Any]) -> ToolResult:
        self._mission_stop.set()
        self.executor.land()
        self._set_progress(status="landing", tool="land", message="Land command sent; active tool mission stopped.")
        return ToolResult(True, "land", "Land command sent")

    def _return_home(self, args: dict[str, Any]) -> ToolResult:
        self._mission_stop.set()
        safe_altitude_m = float(args.get("safe_altitude_m", 8.0))
        self.executor.return_to_launch(altitude_m=safe_altitude_m)
        self._set_progress(status="returning", tool="return_home", message="Return-home command sent.")
        return ToolResult(True, "return_home", "Return-home command sent", {"safe_altitude_m": safe_altitude_m})

    def _goto_local(self, args: dict[str, Any]) -> ToolResult:
        x = float(args.get("x", 0.0))
        y = float(args.get("y", 0.0))
        z = float(args.get("z", -8.0))
        speed_mps = float(args.get("speed_mps", 2.0))
        self.executor.goto_local(x=x, y=y, z=z, speed_mps=speed_mps)
        return ToolResult(
            True,
            "goto_local",
            f"Goto local command sent: x={x:.1f}, y={y:.1f}, z={z:.1f}",
            {"x": x, "y": y, "z": z, "speed_mps": speed_mps},
        )

    def _waypoint_route(self, args: dict[str, Any]) -> ToolResult:
        waypoints = args.get("waypoints", [])
        waypoints = waypoints if isinstance(waypoints, list) else []
        speed_mps = float(args.get("speed_mps", 2.0))
        avoidance = bool(args.get("avoidance", True))
        hold_at_end = bool(args.get("hold_at_end", True))
        route = [
            Waypoint(float(point.get("x", 0.0)), float(point.get("y", 0.0)), float(point.get("z", -8.0)))
            for point in waypoints
            if isinstance(point, dict)
        ]
        if not route:
            return ToolResult(False, "waypoint_route", "航点路线为空")
        self._mission_stop.set()
        old_thread = self._mission_thread
        if old_thread is not None and old_thread.is_alive():
            old_thread.join(timeout=0.2)
        self._mission_stop = threading.Event()
        self._set_progress(
            status="running",
            tool="waypoint_route",
            current_waypoint_index=0,
            total_waypoints=len(route),
            distance_to_waypoint_m=None,
            planner={"ok": True, "reason": "航点路线已生成", "waypoints": [point.to_api() for point in route]},
            message=f"航点飞行已启动，共 {len(route)} 个航点。",
            started_at=time.time(),
        )
        self._mission_thread = threading.Thread(
            target=self._run_waypoint_route,
            args=(route, speed_mps, avoidance, hold_at_end),
            name="aeromind-waypoint-route",
            daemon=True,
        )
        self._mission_thread.start()
        return ToolResult(
            True,
            "waypoint_route",
            f"航点飞行已启动，共 {len(route)} 个航点。",
            {"route": [point.to_api() for point in route], "total_waypoints": len(route), "speed_mps": speed_mps, "avoidance": avoidance, "hold_at_end": hold_at_end},
        )

    def _autonomous_nav(self, args: dict[str, Any]) -> ToolResult:
        goal = Waypoint(
            float(args.get("x", 0.0)),
            float(args.get("y", 0.0)),
            float(args.get("z", -8.0)),
        )
        speed_mps = float(args.get("speed_mps", 1.5))
        replan_interval_s = max(0.3, float(args.get("replan_interval_s", 1.0)))
        control_dt_s = max(0.1, float(args.get("control_dt_s", 0.2)))
        lookahead_m = max(1.0, float(args.get("lookahead_m", 3.0)))
        timeout_s = max(5.0, float(args.get("timeout_s", 120.0)))
        if not self._ensure_airborne(abs(goal.z)):
            return ToolResult(False, "autonomous_nav", "Unable to take off before autonomous navigation")
        collision_baseline = self.adapter.collision_status().time_stamp
        self._mission_stop.set()
        old_thread = self._mission_thread
        if old_thread is not None and old_thread.is_alive():
            old_thread.join(timeout=0.2)
        self._mission_stop = threading.Event()
        self._set_progress(
            status="autonomous",
            tool="autonomous_nav",
            target="goal",
            current_waypoint_index=0,
            total_waypoints=1,
            planner=None,
            collision=None,
            obstacle=None,
            recovery=None,
            message=f"Autonomous navigation started to X {goal.x:.1f}, Y {goal.y:.1f}.",
            started_at=time.time(),
        )
        self._mission_thread = threading.Thread(
            target=self._run_autonomous_nav,
            args=(goal, speed_mps, replan_interval_s, control_dt_s, lookahead_m, timeout_s, collision_baseline),
            name="aeromind-autonomous-nav",
            daemon=True,
        )
        self._mission_thread.start()
        return ToolResult(
            True,
            "autonomous_nav",
            "Autonomous navigation started",
            {
                "goal": goal.to_api(),
                "speed_mps": speed_mps,
                "replan_interval_s": replan_interval_s,
                "control_dt_s": control_dt_s,
                "lookahead_m": lookahead_m,
                "timeout_s": timeout_s,
                "route": [goal.to_api()],
            },
        )

    def _formation_flight(self, args: dict[str, Any]) -> ToolResult:
        count = int(args.get("count", 3))
        shape = str(args.get("shape", "v")).lower()
        distance_m = float(args.get("distance_m", 40.0))
        altitude_m = float(args.get("altitude_m", 8.0))
        spacing_m = float(args.get("spacing_m", 6.0))
        speed_mps = float(args.get("speed_mps", 2.0))
        avoidance = bool(args.get("avoidance", True))
        z = -abs(altitude_m)
        route = [Waypoint(0.0, 0.0, z), Waypoint(distance_m, 0.0, z)]
        slots = self._formation_slots(count, shape, spacing_m)
        vehicles = list(self.adapter.formation_vehicle_names(count))
        self._mission_stop.set()
        old_thread = self._mission_thread
        if old_thread is not None and old_thread.is_alive():
            old_thread.join(timeout=0.2)
        self._mission_stop = threading.Event()
        self._set_progress(
            status="running",
            tool="formation_flight",
            current_waypoint_index=0,
            total_waypoints=len(route),
            formation={"count": count, "shape": shape, "spacing_m": spacing_m, "slots": slots, "vehicles": vehicles},
            formation_targets=self._formation_targets(route[0], slots, vehicles),
            formation_vehicles=self._formation_vehicle_states(vehicles, self._formation_targets(route[0], slots, vehicles), "planned"),
            planner={"ok": True, "reason": "多机编队航线已生成", "waypoints": [point.to_api() for point in route]},
            message=f"编队飞行已启动：{count} 架，队形 {shape.upper()}。",
            started_at=time.time(),
        )
        self._mission_thread = threading.Thread(
            target=self._run_formation_route,
            args=(route, slots, altitude_m, speed_mps, avoidance),
            name="aeromind-formation-flight",
            daemon=True,
        )
        self._mission_thread.start()
        return ToolResult(
            True,
            "formation_flight",
            f"编队飞行已启动：{count} 架，队形 {shape.upper()}",
            {"route": [point.to_api() for point in route], "formation": {"count": count, "shape": shape, "spacing_m": spacing_m, "slots": slots, "vehicles": vehicles}, "speed_mps": speed_mps, "avoidance": avoidance, "execution_mode": "multi_vehicle"},
        )

    def _recover_from_collision(self, args: dict[str, Any]) -> ToolResult:
        self._mission_stop.set()
        climb_m = float(args.get("climb_m", 8.0))
        backoff_m = float(args.get("backoff_m", 10.0))
        force_relocate = bool(args.get("force_relocate", True))
        recovery = self.executor.recover_from_collision(
            climb_m=climb_m,
            backoff_m=backoff_m,
            force_relocate=force_relocate,
        )
        self._set_progress(
            status="recovering",
            recovery=recovery,
            message=f"Recovery command sent: climb={climb_m:.1f}m, backoff={backoff_m:.1f}m.",
        )
        return ToolResult(
            True,
            "recover_from_collision",
            "Recovery command sent",
            {"climb_m": climb_m, "backoff_m": backoff_m, "force_relocate": force_relocate, "recovery": recovery},
        )

    def _search_area(self, args: dict[str, Any]) -> ToolResult:
        area_payload = args.get("area", {})
        area_payload = area_payload if isinstance(area_payload, dict) else {}
        area = MissionArea(
            x_min=float(area_payload.get("x_min", 0.0)),
            x_max=float(area_payload.get("x_max", 60.0)),
            y_min=float(area_payload.get("y_min", -20.0)),
            y_max=float(area_payload.get("y_max", 20.0)),
        )
        altitude_m = float(args.get("altitude_m", 8.0))
        spacing_m = float(args.get("spacing_m", 10.0))
        speed_mps = float(args.get("speed_mps", 2.0))
        avoidance = bool(args.get("avoidance", True))
        planned_avoidance = bool(args.get("planned_avoidance", True))
        obstacle_distance_m = float(args.get("obstacle_distance_m", 8.0))
        avoidance_offset_m = float(args.get("avoidance_offset_m", 6.0))
        scan_margin_m = float(args.get("scan_margin_m", 4.0))
        pre_scan = bool(args.get("pre_scan", True))
        pre_scan_stop_on_high_risk = bool(args.get("pre_scan_stop_on_high_risk", True))
        target = str(args.get("target", "object"))
        effective_area = self._inset_area(area, scan_margin_m)
        route = lawnmower_path(effective_area, altitude_m, spacing_m=spacing_m)
        if not route:
            return ToolResult(False, "search_area", "No waypoints generated for search area")
        collision_baseline = self.adapter.collision_status().time_stamp

        self._mission_stop.set()
        old_thread = self._mission_thread
        if old_thread is not None and old_thread.is_alive():
            old_thread.join(timeout=0.2)
        self._mission_stop = threading.Event()
        self._set_progress(
            status="running",
            tool="search_area",
            target=target,
            current_waypoint_index=0,
            total_waypoints=len(route),
            distance_to_waypoint_m=None,
            collision=None,
            obstacle=None,
            avoidance_count=0,
            planner=None,
            recovery=None,
            message=f"Search area started for target '{target}'.",
            started_at=time.time(),
        )
        self._mission_thread = threading.Thread(
            target=self._run_search_area,
            args=(
                target,
                route,
                altitude_m,
                speed_mps,
                collision_baseline,
                avoidance,
                planned_avoidance,
                obstacle_distance_m,
                avoidance_offset_m,
                effective_area,
                pre_scan,
                pre_scan_stop_on_high_risk,
            ),
            name="aeromind-search-area",
            daemon=True,
        )
        self._mission_thread.start()
        return ToolResult(
            True,
            "search_area",
            f"Search area mission started with {len(route)} waypoints",
            {
                "target": target,
                "area": area.to_api(),
                "effective_area": effective_area.to_api(),
                "scan_margin_m": scan_margin_m,
                "pre_scan": pre_scan,
                "pre_scan_stop_on_high_risk": pre_scan_stop_on_high_risk,
                "altitude_m": altitude_m,
                "spacing_m": spacing_m,
                "speed_mps": speed_mps,
                "avoidance": avoidance,
                "planned_avoidance": planned_avoidance,
                "obstacle_distance_m": obstacle_distance_m,
                "avoidance_offset_m": avoidance_offset_m,
                "route": [point.to_api() for point in route],
                "total_waypoints": len(route),
            },
        )

    def _collect_images(self, args: dict[str, Any]) -> ToolResult:
        area_payload = args.get("area", {})
        area_payload = area_payload if isinstance(area_payload, dict) else {}
        area = MissionArea(
            x_min=float(area_payload.get("x_min", 0.0)),
            x_max=float(area_payload.get("x_max", 40.0)),
            y_min=float(area_payload.get("y_min", -15.0)),
            y_max=float(area_payload.get("y_max", 15.0)),
        )
        altitude_m = float(args.get("altitude_m", 8.0))
        spacing_m = float(args.get("spacing_m", 8.0))
        speed_mps = float(args.get("speed_mps", 1.5))
        scan_margin_m = float(args.get("scan_margin_m", 4.0))
        planned_avoidance = bool(args.get("planned_avoidance", True))
        avoidance = bool(args.get("avoidance", True))
        pre_scan = bool(args.get("pre_scan", True))
        pre_scan_stop_on_high_risk = bool(args.get("pre_scan_stop_on_high_risk", False))
        obstacle_distance_m = float(args.get("obstacle_distance_m", 8.0))
        avoidance_offset_m = float(args.get("avoidance_offset_m", 6.0))
        max_images = max(1, min(int(args.get("max_images", 200)), 5000))
        capture_interval_s = max(0.2, float(args.get("capture_interval_s", 1.0)))
        camera = str(args.get("camera", self.adapter.active_camera_name or "front_center"))
        dataset_name = self._safe_dataset_name(str(args.get("dataset_name", "")) or time.strftime("collect_%Y%m%d_%H%M%S"))
        dataset_dir = DATASET_ROOT / dataset_name
        images_dir = dataset_dir / "images"
        effective_area = self._inset_area(area, scan_margin_m)
        route = lawnmower_path(effective_area, altitude_m, spacing_m=spacing_m)
        if not route:
            return ToolResult(False, "collect_images", "No waypoints generated for image collection")
        collision_baseline = self.adapter.collision_status().time_stamp

        self._mission_stop.set()
        old_thread = self._mission_thread
        if old_thread is not None and old_thread.is_alive():
            old_thread.join(timeout=0.2)
        self._mission_stop = threading.Event()
        self._set_progress(
            status="collecting",
            tool="collect_images",
            target="dataset",
            current_waypoint_index=0,
            total_waypoints=len(route),
            distance_to_waypoint_m=None,
            collision=None,
            obstacle=None,
            avoidance_count=0,
            planner=None,
            recovery=None,
            dataset={
                "name": dataset_name,
                "dir": str(dataset_dir),
                "images_dir": str(images_dir),
                "saved": 0,
                "max_images": max_images,
                "camera": camera,
            },
            message=f"Image collection started: {dataset_name}.",
            started_at=time.time(),
        )
        self._mission_thread = threading.Thread(
            target=self._run_collect_images,
            args=(
                route,
                altitude_m,
                speed_mps,
                collision_baseline,
                avoidance,
                planned_avoidance,
                obstacle_distance_m,
                avoidance_offset_m,
                effective_area,
                pre_scan,
                pre_scan_stop_on_high_risk,
                dataset_dir,
                images_dir,
                dataset_name,
                camera,
                max_images,
                capture_interval_s,
            ),
            name="aeromind-collect-images",
            daemon=True,
        )
        self._mission_thread.start()
        return ToolResult(
            True,
            "collect_images",
            f"Image collection started: {dataset_name}",
            {
                "dataset_name": dataset_name,
                "dataset_dir": str(dataset_dir),
                "images_dir": str(images_dir),
                "camera": camera,
                "area": area.to_api(),
                "effective_area": effective_area.to_api(),
                "altitude_m": altitude_m,
                "spacing_m": spacing_m,
                "speed_mps": speed_mps,
                "max_images": max_images,
                "capture_interval_s": capture_interval_s,
                "avoidance": avoidance,
                "planned_avoidance": planned_avoidance,
                "pre_scan": pre_scan,
                "route": [point.to_api() for point in route],
                "total_waypoints": len(route),
            },
        )

    def _detect_objects(self, args: dict[str, Any]) -> ToolResult:
        frame = self.adapter.camera_frame()
        if frame is None:
            return ToolResult(False, "detect_objects", f"No camera frame available: {self.adapter.last_error}")
        detection_result = self.detector.detect(frame.data)
        return ToolResult(
            detection_result.ok,
            "detect_objects",
            str(detection_result.message),
            {
                "target": str(args.get("target", "object")),
                "camera": frame.camera_name,
                "image_type": frame.image_type,
                "bytes": len(frame.data),
                **detection_result.to_api(),
            },
        )

    def _report_target(self, args: dict[str, Any]) -> ToolResult:
        target_id = str(args.get("target_id", "target-unknown"))
        return ToolResult(
            True,
            "report_target",
            "Target report placeholder generated",
            {"target_id": target_id, "status": "pending_detection_module"},
        )

    @staticmethod
    def _safe_dataset_name(value: str) -> str:
        cleaned = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in value.strip())
        cleaned = cleaned.strip("._-")
        return cleaned[:80] or time.strftime("collect_%Y%m%d_%H%M%S")

    @staticmethod
    def _frame_extension(frame: Any) -> str:
        content_type = str(getattr(frame, "content_type", "")).lower()
        if "png" in content_type:
            return ".png"
        return ".jpg"

    def _save_collection_frame(
        self,
        images_dir: Path,
        metadata_path: Path,
        dataset_name: str,
        camera: str,
        saved_count: int,
        waypoint_index: int,
        total_waypoints: int,
    ) -> dict[str, Any] | None:
        frame = self.adapter.capture_camera_frame(camera)
        if frame is None:
            self._set_progress(message=f"Image capture skipped: {self.adapter.last_error or 'no frame'}")
            return None
        telemetry = self.adapter.telemetry()
        ext = self._frame_extension(frame)
        filename = f"{dataset_name}_{saved_count + 1:06d}{ext}"
        path = images_dir / filename
        path.write_bytes(frame.data)
        row = {
            "file": f"images/{filename}",
            "camera": frame.camera_name,
            "content_type": frame.content_type,
            "image_type": frame.image_type,
            "timestamp": time.time(),
            "vehicle": telemetry.name,
            "position": {"x": telemetry.x, "y": telemetry.y, "z": telemetry.z, "altitude_m": telemetry.altitude_m},
            "speed_mps": telemetry.speed_mps,
            "waypoint_index": waypoint_index,
            "total_waypoints": total_waypoints,
        }
        with metadata_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")
        return row

    def _run_collect_images(
        self,
        route: list[Any],
        altitude_m: float,
        speed_mps: float,
        collision_baseline: int,
        avoidance: bool,
        planned_avoidance: bool,
        obstacle_distance_m: float,
        avoidance_offset_m: float,
        effective_area: MissionArea,
        pre_scan: bool,
        pre_scan_stop_on_high_risk: bool,
        dataset_dir: Path,
        images_dir: Path,
        dataset_name: str,
        camera: str,
        max_images: int,
        capture_interval_s: float,
    ) -> None:
        saved = 0
        last_capture_at = 0.0
        metadata_path = dataset_dir / "metadata.jsonl"
        try:
            images_dir.mkdir(parents=True, exist_ok=True)
            metadata_path.write_text("", encoding="utf-8")
            if not self._ensure_airborne(altitude_m):
                self._hold_current_position_quietly()
                return
            if pre_scan:
                self._set_progress(status="pre_scanning", message="Dataset collection: pre-scanning obstacle map.")
                assessment = self._preflight_area_assessment(effective_area, route)
                self._set_progress(area_assessment=assessment)
                if assessment.get("stop_recommended"):
                    replanned_route = self._replan_route_around_zones(route, assessment.get("blocked_zones", []))
                    replanned_assessment = {
                        **assessment,
                        "route_hits_before_replan": assessment.get("route_hits", 0),
                        "route_hits": self._count_route_hits(replanned_route, assessment.get("blocked_zones", [])),
                        "replanned": True,
                        "replanned_waypoints": len(replanned_route),
                    }
                    replanned_assessment["stop_recommended"] = len(replanned_route) < 2 or replanned_assessment["route_hits"] > 0
                    self._set_progress(area_assessment=replanned_assessment)
                    if replanned_assessment["stop_recommended"] and pre_scan_stop_on_high_risk:
                        self._hold_current_position_quietly()
                        self._mission_stop.set()
                        self._set_progress(status="blocked", message="Dataset collection stopped: pre-scan could not create a safe route.")
                        return
                    if len(replanned_route) >= 2:
                        route = replanned_route
                        self._set_progress(
                            total_waypoints=len(route),
                            replanned_route=[point.to_api() for point in route],
                            planner={
                                "ok": True,
                                "phase": "dataset_pre_scan_replan",
                                "reason": f"采集航线已根据预扫描调整为 {len(route)} 个航点",
                                "waypoints": [point.to_api() for point in route],
                            },
                        )
            for index, waypoint in enumerate(route, start=1):
                if self._mission_stop.is_set() or saved >= max_images:
                    break
                self._set_progress(
                    status="collecting",
                    tool="collect_images",
                    current_waypoint_index=index,
                    total_waypoints=len(route),
                    message=f"Collecting images with autonomous navigation on waypoint {index}/{len(route)}.",
                )

                def capture_tick(_current: Waypoint, _goal_distance: float, _elapsed: float) -> bool:
                    nonlocal saved, last_capture_at
                    now = time.time()
                    if now - last_capture_at < capture_interval_s or saved >= max_images:
                        return saved < max_images
                    row = self._save_collection_frame(
                        images_dir,
                        metadata_path,
                        dataset_name,
                        camera,
                        saved,
                        index,
                        len(route),
                    )
                    last_capture_at = now
                    if row is None:
                        return saved < max_images
                    saved += 1
                    self._set_progress(
                        dataset={
                            "name": dataset_name,
                            "dir": str(dataset_dir),
                            "images_dir": str(images_dir),
                            "metadata": str(metadata_path),
                            "saved": saved,
                            "max_images": max_images,
                            "last_file": row["file"],
                            "camera": camera,
                        },
                        message=f"Saved dataset image {saved}/{max_images}: {row['file']}",
                    )
                    return saved < max_images

                reached = self._closed_loop_drive_to_goal(
                    goal=waypoint,
                    speed_mps=speed_mps,
                    replan_interval_s=1.2 if planned_avoidance else 1.8,
                    control_dt_s=0.25,
                    lookahead_m=max(2.5, avoidance_offset_m),
                    timeout_s=max(20.0, self._distance_to_waypoint(waypoint) / max(speed_mps, 0.5) * 4.0),
                    collision_baseline=collision_baseline,
                    mission_label="Dataset collection",
                    active_status="collecting",
                    blocked_message="Dataset collection planner blocked",
                    reached_message=f"Reached collection waypoint {index}/{len(route)}.",
                    current_waypoint_index=index,
                    total_waypoints=len(route),
                    use_distance_safety=avoidance,
                    effective_area=effective_area,
                    planner_phase="dataset_closed_loop_replan",
                    on_tick=capture_tick,
                )
                if not reached:
                    if saved >= max_images:
                        break
                    self._hold_current_position_quietly()
                    return
            self.executor.hover()
            self._set_progress(
                status="completed",
                current_waypoint_index=len(route),
                total_waypoints=len(route),
                dataset={
                    "name": dataset_name,
                    "dir": str(dataset_dir),
                    "images_dir": str(images_dir),
                    "metadata": str(metadata_path),
                    "saved": saved,
                    "max_images": max_images,
                    "camera": camera,
                },
                message=f"Image collection completed: {saved} images saved.",
            )
        except Exception as exc:
            self._hold_current_position_quietly()
            self._set_progress(status="failed", message=f"Image collection worker failed: {exc}")

    def _hold_current_position_quietly(self) -> None:
        try:
            self.adapter.hold_current_position(duration_s=1.5)
        except Exception:
            try:
                self.executor.hover()
            except Exception:
                pass

    def _closed_loop_drive_to_goal(
        self,
        goal: Waypoint,
        speed_mps: float,
        replan_interval_s: float,
        control_dt_s: float,
        lookahead_m: float,
        timeout_s: float,
        collision_baseline: int,
        mission_label: str,
        active_status: str,
        blocked_message: str,
        reached_message: str,
        current_waypoint_index: int,
        total_waypoints: int,
        use_distance_safety: bool = True,
        effective_area: MissionArea | None = None,
        planner_phase: str = "closed_loop_replan",
        on_tick: Callable[[Waypoint, float, float], bool] | None = None,
    ) -> bool:
        goal = Waypoint(float(goal.x), float(goal.y), -abs(float(goal.z)))
        started_at = time.time()
        last_replan_at = 0.0
        plan_waypoints: list[Waypoint] = []
        bounds = None
        if effective_area is not None:
            bounds_padding_m = 8.0
            bounds = (
                float(effective_area.x_min) - bounds_padding_m,
                float(effective_area.x_max) + bounds_padding_m,
                float(effective_area.y_min) - bounds_padding_m,
                float(effective_area.y_max) + bounds_padding_m,
            )

        while not self._mission_stop.is_set():
            elapsed = time.time() - started_at
            if elapsed > timeout_s:
                self._hold_current_position_quietly()
                self._set_progress(status="timeout", message=f"{mission_label} timeout; holding position.")
                return False

            telemetry = self.adapter.telemetry()
            current = Waypoint(float(telemetry.x), float(telemetry.y), float(goal.z))
            goal_distance = math.hypot(goal.x - current.x, goal.y - current.y)
            altitude_error = float(goal.z) - float(telemetry.z)
            self._set_progress(distance_to_waypoint_m=round(goal_distance, 2))
            if on_tick is not None and not on_tick(current, goal_distance, elapsed):
                self._hold_current_position_quietly()
                self._set_progress(status=active_status, message=f"{mission_label} capture limit reached; holding position.")
                return True
            if goal_distance <= 1.5 and abs(altitude_error) <= 1.2:
                self._hold_current_position_quietly()
                self._set_progress(
                    status=active_status,
                    current_waypoint_index=current_waypoint_index,
                    total_waypoints=total_waypoints,
                    distance_to_waypoint_m=0.0,
                    message=reached_message,
                )
                return True

            collision = self.adapter.collision_status()
            if collision.has_collided and collision.time_stamp > collision_baseline:
                recovery = self._try_recovery()
                self._mission_stop.set()
                self._set_progress(
                    status="recovered" if recovery is not None else "collision",
                    collision=collision.to_api(),
                    recovery=recovery,
                    message=f"{mission_label} stopped after collision; recovery attempted.",
                )
                return False

            if use_distance_safety:
                distances = self.adapter.distance_sensors_status(
                    emergency_threshold_m=2.6,
                    side_emergency_threshold_m=1.6,
                )
                self._set_progress(distance_sensors=distances.to_api())
                if distances.available and distances.emergency:
                    stopped_at = self._try_hard_stop()
                    self._mission_stop.set()
                    self._set_progress(
                        status="blocked",
                        recovery={"mode": "hard_stop", "stopped_at": stopped_at},
                        message=f"{mission_label} stopped by distance safety bubble.",
                    )
                    return False

            now = time.time()
            if now - last_replan_at >= replan_interval_s or not plan_waypoints:
                points = self.adapter.lidar_obstacle_points_world(
                    range_m=35.0,
                    vertical_window_m=2.5,
                    vertical_down_m=1.4,
                    denoise_cell_m=1.5,
                    min_points_per_cell=2,
                )
                plan = plan_local_path(
                    start=current,
                    goal=goal,
                    obstacle_points=points,
                    grid_size_m=55.0,
                    resolution_m=1.0,
                    inflation_m=3.0,
                    max_waypoints=24,
                    bounds=bounds,
                )
                self._set_progress(
                    planner=plan.to_api() | {"points": len(points), "phase": planner_phase},
                    message=plan.reason,
                )
                if not plan.ok:
                    self._hold_current_position_quietly()
                    self._set_progress(status="blocked", message=f"{blocked_message}: {plan.reason}")
                    return False
                plan_waypoints = plan.waypoints or [goal]
                last_replan_at = now

            target = self._lookahead_target(current, plan_waypoints, lookahead_m) or goal
            dx = float(target.x) - float(telemetry.x)
            dy = float(target.y) - float(telemetry.y)
            dz = float(goal.z) - float(telemetry.z)
            horizontal_distance = max(0.001, math.hypot(dx, dy))
            speed = min(max(0.4, speed_mps), max(0.4, horizontal_distance / max(control_dt_s, 0.1)))
            vx = dx / horizontal_distance * speed
            vy = dy / horizontal_distance * speed
            vz = max(-1.0, min(1.0, dz / max(control_dt_s * 4.0, 0.4)))
            self.executor.move_velocity(vx=vx, vy=vy, vz=vz, duration_s=control_dt_s)
            self._set_progress(
                status=active_status,
                current_waypoint_index=current_waypoint_index,
                total_waypoints=total_waypoints,
                message=f"{mission_label} velocity cmd vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}.",
            )
            time.sleep(max(0.02, control_dt_s * 0.65))
        return False

    def _run_autonomous_nav(
        self,
        goal: Waypoint,
        speed_mps: float,
        replan_interval_s: float,
        control_dt_s: float,
        lookahead_m: float,
        timeout_s: float,
        collision_baseline: int,
    ) -> None:
        try:
            reached = self._closed_loop_drive_to_goal(
                goal=goal,
                speed_mps=speed_mps,
                replan_interval_s=replan_interval_s,
                control_dt_s=control_dt_s,
                lookahead_m=lookahead_m,
                timeout_s=timeout_s,
                collision_baseline=collision_baseline,
                mission_label="Autonomous navigation",
                active_status="autonomous",
                blocked_message="Autonomous navigation planner blocked",
                reached_message="Autonomous navigation reached goal and is holding.",
                current_waypoint_index=1,
                total_waypoints=1,
                use_distance_safety=True,
                planner_phase="closed_loop_replan",
            )
            if reached:
                self._set_progress(status="completed", current_waypoint_index=1, distance_to_waypoint_m=0.0)
        except Exception as exc:
            self._set_progress(status="failed", message=f"Autonomous navigation worker failed: {exc}")

    @staticmethod
    def _lookahead_target(current: Waypoint, waypoints: list[Waypoint], lookahead_m: float) -> Waypoint | None:
        if not waypoints:
            return None
        for point in waypoints:
            if math.hypot(point.x - current.x, point.y - current.y) >= lookahead_m:
                return point
        return waypoints[-1]

    def _run_search_area(
        self,
        target: str,
        route: list[Any],
        altitude_m: float,
        speed_mps: float,
        collision_baseline: int,
        avoidance: bool,
        planned_avoidance: bool,
        obstacle_distance_m: float,
        avoidance_offset_m: float,
        effective_area: MissionArea,
        pre_scan: bool,
        pre_scan_stop_on_high_risk: bool,
    ) -> None:
        try:
            if not self._ensure_airborne(altitude_m):
                return
            if pre_scan:
                self._set_progress(
                    status="pre_scanning",
                    planner={
                        "ok": None,
                        "phase": "pre_scan",
                        "reason": "升空后正在确认任务区域障碍物信息",
                    },
                    message="Pre-scan: airborne, confirming obstacle map before A* planning.",
                )
                assessment = self._preflight_area_assessment(effective_area, route)
                self._set_progress(area_assessment=assessment)
                if assessment.get("stop_recommended"):
                    replanned_route = self._replan_route_around_zones(route, assessment.get("blocked_zones", []))
                    replanned_assessment = {
                        **assessment,
                        "route_hits_before_replan": assessment.get("route_hits", 0),
                        "route_hits": self._count_route_hits(replanned_route, assessment.get("blocked_zones", [])),
                        "replanned": True,
                        "replanned_waypoints": len(replanned_route),
                    }
                    replanned_assessment["stop_recommended"] = len(replanned_route) < 2 or replanned_assessment["route_hits"] > 0
                    self._set_progress(area_assessment=replanned_assessment)
                    if replanned_assessment["stop_recommended"] and pre_scan_stop_on_high_risk:
                        self._mission_stop.set()
                        self._set_progress(
                            status="blocked",
                            message=f"Pre-scan could not create a safe scan route: {assessment.get('recommendation', 'unsafe scan area')}",
                        )
                        return
                    if replanned_assessment["stop_recommended"]:
                        if len(replanned_route) >= 2:
                            route = replanned_route
                            self._set_progress(
                                total_waypoints=len(route),
                                replanned_route=[point.to_api() for point in route],
                                planner={
                                    "ok": True,
                                    "phase": "pre_scan_replan",
                                    "reason": f"预扫描已调整航线，但仍有 {replanned_assessment['route_hits']} 处风险交叉",
                                    "waypoints": [point.to_api() for point in route],
                                },
                            )
                        self._set_progress(
                            message="Pre-scan replan still has risks; continuing because pre_scan_stop_on_high_risk=false.",
                        )
                    elif len(replanned_route) >= 2:
                        route = replanned_route
                        self._set_progress(
                            total_waypoints=len(route),
                            replanned_route=[point.to_api() for point in route],
                            planner={
                                "ok": True,
                                "phase": "pre_scan_replan",
                                "reason": f"预扫描已绕开障碍物，航线调整为 {len(route)} 个航点",
                                "waypoints": [point.to_api() for point in route],
                            },
                            message=f"Pre-scan replanned scan route around obstacles: {len(route)} waypoints.",
                        )
                else:
                    self._set_progress(
                        planner={
                            "ok": True,
                            "phase": "pre_scan_clear",
                            "reason": f"预扫描完成，发现 {assessment.get('obstacle_points', 0)} 个障碍点，原航线可执行",
                            "waypoints": [point.to_api() for point in route],
                        },
                        message="Pre-scan complete; route cleared for A* segment planning.",
                    )
            for index, waypoint in enumerate(route, start=1):
                if self._mission_stop.is_set():
                    self._set_progress(status="stopped", message="Search area stopped by command.")
                    return
                self._set_progress(
                    status="running",
                    tool="search_area",
                    target=target,
                    current_waypoint_index=index,
                    total_waypoints=len(route),
                    message=f"Flying to waypoint {index}/{len(route)}.",
                )
                sub_waypoints = self._planned_sub_waypoints(waypoint, planned_avoidance, effective_area)
                if not sub_waypoints:
                    return
                for sub_index, sub_waypoint in enumerate(sub_waypoints, start=1):
                    if self._mission_stop.is_set():
                        return
                    self._set_progress(
                        message=f"Flying to waypoint {index}/{len(route)} segment {sub_index}/{len(sub_waypoints)}.",
                    )
                    commanded_speed = float(speed_mps)
                    with self._mission_lock:
                        planner_state = self._mission_progress.get("planner")
                    if isinstance(planner_state, dict) and str(planner_state.get("fallback", "")).startswith("direct_"):
                        commanded_speed = min(commanded_speed, 1.2)
                    try:
                        self.executor.goto_local(
                            sub_waypoint.x,
                            sub_waypoint.y,
                            sub_waypoint.z,
                            speed_mps=commanded_speed,
                            face_target=not planned_avoidance,
                        )
                    except Exception as exc:
                        self._set_progress(status="failed", message=f"Waypoint {index} command failed: {exc}")
                        return
                    if not self._wait_for_waypoint(
                        sub_waypoint,
                        speed_mps,
                        collision_baseline,
                        avoidance=avoidance,
                        reactive_avoidance=not planned_avoidance,
                        obstacle_distance_m=obstacle_distance_m,
                        avoidance_offset_m=avoidance_offset_m,
                    ):
                        return
                if not self._wait_for_waypoint(
                    waypoint,
                    speed_mps,
                    collision_baseline,
                    avoidance=avoidance,
                    reactive_avoidance=not planned_avoidance,
                    obstacle_distance_m=obstacle_distance_m,
                    avoidance_offset_m=avoidance_offset_m,
                ):
                    return
            self._set_progress(
                status="completed",
                current_waypoint_index=len(route),
                total_waypoints=len(route),
                distance_to_waypoint_m=0.0,
                message=f"Search area completed for target '{target}'.",
            )
        except Exception as exc:
            self._set_progress(status="failed", message=f"Search area worker failed: {exc}")

    def _run_waypoint_route(
        self,
        route: list[Waypoint],
        speed_mps: float,
        avoidance: bool,
        hold_at_end: bool,
    ) -> None:
        try:
            if route and not self._ensure_airborne(abs(float(route[0].z))):
                return
            collision_baseline = self.adapter.collision_status().time_stamp
            for index, waypoint in enumerate(route, start=1):
                if self._mission_stop.is_set():
                    self._set_progress(status="stopped", message="航点飞行已停止。")
                    return
                collision = self.adapter.collision_status()
                if collision.has_collided and collision.time_stamp != collision_baseline:
                    self.executor.hover()
                    self._set_progress(status="collision", collision=collision.to_api(), message=f"检测到碰撞：{collision.object_name or 'unknown'}，航点飞行停止。")
                    return
                if avoidance:
                    obstacle = self.adapter.obstacle_status(lookahead_m=8.0)
                    if obstacle.blocked and obstacle.risk_level in {"high", "critical"}:
                        self.executor.hover()
                        self._set_progress(status="blocked", obstacle=obstacle.to_api(), message="前方障碍风险过高，航点飞行已暂停。")
                        return
                self.executor.goto_local(waypoint.x, waypoint.y, waypoint.z, speed_mps=speed_mps)
                while not self._mission_stop.is_set():
                    distance = self._distance_to_waypoint(waypoint)
                    self._set_progress(current_waypoint_index=index, total_waypoints=len(route), distance_to_waypoint_m=round(distance, 2), message=f"正在飞向航点 {index}/{len(route)}。")
                    if distance <= 2.0:
                        break
                    time.sleep(0.25)
            if hold_at_end:
                self.executor.hover()
            self._set_progress(status="completed", message="航点飞行完成。")
        except Exception as exc:
            self._set_progress(status="failed", message=f"航点飞行失败：{exc}")

    def _run_formation_route(
        self,
        route: list[Waypoint],
        slots: list[dict[str, Any]],
        altitude_m: float,
        speed_mps: float,
        avoidance: bool,
    ) -> None:
        try:
            vehicles = list(self.adapter.formation_vehicle_names(len(slots)))
            self._set_progress(status="preparing", message=f"编队任务准备中，共 {len(vehicles)} 架无人机。")
            
            self._set_progress(status="taking_off", message="开始编队起飞。")
            result = self.adapter.execute_formation_route(
                route=[point.to_api() for point in route],
                slots=slots,
                speed_mps=speed_mps,
                altitude_m=altitude_m,
                progress_callback=lambda **updates: self._set_formation_progress(vehicles, **updates),
            )
            
            self._set_progress(status="completed", formation=result, message="多无人机编队飞行完成。")
        except Exception as exc:
            self._set_progress(status="failed", message=f"多无人机编队飞行失败：{exc}")
    @staticmethod
    def _formation_slots(count: int, shape: str, spacing_m: float) -> list[dict[str, float | str]]:
        slots: list[dict[str, float | str]] = [{"vehicle": "leader", "x": 0.0, "y": 0.0, "z": 0.0}]
        for index in range(1, count):
            if shape == "line":
                x = -spacing_m * index
                y = 0.0
            else:
                side = -1.0 if index % 2 else 1.0
                rank = (index + 1) // 2
                x = -spacing_m * rank
                y = side * spacing_m * rank
            slots.append({"vehicle": f"follower-{index}", "x": round(x, 2), "y": round(y, 2), "z": 0.0})
        return slots

    @staticmethod
    def _formation_targets(leader: Waypoint, slots: list[dict[str, Any]], vehicles: list[str]) -> list[dict[str, Any]]:
        targets: list[dict[str, Any]] = []
        for index, name in enumerate(vehicles):
            slot = slots[index] if index < len(slots) else {"x": 0.0, "y": 0.0, "z": 0.0}
            targets.append(
                {
                    "vehicle": name,
                    "x": float(leader.x) + float(slot.get("x", 0.0)),
                    "y": float(leader.y) + float(slot.get("y", 0.0)),
                    "z": float(leader.z) + float(slot.get("z", 0.0)),
                    "slot": slot,
                }
            )
        return targets

    def _formation_vehicle_states(
        self,
        vehicles: list[str],
        targets: list[dict[str, Any]],
        phase: str,
    ) -> list[dict[str, Any]]:
        by_vehicle = {str(target.get("vehicle", "")): target for target in targets if isinstance(target, dict)}
        states: list[dict[str, Any]] = []
        for name in vehicles:
            telemetry = self.adapter.telemetry(vehicle_name=name)
            target = by_vehicle.get(name)
            error_m = None
            arrived = False
            if target is not None:
                dx = float(telemetry.x) - float(target.get("x", 0.0))
                dy = float(telemetry.y) - float(target.get("y", 0.0))
                dz = float(telemetry.z) - float(target.get("z", 0.0))
                error_m = round((dx * dx + dy * dy + dz * dz) ** 0.5, 2)
                arrived = error_m <= 2.0
            states.append(
                {
                    "vehicle": name,
                    "phase": phase,
                    "target": target,
                    "telemetry": telemetry.to_api(),
                    "error_m": error_m,
                    "arrived": arrived,
                }
            )
        return states

    def _set_formation_progress(self, vehicles: list[str], **updates: Any) -> None:
        targets = updates.get("formation_targets")
        targets = targets if isinstance(targets, list) else []
        phase = str(updates.get("status", "running"))
        updates["formation_targets"] = targets
        updates["formation_vehicles"] = self._formation_vehicle_states(vehicles, targets, phase)
        self._set_progress(**updates)

    def _ensure_airborne(self, altitude_m: float) -> bool:
        target_altitude = abs(float(altitude_m))
        telemetry = self.adapter.telemetry()
        required_altitude = max(1.0, target_altitude - 0.5)
        if float(telemetry.altitude_m) >= required_altitude:
            return True
        self._set_progress(
            status="taking_off",
            message=f"Taking off before search_area: target altitude {target_altitude:.1f}m.",
        )
        try:
            self.executor.takeoff(altitude_m=target_altitude)
        except Exception as exc:
            self._set_progress(status="failed", message=f"Takeoff before search_area failed: {exc}")
            return False
        started_at = time.time()
        while time.time() - started_at <= max(18.0, target_altitude * 2.0):
            if self._mission_stop.is_set():
                self._set_progress(status="stopped", message="Search area stopped during takeoff.")
                return False
            telemetry = self.adapter.telemetry()
            self._set_progress(
                distance_to_waypoint_m=round(max(0.0, target_altitude - float(telemetry.altitude_m)), 2),
                message=f"Climbing before search_area: ALT {float(telemetry.altitude_m):.1f}/{target_altitude:.1f}m.",
            )
            if float(telemetry.altitude_m) >= required_altitude:
                return True
            time.sleep(0.6)
        self._set_progress(status="failed", message="Takeoff before search_area timed out.")
        return False

    def _wait_for_waypoint(
        self,
        waypoint: Any,
        speed_mps: float,
        collision_baseline: int,
        avoidance: bool,
        reactive_avoidance: bool,
        obstacle_distance_m: float,
        avoidance_offset_m: float,
    ) -> bool:
        started_at = time.time()
        timeout_s = max(12.0, self._distance_to_waypoint(waypoint) / max(0.5, speed_mps) + 8.0)
        avoidance_attempts = 0
        last_distance: float | None = None
        stagnant_since: float | None = None
        while time.time() - started_at <= timeout_s:
            if self._mission_stop.is_set():
                self._set_progress(status="stopped", message="Search area stopped by command.")
                return False
            collision = self.adapter.collision_status()
            if collision.has_collided and collision.time_stamp > collision_baseline:
                recovery = self._try_recovery()
                self._mission_stop.set()
                self._set_progress(
                    status="recovered" if recovery is not None else "collision",
                    collision=collision.to_api(),
                    recovery=recovery,
                    message=f"Collision detected with '{collision.object_name or 'unknown'}'; recovery attempted and search stopped.",
                )
                return False
            if avoidance:
                distances = self.adapter.distance_sensors_status(
                    emergency_threshold_m=2.8,
                    side_emergency_threshold_m=1.8,
                )
                self._set_progress(distance_sensors=distances.to_api())
                if distances.available and distances.emergency:
                    stopped_at = self._try_hard_stop()
                    self._mission_stop.set()
                    self._set_progress(
                        status="blocked",
                        recovery={"mode": "hard_stop", "stopped_at": stopped_at},
                        message=(
                            "Distance safety bubble triggered: "
                            f"{distances.emergency_direction} near obstacle "
                            f"(front={distances.front_m}m, left={distances.left_m}m, right={distances.right_m}m); "
                            "search stopped."
                        ),
                    )
                    return False
            if avoidance and reactive_avoidance and avoidance_attempts < 3:
                obstacle = self.adapter.obstacle_status(lookahead_m=obstacle_distance_m)
                self._set_progress(obstacle=obstacle.to_api())
                if obstacle.available and obstacle.blocked:
                    if getattr(obstacle, "risk_level", "") == "critical":
                        recovery = self._try_recovery()
                        self._mission_stop.set()
                        self._set_progress(
                            status="recovered" if recovery is not None else "blocked",
                            recovery=recovery,
                            message="Critical wall-like obstacle detected; recovery attempted and search stopped.",
                        )
                        return False
                    avoidance_attempts += 1
                    if not self._avoid_obstacle(waypoint, obstacle, speed_mps, avoidance_offset_m, avoidance_attempts, collision_baseline):
                        return False
                    started_at = time.time()
                    timeout_s = max(12.0, self._distance_to_waypoint(waypoint) / max(0.5, speed_mps) + 8.0)
                    try:
                        self.executor.goto_local(
                            waypoint.x,
                            waypoint.y,
                            waypoint.z,
                            speed_mps=speed_mps,
                            face_target=True,
                        )
                    except Exception as exc:
                        self._set_progress(status="failed", message=f"Resume waypoint command failed: {exc}")
                        return False
            distance = self._distance_to_waypoint(waypoint)
            self._set_progress(distance_to_waypoint_m=round(distance, 2))
            if distance <= 2.0:
                return True
            telemetry = self.adapter.telemetry()
            if last_distance is not None and abs(last_distance - distance) < 0.25 and float(telemetry.speed_mps) < 0.4:
                stagnant_since = stagnant_since or time.time()
                if time.time() - stagnant_since >= 4.0:
                    recovery = self._try_recovery()
                    self._mission_stop.set()
                    self._set_progress(
                        status="stuck_recovered" if recovery is not None else "stuck",
                        recovery=recovery,
                        message="UAV appears stuck; recovery attempted and search stopped.",
                    )
                    return False
            else:
                stagnant_since = None
            last_distance = distance
            time.sleep(0.5)
        recovery = self._try_recovery()
        self._set_progress(status="recovered" if recovery is not None else "timeout", recovery=recovery, message="Waypoint timeout; recovery attempted.")
        return False

    def _distance_to_waypoint(self, waypoint: Any) -> float:
        telemetry = self.adapter.telemetry()
        return math.sqrt(
            (float(telemetry.x) - float(waypoint.x)) ** 2
            + (float(telemetry.y) - float(waypoint.y)) ** 2
            + (float(telemetry.z) - float(waypoint.z)) ** 2
        )

    def _planned_sub_waypoints(self, goal: Any, enabled: bool, effective_area: MissionArea | None = None) -> list[Waypoint]:
        if not enabled:
            return [goal]
        telemetry = self.adapter.telemetry()
        start = Waypoint(float(telemetry.x), float(telemetry.y), float(goal.z))
        target = Waypoint(float(goal.x), float(goal.y), float(goal.z))
        points = self.adapter.lidar_obstacle_points_world(
            range_m=35.0,
            vertical_window_m=2.5,
            vertical_down_m=1.4,
            denoise_cell_m=2.0,
            min_points_per_cell=3,
        )
        if len(points) < 3:
            distances = self.adapter.distance_sensors_status(
                emergency_threshold_m=2.8,
                side_emergency_threshold_m=1.8,
            )
            if distances.available and not distances.emergency:
                self._set_progress(
                    planner={
                        "ok": True,
                        "reason": "not enough LiDAR obstacle points; direct segment allowed by distance safety bubble",
                        "points": len(points),
                        "fallback": "direct_safe_segment",
                    },
                    distance_sensors=distances.to_api(),
                    message="Planner fallback: no nearby LiDAR obstacles; flying direct segment with distance safety bubble.",
                )
                return [target]
            obstacle = self.adapter.obstacle_status(lookahead_m=10.0)
            if not obstacle.blocked or obstacle.risk_level in {"unknown", "low", "medium"}:
                self._set_progress(
                    planner={
                        "ok": True,
                        "reason": "LiDAR sparse and distance bubble unavailable; cautious direct segment fallback enabled",
                        "points": len(points),
                        "fallback": "direct_cautious_segment",
                    },
                    obstacle=obstacle.to_api(),
                    distance_sensors=distances.to_api(),
                    message="Planner fallback: sparse sensing, no high-risk obstacle detected; continuing with cautious direct segment.",
                )
                return [target]
            self._set_progress(
                status="blocked",
                planner={
                    "ok": False,
                    "reason": "not enough LiDAR points and distance safety bubble is unavailable or unsafe",
                    "points": len(points),
                },
                distance_sensors=distances.to_api(),
                message="Planned avoidance needs LiDAR or safe distance readings; search stopped to avoid blind flight.",
            )
            self._mission_stop.set()
            return []
        plan = plan_local_path(
            start=start,
            goal=target,
            obstacle_points=points,
            grid_size_m=70.0,
            resolution_m=1.5,
            inflation_m=4.0,
            max_waypoints=14,
            bounds=(
                effective_area.x_min,
                effective_area.x_max,
                effective_area.y_min,
                effective_area.y_max,
            ) if effective_area is not None else None,
        )
        self._set_progress(planner=plan.to_api() | {"points": len(points)})
        if not plan.ok:
            obstacle = self.adapter.obstacle_status(lookahead_m=12.0)
            if obstacle.blocked and obstacle.risk_level in {"high", "critical"}:
                recovery = self._try_recovery()
                self._mission_stop.set()
                self._set_progress(
                    status="recovered" if recovery is not None else "blocked",
                    obstacle=obstacle.to_api(),
                    recovery=recovery,
                    message="Planner found no safe path near obstacle; recovery attempted.",
                )
                return []
            self._set_progress(
                planner=plan.to_api() | {"points": len(points), "fallback": "direct_after_plan_fail"},
                obstacle=obstacle.to_api(),
                message="A* planner failed but risk is not high; falling back to direct cautious segment.",
            )
            return [target]
        return plan.waypoints

    def _preflight_area_assessment(self, area: MissionArea, route: list[Any]) -> dict[str, Any]:
        self._set_progress(
            status="pre_scanning",
            message="Pre-scan: collecting LiDAR samples before area search.",
        )
        sample_targets = [
            ((area.x_min + area.x_max) / 2.0, (area.y_min + area.y_max) / 2.0),
            (area.x_min, area.y_min),
            (area.x_max, area.y_min),
            (area.x_max, area.y_max),
            (area.x_min, area.y_max),
        ]
        sampled_points: set[tuple[float, float]] = set()
        for index, (x, y) in enumerate(sample_targets, start=1):
            if self._mission_stop.is_set():
                break
            self._set_progress(message=f"Pre-scan: sampling {index}/{len(sample_targets)} at X {x:.1f}, Y {y:.1f}.")
            try:
                self.adapter.hold_current_position(duration_s=0.6)
                self.executor.face_local(x, y)
                time.sleep(0.25)
            except Exception:
                pass
            points = self.adapter.lidar_obstacle_points_world(
                range_m=55.0,
                vertical_window_m=2.5,
                vertical_down_m=1.4,
                denoise_cell_m=2.0,
                min_points_per_cell=3,
            )
            for px, py in points:
                if area.x_min - 2.0 <= px <= area.x_max + 2.0 and area.y_min - 2.0 <= py <= area.y_max + 2.0:
                    sampled_points.add((round(float(px), 1), round(float(py), 1)))
            self._set_progress(
                message=f"Pre-scan: sample {index}/{len(sample_targets)} done, lidar {len(points)} pts, mapped {len(sampled_points)} obstacle points.",
            )

        zones = self._blocked_zones_from_points(area, sampled_points)
        route_hits = self._count_route_hits(route, zones)
        coverage = self._area_coverage(area, zones)
        risk = self._assessment_risk(coverage, route_hits)
        stop_recommended = risk in {"high", "critical"} and route_hits > 0
        recommendation = self._assessment_recommendation(risk, route_hits, coverage)
        return {
            "available": bool(sampled_points),
            "area": area.to_api(),
            "risk": risk,
            "obstacle_points": len(sampled_points),
            "obstacle_coverage": round(coverage, 3),
            "route_hits": route_hits,
            "blocked_zones": zones[:80],
            "stop_recommended": stop_recommended,
            "recommendation": recommendation,
        }

    @staticmethod
    def _blocked_zones_from_points(area: MissionArea, points: set[tuple[float, float]]) -> list[dict[str, Any]]:
        if not points:
            return []
        cell_size = 2.0
        cells: dict[tuple[int, int], int] = {}
        for x, y in points:
            if not (area.x_min <= x <= area.x_max and area.y_min <= y <= area.y_max):
                continue
            col = int((x - area.x_min) / cell_size)
            row = int((y - area.y_min) / cell_size)
            cells[(col, row)] = cells.get((col, row), 0) + 1
        zones: list[dict[str, Any]] = []
        for (col, row), count in cells.items():
            if count < 4:
                continue
            x_min = area.x_min + col * cell_size
            y_min = area.y_min + row * cell_size
            zones.append(
                {
                    "x_min": round(x_min, 2),
                    "x_max": round(min(area.x_max, x_min + cell_size), 2),
                    "y_min": round(y_min, 2),
                    "y_max": round(min(area.y_max, y_min + cell_size), 2),
                    "points": count,
                    "type": "obstacle",
                }
            )
        zones.sort(key=lambda zone: int(zone["points"]), reverse=True)
        return zones

    @staticmethod
    def _count_route_hits(route: list[Any], zones: list[dict[str, Any]]) -> int:
        hits = 0
        for start, end in zip(route, route[1:]):
            sx = float(getattr(start, "x", 0.0))
            sy = float(getattr(start, "y", 0.0))
            ex = float(getattr(end, "x", 0.0))
            ey = float(getattr(end, "y", 0.0))
            for zone in zones:
                if AgentToolRuntime._segment_intersects_zone(sx, sy, ex, ey, zone, padding_m=1.5):
                    hits += 1
                    break
        return hits

    @staticmethod
    def _segment_intersects_zone(
        sx: float,
        sy: float,
        ex: float,
        ey: float,
        zone: dict[str, Any],
        padding_m: float,
    ) -> bool:
        try:
            x_min = float(zone["x_min"]) - padding_m
            x_max = float(zone["x_max"]) + padding_m
            y_min = float(zone["y_min"]) - padding_m
            y_max = float(zone["y_max"]) + padding_m
        except Exception:
            return False
        if max(sx, ex) < x_min or min(sx, ex) > x_max or max(sy, ey) < y_min or min(sy, ey) > y_max:
            return False
        if abs(sy - ey) <= 0.2:
            return y_min <= sy <= y_max
        if abs(sx - ex) <= 0.2:
            return x_min <= sx <= x_max
        return True

    @staticmethod
    def _replan_route_around_zones(route: list[Any], zones: Any, padding_m: float = 2.0) -> list[Waypoint]:
        if not isinstance(zones, list) or not zones:
            return [Waypoint(float(point.x), float(point.y), float(point.z)) for point in route]
        safe_route: list[Waypoint] = []
        min_segment_m = 3.0
        for start, end in zip(route, route[1:]):
            sx = float(getattr(start, "x", 0.0))
            sy = float(getattr(start, "y", 0.0))
            sz = float(getattr(start, "z", -10.0))
            ex = float(getattr(end, "x", 0.0))
            ey = float(getattr(end, "y", 0.0))
            ez = float(getattr(end, "z", sz))
            if abs(sy - ey) > 0.2:
                continue
            lane_y = sy
            x_low = min(sx, ex)
            x_high = max(sx, ex)
            blocked_intervals: list[tuple[float, float]] = []
            for zone in zones:
                try:
                    zone_y_min = float(zone["y_min"]) - padding_m
                    zone_y_max = float(zone["y_max"]) + padding_m
                    if zone_y_min <= lane_y <= zone_y_max:
                        blocked_intervals.append(
                            (
                                max(x_low, float(zone["x_min"]) - padding_m),
                                min(x_high, float(zone["x_max"]) + padding_m),
                            )
                        )
                except Exception:
                    continue
            intervals = AgentToolRuntime._subtract_intervals(x_low, x_high, blocked_intervals)
            if sx > ex:
                intervals = list(reversed(intervals))
            for seg_start, seg_end in intervals:
                if abs(seg_end - seg_start) < min_segment_m:
                    continue
                a, b = (seg_start, seg_end) if sx <= ex else (seg_end, seg_start)
                if not safe_route or math.hypot(safe_route[-1].x - a, safe_route[-1].y - lane_y) > 0.5:
                    safe_route.append(Waypoint(a, lane_y, sz))
                safe_route.append(Waypoint(b, lane_y, ez))
        return safe_route

    @staticmethod
    def _subtract_intervals(x_low: float, x_high: float, blocked: list[tuple[float, float]]) -> list[tuple[float, float]]:
        intervals = [(float(x_low), float(x_high))]
        for block_start, block_end in sorted(blocked):
            next_intervals: list[tuple[float, float]] = []
            for start, end in intervals:
                if block_end <= start or block_start >= end:
                    next_intervals.append((start, end))
                    continue
                if block_start > start:
                    next_intervals.append((start, min(block_start, end)))
                if block_end < end:
                    next_intervals.append((max(block_end, start), end))
            intervals = next_intervals
        return intervals

    @staticmethod
    def _area_coverage(area: MissionArea, zones: list[dict[str, Any]]) -> float:
        area_size = max(1.0, (float(area.x_max) - float(area.x_min)) * (float(area.y_max) - float(area.y_min)))
        blocked = 0.0
        for zone in zones:
            blocked += max(0.0, float(zone["x_max"]) - float(zone["x_min"])) * max(0.0, float(zone["y_max"]) - float(zone["y_min"]))
        return min(1.0, blocked / area_size)

    @staticmethod
    def _assessment_risk(coverage: float, route_hits: int) -> str:
        if route_hits >= 3 or coverage >= 0.28:
            return "critical"
        if route_hits >= 1 or coverage >= 0.12:
            return "high"
        if coverage >= 0.04:
            return "medium"
        return "low"

    @staticmethod
    def _assessment_recommendation(risk: str, route_hits: int, coverage: float) -> str:
        if risk == "critical":
            return "Area contains dense obstacles or the planned scan route intersects blocked cells; split or shrink the mission area."
        if risk == "high" and route_hits > 0:
            return "Planned scan route crosses suspected obstacles; increase scan_margin_m or split the area."
        if risk == "high":
            return "Area has significant obstacle coverage; proceed only with planned avoidance enabled."
        if risk == "medium":
            return "Area has scattered obstacles; planned avoidance is recommended."
        return "Area appears flyable from pre-scan samples."

    @staticmethod
    def _inset_area(area: MissionArea, margin_m: float) -> MissionArea:
        margin = max(0.0, float(margin_m))
        width = float(area.x_max) - float(area.x_min)
        height = float(area.y_max) - float(area.y_min)
        max_margin = max(0.0, min(width, height) / 2.0 - 1.0)
        margin = min(margin, max_margin)
        if margin <= 0.0:
            return area
        return MissionArea(
            x_min=float(area.x_min) + margin,
            x_max=float(area.x_max) - margin,
            y_min=float(area.y_min) + margin,
            y_max=float(area.y_max) - margin,
        )

    def _avoid_obstacle(
        self,
        waypoint: Any,
        obstacle: Any,
        speed_mps: float,
        offset_m: float,
        attempt: int,
        collision_baseline: int,
    ) -> bool:
        telemetry = self.adapter.telemetry()
        side = str(getattr(obstacle, "recommended_side", "left") or "left")
        side_sign = -1.0 if side == "left" else 1.0
        bypass_x = float(telemetry.x) + min(4.0, max(1.5, float(offset_m) * 0.5))
        bypass_y = float(telemetry.y) + side_sign * max(2.0, float(offset_m))
        bypass_z = float(waypoint.z)
        with self._mission_lock:
            count = int(self._mission_progress.get("avoidance_count") or 0) + 1
        self._set_progress(
            status="avoiding",
            avoidance_count=count,
            obstacle=obstacle.to_api(),
            message=f"Obstacle ahead; bypassing {side} via temporary waypoint {attempt}.",
        )
        try:
            self.executor.goto_local(bypass_x, bypass_y, bypass_z, speed_mps=max(1.0, min(speed_mps, 2.0)))
        except Exception as exc:
            self._set_progress(status="failed", message=f"Avoidance command failed: {exc}")
            return False

        started_at = time.time()
        timeout_s = max(8.0, self._distance_to_xyz(bypass_x, bypass_y, bypass_z) / max(0.5, speed_mps) + 5.0)
        while time.time() - started_at <= timeout_s:
            if self._mission_stop.is_set():
                self._set_progress(status="stopped", message="Search area stopped during avoidance.")
                return False
            collision = self.adapter.collision_status()
            if collision.has_collided and collision.time_stamp > collision_baseline:
                recovery = self._try_recovery()
                self._set_progress(
                    status="recovered" if recovery is not None else "collision",
                    collision=collision.to_api(),
                    recovery=recovery,
                    message="Collision detected during avoidance; recovery attempted.",
                )
                return False
            distance = self._distance_to_xyz(bypass_x, bypass_y, bypass_z)
            self._set_progress(distance_to_waypoint_m=round(distance, 2))
            if distance <= 2.0:
                self._set_progress(status="running", message="Avoidance waypoint reached; resuming route.")
                return True
            time.sleep(0.5)
        recovery = self._try_recovery()
        self._set_progress(status="recovered" if recovery is not None else "timeout", recovery=recovery, message="Avoidance waypoint timeout; recovery attempted.")
        return False

    def _distance_to_xyz(self, x: float, y: float, z: float) -> float:
        telemetry = self.adapter.telemetry()
        return math.sqrt(
            (float(telemetry.x) - float(x)) ** 2
            + (float(telemetry.y) - float(y)) ** 2
            + (float(telemetry.z) - float(z)) ** 2
        )

    def _try_recovery(self) -> dict[str, Any] | None:
        try:
            return self.executor.recover_from_collision(climb_m=8.0, backoff_m=10.0, force_relocate=True)
        except Exception:
            try:
                return {"mode": "hard_stop", "stopped_at": self.executor.hard_stop()}
            except Exception:
                pass
            return None

    def _try_hard_stop(self) -> dict[str, float] | None:
        try:
            return self.executor.hard_stop()
        except Exception:
            try:
                self.executor.hover()
            except Exception:
                pass
            return None

    def _set_progress(self, **updates: Any) -> None:
        if "telemetry" not in updates:
            try:
                updates["telemetry"] = self.adapter.telemetry().to_api()
            except Exception:
                pass
        status_value = str(updates.get("status", "") or "")
        with self._mission_lock:
            self._mission_progress.update(updates)
            self._mission_progress["updated_at"] = time.time()
        self._apply_post_mission_behavior(status_value)

    def _apply_post_mission_behavior(self, status: str) -> None:
        if self._post_mission_behavior != "keep_state":
            return
        if status not in {"completed", "recovered", "stuck_recovered", "blocked", "timeout", "failed"}:
            return
        now = time.time()
        # De-duplicate rapid terminal updates from the same mission thread.
        with self._terminal_hold_guard:
            if now - self._terminal_hold_applied_at < 0.8:
                return
            self._terminal_hold_applied_at = now
        try:
            self._hold_current_position_quietly()
        except Exception:
            pass
