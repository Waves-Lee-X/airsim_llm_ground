from __future__ import annotations

from dataclasses import dataclass, field
import json
import logging
import math
from pathlib import Path
import threading
import time
from typing import Any, Callable

logger = logging.getLogger(__name__)

from config import nav_config
from core.airsim_adapter import AirSimAdapter
from core.obstacle_avoidance import plan_local_path, plan_local_path_on_slice, AvoidancePlan
from core.path_planner import lawnmower_path
from core.path_planner import Waypoint
from core.temporal_grid import TemporalVoxelGrid
from core.depth_camera import DepthCamera
from core.structured_log import log_event, log_mission_event
from core import preflight
from core import replay
from core.task_executor import TaskExecutor
from core.task_schema import MissionArea
from core.vision_detector import VisionDetector


DATASET_ROOT = Path(__file__).resolve().parents[1] / "datasets" / "yolo_collect"
VLA_ROOT = Path(__file__).resolve().parents[2] / "VLA"
DEFAULT_MAP_SPAWN_JSON = VLA_ROOT / "data" / "meta" / "map_spawnarea_info.json"


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


class ActiveMission:
    """Thread-safe mission handle — ensures only one mission runs at a time."""

    def __init__(self, stop_event: threading.Event, name: str = "pending") -> None:
        self.thread: threading.Thread | None = None
        self.stop_event = stop_event
        self.name = name

    def stop(self) -> None:
        self.stop_event.set()

    def is_stopped(self) -> bool:
        return self.stop_event.is_set()


class AgentToolRuntime:
    """Mission-level tools that an LLM agent is allowed to call."""

    def __init__(self, adapter: AirSimAdapter, executor: TaskExecutor, detector: VisionDetector | None = None) -> None:
        self.adapter = adapter
        self.executor = executor
        self.detector = detector or VisionDetector()
        self._temporal_grid: TemporalVoxelGrid | None = None
        self._depth_camera: DepthCamera | None = None
        self._mission_lock = threading.RLock()
        self._active_mission: ActiveMission | None = None
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
            "replay_trajectory": self._replay_trajectory,
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
                "replay_trajectory",
                "Replay a recorded AirSim log trajectory and optionally spawn target object from mark.json metadata.",
                ("log_folder",),
                {
                    "log_folder": "Directory containing per-frame JSON logs such as 000001.json.",
                    "speed_mps": "Replay speed in meters per second.",
                    "turn_threshold_deg": "Split route at corners larger than this threshold.",
                    "min_dist_m": "Minimum distance for waypoint deduplication.",
                    "spawn_target": "Spawn target object from mark.json and map spawn metadata.",
                    "map_spawn_json": "Optional path to map_spawnarea_info.json.",
                    "target_object_name": "Spawned target object name in UE.",
                    "draw_trail": "Draw persistent red trail while replaying.",
                    "trail_thickness": "Trail line thickness.",
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
        altitude_m = float(args.get("altitude_m", nav_config.default_altitude_m))
        self.executor.takeoff(altitude_m=altitude_m)
        self._set_progress(status="holding", tool="takeoff", message=f"Takeoff complete; holding at {altitude_m:.1f}m.")
        return ToolResult(True, "takeoff", f"Takeoff command sent to {altitude_m:.1f}m", {"altitude_m": altitude_m})

    def _hover(self, args: dict[str, Any]) -> ToolResult:
        if self._active_mission:
            self._active_mission.stop()
        self.executor.hover()
        self._set_progress(status="paused", message="Hover command sent; active tool mission paused.")
        return ToolResult(True, "hover", "Hover command sent")

    def _stop(self, args: dict[str, Any]) -> ToolResult:
        if self._active_mission:
            self._active_mission.stop()
        self.executor.stop()
        self._set_progress(status="stopped", message="Stop command sent; active tool mission stopped.")
        return ToolResult(True, "stop", "Stop command sent")

    def _land(self, args: dict[str, Any]) -> ToolResult:
        if self._active_mission:
            self._active_mission.stop()
        self.executor.land()
        self._set_progress(status="landing", tool="land", message="Land command sent; active tool mission stopped.")
        return ToolResult(True, "land", "Land command sent")

    def _return_home(self, args: dict[str, Any]) -> ToolResult:
        if self._active_mission:
            self._active_mission.stop()
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
        self._prepare_new_mission()
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
        thread = threading.Thread(
            target=self._run_waypoint_route,
            args=(route, speed_mps, avoidance, hold_at_end),
            name="aeromind-waypoint-route",
            daemon=True,
        )
        self._active_mission.thread = thread
        self._active_mission.name = "aeromind-waypoint-route"
        thread.start()
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
        self._prepare_new_mission()
        if not self._ensure_airborne(abs(goal.z)):
            return ToolResult(False, "autonomous_nav", "Unable to take off before autonomous navigation")
        collision_baseline = self.adapter.collision_status().time_stamp
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
        thread = threading.Thread(
            target=self._run_autonomous_nav,
            args=(goal, speed_mps, replan_interval_s, control_dt_s, lookahead_m, timeout_s, collision_baseline),
            name="aeromind-autonomous-nav",
            daemon=True,
        )
        self._active_mission.thread = thread
        self._active_mission.name = "aeromind-autonomous-nav"
        thread.start()
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
        self._prepare_new_mission()
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
        thread = threading.Thread(
            target=self._run_formation_route,
            args=(route, slots, altitude_m, speed_mps, avoidance),
            name="aeromind-formation-flight",
            daemon=True,
        )
        self._active_mission.thread = thread
        self._active_mission.name = "aeromind-formation-flight"
        thread.start()
        return ToolResult(
            True,
            "formation_flight",
            f"编队飞行已启动：{count} 架，队形 {shape.upper()}",
            {"route": [point.to_api() for point in route], "formation": {"count": count, "shape": shape, "spacing_m": spacing_m, "slots": slots, "vehicles": vehicles}, "speed_mps": speed_mps, "avoidance": avoidance, "execution_mode": "multi_vehicle"},
        )

    def _recover_from_collision(self, args: dict[str, Any]) -> ToolResult:
        if self._active_mission:
            self._active_mission.stop()
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
        pre_scan = bool(args.get("pre_scan", False))
        pre_scan_stop_on_high_risk = bool(args.get("pre_scan_stop_on_high_risk", True))
        target = str(args.get("target", "object"))
        effective_area = self._inset_area(area, scan_margin_m)
        route = lawnmower_path(effective_area, altitude_m, spacing_m=spacing_m)
        if not route:
            return ToolResult(False, "search_area", "No waypoints generated for search area")
        collision_baseline = self.adapter.collision_status().time_stamp

        self._prepare_new_mission()
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
        thread = threading.Thread(
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
        self._active_mission.thread = thread
        self._active_mission.name = "aeromind-search-area"
        thread.start()
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
        pre_scan = bool(args.get("pre_scan", False))
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

        self._prepare_new_mission()
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
        thread = threading.Thread(
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
        self._active_mission.thread = thread
        self._active_mission.name = "aeromind-collect-images"
        thread.start()
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

    def _replay_trajectory(self, args: dict[str, Any]) -> ToolResult:
        log_folder = Path(str(args.get("log_folder", "")).strip())
        speed_mps = float(args.get("speed_mps", 3.5))
        turn_threshold_deg = float(args.get("turn_threshold_deg", 30.0))
        min_dist_m = float(args.get("min_dist_m", 0.5))
        spawn_target = bool(args.get("spawn_target", True))
        map_spawn_json = Path(str(args.get("map_spawn_json", "")).strip()) if args.get("map_spawn_json") else DEFAULT_MAP_SPAWN_JSON
        target_object_name = str(args.get("target_object_name", "ReplayTarget")).strip() or "ReplayTarget"
        draw_trail = bool(args.get("draw_trail", True))
        trail_thickness = float(args.get("trail_thickness", 8.0))

        if not log_folder.exists() or not log_folder.is_dir():
            return ToolResult(False, "replay_trajectory", f"log_folder does not exist: {log_folder}")

        frames = self._load_replay_frames(log_folder)
        if len(frames) < 2:
            return ToolResult(False, "replay_trajectory", "Not enough replay frames in log_folder")
        waypoints = self._deduplicate_replay_frames(frames, min_dist_m=min_dist_m)
        if len(waypoints) < 2:
            return ToolResult(False, "replay_trajectory", "Not enough waypoints after deduplication")

        self._prepare_new_mission()
        self._set_progress(
            status="running",
            tool="replay_trajectory",
            current_waypoint_index=0,
            total_waypoints=len(waypoints),
            planner={
                "ok": True,
                "phase": "replay_prepare",
                "reason": f"Replay prepared with {len(waypoints)} waypoints",
            },
            message="Trajectory replay started.",
            started_at=time.time(),
        )
        thread = threading.Thread(
            target=self._run_replay_trajectory,
            args=(
                log_folder,
                waypoints,
                speed_mps,
                turn_threshold_deg,
                spawn_target,
                map_spawn_json,
                target_object_name,
                draw_trail,
                trail_thickness,
            ),
            name="aeromind-replay-trajectory",
            daemon=True,
        )
        self._active_mission.thread = thread
        self._active_mission.name = "aeromind-replay-trajectory"
        thread.start()
        return ToolResult(
            True,
            "replay_trajectory",
            f"Trajectory replay started with {len(waypoints)} waypoints.",
            {
                "log_folder": str(log_folder),
                "waypoint_count": len(waypoints),
                "speed_mps": speed_mps,
                "turn_threshold_deg": turn_threshold_deg,
                "spawn_target": spawn_target,
                "map_spawn_json": str(map_spawn_json),
                "target_object_name": target_object_name,
                "draw_trail": draw_trail,
            },
        )

    def _run_replay_trajectory(
        self,
        log_folder: Path,
        waypoints: list[dict[str, Any]],
        speed_mps: float,
        turn_threshold_deg: float,
        spawn_target: bool,
        map_spawn_json: Path,
        target_object_name: str,
        draw_trail: bool,
        trail_thickness: float,
    ) -> None:
        try:
            if spawn_target:
                spawned = self._spawn_target_from_replay_mark(log_folder, map_spawn_json, target_object_name)
                if spawned:
                    self._set_progress(message=f"Replay target spawned: {spawned}")
            self.adapter.flush_persistent_markers()
            first = waypoints[0]
            first_pos = first["position"]
            first_ori = first["orientation"]
            self.adapter.set_vehicle_pose(
                first_pos[0],
                first_pos[1],
                first_pos[2],
                first_ori[0],
                first_ori[1],
                first_ori[2],
                first_ori[3],
                ignore_collision=True,
            )
            time.sleep(1.0)
            self.executor.takeoff(altitude_m=max(2.0, abs(float(first_pos[2]))))
            break_idx = self._split_replay_by_turns(waypoints, threshold_deg=turn_threshold_deg)
            trail_points: list[dict[str, float]] = []
            segment_total = max(1, len(break_idx) - 1)
            for segment_idx in range(segment_total):
                if self._active_mission.stop_event.is_set():
                    self._set_progress(status="stopped", message="Trajectory replay stopped by command.")
                    return
                start_i, end_i = break_idx[segment_idx], break_idx[segment_idx + 1]
                segment = waypoints[start_i + 1 : end_i + 1]
                if not segment:
                    continue
                first_target = segment[0]["position"]
                yaw_deg = self._yaw_to_target(first_target[0], first_target[1])
                self.adapter.rotate_to_yaw(yaw_deg, timeout_s=5.0, margin_deg=2.0)
                path = [
                    {"x": float(item["position"][0]), "y": float(item["position"][1]), "z": float(item["position"][2])}
                    for item in segment
                ]
                self._set_progress(
                    status="running",
                    current_waypoint_index=end_i,
                    total_waypoints=len(waypoints),
                    message=f"Replay segment {segment_idx + 1}/{segment_total}",
                )
                self.adapter.move_on_path(
                    path=path,
                    speed_mps=speed_mps,
                    lookahead=-1.0,
                    adaptive_lookahead=1.0,
                    timeout_s=max(30.0, len(path) * 2.0),
                    forward_only=True,
                )
                if draw_trail:
                    trail_points.extend(path)
                    self.adapter.plot_line_strip(
                        points=trail_points,
                        color_rgba=[1.0, 0.0, 0.0, 1.0],
                        thickness=trail_thickness,
                    )
            self.executor.land()
            self._set_progress(status="completed", message="Trajectory replay completed.")
        except Exception as exc:
            self._set_progress(status="failed", message=f"Trajectory replay failed: {exc}")

    @staticmethod
    def _load_replay_frames(log_folder: Path) -> list[dict[str, Any]]:
        return replay.load_replay_frames(log_folder)

    @staticmethod
    def _deduplicate_replay_frames(frames: list[dict[str, Any]], min_dist_m: float = 0.5) -> list[dict[str, Any]]:
        return replay.deduplicate_replay_frames(frames, min_dist_m)

    @staticmethod
    def _turn_angle_deg(waypoints: list[dict[str, Any]], i: int) -> float:
        return replay.turn_angle_deg(waypoints, i)

    def _split_replay_by_turns(self, waypoints: list[dict[str, Any]], threshold_deg: float) -> list[int]:
        return replay.split_replay_by_turns(waypoints, threshold_deg)

    def _spawn_target_from_replay_mark(self, log_folder: Path, map_spawn_json: Path, object_name: str) -> str | None:
        seq_dir = log_folder.parent
        mark_path = seq_dir / "mark.json"
        if not mark_path.exists() or not map_spawn_json.exists():
            return None
        try:
            mark = json.loads(mark_path.read_text(encoding="utf-8"))
            target = mark.get("target") if isinstance(mark, dict) else None
            coord = target.get("position") if isinstance(target, dict) else None
            if not isinstance(coord, list) or len(coord) < 3:
                return None
            map_name = seq_dir.parent.name
            meta = json.loads(map_spawn_json.read_text(encoding="utf-8"))
            areas = meta.get(map_name, []) if isinstance(meta, dict) else []
            matched = self._closest_spawn_area(coord, areas)
            if matched is None or len(matched) < 18:
                return None
            position = {"x": float(matched[9]), "y": float(matched[10]), "z": float(matched[11])}
            orientation = {"x": float(matched[13]), "y": float(matched[14]), "z": float(matched[15]), "w": float(matched[12])}
            asset_name = str(matched[16])
            scale = float(matched[17])
            return self.adapter.spawn_object(
                object_name=object_name,
                asset_name=asset_name,
                position=position,
                orientation=orientation,
                scale=scale,
                destroy_existing=True,
            )
        except Exception as spawn_err:
            logger.warning("Failed to spawn asset %s at map %s: %s", object_name, map_name, spawn_err)
            return None

    @staticmethod
    def _closest_spawn_area(coord: list[float], areas: Any) -> list[Any] | None:
        return replay.closest_spawn_area(coord, areas)

    def _yaw_to_target(self, target_x: float, target_y: float) -> float:
        telemetry = self.adapter.telemetry()
        dx = float(target_x) - float(telemetry.x)
        dy = float(target_y) - float(telemetry.y)
        if abs(dx) <= 0.05 and abs(dy) <= 0.05:
            return 0.0
        return math.degrees(math.atan2(dy, dx))

    def _detect_objects(self, args: dict[str, Any]) -> ToolResult:
        frame = self.adapter.camera_frame()
        if frame is None:
            return ToolResult(False, "detect_objects", f"No camera frame available: {self.adapter.last_error}")
        timeout_s = max(3.0, float(args.get("timeout_s", 10.0)))
        try:
            detection_result = self.detector.detect_async(frame.data).result(timeout=timeout_s)
        except Exception as detect_err:
            return ToolResult(
                False,
                "detect_objects",
                f"Detection timed out or failed: {detect_err}",
                {
                    "target": str(args.get("target", "object")),
                    "camera": frame.camera_name,
                    "image_type": frame.image_type,
                    "bytes": len(frame.data),
                    "ok": False,
                    "detections": [],
                },
            )
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
                        self._active_mission.stop_event.set()
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
                if self._active_mission.stop_event.is_set() or saved >= max_images:
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
        except Exception as hold_err:
            logger.warning("hold_current_position failed, trying hover fallback: %s", hold_err)
            try:
                self.executor.hover()
            except Exception as hover_err:
                self._set_progress(message=f"hold_current_position failed, hover fallback also failed: {hover_err}")

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
        engine = self.adapter._engine

        # Disable position-hold for the entire nav loop; we manage velocity
        # via engine nav-control mode.  Restored on exit.
        self.adapter._stop_position_hold()

        # -- seed tick: read telemetry (no velocity yet) --------------------
        nav = self.adapter.nav_telemetry()
        tel: Any = nav["telemetry"]
        self._ensure_temporal_grid(float(tel.x), float(tel.y), float(tel.z))

        # -- start engine nav-control loop (zero queue overhead) -------------
        engine.start_nav_control()

        current = Waypoint(float(tel.x), float(tel.y), float(goal.z))
        target = goal
        raw_vx, raw_vy, raw_vz = 0.0, 0.0, 0.0
        vx_smooth, vy_smooth, vz_smooth = 0.0, 0.0, 0.0
        ema_alpha = 0.65  # smoothing factor: higher = more responsive

        try:
            while not self._active_mission.stop_event.is_set():
                # -- read telemetry from shared cache (no RPC!) ---------------
                cache = self.adapter.get_nav_cache()
                cache_age = time.time() - cache.get("updated_at", 0.0)

                # Health check: if engine nav mode died or cache is stale,
                # restart nav control and fall back to direct RPC for this tick.
                if cache_age > 1.0 or not engine._nav_active:
                    if not engine._nav_active:
                        logger.warning("Engine nav control stopped; restarting")
                        engine.start_nav_control()
                    # Fallback: direct RPC to refresh the cache
                    nav = self.adapter.nav_telemetry()
                    tel = nav["telemetry"]
                else:
                    tel = cache["telemetry"]

                current = Waypoint(float(tel.x), float(tel.y), float(goal.z))

                # -- safety checks with pre-fetched collision & distances -----
                result = self._check_flight_safety(
                    telemetry=tel, goal=goal, current=current,
                    started_at=started_at, timeout_s=timeout_s,
                    collision_baseline=collision_baseline,
                    use_distance_safety=use_distance_safety,
                    mission_label=mission_label, active_status=active_status,
                    reached_message=reached_message,
                    current_waypoint_index=current_waypoint_index,
                    total_waypoints=total_waypoints,
                    on_tick=on_tick,
                    _prefetched_collision=cache["collision"],
                    _prefetched_distances=cache["distances"],
                )
                if result is not None:
                    return result  # True=reached, False=blocked/timeout/collision

                # -- replan (batched lidar read every replan_interval_s) ------
                now = time.time()
                if now - last_replan_at >= replan_interval_s or not plan_waypoints:
                    plan_waypoints = self._replan_to_goal(
                        current=current, goal=goal,
                        planner_phase=planner_phase,
                        blocked_message=blocked_message,
                    )
                    if plan_waypoints is None:
                        return False
                    last_replan_at = now

                # -- compute velocity, apply EMA smoothing --------------------
                target = self._lookahead_target(current, plan_waypoints, lookahead_m) or goal
                raw_vx, raw_vy, raw_vz = self._compute_velocity(
                    telemetry=tel, target=target, goal_z=float(goal.z),
                    speed_mps=speed_mps, control_dt_s=control_dt_s,
                )
                vx_smooth = ema_alpha * raw_vx + (1 - ema_alpha) * vx_smooth
                vy_smooth = ema_alpha * raw_vy + (1 - ema_alpha) * vy_smooth
                vz_smooth = ema_alpha * raw_vz + (1 - ema_alpha) * vz_smooth

                # -- set velocity for engine tick (shared memory, no RPC) -----
                engine.set_nav_velocity(vx_smooth, vy_smooth, vz_smooth, max(control_dt_s, 0.6))

                self._set_progress(
                    status=active_status,
                    current_waypoint_index=current_waypoint_index,
                    total_waypoints=total_waypoints,
                    message=f"{mission_label} v={math.hypot(vx_smooth, vy_smooth):.2f} m/s to "
                            f"({target.x:.1f}, {target.y:.1f})",
                )

                time.sleep(max(0.02, control_dt_s * 0.35))
            return False
        finally:
            engine.stop_nav_control()
            self._hold_current_position_quietly()

    @staticmethod
    def _compute_velocity(
        telemetry: Any,
        target: Waypoint,
        goal_z: float,
        speed_mps: float,
        control_dt_s: float,
    ) -> tuple[float, float, float]:
        """Compute (vx, vy, vz) for the given telemetry and target.  Does NOT
        send anything — caller pipes the result into ``nav_tick`` on the next
        iteration."""
        dx = float(target.x) - float(telemetry.x)
        dy = float(target.y) - float(telemetry.y)
        dz = goal_z - float(telemetry.z)
        horizontal_distance = max(0.001, math.hypot(dx, dy))
        speed = min(float(speed_mps), max(0.4, horizontal_distance / max(control_dt_s, 0.1)))
        vx = dx / horizontal_distance * speed
        vy = dy / horizontal_distance * speed
        vz = max(-1.0, min(1.0, dz / max(control_dt_s * 4.0, 0.4)))
        return vx, vy, vz

    def _check_flight_safety(
        self,
        telemetry: Any,
        goal: Waypoint,
        current: Waypoint,
        started_at: float,
        timeout_s: float,
        collision_baseline: int,
        use_distance_safety: bool,
        mission_label: str,
        active_status: str,
        reached_message: str,
        current_waypoint_index: int,
        total_waypoints: int,
        on_tick: Callable | None = None,
        _prefetched_collision: Any = None,
        _prefetched_distances: Any = None,
    ) -> bool | None:
        """Return True (reached), False (blocked), or None (continue).

        Set *_prefetched_collision* / *_prefetched_distances* to avoid extra
        RPC round-trips inside the closed-loop drive hot path.
        """
        elapsed = time.time() - started_at
        if elapsed > timeout_s:
            self._hold_current_position_quietly()
            self._set_progress(status="timeout", message=f"{mission_label} timeout; holding position.")
            return False

        goal_distance = math.hypot(goal.x - current.x, goal.y - current.y)
        altitude_error = float(goal.z) - float(telemetry.z)
        self._set_progress(distance_to_waypoint_m=round(goal_distance, 2))

        if on_tick is not None and not on_tick(current, goal_distance, elapsed):
            self._hold_current_position_quietly()
            self._set_progress(status=active_status, message=f"{mission_label} capture limit reached; holding position.")
            return True

        if goal_distance <= nav_config.arrival_distance_m and abs(altitude_error) <= nav_config.arrival_altitude_error_m:
            self._hold_current_position_quietly()
            self._set_progress(
                status=active_status,
                current_waypoint_index=current_waypoint_index,
                total_waypoints=total_waypoints,
                distance_to_waypoint_m=0.0,
                message=reached_message,
            )
            return True

        collision = _prefetched_collision if _prefetched_collision is not None else self.adapter.collision_status()
        if collision.has_collided and collision.time_stamp > collision_baseline:
            recovery = self._try_recovery()
            self._active_mission.stop_event.set()
            self._set_progress(
                status="recovered" if recovery is not None else "collision",
                collision=collision.to_api(),
                recovery=recovery,
                message=f"{mission_label} stopped after collision; recovery attempted.",
            )
            return False

        if use_distance_safety:
            distances = (
                _prefetched_distances
                if _prefetched_distances is not None
                else self.adapter.distance_sensors_status(
                    emergency_threshold_m=nav_config.distance_emergency_threshold_m,
                    side_emergency_threshold_m=nav_config.side_emergency_threshold_m,
                )
            )
            self._set_progress(distance_sensors=distances.to_api())
            if distances.available and distances.emergency:
                stopped_at = self._try_hard_stop()
                self._active_mission.stop_event.set()
                self._set_progress(
                    status="blocked",
                    recovery={"mode": "hard_stop", "stopped_at": stopped_at},
                    message=f"{mission_label} stopped by distance safety bubble.",
                )
                return False

        return None

    def _replan_to_goal(
        self,
        current: Waypoint,
        goal: Waypoint,
        planner_phase: str,
        blocked_message: str,
    ) -> list[Waypoint] | None:
        """Replan path from current position to goal. Returns waypoints or None if blocked."""
        self._update_temporal_grid()
        grid_slice = self._get_2d_slice_for_planning(z_center=float(goal.z), half_height=nav_config.grid_half_height)
        if grid_slice is not None:
            plan = plan_local_path_on_slice(
                start=current, goal=goal, grid_slice=grid_slice, max_waypoints=nav_config.max_waypoints,
            )
            grid_info = grid_slice.to_api()
        else:
            plan = AvoidancePlan(False, [], "temporal grid not available", {})
            grid_info = {}
        self._set_progress(
            planner=plan.to_api() | {"phase": planner_phase, "grid": grid_info},
            message=plan.reason,
        )
        if not plan.ok:
            alt_goal, alt_reason = self._try_alt_replan(current, goal)
            if alt_goal is not None:
                self._set_progress(
                    planner=plan.to_api() | {"phase": planner_phase, "replanned": True,
                                             "reason": f"altitude adjusted: {alt_reason}"},
                    message=f"Replanned at alternative altitude: {alt_reason}",
                )
                return [alt_goal]
            alt_info = self._check_alt_slices(current, goal)
            self._hold_current_position_quietly()
            self._set_progress(status="blocked", message=f"{blocked_message}: {plan.reason}{alt_info}")
            return None
        if grid_slice is not None and hasattr(grid_slice, 'blocked') and grid_slice.width > 0:
            total_cells = grid_slice.width * grid_slice.height
            self._obstacle_density = len(getattr(grid_slice, 'blocked', set())) / max(1, total_cells)
        else:
            self._obstacle_density = 0.0
        return plan.waypoints or [goal]

    def _try_alt_replan(self, current: Waypoint, goal: Waypoint) -> tuple[Waypoint | None, str]:
        """If the current altitude slice is blocked but an upper/lower slice is
        free, return an adjusted goal at the viable altitude.  Prefers climbing
        (upper slice) over descending for safety."""
        if self._temporal_grid is None:
            return None, ""
        for offset, label in [(-nav_config.alt_offset_m, "climb"), (nav_config.alt_offset_m, "descend")]:
            alt_slice = self._temporal_grid.extract_2d_slice(
                z_center=float(goal.z) + offset, half_height=nav_config.alt_slice_half_height,
                treat_unknown_as="free",
            )
            alt_plan = plan_local_path_on_slice(current, goal, alt_slice, max_waypoints=nav_config.max_alt_waypoints)
            if alt_plan.ok:
                alt_z = float(goal.z) + offset
                return Waypoint(float(goal.x), float(goal.y), alt_z), f"{label} {abs(offset):.0f}m to z={alt_z:.1f}"
        return None, ""

    def _check_alt_slices(self, current: Waypoint, goal: Waypoint) -> str:
        """Check upper/lower altitude slices for alternative paths. Returns info string."""
        if self._temporal_grid is None:
            return ""
        upper_slice = self._temporal_grid.extract_2d_slice(
            z_center=float(goal.z) - nav_config.alt_offset_m, half_height=nav_config.alt_slice_half_height, treat_unknown_as="free",
        )
        lower_slice = self._temporal_grid.extract_2d_slice(
            z_center=float(goal.z) + nav_config.alt_offset_m, half_height=nav_config.alt_slice_half_height, treat_unknown_as="free",
        )
        upper_free = (plan_local_path_on_slice(current, goal, upper_slice, max_waypoints=nav_config.max_alt_waypoints)).ok
        lower_free = (plan_local_path_on_slice(current, goal, lower_slice, max_waypoints=nav_config.max_alt_waypoints)).ok
        if upper_free and lower_free:
            return " (upper +4m and lower -4m slices are free — consider altitude change)"
        elif upper_free:
            return " (upper +4m slice is free — consider climbing)"
        elif lower_free:
            return " (lower -4m slice is free — consider descending)"
        return ""

    def _apply_velocity_command(
        self,
        telemetry: Any,
        target: Waypoint,
        goal_z: float,
        speed_mps: float,
        control_dt_s: float,
        active_status: str,
        current_waypoint_index: int,
        total_waypoints: int,
        mission_label: str,
    ) -> None:
        """Compute and send velocity command with density-adjusted speed."""
        dx = float(target.x) - float(telemetry.x)
        dy = float(target.y) - float(telemetry.y)
        dz = goal_z - float(telemetry.z)
        horizontal_distance = max(0.001, math.hypot(dx, dy))
        density_factor = max(nav_config.density_factor_min, 1.0 - getattr(self, '_obstacle_density', 0.0) * nav_config.density_factor_multiplier)
        effective_speed = max(nav_config.speed_min_mps, float(speed_mps) * density_factor)
        speed = min(effective_speed, max(0.4, horizontal_distance / max(control_dt_s, 0.1)))
        vx = dx / horizontal_distance * speed
        vy = dy / horizontal_distance * speed
        vz = max(-1.0, min(1.0, dz / max(control_dt_s * 4.0, 0.4)))
        self.executor.move_velocity(vx=vx, vy=vy, vz=vz, duration_s=max(control_dt_s, 0.5))
        self._set_progress(
            status=active_status,
            current_waypoint_index=current_waypoint_index,
            total_waypoints=total_waypoints,
            message=f"{mission_label} velocity cmd vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}.",
        )

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
            logger.exception("Autonomous navigation worker failed")
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
                route = self._execute_pre_scan(route, effective_area, pre_scan_stop_on_high_risk)
                if route is None:
                    return
            self._fly_search_route(
                target=target, route=route, speed_mps=speed_mps,
                collision_baseline=collision_baseline,
                avoidance=avoidance, planned_avoidance=planned_avoidance,
                obstacle_distance_m=obstacle_distance_m,
                avoidance_offset_m=avoidance_offset_m,
                effective_area=effective_area,
            )
        except Exception as exc:
            logger.exception("Search area worker failed")
            self._set_progress(status="failed", message=f"Search area worker failed: {exc}")

    def _execute_pre_scan(
        self,
        route: list[Any],
        effective_area: MissionArea,
        pre_scan_stop_on_high_risk: bool,
    ) -> list[Any] | None:
        """Pre-scan the mission area and replan route if needed. Returns adjusted route or None to abort."""
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
        if not assessment.get("stop_recommended"):
            self._set_progress(
                planner={
                    "ok": True,
                    "phase": "pre_scan_clear",
                    "reason": f"预扫描完成，发现 {assessment.get('obstacle_points', 0)} 个障碍点，原航线可执行",
                    "waypoints": [point.to_api() for point in route],
                },
                message="Pre-scan complete; route cleared for A* segment planning.",
            )
            return route

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
            self._active_mission.stop_event.set()
            self._set_progress(
                status="blocked",
                message=f"Pre-scan could not create a safe scan route: {assessment.get('recommendation', 'unsafe scan area')}",
            )
            return None

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
        return route

    def _fly_search_route(
        self,
        target: str,
        route: list[Any],
        speed_mps: float,
        collision_baseline: int,
        avoidance: bool,
        planned_avoidance: bool,
        obstacle_distance_m: float,
        avoidance_offset_m: float,
        effective_area: MissionArea,
    ) -> None:
        """Execute the search route waypoint by waypoint with A* sub-planning."""
        for index, waypoint in enumerate(route, start=1):
            if self._active_mission.stop_event.is_set():
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
                if self._active_mission.stop_event.is_set():
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
                        sub_waypoint.x, sub_waypoint.y, sub_waypoint.z,
                        speed_mps=commanded_speed,
                        face_target=not planned_avoidance,
                    )
                except Exception as exc:
                    self._set_progress(status="failed", message=f"Waypoint {index} command failed: {exc}")
                    return
                if not self._wait_for_waypoint(
                    sub_waypoint, speed_mps, collision_baseline,
                    avoidance=avoidance, reactive_avoidance=not planned_avoidance,
                    obstacle_distance_m=obstacle_distance_m,
                    avoidance_offset_m=avoidance_offset_m,
                ):
                    return
            if not self._wait_for_waypoint(
                waypoint, speed_mps, collision_baseline,
                avoidance=avoidance, reactive_avoidance=not planned_avoidance,
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
                if self._active_mission.stop_event.is_set():
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
                while not self._active_mission.stop_event.is_set():
                    distance = self._distance_to_waypoint(waypoint)
                    self._set_progress(current_waypoint_index=index, total_waypoints=len(route), distance_to_waypoint_m=round(distance, 2), message=f"正在飞向航点 {index}/{len(route)}。")
                    if distance <= 2.0:
                        break
                    time.sleep(0.25)
            if hold_at_end:
                self.executor.hover()
            self._set_progress(status="completed", message="航点飞行完成。")
        except Exception as exc:
            logger.exception("Waypoint route worker failed")
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
            if self._active_mission.stop_event.is_set():
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
            if self._active_mission.stop_event.is_set():
                self._set_progress(status="stopped", message="Search area stopped by command.")
                return False
            collision = self.adapter.collision_status()
            if collision.has_collided and collision.time_stamp > collision_baseline:
                recovery = self._try_recovery()
                self._active_mission.stop_event.set()
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
                    self._active_mission.stop_event.set()
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
                        self._active_mission.stop_event.set()
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
                    self._active_mission.stop_event.set()
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

    def _replan_blocked_waypoint(
        self, start: Waypoint, target: Waypoint, grid_slice: Any,
    ) -> list[Waypoint] | None:
        """Try to find an alternative waypoint when the original is blocked."""
        goal_cell = grid_slice.world_to_cell(target.x, target.y)
        if goal_cell is not None:
            free_goal = grid_slice.nearest_free(goal_cell, max_radius=16)
            if free_goal is not None and free_goal != goal_cell:
                wx, wy = grid_slice.cell_to_world(free_goal)
                alt_target = Waypoint(wx, wy, target.z)
                plan = plan_local_path_on_slice(start, alt_target, grid_slice, max_waypoints=14)
                if plan.ok:
                    self._set_progress(
                        planner=plan.to_api() | {"replanned": True, "reason": f"waypoint shifted to nearest free cell"},
                        message=f"Replanned: waypoint ({target.x:.1f},{target.y:.1f}) shifted to free cell ({wx:.1f},{wy:.1f}).",
                    )
                    return plan.waypoints

        if self._temporal_grid is not None:
            for alt_offset in (-4.0, 4.0):
                alt_z = float(target.z) + alt_offset
                alt_slice = self._temporal_grid.extract_2d_slice(
                    z_center=alt_z, half_height=2.0, treat_unknown_as="free",
                )
                alt_plan = plan_local_path_on_slice(start, target, alt_slice, max_waypoints=12)
                if alt_plan.ok:
                    alt_target = Waypoint(target.x, target.y, alt_z)
                    alt_plan_full = plan_local_path_on_slice(start, alt_target, alt_slice, max_waypoints=14)
                    if alt_plan_full.ok:
                        self._set_progress(
                            planner=alt_plan_full.to_api() | {"replanned": True, "reason": f"altitude adjusted by {alt_offset:+.0f}m"},
                            message=f"Replanned: waypoint altitude shifted {alt_offset:+.0f}m to z={alt_z:.1f} for obstacle clearance.",
                        )
                        return alt_plan_full.waypoints

        return None

    def _planned_sub_waypoints(self, goal: Any, enabled: bool, effective_area: MissionArea | None = None) -> list[Waypoint]:
        if not enabled:
            return [goal]
        telemetry = self.adapter.telemetry()
        start = Waypoint(float(telemetry.x), float(telemetry.y), float(goal.z))
        target = Waypoint(float(goal.x), float(goal.y), float(goal.z))
        self._update_temporal_grid()
        grid_slice = self._get_2d_slice_for_planning(z_center=float(goal.z), half_height=8.0)

        if grid_slice is None:
            distances = self.adapter.distance_sensors_status(
                emergency_threshold_m=2.8,
                side_emergency_threshold_m=1.8,
            )
            if distances.available and not distances.emergency:
                self._set_progress(
                    planner={
                        "ok": True,
                        "reason": "temporal grid not available; direct segment allowed by distance safety bubble",
                        "fallback": "direct_safe_segment",
                    },
                    distance_sensors=distances.to_api(),
                    message="Planner fallback: temporal grid unavailable; flying direct segment with distance safety bubble.",
                )
                return [target]
            obstacle = self.adapter.obstacle_status(lookahead_m=10.0)
            if not obstacle.blocked or obstacle.risk_level in {"unknown", "low", "medium"}:
                self._set_progress(
                    planner={
                        "ok": True,
                        "reason": "temporal grid unavailable; cautious direct segment fallback enabled",
                        "fallback": "direct_cautious_segment",
                    },
                    obstacle=obstacle.to_api(),
                    distance_sensors=distances.to_api(),
                    message="Planner fallback: temporal grid unavailable; continuing with cautious direct segment.",
                )
                return [target]
            self._set_progress(
                status="blocked",
                planner={"ok": False, "reason": "temporal grid unavailable and distance safety bubble is unsafe"},
                distance_sensors=distances.to_api(),
                message="Planned avoidance needs temporal grid or safe distance readings; search stopped to avoid blind flight.",
            )
            self._active_mission.stop_event.set()
            return []

        plan = plan_local_path_on_slice(
            start=start,
            goal=target,
            grid_slice=grid_slice,
            max_waypoints=14,
        )
        self._set_progress(planner=plan.to_api())
        if not plan.ok:
            replanned = self._replan_blocked_waypoint(start, target, grid_slice)
            if replanned is not None:
                return replanned
            obstacle = self.adapter.obstacle_status(lookahead_m=12.0)
            if obstacle.blocked and obstacle.risk_level in {"high", "critical"}:
                recovery = self._try_recovery()
                self._active_mission.stop_event.set()
                self._set_progress(
                    status="recovered" if recovery is not None else "blocked",
                    obstacle=obstacle.to_api(),
                    recovery=recovery,
                    message="Planner found no safe path and no alternative waypoint near obstacle; recovery attempted.",
                )
                return []
            self._set_progress(
                planner=plan.to_api() | {"fallback": "skip_blocked_waypoint"},
                obstacle=obstacle.to_api(),
                message=f"A* planner failed to reach waypoint ({target.x:.1f}, {target.y:.1f}); skipping blocked waypoint.",
            )
            return []
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
            if self._active_mission.stop_event.is_set():
                break
            self._set_progress(message=f"Pre-scan: sampling {index}/{len(sample_targets)} at X {x:.1f}, Y {y:.1f}.")
            try:
                self.executor.face_local(x, y)
                time.sleep(0.15)
            except Exception as face_err:
                self._set_progress(message=f"Pre-scan face_local failed at ({x:.1f}, {y:.1f}): {face_err}")
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
        return preflight.blocked_zones_from_points(area, points)

    @staticmethod
    def _count_route_hits(route: list[Any], zones: list[dict[str, Any]]) -> int:
        return preflight.count_route_hits(route, zones)

    @staticmethod
    def _segment_intersects_zone(
        sx: float,
        sy: float,
        ex: float,
        ey: float,
        zone: dict[str, Any],
        padding_m: float,
    ) -> bool:
        return preflight.segment_intersects_zone(sx, sy, ex, ey, zone, padding_m)

    @staticmethod
    def _replan_route_around_zones(route: list[Any], zones: Any, padding_m: float = 2.0) -> list[Waypoint]:
        return preflight.replan_route_around_zones(route, zones, padding_m)

    @staticmethod
    def _subtract_intervals(x_low: float, x_high: float, blocked: list[tuple[float, float]]) -> list[tuple[float, float]]:
        return preflight.subtract_intervals(x_low, x_high, blocked)

    @staticmethod
    def _area_coverage(area: MissionArea, zones: list[dict[str, Any]]) -> float:
        return preflight.area_coverage(area, zones)

    @staticmethod
    def _assessment_risk(coverage: float, route_hits: int) -> str:
        return preflight.assessment_risk(coverage, route_hits)

    @staticmethod
    def _assessment_recommendation(risk: str, route_hits: int, coverage: float) -> str:
        return preflight.assessment_recommendation(risk, route_hits, coverage)

    @staticmethod
    def _inset_area(area: MissionArea, margin_m: float) -> MissionArea:
        return preflight.inset_area(area, margin_m)

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
            if self._active_mission.stop_event.is_set():
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
            safe_heading_deg = None
            if self._temporal_grid is not None:
                telemetry = self.adapter.telemetry()
                drone_x, drone_y, drone_z = float(telemetry.x), float(telemetry.y), float(telemetry.z)
                check_distance = 6.0
                best_dir = 0.0
                best_blocked = 999999
                for angle_deg in range(0, 360, 45):
                    rad = math.radians(angle_deg)
                    check_x = drone_x + math.cos(rad) * check_distance
                    check_y = drone_y + math.sin(rad) * check_distance
                    if self._temporal_grid.is_path_blocked(check_x, check_y, drone_z, half_height=6.0):
                        continue
                    idx = self._temporal_grid._world_to_voxel(check_x, check_y, drone_z)
                    if idx is not None:
                        col, row, _layer = idx
                        blocked_in_dir = 0
                        for dc in range(-2, 3):
                            for dr in range(-2, 3):
                                if self._temporal_grid.is_confirmed_occupied(col + dc, row + dr, 0):
                                    blocked_in_dir += 1
                        if blocked_in_dir < best_blocked:
                            best_blocked = blocked_in_dir
                            best_dir = float(angle_deg)
                if best_blocked < 999999:
                    safe_heading_deg = best_dir

            if safe_heading_deg is not None:
                rad = math.radians(safe_heading_deg)
                retreat_x = float(telemetry.x) + math.cos(rad) * nav_config.recovery_backoff_m
                retreat_y = float(telemetry.y) + math.sin(rad) * nav_config.recovery_backoff_m
                self.adapter.recover_from_collision_with_direction(
                    climb_m=nav_config.recovery_climb_m, retreat_x=retreat_x, retreat_y=retreat_y,
                )
                return {"mode": "smart_recovery", "heading_deg": safe_heading_deg,
                        "retreat_x": retreat_x, "retreat_y": retreat_y}
            return self.executor.recover_from_collision(
                climb_m=nav_config.recovery_climb_m, backoff_m=nav_config.recovery_backoff_m, force_relocate=True,
            )
        except Exception:
            logger.exception("Collision recovery failed")
            try:
                return {"mode": "hard_stop", "stopped_at": self.executor.hard_stop()}
            except Exception:
                logger.exception("Recovery hard_stop fallback also failed")
            return None

    def _try_hard_stop(self) -> dict[str, float] | None:
        try:
            return self.executor.hard_stop()
        except Exception as hs_err:
            self._set_progress(message=f"hard_stop failed: {hs_err}")
            try:
                self.executor.hover()
            except Exception as hover_err:
                self._set_progress(message=f"hard_stop hover fallback also failed: {hover_err}")
            return None

    def _ensure_temporal_grid(self, x: float, y: float, z: float) -> TemporalVoxelGrid:
        self._temporal_grid = TemporalVoxelGrid(
            center_x=x, center_y=y, center_z=z,
            size_xy_m=100.0, size_z_m=24.0, resolution_m=1.0,
            hit_confirm=3, miss_confirm=5, inflation_m=1.5,
        )
        self._depth_camera = DepthCamera(
            camera_name=self.adapter.active_camera_name or "front_center",
            max_range_m=40.0, sample_step=8, max_points_per_frame=2000,
        )
        return self._temporal_grid

    def _update_temporal_grid(self) -> None:
        if self._temporal_grid is None:
            return

        engine = self.adapter._engine
        if engine._nav_active:
            # Nav mode: read LiDAR from the shared cache that the engine
            # refreshes during _nav_tick_impl.  No _exec_rpc → no queue
            # round-trip → nav loop stays unblocked.
            raw_lidar, (px, py, pz) = self.adapter.get_shared_lidar()
            if not raw_lidar:
                return
        else:
            raw_lidar, (px, py, pz) = self.adapter.lidar_obstacle_points_world(
                range_m=35.0, vertical_window_m=3.0, vertical_down_m=2.0,
                denoise_cell_m=1.0, min_points_per_cell=1,
                return_3d=True, return_position=True,
            )

        if raw_lidar:
            self._temporal_grid.add_points_3d(raw_lidar)
            # Write 2D projection to shared nav-cache so the WebSocket map
            # can show obstacle points without any RPC contention.
            lidar_2d: list[tuple[float, float]] = [(x, y) for (x, y, _z) in raw_lidar]
            self.adapter.update_nav_cache({"lidar_points": lidar_2d})

        if self._temporal_grid.should_recenter(px, py, pz):
            self._temporal_grid.recenter(px, py, pz)

        if self._depth_camera is not None and not engine._nav_active:
            cloud = self._depth_camera.capture(self.adapter)
            if cloud.ok and cloud.sampled_point_count > 0:
                self._temporal_grid.add_ray_casts(
                    origin=cloud.camera_position,
                    endpoints=cloud.points_world,
                )

    def _get_2d_slice_for_planning(self, z_center: float, half_height: float = 8.0):
        if self._temporal_grid is None:
            return None
        return self._temporal_grid.extract_2d_slice(
            z_center=z_center, half_height=half_height, treat_unknown_as="free",
        )

    def _prepare_new_mission(self, timeout: float | None = None) -> threading.Event:
        if timeout is None:
            timeout = nav_config.mission_prepare_timeout_s
        if self._active_mission is not None:
            self._active_mission.stop()
            old_thread = self._active_mission.thread
            if old_thread is not None and old_thread.is_alive():
                old_thread.join(timeout=timeout)
                if old_thread.is_alive():
                    logger.warning(
                        "Previous mission thread '%s' did not stop within %.1fs",
                        self._active_mission.name, timeout,
                    )
        stop_event = threading.Event()
        self._active_mission = ActiveMission(stop_event=stop_event, name="pending")
        return stop_event

    def _set_progress(self, **updates: Any) -> None:
        if "telemetry" not in updates:
            try:
                updates["telemetry"] = self.adapter.telemetry().to_api()
            except Exception as telem_err:
                logger.warning("Telemetry fetch in _set_progress failed: %s", telem_err)
        status_value = str(updates.get("status", "") or "")
        with self._mission_lock:
            self._mission_progress.update(updates)
            self._mission_progress["updated_at"] = time.time()
        # Emit structured log for terminal/significant state transitions
        if status_value in {"completed", "failed", "blocked", "collision", "recovered", "timeout", "stopped", "running"}:
            log_mission_event(
                "status_change",
                progress=self._mission_progress,
                new_status=status_value,
                message=str(updates.get("message", "") or ""),
            )
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
        except Exception as hold_err:
            logger.warning("Post-mission hold_current_position failed: %s", hold_err)
