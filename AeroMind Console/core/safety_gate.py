from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from core.validation import ToolValidator, ValidationResult


@dataclass(frozen=True)
class SafetyLimits:
    min_altitude_m: float = 1.0
    max_altitude_m: float = 30.0
    max_speed_mps: float = 4.0
    boundary_x_min: float = -120.0
    boundary_x_max: float = 120.0
    boundary_y_min: float = -120.0
    boundary_y_max: float = 120.0
    allowed_vehicles: tuple[str, ...] = ("Drone1", "Drone2", "Drone3", "Drone4", "Drone5", "Drone6")
    allowed_cameras: tuple[str, ...] = ("front_center", "bottom_center", "0", "front_left", "front_right")
    critical_tools: tuple[str, ...] = ("land", "return_home")
    emergency_altitude_m: float = 3.0
    emergency_speed_mps: float = 2.0


@dataclass(frozen=True)
class SafetyDecision:
    accepted: bool
    tool: str
    args: dict[str, Any] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    reason: str = ""
    preflight_checks: dict[str, Any] = field(default_factory=dict)

    def to_api(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "accepted": self.accepted,
            "tool": self.tool,
            "args": self.args,
            "warnings": self.warnings,
        }
        if self.reason:
            payload["reason"] = self.reason
        if self.preflight_checks:
            payload["preflight_checks"] = self.preflight_checks
        return payload


@dataclass(frozen=True)
class SafetyState:
    has_collision: bool = False
    collision_object: str = ""
    obstacle_risk_level: str = "unknown"
    obstacle_blocked: bool = False
    distance_emergency: bool = False
    distance_direction: str = ""
    current_altitude_m: float = 0.0
    current_speed_mps: float = 0.0
    current_x: float = 0.0
    current_y: float = 0.0
    current_z: float = 0.0


class SafetyGate:
    def __init__(self, limits: SafetyLimits | None = None) -> None:
        self.limits = limits or SafetyLimits()
        self._state_checker: Callable[[], SafetyState] | None = None

    def set_state_checker(self, checker: Callable[[], SafetyState]) -> None:
        self._state_checker = checker

    def validate(self, tool: str, args: dict[str, Any] | None, connected: bool = False) -> SafetyDecision:
        normalized_tool = str(tool or "").strip()
        normalized_args = dict(args or {})
        warnings: list[str] = []
        preflight_checks: dict[str, Any] = {}

        if normalized_tool not in self.allowed_tools():
            return self._reject(normalized_tool, f"Unknown tool: {normalized_tool}")

        if normalized_tool not in {"detect_objects", "report_target"} and not connected:
            warnings.append("AirSim is not currently marked connected; the tool runtime will retry connection.")

        confirmed = bool(normalized_args.pop("confirmed", False) or normalized_args.pop("confirm", False))
        if normalized_tool in self.limits.critical_tools and not confirmed:
            return self._reject(normalized_tool, f"Tool '{normalized_tool}' requires confirmed=true")

        altitude_key = "safe_altitude_m" if normalized_tool == "return_home" else "altitude_m"
        if normalized_tool in {"takeoff", "return_home", "search_area", "formation_flight", "collect_images"}:
            altitude = self._float_arg(normalized_args, altitude_key, 8.0)
            rejected = self._validate_altitude(altitude)
            if rejected:
                return self._reject(normalized_tool, rejected)

        validation_result = ToolValidator.validate(normalized_tool, normalized_args)
        if not validation_result.ok:
            error_msg = "; ".join(validation_result.errors)
            return self._reject(normalized_tool, f"Validation failed: {error_msg}")
        normalized_args = validation_result.data

        if self._state_checker:
            safety_state = self._state_checker()
            preflight_checks = self._run_preflight_checks(normalized_tool, normalized_args, safety_state)
            
            if preflight_checks.get("blocked"):
                reason = preflight_checks.get("reason", "Preflight check failed")
                return self._reject(normalized_tool, reason, preflight_checks=preflight_checks)
            
            for check in preflight_checks.get("checks", []):
                if check.get("status") == "warn":
                    warnings.append(check.get("detail", ""))

        if normalized_tool == "takeoff":
            altitude = self._float_arg(normalized_args, "altitude_m", 8.0)
            rejected = self._validate_altitude(altitude)
            if rejected:
                return self._reject(normalized_tool, rejected)
            normalized_args["altitude_m"] = altitude

        elif normalized_tool == "return_home":
            altitude = self._float_arg(normalized_args, "safe_altitude_m", 8.0)
            rejected = self._validate_altitude(altitude)
            if rejected:
                return self._reject(normalized_tool, rejected)
            normalized_args["safe_altitude_m"] = altitude

        elif normalized_tool == "goto_local":
            x = self._float_arg(normalized_args, "x", 0.0)
            y = self._float_arg(normalized_args, "y", 0.0)
            z = self._float_arg(normalized_args, "z", -8.0)
            speed = self._float_arg(normalized_args, "speed_mps", 2.0)
            rejected = self._validate_local_position(x, y, z, speed)
            if rejected:
                return self._reject(normalized_tool, rejected)
            normalized_args.update({"x": x, "y": y, "z": z, "speed_mps": speed})

        elif normalized_tool == "autonomous_nav":
            x = self._float_arg(normalized_args, "x", 0.0)
            y = self._float_arg(normalized_args, "y", 0.0)
            z = self._float_arg(normalized_args, "z", -8.0)
            speed = self._float_arg(normalized_args, "speed_mps", 1.5)
            rejected = self._validate_local_position(x, y, z, speed)
            if rejected:
                return self._reject(normalized_tool, rejected)
            normalized_args.update(
                {
                    "x": x,
                    "y": y,
                    "z": z,
                    "speed_mps": speed,
                    "replan_interval_s": self._float_arg(normalized_args, "replan_interval_s", 1.0),
                    "control_dt_s": self._float_arg(normalized_args, "control_dt_s", 0.2),
                    "lookahead_m": self._float_arg(normalized_args, "lookahead_m", 3.0),
                    "timeout_s": self._float_arg(normalized_args, "timeout_s", 120.0),
                }
            )

        elif normalized_tool == "waypoint_route":
            waypoints = normalized_args.get("waypoints", [])
            waypoints = waypoints if isinstance(waypoints, list) else []
            if not waypoints:
                return self._reject(normalized_tool, "waypoint_route requires at least one waypoint")
            if len(waypoints) > 40:
                return self._reject(normalized_tool, "waypoint_route supports at most 40 waypoints")
            speed = self._float_arg(normalized_args, "speed_mps", 2.0)
            clean_waypoints: list[dict[str, float]] = []
            for index, item in enumerate(waypoints, start=1):
                if not isinstance(item, dict):
                    return self._reject(normalized_tool, f"waypoint {index} must be an object")
                x = self._float_arg(item, "x", 0.0)
                y = self._float_arg(item, "y", 0.0)
                z = self._float_arg(item, "z", -8.0)
                rejected = self._validate_local_position(x, y, z, speed)
                if rejected:
                    return self._reject(normalized_tool, f"waypoint {index}: {rejected}")
                clean_waypoints.append({"x": x, "y": y, "z": z})
            normalized_args.update(
                {
                    "waypoints": clean_waypoints,
                    "speed_mps": speed,
                    "avoidance": bool(normalized_args.get("avoidance", True)),
                    "hold_at_end": bool(normalized_args.get("hold_at_end", True)),
                }
            )

        elif normalized_tool == "formation_flight":
            count = int(self._float_arg(normalized_args, "count", 3.0))
            if count < 2 or count > 6:
                return self._reject(normalized_tool, "formation count must be within 2..6")
            altitude = self._float_arg(normalized_args, "altitude_m", 8.0)
            speed = self._float_arg(normalized_args, "speed_mps", 2.0)
            distance = self._float_arg(normalized_args, "distance_m", 40.0)
            spacing = self._float_arg(normalized_args, "spacing_m", 6.0)
            rejected = self._validate_local_position(distance, 0.0, -abs(altitude), speed)
            if rejected:
                return self._reject(normalized_tool, rejected)
            if spacing < 2.0 or spacing > 20.0:
                return self._reject(normalized_tool, "formation spacing_m must be within 2m..20m")
            normalized_args.update(
                {
                    "count": count,
                    "shape": str(normalized_args.get("shape", "v")).strip() or "v",
                    "distance_m": distance,
                    "altitude_m": altitude,
                    "spacing_m": spacing,
                    "speed_mps": speed,
                    "avoidance": bool(normalized_args.get("avoidance", True)),
                }
            )

        elif normalized_tool == "recover_from_collision":
            climb = self._float_arg(normalized_args, "climb_m", 8.0)
            backoff = self._float_arg(normalized_args, "backoff_m", 10.0)
            force_relocate = bool(normalized_args.get("force_relocate", True))
            if climb < 2.0 or climb > 15.0:
                return self._reject(normalized_tool, "climb_m must be within 2m..15m")
            if backoff < 2.0 or backoff > 20.0:
                return self._reject(normalized_tool, "backoff_m must be within 2m..20m")
            normalized_args.update({"climb_m": climb, "backoff_m": backoff, "force_relocate": force_relocate})

        elif normalized_tool == "search_area":
            area = normalized_args.get("area", {})
            area = area if isinstance(area, dict) else {}
            altitude = self._float_arg(normalized_args, "altitude_m", 8.0)
            speed = self._float_arg(normalized_args, "speed_mps", 2.0)
            spacing = self._float_arg(normalized_args, "spacing_m", 10.0)
            obstacle_distance = self._float_arg(normalized_args, "obstacle_distance_m", 8.0)
            avoidance_offset = self._float_arg(normalized_args, "avoidance_offset_m", 6.0)
            scan_margin = self._float_arg(normalized_args, "scan_margin_m", 4.0)
            pre_scan = bool(normalized_args.get("pre_scan", False))
            pre_scan_stop_on_high_risk = bool(normalized_args.get("pre_scan_stop_on_high_risk", True))
            x_min = self._float_arg(area, "x_min", 0.0)
            x_max = self._float_arg(area, "x_max", 60.0)
            y_min = self._float_arg(area, "y_min", -20.0)
            y_max = self._float_arg(area, "y_max", 20.0)
            rejected = self._validate_search_area(
                x_min,
                x_max,
                y_min,
                y_max,
                altitude,
                speed,
                spacing,
                obstacle_distance,
                avoidance_offset,
                scan_margin,
            )
            if rejected:
                return self._reject(normalized_tool, rejected)
            normalized_args.update(
                {
                    "target": str(normalized_args.get("target", "object")).strip() or "object",
                    "area": {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
                    "altitude_m": altitude,
                    "speed_mps": speed,
                    "spacing_m": spacing,
                    "avoidance": bool(normalized_args.get("avoidance", True)),
                    "planned_avoidance": bool(normalized_args.get("planned_avoidance", True)),
                    "obstacle_distance_m": obstacle_distance,
                    "avoidance_offset_m": avoidance_offset,
                    "scan_margin_m": scan_margin,
                    "pre_scan": pre_scan,
                    "pre_scan_stop_on_high_risk": pre_scan_stop_on_high_risk,
                    "strategy": str(normalized_args.get("strategy", "lawnmower")).strip() or "lawnmower",
                }
            )

        elif normalized_tool == "collect_images":
            area = normalized_args.get("area", {})
            area = area if isinstance(area, dict) else {}
            altitude = self._float_arg(normalized_args, "altitude_m", 8.0)
            speed = self._float_arg(normalized_args, "speed_mps", 1.5)
            spacing = self._float_arg(normalized_args, "spacing_m", 8.0)
            obstacle_distance = self._float_arg(normalized_args, "obstacle_distance_m", 8.0)
            avoidance_offset = self._float_arg(normalized_args, "avoidance_offset_m", 6.0)
            scan_margin = self._float_arg(normalized_args, "scan_margin_m", 4.0)
            x_min = self._float_arg(area, "x_min", 0.0)
            x_max = self._float_arg(area, "x_max", 40.0)
            y_min = self._float_arg(area, "y_min", -15.0)
            y_max = self._float_arg(area, "y_max", 15.0)
            rejected = self._validate_search_area(
                x_min,
                x_max,
                y_min,
                y_max,
                altitude,
                speed,
                spacing,
                obstacle_distance,
                avoidance_offset,
                scan_margin,
            )
            if rejected:
                return self._reject(normalized_tool, rejected)
            normalized_args.update(
                {
                    "area": {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
                    "altitude_m": altitude,
                    "speed_mps": speed,
                    "spacing_m": spacing,
                    "camera": str(normalized_args.get("camera", "front_center")).strip() or "front_center",
                    "dataset_name": str(normalized_args.get("dataset_name", "collect")).strip() or "collect",
                    "max_images": int(normalized_args.get("max_images", 200)),
                    "capture_interval_s": self._float_arg(normalized_args, "capture_interval_s", 1.0),
                    "avoidance": bool(normalized_args.get("avoidance", True)),
                    "planned_avoidance": bool(normalized_args.get("planned_avoidance", True)),
                    "obstacle_distance_m": obstacle_distance,
                    "avoidance_offset_m": avoidance_offset,
                    "scan_margin_m": scan_margin,
                    "pre_scan": bool(normalized_args.get("pre_scan", False)),
                    "pre_scan_stop_on_high_risk": bool(normalized_args.get("pre_scan_stop_on_high_risk", True)),
                }
            )

        elif normalized_tool == "detect_objects":
            camera = str(normalized_args.get("camera", "front_center")).strip() or "front_center"
            if camera not in self.limits.allowed_cameras:
                return self._reject(normalized_tool, f"Camera '{camera}' is not allowed")
            normalized_args["camera"] = camera
            normalized_args["target"] = str(normalized_args.get("target", "object")).strip() or "object"
        elif normalized_tool == "replay_trajectory":
            log_folder = str(normalized_args.get("log_folder", "")).strip()
            if not log_folder:
                return self._reject(normalized_tool, "log_folder is required")
            normalized_args["log_folder"] = log_folder
            normalized_args["speed_mps"] = self._float_arg(normalized_args, "speed_mps", 3.5)
            normalized_args["turn_threshold_deg"] = self._float_arg(normalized_args, "turn_threshold_deg", 30.0)
            normalized_args["min_dist_m"] = self._float_arg(normalized_args, "min_dist_m", 0.5)
            normalized_args["spawn_target"] = bool(normalized_args.get("spawn_target", True))
            map_spawn_json = normalized_args.get("map_spawn_json")
            normalized_args["map_spawn_json"] = str(map_spawn_json).strip() if isinstance(map_spawn_json, str) else None
            normalized_args["target_object_name"] = str(normalized_args.get("target_object_name", "ReplayTarget")).strip() or "ReplayTarget"
            normalized_args["draw_trail"] = bool(normalized_args.get("draw_trail", True))
            normalized_args["trail_thickness"] = self._float_arg(normalized_args, "trail_thickness", 8.0)

        return SafetyDecision(True, normalized_tool, normalized_args, warnings, preflight_checks=preflight_checks)

    def _run_preflight_checks(self, tool: str, args: dict[str, Any], state: SafetyState) -> dict[str, Any]:
        checks: list[dict[str, str]] = []
        blocked = False
        reason = ""

        def add_check(key: str, label: str, status: str, detail: str) -> None:
            checks.append({"key": key, "label": label, "status": status, "detail": detail})

        if state.has_collision:
            status = "error" if tool != "recover_from_collision" else "warn"
            detail = f"Current collision detected with '{state.collision_object}'" if state.collision_object else "Current collision detected"
            add_check("collision", "Collision Status", status, detail)
            if tool != "recover_from_collision" and tool != "stop":
                blocked = True
                reason = "Cannot execute flight command while in collision state"

        if state.obstacle_blocked and state.obstacle_risk_level in {"high", "critical"}:
            add_check("obstacle", "Obstacle Risk", "warn", f"Sensor reports obstacle risk: {state.obstacle_risk_level}")

        if state.distance_emergency:
            add_check("distance", "Distance Sensor", "warn", f"Distance sensor reports a close object in {state.distance_direction} direction")

        if tool in {"takeoff", "goto_local", "autonomous_nav", "waypoint_route", "search_area", "formation_flight", "collect_images"}:
            if state.current_altitude_m < self.limits.emergency_altitude_m:
                add_check("altitude", "Current Altitude", "warn", f"Current altitude {state.current_altitude_m:.1f}m is below emergency threshold")

            target_alt = args.get("altitude_m") or args.get("safe_altitude_m")
            if target_alt and state.current_altitude_m > target_alt:
                add_check("altitude", "Target Altitude", "warn", f"Current altitude higher than target; will descend")

        return {"overall": "blocked" if blocked else "caution" if any(c["status"] == "warn" for c in checks) else "ready", "checks": checks, "blocked": blocked, "reason": reason}

    def allowed_tools(self) -> tuple[str, ...]:
        return (
            "takeoff",
            "hover",
            "stop",
            "land",
            "return_home",
            "goto_local",
            "autonomous_nav",
            "waypoint_route",
            "formation_flight",
            "recover_from_collision",
            "search_area",
            "collect_images",
            "detect_objects",
            "report_target",
            "replay_trajectory",
        )

    def _validate_altitude(self, altitude_m: float) -> str:
        if altitude_m < self.limits.min_altitude_m:
            return f"Altitude {altitude_m:.1f}m is below minimum {self.limits.min_altitude_m:.1f}m"
        if altitude_m > self.limits.max_altitude_m:
            return f"Altitude {altitude_m:.1f}m exceeds maximum {self.limits.max_altitude_m:.1f}m"
        return ""

    def _validate_local_position(self, x: float, y: float, z: float, speed_mps: float) -> str:
        altitude = abs(z)
        altitude_error = self._validate_altitude(altitude)
        if altitude_error:
            return altitude_error
        if not (self.limits.boundary_x_min <= x <= self.limits.boundary_x_max):
            return f"x={x:.1f} is outside allowed boundary [{self.limits.boundary_x_min:.0f}..{self.limits.boundary_x_max:.0f}]"
        if not (self.limits.boundary_y_min <= y <= self.limits.boundary_y_max):
            return f"y={y:.1f} is outside allowed boundary [{self.limits.boundary_y_min:.0f}..{self.limits.boundary_y_max:.0f}]"
        if speed_mps <= 0:
            return "speed_mps must be greater than 0"
        if speed_mps > self.limits.max_speed_mps:
            return f"speed_mps {speed_mps:.1f} exceeds maximum {self.limits.max_speed_mps:.1f} m/s"
        return ""

    def _validate_search_area(
        self,
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
        altitude_m: float,
        speed_mps: float,
        spacing_m: float,
        obstacle_distance_m: float,
        avoidance_offset_m: float,
        scan_margin_m: float,
    ) -> str:
        altitude_error = self._validate_altitude(altitude_m)
        if altitude_error:
            return altitude_error
        if x_min >= x_max:
            return "area.x_min must be less than area.x_max"
        if y_min >= y_max:
            return "area.y_min must be less than area.y_max"
        if x_min < self.limits.boundary_x_min or x_max > self.limits.boundary_x_max:
            return f"search area x range [{x_min:.1f}..{x_max:.1f}] is outside allowed boundary"
        if y_min < self.limits.boundary_y_min or y_max > self.limits.boundary_y_max:
            return f"search area y range [{y_min:.1f}..{y_max:.1f}] is outside allowed boundary"
        if speed_mps <= 0 or speed_mps > self.limits.max_speed_mps:
            return f"speed_mps must be within 0..{self.limits.max_speed_mps:.1f}"
        if spacing_m < 2.0:
            return "spacing_m must be at least 2m"
        if spacing_m > 30.0:
            return "spacing_m must be 30m or less"
        if obstacle_distance_m < 3.0 or obstacle_distance_m > 20.0:
            return "obstacle_distance_m must be within 3m..20m"
        if avoidance_offset_m < 2.0 or avoidance_offset_m > 15.0:
            return "avoidance_offset_m must be within 2m..15m"
        if scan_margin_m < 0.0 or scan_margin_m > 12.0:
            return "scan_margin_m must be within 0m..12m"
        return ""

    @staticmethod
    def _float_arg(args: dict[str, Any], key: str, default: float) -> float:
        try:
            return float(args.get(key, default))
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _reject(tool: str, reason: str, preflight_checks: dict[str, Any] | None = None) -> SafetyDecision:
        return SafetyDecision(False, tool, {}, [], reason, preflight_checks=preflight_checks or {})
