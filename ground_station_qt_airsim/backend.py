from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Callable

from ground_station_qt_airsim.config import GroundStationAirSimConfig
from ground_station_qt_airsim.models import CommandRecord, TeamAssignment, TelemetryData, VehicleBinding, Waypoint, metres_between_local


LogFn = Callable[[str, str], None]


@dataclass
class ActiveWaypointMission:
    points: list[Waypoint]
    current_index: int = 0


@dataclass
class ExpertNavigationMission:
    target_x: float
    target_y: float
    target_z: float
    started_at: float
    max_time_s: float = 90.0
    safety_radius_m: float = 2.2
    max_speed_mps: float = 2.0
    step_duration_s: float = 0.35
    altitude_gain: float = 0.35
    last_tick_at: float = 0.0
    previous_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    avoid_direction: str | None = None
    avoid_counter: int = 0
    recovery_counter: int = 0
    recovery_direction: str | None = None
    last_collision_stamp: int = 0
    phase: str = "direct"
    clear_ticks: int = 0
    stuck_recovery_count: int = 0
    distance_history: list[float] | None = None
    best_horizontal_m: float = float("inf")
    no_progress_ticks: int = 0


@dataclass
class HoldPositionMission:
    x: float
    y: float
    z: float
    yaw_deg: float
    last_tick_at: float = 0.0
    refresh_s: float = 0.8


class AirSimBackend:
    def __init__(self, config: GroundStationAirSimConfig, log: LogFn) -> None:
        self.config = config
        self.log = log
        self.client = None
        self.airsim = None
        self.connected = False
        self.vehicle_bindings: list[VehicleBinding] = []
        self.telemetry: dict[int, TelemetryData] = {}
        self.team_state: dict[int, TeamAssignment] = {}
        self.command_history: list[CommandRecord] = []
        self._command_index = 0
        self.shape_name = "line"
        self.spacing_m = float(config.formation.default_spacing_m)
        self.altitude_offset_m = float(config.formation.default_altitude_offset_m)
        self.command_speed_mps = float(config.airsim.default_speed_mps)
        self._last_formation_tick = 0.0
        self._last_refresh_at = 0.0
        self._waypoint_missions: dict[int, ActiveWaypointMission] = {}
        self._expert_missions: dict[int, ExpertNavigationMission] = {}
        self._hold_missions: dict[int, HoldPositionMission] = {}
        self._home_positions: dict[int, tuple[float, float, float]] = {}

    def connect(self) -> None:
        try:
            import airsim  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(
                "AirSim Python package is required. Install it with: pip install airsim"
            ) from exc

        self.airsim = airsim
        self.client = airsim.MultirotorClient(
            ip=self.config.airsim.host,
            port=self.config.airsim.port,
            timeout_value=self.config.airsim.timeout_s,
        )
        self.client.confirmConnection()
        self.connected = True
        self.vehicle_bindings = self._discover_vehicles()
        self.telemetry = {
            binding.sysid: TelemetryData(
                sysid=binding.sysid,
                vehicle_name=binding.vehicle_name,
                display_name=binding.display_name,
            )
            for binding in self.vehicle_bindings
        }
        self.team_state = {binding.sysid: TeamAssignment() for binding in self.vehicle_bindings}
        if self.config.airsim.auto_enable_api_control:
            for binding in self.vehicle_bindings:
                self.client.enableApiControl(True, vehicle_name=binding.vehicle_name)
        self._capture_home_positions()
        self.log(
            f"AirSim connected: {self.config.airsim.host}:{self.config.airsim.port}, vehicles={len(self.vehicle_bindings)}",
            "INFO",
        )

    def close(self) -> None:
        if self.client is None:
            return
        for binding in self.vehicle_bindings:
            try:
                self.client.cancelLastTask(vehicle_name=binding.vehicle_name)
            except Exception:
                pass
        self.connected = False

    def _discover_vehicles(self) -> list[VehicleBinding]:
        names: list[str] = []
        try:
            names = list(self.client.listVehicles() or [])
        except Exception:
            names = []

        if not names:
            names = self._vehicle_names_from_settings()
        if not names:
            configured = self.config.airsim.vehicle_names or []
            names = list(configured) if configured else [""]

        bindings: list[VehicleBinding] = []
        for index, vehicle_name in enumerate(names, start=1):
            display = vehicle_name or f"Drone{index}"
            bindings.append(VehicleBinding(sysid=index, vehicle_name=vehicle_name, display_name=display))
        return bindings

    def _vehicle_names_from_settings(self) -> list[str]:
        try:
            raw = self.client.getSettingsString()
            parsed = json.loads(raw or "{}")
            vehicles = parsed.get("Vehicles")
            if isinstance(vehicles, dict):
                return [str(name).strip() for name in vehicles.keys() if str(name).strip()]
        except Exception:
            return []
        return []

    def _capture_home_positions(self) -> None:
        self._home_positions.clear()
        for binding in self.vehicle_bindings:
            try:
                state = self.client.getMultirotorState(vehicle_name=binding.vehicle_name)
                position = state.kinematics_estimated.position
                self._home_positions[binding.sysid] = (
                    float(position.x_val),
                    float(position.y_val),
                    float(position.z_val),
                )
            except Exception:
                self._home_positions[binding.sysid] = (0.0, 0.0, 0.0)

    def refresh(self) -> dict[int, TelemetryData]:
        if not self.connected or self.client is None:
            return self.telemetry

        now = time.time()
        if now - self._last_refresh_at < self.config.airsim.poll_interval_s:
            return self.telemetry
        self._last_refresh_at = now

        for binding in self.vehicle_bindings:
            self.telemetry[binding.sysid] = self._read_vehicle(binding, now)
        self._tick_hold_positions(now)
        self._tick_expert_navigation(now)
        self._tick_waypoint_missions()
        self._tick_formation(now)
        return self.telemetry

    def _read_vehicle(self, binding: VehicleBinding, now: float) -> TelemetryData:
        tele = self.telemetry.get(binding.sysid) or TelemetryData(
            sysid=binding.sysid,
            vehicle_name=binding.vehicle_name,
            display_name=binding.display_name,
        )
        team = self.team_state.get(binding.sysid, TeamAssignment())

        try:
            state = self.client.getMultirotorState(vehicle_name=binding.vehicle_name)
            kinematics = state.kinematics_estimated
            position = kinematics.position
            orientation = kinematics.orientation
            linear_velocity = kinematics.linear_velocity
            landed_state = getattr(state, "landed_state", None)

            speed = math.sqrt(
                float(linear_velocity.x_val) ** 2
                + float(linear_velocity.y_val) ** 2
                + float(linear_velocity.z_val) ** 2
            )
            roll, pitch, yaw = self.airsim.to_eularian_angles(orientation)
            gps = self.client.getGpsData(vehicle_name=binding.vehicle_name)
            geo_point = getattr(getattr(gps, "gnss", None), "geo_point", None)
            lat = float(getattr(geo_point, "latitude", 0.0) or 0.0)
            lon = float(getattr(geo_point, "longitude", 0.0) or 0.0)
            alt = float(getattr(geo_point, "altitude", 0.0) or 0.0)
            gps_valid = bool(lat and lon)
            if not gps_valid:
                lat, lon, alt = self._local_to_geo(
                    float(position.x_val),
                    float(position.y_val),
                    float(position.z_val),
                )

            tele.lat = lat
            tele.lon = lon
            tele.alt = -float(position.z_val)
            tele.speed = speed
            tele.roll_deg = math.degrees(roll)
            tele.pitch_deg = math.degrees(pitch)
            tele.yaw_deg = math.degrees(yaw)
            tele.armed = bool(getattr(state, "ready", False)) and landed_state != 0
            tele.mode = "FOLLOW" if team.follow_enabled else "SIM"
            tele.gps_valid = gps_valid
            tele.local_valid = True
            tele.local_x = float(position.x_val)
            tele.local_y = float(position.y_val)
            tele.local_z = float(position.z_val)
            tele.vehicle_name = binding.vehicle_name
            tele.display_name = binding.display_name
            tele.leader_id = team.leader_id
            tele.team_no = team.team_no
            tele.follow_enabled = team.follow_enabled
            tele.last_update = now
            tele.stars = 0
            tele.batt = 0.0
            if not tele.trail or (now - tele.trail[-1][4]) >= 0.3:
                tele.trail.append((tele.lat, tele.lon, tele.local_x, tele.local_y, now))
        except Exception as exc:
            self.log(f"Telemetry read failed for {binding.display_name}: {exc}", "WARN")
        return tele

    def _local_to_geo(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        lat0 = float(self.config.ui.default_map_center_lat)
        lon0 = float(self.config.ui.default_map_center_lon)
        earth_radius = 6378137.0
        lat = lat0 + (x / earth_radius) * 180.0 / math.pi
        lon = lon0 + (y / (earth_radius * math.cos(math.radians(lat0)))) * 180.0 / math.pi
        alt = max(0.0, self.config.airsim.takeoff_height_m - (-z))
        return lat, lon, alt

    def _geo_to_local(self, lat: float, lon: float, target_z: float) -> tuple[float, float, float]:
        lat0 = float(self.config.ui.default_map_center_lat)
        lon0 = float(self.config.ui.default_map_center_lon)
        earth_radius = 6378137.0
        x = math.radians(lat - lat0) * earth_radius
        y = math.radians(lon - lon0) * earth_radius * math.cos(math.radians(lat0))
        z = float(target_z)
        return x, y, z

    def record_command(self, target_id: int, command: str, status: str, detail: str = "") -> None:
        self._command_index += 1
        self.command_history.insert(
            0,
            CommandRecord(
                index=self._command_index,
                target_id=target_id,
                command=command,
                status=status,
                issued_at=datetime.now().isoformat(timespec="seconds"),
                detail=detail,
            ),
        )
        self.command_history = self.command_history[:100]

    def _binding(self, target_id: int) -> VehicleBinding:
        for binding in self.vehicle_bindings:
            if binding.sysid == target_id:
                return binding
        raise KeyError(f"Unknown vehicle id: {target_id}")

    def _ensure_api_control(self, binding: VehicleBinding) -> None:
        if self.config.airsim.auto_enable_api_control:
            self.client.enableApiControl(True, vehicle_name=binding.vehicle_name)

    def send_basic_command(self, target_id: int, command: str) -> None:
        binding = self._binding(target_id)
        try:
            self._ensure_api_control(binding)
            if command == "arm":
                self.client.armDisarm(True, vehicle_name=binding.vehicle_name)
            elif command == "disarm":
                self._expert_missions.pop(target_id, None)
                self._waypoint_missions.pop(target_id, None)
                self._hold_missions.pop(target_id, None)
                self.client.cancelLastTask(vehicle_name=binding.vehicle_name)
                self.client.armDisarm(False, vehicle_name=binding.vehicle_name)
            elif command == "takeoff":
                self._hold_missions.pop(target_id, None)
                self._store_home_from_current_position(target_id)
                self.client.armDisarm(True, vehicle_name=binding.vehicle_name)
                self.client.takeoffAsync(vehicle_name=binding.vehicle_name)
                self.client.moveToZAsync(
                    z=-float(self.config.airsim.takeoff_height_m),
                    velocity=self.command_speed_mps,
                    vehicle_name=binding.vehicle_name,
                )
            elif command == "land":
                self._expert_missions.pop(target_id, None)
                self._waypoint_missions.pop(target_id, None)
                self._hold_missions.pop(target_id, None)
                self.client.landAsync(vehicle_name=binding.vehicle_name)
            elif command == "rtl":
                self.start_expert_rtl(target_id)
            elif command == "hover":
                self.start_hold_position(target_id)
            elif command == "follow_ap":
                team = self.team_state.setdefault(target_id, TeamAssignment())
                team.follow_enabled = True
            elif command == "team":
                team = self.team_state.setdefault(target_id, TeamAssignment())
                if team.team_no <= 0:
                    team.team_no = max(1, target_id)
                team.follow_enabled = target_id != team.leader_id and team.leader_id > 0
            elif command in {"guided", "solo"}:
                team = self.team_state.setdefault(target_id, TeamAssignment())
                team.follow_enabled = False
                if command == "solo":
                    team.leader_id = 0
                    team.team_no = 0
            elif command == "shape1":
                self.shape_name = "line"
            elif command == "shape_square":
                self.shape_name = "square"
            elif command == "shapeV":
                self.shape_name = "v"
            elif command == "dist+1":
                self.spacing_m = min(30.0, self.spacing_m + 1.0)
            elif command == "dist-1":
                self.spacing_m = max(2.0, self.spacing_m - 1.0)
            elif command == "alt+1":
                self.altitude_offset_m = min(20.0, self.altitude_offset_m + 1.0)
            elif command == "alt-1":
                self.altitude_offset_m = max(-20.0, self.altitude_offset_m - 1.0)
            elif command == "speed+20":
                self.command_speed_mps = min(20.0, self.command_speed_mps + 2.0)
            elif command == "speed-20":
                self.command_speed_mps = max(1.0, self.command_speed_mps - 2.0)
            else:
                raise ValueError(f"Unsupported AirSim command: {command}")
            self.record_command(target_id, command, "SENT")
        except Exception as exc:
            self.record_command(target_id, command, "FAILED", str(exc))
            raise

    def set_team_config(self, target_id: int, leader_id: int, team_no: int) -> None:
        team = self.team_state.setdefault(target_id, TeamAssignment())
        team.leader_id = int(leader_id)
        team.team_no = int(team_no)
        team.follow_enabled = target_id != leader_id and leader_id > 0
        self.record_command(target_id, "team_config", "APPLIED", f"leader={leader_id}, team={team_no}")

    def _store_home_from_current_position(self, target_id: int) -> None:
        binding = self._binding(target_id)
        try:
            state = self.client.getMultirotorState(vehicle_name=binding.vehicle_name)
            position = state.kinematics_estimated.position
            self._home_positions[target_id] = (
                float(position.x_val),
                float(position.y_val),
                float(position.z_val),
            )
        except Exception:
            self._home_positions.setdefault(target_id, (0.0, 0.0, 0.0))

    def start_expert_rtl(self, target_id: int) -> None:
        team = self.team_state.setdefault(target_id, TeamAssignment())
        team.follow_enabled = False
        tele = self.telemetry.get(target_id)
        home_x, home_y, home_z = self._home_positions.get(target_id, (0.0, 0.0, 0.0))
        if tele is not None and tele.local_valid:
            safe_z = min(float(tele.local_z), -float(self.config.airsim.takeoff_height_m))
        else:
            safe_z = min(float(home_z), -float(self.config.airsim.takeoff_height_m))
        self.start_expert_goto_local(target_id, home_x, home_y, safe_z)
        self.record_command(target_id, "rtl", "RUNNING", f"x={home_x:.1f}, y={home_y:.1f}, z={safe_z:.1f}")

    def goto_gps(self, target_id: int, lat: float, lon: float, target_z: float) -> None:
        binding = self._binding(target_id)
        self._ensure_api_control(binding)
        self._expert_missions.pop(target_id, None)
        self._hold_missions.pop(target_id, None)
        x, y, z = self._geo_to_local(lat, lon, target_z)
        self.client.moveToPositionAsync(
            x=x,
            y=y,
            z=z,
            velocity=self.command_speed_mps,
            vehicle_name=binding.vehicle_name,
        )
        self.record_command(target_id, "goto_map", "SENT", f"lat={lat:.6f}, lon={lon:.6f}, z={target_z:.1f}")

    def goto_local(self, target_id: int, x: float, y: float, z: float) -> None:
        binding = self._binding(target_id)
        self._ensure_api_control(binding)
        self._expert_missions.pop(target_id, None)
        self._hold_missions.pop(target_id, None)
        self.client.moveToPositionAsync(
            x=float(x),
            y=float(y),
            z=float(z),
            velocity=self.command_speed_mps,
            vehicle_name=binding.vehicle_name,
        )
        self.record_command(target_id, "goto_local", "SENT", f"x={x:.1f}, y={y:.1f}, z={z:.1f}")

    def start_expert_goto_gps(self, target_id: int, lat: float, lon: float, target_z: float) -> None:
        x, y, z = self._geo_to_local(lat, lon, target_z)
        self.start_expert_goto_local(target_id, x, y, z)
        self.record_command(target_id, "expert_goto_map", "RUNNING", f"lat={lat:.6f}, lon={lon:.6f}, z={target_z:.1f}")

    def start_expert_goto_local(self, target_id: int, x: float, y: float, z: float) -> None:
        self._binding(target_id)
        self._hold_missions.pop(target_id, None)
        self._waypoint_missions.pop(target_id, None)
        self._expert_missions[target_id] = ExpertNavigationMission(
            target_x=float(x),
            target_y=float(y),
            target_z=float(z),
            started_at=time.time(),
            max_speed_mps=max(1.0, min(3.0, self.command_speed_mps)),
        )
        self.record_command(target_id, "expert_goto_local", "RUNNING", f"x={x:.1f}, y={y:.1f}, z={z:.1f}")

    def stop_motion(self, target_id: int) -> None:
        self.start_hold_position(target_id)
        self.record_command(target_id, "stop_motion", "SENT")

    def run_waypoints(self, target_id: int, points: list[Waypoint]) -> None:
        if not points:
            return
        self._expert_missions.pop(target_id, None)
        self._hold_missions.pop(target_id, None)
        self._waypoint_missions[target_id] = ActiveWaypointMission(points=list(points))
        self._dispatch_waypoint(target_id)
        self.record_command(target_id, "waypoints", "RUNNING", f"count={len(points)}")

    def start_hold_position(self, target_id: int) -> None:
        binding = self._binding(target_id)
        self._ensure_api_control(binding)
        self._expert_missions.pop(target_id, None)
        self._waypoint_missions.pop(target_id, None)
        tele = self.telemetry.get(target_id)
        if tele is None or not tele.local_valid:
            state = self.client.getMultirotorState(vehicle_name=binding.vehicle_name)
            kinematics = state.kinematics_estimated
            position = kinematics.position
            roll, pitch, yaw = self.airsim.to_eularian_angles(kinematics.orientation)
            hold = HoldPositionMission(
                x=float(position.x_val),
                y=float(position.y_val),
                z=float(position.z_val),
                yaw_deg=math.degrees(yaw),
            )
        else:
            hold = HoldPositionMission(
                x=float(tele.local_x),
                y=float(tele.local_y),
                z=float(tele.local_z),
                yaw_deg=float(tele.yaw_deg),
            )
        self._hold_missions[target_id] = hold
        try:
            self.client.cancelLastTask(vehicle_name=binding.vehicle_name)
        except Exception:
            pass
        self._issue_hold_position(binding, hold)

    def _tick_hold_positions(self, now: float) -> None:
        for target_id, hold in list(self._hold_missions.items()):
            if now - hold.last_tick_at < hold.refresh_s:
                continue
            binding = self._binding(target_id)
            try:
                self._issue_hold_position(binding, hold)
            except Exception as exc:
                self.record_command(target_id, "hover", "FAILED", str(exc))
                self._hold_missions.pop(target_id, None)

    def _issue_hold_position(self, binding: VehicleBinding, hold: HoldPositionMission) -> None:
        hold.last_tick_at = time.time()
        self._ensure_api_control(binding)
        try:
            self.client.moveToPositionAsync(
                x=float(hold.x),
                y=float(hold.y),
                z=float(hold.z),
                velocity=0.8,
                drivetrain=self.airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=self.airsim.YawMode(False, float(hold.yaw_deg)),
                vehicle_name=binding.vehicle_name,
            )
        except TypeError:
            self.client.moveToPositionAsync(
                x=float(hold.x),
                y=float(hold.y),
                z=float(hold.z),
                velocity=0.8,
                vehicle_name=binding.vehicle_name,
            )

    def _dispatch_waypoint(self, target_id: int) -> None:
        mission = self._waypoint_missions.get(target_id)
        if mission is None or mission.current_index >= len(mission.points):
            return
        point = mission.points[mission.current_index]
        self.goto_local(target_id, point.x, point.y, point.z)

    def _tick_waypoint_missions(self) -> None:
        finished: list[int] = []
        for target_id, mission in list(self._waypoint_missions.items()):
            if mission.current_index >= len(mission.points):
                finished.append(target_id)
                continue
            tele = self.telemetry.get(target_id)
            if tele is None or not tele.local_valid:
                continue
            point = mission.points[mission.current_index]
            if metres_between_local(tele, point.x, point.y, point.z) <= 2.0:
                mission.current_index += 1
                if mission.current_index >= len(mission.points):
                    self.record_command(target_id, "waypoints", "DONE", f"count={len(mission.points)}")
                    finished.append(target_id)
                else:
                    self._dispatch_waypoint(target_id)
        for target_id in finished:
            self._waypoint_missions.pop(target_id, None)

    def _tick_expert_navigation(self, now: float) -> None:
        finished: list[int] = []
        for target_id, mission in list(self._expert_missions.items()):
            if now - mission.last_tick_at < mission.step_duration_s:
                continue
            mission.last_tick_at = now
            tele = self.telemetry.get(target_id)
            if tele is None or not tele.local_valid:
                continue
            binding = self._binding(target_id)
            horizontal = math.hypot(tele.local_x - mission.target_x, tele.local_y - mission.target_y)
            altitude_error = mission.target_z - tele.local_z
            if horizontal < mission.best_horizontal_m - 0.15:
                mission.best_horizontal_m = horizontal
                mission.no_progress_ticks = 0
            else:
                mission.no_progress_ticks += 1
            if horizontal <= mission.safety_radius_m and abs(altitude_error) <= max(1.8, mission.safety_radius_m):
                mission.previous_velocity = (0.0, 0.0, 0.0)
                self.start_hold_position(target_id)
                self.record_command(target_id, "expert_goto", "DONE", f"distance={horizontal:.2f}")
                finished.append(target_id)
                continue
            if now - mission.started_at > mission.max_time_s:
                self.start_hold_position(target_id)
                self.record_command(target_id, "expert_goto", "FAILED", "timeout")
                finished.append(target_id)
                continue

            front, left_space, right_space, _left_edge, _right_edge = self._read_lidar_summary(binding.vehicle_name)
            collision_stamp = self._collision_stamp(binding.vehicle_name)
            collided = collision_stamp > 0 and collision_stamp != mission.last_collision_stamp
            if collided:
                mission.last_collision_stamp = collision_stamp
                if mission.recovery_direction == "right" and left_space <= right_space + 1.2:
                    mission.recovery_direction = "right"
                elif mission.recovery_direction == "left" and right_space <= left_space + 1.2:
                    mission.recovery_direction = "left"
                else:
                    mission.recovery_direction = "right" if right_space > left_space else "left"
                mission.recovery_counter = max(mission.recovery_counter, 12)
                try:
                    self.client.cancelLastTask(vehicle_name=binding.vehicle_name)
                except Exception:
                    pass
            no_progress_s = mission.no_progress_ticks * mission.step_duration_s
            if no_progress_s > 8.0 and horizontal > mission.safety_radius_m * 2.0 and front < 4.5:
                mission.phase = "recovery"
                if mission.recovery_direction == "right" and left_space <= right_space + 1.2:
                    mission.recovery_direction = "right"
                elif mission.recovery_direction == "left" and right_space <= left_space + 1.2:
                    mission.recovery_direction = "left"
                else:
                    mission.recovery_direction = "right" if right_space > left_space else "left"
                mission.recovery_counter = max(mission.recovery_counter, 12)
                mission.stuck_recovery_count += 1
                mission.no_progress_ticks = 0
            if mission.stuck_recovery_count >= 4 and no_progress_s > 10.0:
                self.start_hold_position(target_id)
                self.record_command(target_id, "expert_goto", "FAILED", "target may be unreachable")
                finished.append(target_id)
                continue

            near_goal = horizontal < 5.0
            if near_goal and front >= 4.0 and mission.recovery_counter <= 0:
                mission.avoid_counter = 0
                mission.avoid_direction = None
            vx, vy = self._expert_xy_velocity(mission, tele, front, left_space, right_space, collided)
            vertical_limit = mission.max_speed_mps if not near_goal else 0.8
            vz = self._clamp(mission.altitude_gain * altitude_error, -vertical_limit, vertical_limit)
            if collided or front < 4.0 or mission.recovery_counter > 0:
                vz = max(0.0, vz)
            vx, vy, vz = self._smooth_velocity(mission, vx, vy, vz)
            try:
                self._ensure_api_control(binding)
                self._move_by_velocity_facing(binding.vehicle_name, vx, vy, vz, mission.step_duration_s, tele.yaw_deg)
            except Exception as exc:
                self.record_command(target_id, "expert_goto", "FAILED", str(exc))
                finished.append(target_id)
        for target_id in finished:
            self._expert_missions.pop(target_id, None)

    def _expert_xy_velocity(
        self,
        mission: ExpertNavigationMission,
        tele: TelemetryData,
        front: float,
        left_space: float,
        right_space: float,
        collided: bool,
    ) -> tuple[float, float]:
        dx = mission.target_x - tele.local_x
        dy = mission.target_y - tele.local_y
        norm = math.hypot(dx, dy)
        if norm > 1e-6:
            ux = dx / norm
            uy = dy / norm
        else:
            ux = 1.0
            uy = 0.0

        approach_scale = self._clamp(norm / 8.0, 0.18, 1.0)

        def steer(forward: float, lateral: float, direction: str | None, *, scale_near_goal: bool = True) -> tuple[float, float]:
            if direction == "right":
                lx, ly = uy, -ux
            else:
                lx, ly = -uy, ux
            speed = mission.max_speed_mps * (approach_scale if scale_near_goal else 1.0)
            return (
                speed * (forward * ux + lateral * lx),
                speed * (forward * uy + lateral * ly),
            )

        def choose_side(current: str | None) -> str:
            if current == "right" and left_space <= right_space + 1.2:
                return "right"
            if current == "left" and right_space <= left_space + 1.2:
                return "left"
            return "right" if right_space > left_space else "left"

        if mission.distance_history is None:
            mission.distance_history = []
        mission.distance_history.append(front)
        mission.distance_history = mission.distance_history[-5:]
        closing_in = (
            len(mission.distance_history) == 5
            and all(mission.distance_history[i] > mission.distance_history[i + 1] for i in range(4))
            and front < 3.0
        )

        if front >= 5.5 and not collided and mission.recovery_counter <= 0:
            mission.clear_ticks += 1
            if mission.clear_ticks >= 3:
                mission.phase = "direct"
                mission.avoid_direction = None
                mission.avoid_counter = 0
        else:
            mission.clear_ticks = 0

        if collided or front < 1.2:
            mission.phase = "recovery"
            if mission.recovery_counter <= 0:
                mission.recovery_direction = choose_side(mission.recovery_direction)
                mission.recovery_counter = 10
            mission.recovery_counter -= 1
            return steer(-0.9, 0.9, mission.recovery_direction, scale_near_goal=False)

        if mission.recovery_counter > 0:
            mission.phase = "recovery"
            mission.recovery_counter -= 1
            return steer(-0.35, 1.0, mission.recovery_direction, scale_near_goal=False)

        if mission.avoid_counter > 0 and mission.avoid_direction:
            mission.phase = "avoid"
            mission.avoid_counter -= 1
            return steer(0.15, 1.0, mission.avoid_direction)

        if front < 1.8:
            mission.phase = "recovery"
            mission.avoid_direction = None
            mission.avoid_counter = 0
            mission.recovery_direction = choose_side(mission.recovery_direction)
            mission.recovery_counter = 8
            return steer(-0.75, 0.85, mission.recovery_direction)

        if front < 3.0 or closing_in:
            mission.phase = "avoid"
            if mission.avoid_direction is None or mission.avoid_counter <= 0:
                mission.avoid_direction = choose_side(mission.avoid_direction)
                mission.avoid_counter = 8
            return steer(0.1, 1.0, mission.avoid_direction)

        if front < 5.5:
            mission.phase = "slow"
            mission.avoid_direction = None
            mission.avoid_counter = 0
            return steer(0.65, 0.0, None)

        mission.phase = "direct"
        mission.avoid_direction = None
        mission.avoid_counter = 0
        return ux * mission.max_speed_mps * approach_scale, uy * mission.max_speed_mps * approach_scale

    def _read_lidar_summary(self, vehicle_name: str) -> tuple[float, float, float, float, float]:
        try:
            data = self.client.getLidarData(lidar_name="LidarSensor1", vehicle_name=vehicle_name)
            if len(data.point_cloud) < 3:
                return 10.0, 10.0, 10.0, 0.0, 0.0
            try:
                import numpy as np  # type: ignore
            except Exception:
                return 10.0, 10.0, 10.0, 0.0, 0.0
            points = np.array(data.point_cloud, dtype=np.float32).reshape(-1, 3)
            valid = points[(points[:, 0] > 0.1) & (points[:, 0] < 100)]
            if valid.size == 0:
                return 10.0, 10.0, 10.0, 0.0, 0.0
            front_points = valid[(valid[:, 0] > 0) & (valid[:, 0] < 10.0) & (abs(valid[:, 1]) < 1.3)]
            left_points = valid[(valid[:, 0] > 0) & (valid[:, 0] < 8.0) & (valid[:, 1] > 0.4)]
            right_points = valid[(valid[:, 0] > 0) & (valid[:, 0] < 8.0) & (valid[:, 1] < -0.4)]
            front = float(np.min(front_points[:, 0])) if len(front_points) else 10.0
            left = float(np.min(np.linalg.norm(left_points[:, :2], axis=1))) if len(left_points) else 10.0
            right = float(np.min(np.linalg.norm(right_points[:, :2], axis=1))) if len(right_points) else 10.0
            left_edge = float(np.min(front_points[:, 1])) if len(front_points) else 0.0
            right_edge = float(np.max(front_points[:, 1])) if len(front_points) else 0.0
            return front, left, right, left_edge, right_edge
        except Exception:
            return 10.0, 10.0, 10.0, 0.0, 0.0

    def _collision_stamp(self, vehicle_name: str) -> int:
        try:
            info = self.client.simGetCollisionInfo(vehicle_name=vehicle_name)
            if not bool(getattr(info, "has_collided", False)):
                return 0
            return int(getattr(info, "time_stamp", 0) or 0)
        except Exception:
            return 0

    def _move_by_velocity_facing(
        self,
        vehicle_name: str,
        vx: float,
        vy: float,
        vz: float,
        duration: float,
        current_yaw_deg: float,
    ) -> None:
        yaw_deg = current_yaw_deg
        if math.hypot(vx, vy) > 0.6:
            yaw_deg = math.degrees(math.atan2(vy, vx))
        try:
            self.client.moveByVelocityAsync(
                vx=float(vx),
                vy=float(vy),
                vz=float(vz),
                duration=float(duration),
                drivetrain=self.airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=self.airsim.YawMode(False, float(yaw_deg)),
                vehicle_name=vehicle_name,
            )
        except TypeError:
            self.client.moveByVelocityAsync(
                vx=float(vx),
                vy=float(vy),
                vz=float(vz),
                duration=float(duration),
                vehicle_name=vehicle_name,
            )

    def _smooth_velocity(self, mission: ExpertNavigationMission, vx: float, vy: float, vz: float) -> tuple[float, float, float]:
        max_delta = 1.6 * mission.step_duration_s
        pvx, pvy, pvz = mission.previous_velocity
        smoothed = (
            self._approach(pvx, vx, max_delta),
            self._approach(pvy, vy, max_delta),
            self._approach(pvz, vz, max_delta),
        )
        mission.previous_velocity = smoothed
        return smoothed

    @staticmethod
    def _approach(current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        return current + math.copysign(max_delta, delta)

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def _tick_formation(self, now: float) -> None:
        if not self.config.formation.enabled:
            return
        if now - self._last_formation_tick < self.config.formation.refresh_interval_s:
            return
        self._last_formation_tick = now

        for sysid, team in self.team_state.items():
            if sysid in self._hold_missions:
                continue
            if sysid in self._expert_missions:
                continue
            if sysid in self._waypoint_missions:
                continue
            if not team.follow_enabled or team.leader_id <= 0 or sysid == team.leader_id:
                continue
            leader = self.telemetry.get(team.leader_id)
            follower = self.telemetry.get(sysid)
            if leader is None or follower is None or not leader.local_valid:
                continue
            target = self._formation_target(leader.local_x, leader.local_y, leader.local_z, team.team_no)
            binding = self._binding(sysid)
            try:
                self._ensure_api_control(binding)
                self.client.moveToPositionAsync(
                    x=target[0],
                    y=target[1],
                    z=target[2],
                    velocity=self.command_speed_mps,
                    vehicle_name=binding.vehicle_name,
                )
            except Exception as exc:
                self.log(f"Formation update failed for {binding.display_name}: {exc}", "WARN")

    def _formation_target(self, leader_x: float, leader_y: float, leader_z: float, team_no: int) -> tuple[float, float, float]:
        slot = max(1, team_no) - 1
        spacing = self.spacing_m
        if self.shape_name == "square":
            row = slot // 2
            col = slot % 2
            offset_x = -(row + 1) * spacing
            offset_y = (-0.5 if col == 0 else 0.5) * spacing * 2.0
        elif self.shape_name == "v":
            arm = slot // 2 + 1
            side = -1.0 if slot % 2 == 0 else 1.0
            offset_x = -arm * spacing
            offset_y = side * arm * spacing
        else:
            offset_x = -slot * spacing
            offset_y = 0.0
        offset_z = -self.altitude_offset_m
        return leader_x + offset_x, leader_y + offset_y, leader_z + offset_z
