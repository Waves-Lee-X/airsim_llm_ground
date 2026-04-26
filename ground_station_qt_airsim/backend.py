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

    def refresh(self) -> dict[int, TelemetryData]:
        if not self.connected or self.client is None:
            return self.telemetry

        now = time.time()
        if now - self._last_refresh_at < self.config.airsim.poll_interval_s:
            return self.telemetry
        self._last_refresh_at = now

        self._tick_waypoint_missions()
        self._tick_formation(now)

        for binding in self.vehicle_bindings:
            self.telemetry[binding.sysid] = self._read_vehicle(binding, now)
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
            tele.alt = alt
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
                self.client.hoverAsync(vehicle_name=binding.vehicle_name)
                self.client.armDisarm(False, vehicle_name=binding.vehicle_name)
            elif command == "takeoff":
                self.client.armDisarm(True, vehicle_name=binding.vehicle_name)
                self.client.takeoffAsync(vehicle_name=binding.vehicle_name)
                self.client.moveToZAsync(
                    z=-float(self.config.airsim.takeoff_height_m),
                    velocity=self.command_speed_mps,
                    vehicle_name=binding.vehicle_name,
                )
            elif command == "land":
                self.client.landAsync(vehicle_name=binding.vehicle_name)
            elif command == "rtl":
                self.client.goHomeAsync(vehicle_name=binding.vehicle_name)
            elif command == "hover":
                self.client.hoverAsync(vehicle_name=binding.vehicle_name)
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

    def goto_gps(self, target_id: int, lat: float, lon: float, target_z: float) -> None:
        binding = self._binding(target_id)
        self._ensure_api_control(binding)
        x, y, z = self._geo_to_local(lat, lon, target_z)
        self.client.moveToPositionAsync(
            x=x,
            y=y,
            z=z,
            velocity=self.command_speed_mps,
            vehicle_name=binding.vehicle_name,
        )
        self.record_command(target_id, "goto_map", "SENT", f"lat={lat:.6f}, lon={lon:.6f}, z={target_z:.1f}")

    def run_waypoints(self, target_id: int, points: list[Waypoint]) -> None:
        if not points:
            return
        self._waypoint_missions[target_id] = ActiveWaypointMission(points=list(points))
        self._dispatch_waypoint(target_id)
        self.record_command(target_id, "waypoints", "RUNNING", f"count={len(points)}")

    def _dispatch_waypoint(self, target_id: int) -> None:
        mission = self._waypoint_missions.get(target_id)
        if mission is None or mission.current_index >= len(mission.points):
            return
        point = mission.points[mission.current_index]
        self.goto_gps(target_id, point.lat, point.lon, point.alt_m)

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
            x, y, z = self._geo_to_local(point.lat, point.lon, point.alt_m)
            if metres_between_local(tele, x, y, z) <= 2.0:
                mission.current_index += 1
                if mission.current_index >= len(mission.points):
                    self.record_command(target_id, "waypoints", "DONE", f"count={len(mission.points)}")
                    finished.append(target_id)
                else:
                    self._dispatch_waypoint(target_id)
        for target_id in finished:
            self._waypoint_missions.pop(target_id, None)

    def _tick_formation(self, now: float) -> None:
        if not self.config.formation.enabled:
            return
        if now - self._last_formation_tick < self.config.formation.refresh_interval_s:
            return
        self._last_formation_tick = now

        for sysid, team in self.team_state.items():
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
