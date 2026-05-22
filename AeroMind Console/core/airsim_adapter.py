from __future__ import annotations

from dataclasses import dataclass
import math
import queue
import threading
import time
from typing import Any


@dataclass(frozen=True)
class UavTelemetry:
    name: str = "UAV1"
    mode: str = "LLM-MISSION"
    altitude_m: float = 0.0
    speed_mps: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_api(self) -> dict[str, float | str]:
        return {
            "name": self.name,
            "mode": self.mode,
            "altitude_m": self.altitude_m,
            "speed_mps": self.speed_mps,
            "x": self.x,
            "y": self.y,
            "z": self.z,
        }


@dataclass(frozen=True)
class CameraFrame:
    data: bytes
    content_type: str
    camera_name: str
    image_type: int


@dataclass(frozen=True)
class CollisionStatus:
    has_collided: bool = False
    object_name: str = ""
    object_id: int = -1
    impact_point: dict[str, float] | None = None
    normal: dict[str, float] | None = None
    time_stamp: int = 0

    def to_api(self) -> dict[str, Any]:
        return {
            "has_collided": self.has_collided,
            "object_name": self.object_name,
            "object_id": self.object_id,
            "impact_point": self.impact_point,
            "normal": self.normal,
            "time_stamp": self.time_stamp,
        }


@dataclass(frozen=True)
class ObstacleStatus:
    available: bool = False
    blocked: bool = False
    nearest_distance_m: float | None = None
    front_points: int = 0
    left_points: int = 0
    right_points: int = 0
    recommended_side: str = "none"
    risk_level: str = "unknown"
    sector_clearance: dict[str, float | None] | None = None
    sector_points: dict[str, int] | None = None
    clusters: list[dict[str, Any]] | None = None
    decision: dict[str, Any] | None = None
    sensor_name: str = "LidarSensor1"
    error: str = ""

    def to_api(self) -> dict[str, Any]:
        return {
            "available": self.available,
            "blocked": self.blocked,
            "nearest_distance_m": self.nearest_distance_m,
            "front_points": self.front_points,
            "left_points": self.left_points,
            "right_points": self.right_points,
            "recommended_side": self.recommended_side,
            "risk_level": self.risk_level,
            "sector_clearance": self.sector_clearance or {},
            "sector_points": self.sector_points or {},
            "clusters": self.clusters or [],
            "decision": self.decision or {},
            "sensor_name": self.sensor_name,
            "error": self.error,
        }


@dataclass(frozen=True)
class DistanceSensorsStatus:
    available: bool = False
    front_m: float | None = None
    left_m: float | None = None
    right_m: float | None = None
    min_m: float | None = None
    nearest_direction: str = "none"
    emergency_direction: str = "none"
    emergency: bool = False
    error: str = ""

    def to_api(self) -> dict[str, Any]:
        return {
            "available": self.available,
            "front_m": self.front_m,
            "left_m": self.left_m,
            "right_m": self.right_m,
            "min_m": self.min_m,
            "nearest_direction": self.nearest_direction,
            "emergency_direction": self.emergency_direction,
            "emergency": self.emergency,
            "error": self.error,
        }


class CommandJob:
    def __init__(self, fn: Any) -> None:
        self.fn = fn
        self.done = threading.Event()
        self.result: Any = None
        self.error: BaseException | None = None


class AirSimAdapter:
    """Small AirSim integration layer for telemetry and camera frames."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 41451,
        vehicle_name: str = "Drone1",
        vehicle_names: tuple[str, ...] = ("Drone1", "Drone2", "Drone3"),
    ) -> None:
        self.host = host
        self.port = int(port)
        self.vehicle_name = vehicle_name
        self.vehicle_names = tuple(vehicle_names)
        self.connected = False
        self.client: Any = None
        self.airsim: Any = None
        self.last_error = ""
        self.last_camera_name = ""
        self.last_image_type = -1
        self.camera_candidates = ("front_center", "bottom_center", "0", "front_left", "front_right")
        self.active_camera_name = "front_center"
        self.last_success_at = 0.0
        self.failure_count = 0
        self._lock = threading.RLock()
        self._position_hold_stop = threading.Event()
        self._position_hold_thread: threading.Thread | None = None
        self._position_hold_x: float | None = None
        self._position_hold_y: float | None = None
        self._position_hold_z: float | None = None
        self._command_queue: queue.Queue[CommandJob | None] = queue.Queue()
        self._command_thread = threading.Thread(
            target=self._command_loop,
            name="aeromind-airsim-command",
            daemon=True,
        )
        self._command_thread.start()
        self._reconnect_stop = threading.Event()
        self._reconnect_thread: threading.Thread | None = None
        self._heartbeat_failures = 0
        self._start_reconnect_worker()

    def _command_loop(self) -> None:
        while True:
            job = self._command_queue.get()
            if job is None:
                self._command_queue.task_done()
                return
            try:
                job.result = job.fn()
            except BaseException as exc:
                job.error = exc
            finally:
                job.done.set()
                self._command_queue.task_done()

    def _run_command(self, fn: Any, timeout_s: float | None = None) -> Any:
        job = CommandJob(fn)
        self._command_queue.put(job)
        if not job.done.wait(timeout=timeout_s):
            raise TimeoutError("AirSim command timed out")
        if job.error is not None:
            raise job.error
        return job.result

    @staticmethod
    def _join_async(task: Any, timeout_s: float | None = None) -> None:
        join = getattr(task, "join", None)
        if not callable(join):
            return
        try:
            join(timeout_s)
        except TypeError:
            join()

    def _stop_position_hold(self) -> None:
        self._position_hold_stop.set()
        thread = self._position_hold_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._position_hold_thread = None

    def _start_position_hold(self, x: float | None = None, y: float | None = None, z: float | None = None) -> None:
        self._stop_position_hold()
        self._position_hold_x = float(x) if x is not None else None
        self._position_hold_y = float(y) if y is not None else None
        self._position_hold_z = float(z) if z is not None else None
        self._position_hold_stop = threading.Event()
        self._position_hold_thread = threading.Thread(
            target=self._position_hold_loop,
            name=f"aeromind-position-hold-{self.vehicle_name}",
            daemon=True,
        )
        self._position_hold_thread.start()

    def _position_hold_loop(self) -> None:
        refresh_s = 0.35
        while not self._position_hold_stop.wait(refresh_s):
            try:
                client = self._require_client()
                with self._lock:
                    client.enableApiControl(True, vehicle_name=self.vehicle_name)
                    client.armDisarm(True, vehicle_name=self.vehicle_name)
                    state = client.getMultirotorState(vehicle_name=self.vehicle_name)
                    position = state.kinematics_estimated.position
                    hold_x = self._position_hold_x
                    hold_y = self._position_hold_y
                    target_z = self._position_hold_z if self._position_hold_z is not None else float(position.z_val)
                px = float(position.x_val)
                py = float(position.y_val)
                if hold_x is None:
                    hold_x = px
                    self._position_hold_x = hold_x
                if hold_y is None:
                    hold_y = py
                    self._position_hold_y = hold_y
                vx = max(-0.45, min(0.45, (float(hold_x) - px) * 0.35))
                vy = max(-0.45, min(0.45, (float(hold_y) - py) * 0.35))
                try:
                    with self._lock:
                        task = client.moveByVelocityZAsync(
                            vx=float(vx),
                            vy=float(vy),
                            z=float(target_z),
                            duration=float(refresh_s * 1.4),
                            drivetrain=self.airsim.DrivetrainType.MaxDegreeOfFreedom if self.airsim is not None else 0,
                            yaw_mode=self.airsim.YawMode(True, 0.0) if self.airsim is not None else None,
                            vehicle_name=self.vehicle_name,
                        )
                except TypeError:
                    with self._lock:
                        task = client.moveByVelocityZAsync(
                            float(vx),
                            float(vy),
                            float(target_z),
                            float(refresh_s * 1.4),
                            vehicle_name=self.vehicle_name,
                        )
                self._join_async(task, timeout_s=max(0.7, refresh_s * 2.5))
                self.last_success_at = time.time()
            except Exception as exc:
                self.last_error = str(exc)

    def _start_reconnect_worker(self) -> None:
        if self._reconnect_thread is not None and self._reconnect_thread.is_alive():
            return
        self._reconnect_stop.clear()
        self._reconnect_thread = threading.Thread(
            target=self._reconnect_loop,
            name="aeromind-airsim-reconnect",
            daemon=True,
        )
        self._reconnect_thread.start()

    def _reconnect_loop(self) -> None:
        reconnect_interval_s = 3.0
        while not self._reconnect_stop.is_set():
            if not self.connected:
                self._try_connect()
            else:
                # Heartbeat check to detect stale connections
                client = self.client
                if client is not None:
                    try:
                        with self._lock:
                            client.getMultirotorState(vehicle_name=self.vehicle_name)
                        self._heartbeat_failures = 0
                    except Exception:
                        self._heartbeat_failures += 1
                        if self._heartbeat_failures >= 3:
                            with self._lock:
                                self.connected = False
                                self.client = None
                                self.last_error = "Connection lost"
                                self.failure_count += 1
                            self._heartbeat_failures = 0
            time.sleep(reconnect_interval_s)

    def _try_connect(self) -> None:
        try:
            import airsim
            client = airsim.MultirotorClient(ip=self.host, port=self.port, timeout_value=1.5)
            client.confirmConnection()
            discovered_names: tuple[str, ...] = self.vehicle_names
            try:
                listed = client.listVehicles()
                listed = [str(name).strip() for name in listed if str(name).strip()]
                if listed:
                    discovered_names = tuple(listed)
            except Exception:
                pass
            with self._lock:
                self.airsim = airsim
                self.client = client
                self.vehicle_names = discovered_names
                if self.vehicle_name not in self.vehicle_names and self.vehicle_names:
                    self.vehicle_name = self.vehicle_names[0]
                for name in self.vehicle_names:
                    try:
                        self.client.enableApiControl(True, vehicle_name=name)
                    except Exception:
                        pass
                self.connected = True
                self.last_success_at = time.time()
                self.failure_count = 0
                self._heartbeat_failures = 0
                self.last_error = ""
            self._warmup_camera()
        except Exception as exc:
            with self._lock:
                self.last_error = str(exc)
                self.failure_count += 1
                self.connected = False
                self.client = None

    def stop_reconnect_worker(self) -> None:
        self._stop_position_hold()
        self._reconnect_stop.set()
        self._command_queue.put(None)

    def switch_vehicle(self, vehicle_name: str) -> None:
        target = str(vehicle_name).strip()
        if target not in self.vehicle_names:
            raise ValueError(f"Vehicle '{target}' is not available")
        self._stop_position_hold()
        with self._lock:
            self.vehicle_name = target
            self.last_camera_name = ""
            self.last_image_type = -1
        if self.connected and self.client is not None:
            try:
                self.client.enableApiControl(True, vehicle_name=target)
            except Exception:
                pass

    def vehicle_options(self) -> dict[str, Any]:
        return {
            "selected": self.vehicle_name,
            "vehicles": list(self.vehicle_names),
        }

    def connect(self) -> bool:
        with self._lock:
            if self.connected and self.client is not None:
                return True
            try:
                import airsim  # type: ignore

                self.airsim = airsim
                self.client = airsim.MultirotorClient(ip=self.host, port=self.port, timeout_value=2)
                self.client.confirmConnection()
                discovered_names: tuple[str, ...] = self.vehicle_names
                try:
                    listed = self.client.listVehicles()
                    listed = [str(name).strip() for name in listed if str(name).strip()]
                    if listed:
                        discovered_names = tuple(listed)
                except Exception:
                    pass
                self.vehicle_names = discovered_names
                if self.vehicle_name not in self.vehicle_names and self.vehicle_names:
                    self.vehicle_name = self.vehicle_names[0]
                for name in self.vehicle_names:
                    try:
                        self.client.enableApiControl(True, vehicle_name=name)
                    except Exception:
                        pass
                self.connected = True
                self.last_success_at = time.time()
                self.failure_count = 0
                self.last_error = ""
                self._warmup_camera()
                return True
            except Exception as exc:
                self._note_failure()
                self.client = None
                self.last_error = str(exc)
                return False

    def _warmup_camera(self) -> None:
        if not self.client or not self.airsim:
            return
        for _ in range(3):
            self.camera_frame()

    def is_connected(self, grace_s: float = 5.0) -> bool:
        return self.connected or (self.last_success_at > 0 and (time.time() - self.last_success_at) <= grace_s)

    def telemetry(self, vehicle_name: str | None = None) -> UavTelemetry:
        target_name = vehicle_name or self.vehicle_name
        if not self.connect() or self.client is None:
            return UavTelemetry(name=target_name)
        try:
            with self._lock:
                state = self.client.getMultirotorState(vehicle_name=target_name)
            kinematics = state.kinematics_estimated
            position = kinematics.position
            velocity = kinematics.linear_velocity
            speed = math.sqrt(
                float(velocity.x_val) ** 2
                + float(velocity.y_val) ** 2
                + float(velocity.z_val) ** 2
            )
            return UavTelemetry(
                name=target_name,
                mode="AIRSIM-LIVE",
                altitude_m=max(0.0, -float(position.z_val)),
                speed_mps=speed,
                x=float(position.x_val),
                y=float(position.y_val),
                z=float(position.z_val),
            )
        except Exception as exc:
            self._note_failure()
            self.last_error = str(exc)
            return UavTelemetry(name=target_name)

    def takeoff(self, altitude_m: float = 8.0) -> None:
        client = self._require_client()
        target_z = -abs(float(altitude_m))
        self._stop_position_hold()
        with self._lock:
            client.enableApiControl(True, vehicle_name=self.vehicle_name)
            client.armDisarm(True, vehicle_name=self.vehicle_name)
            try:
                takeoff_task = client.takeoffAsync(timeout_sec=10, vehicle_name=self.vehicle_name)
            except TypeError:
                takeoff_task = client.takeoffAsync(vehicle_name=self.vehicle_name)
            self._join_async(takeoff_task, timeout_s=12.0)
            climb_task = client.moveToZAsync(
                z=target_z,
                velocity=2.0,
                vehicle_name=self.vehicle_name,
            )
            self._join_async(climb_task, timeout_s=max(6.0, abs(target_z) / 2.0 + 4.0))
            settle_deadline = time.time() + max(6.0, abs(target_z) * 1.2)
            while time.time() < settle_deadline:
                state = client.getMultirotorState(vehicle_name=self.vehicle_name)
                current_z = float(state.kinematics_estimated.position.z_val)
                if abs(current_z - target_z) <= 0.6:
                    break
                retry_task = client.moveToZAsync(z=target_z, velocity=1.8, vehicle_name=self.vehicle_name)
                self._join_async(retry_task, timeout_s=2.0)
                time.sleep(0.12)
            final_state = client.getMultirotorState(vehicle_name=self.vehicle_name)
            final_position = final_state.kinematics_estimated.position
            self.last_success_at = time.time()
        self._start_position_hold(
            x=float(final_position.x_val),
            y=float(final_position.y_val),
            z=float(target_z),
        )

    def formation_vehicle_names(self, count: int) -> tuple[str, ...]:
        count = max(1, min(int(count), len(self.vehicle_names)))
        return self.vehicle_names[:count]

    def execute_formation_route(
        self,
        route: list[dict[str, float]],
        slots: list[dict[str, Any]],
        speed_mps: float = 2.0,
        altitude_m: float = 8.0,
        progress_callback: Any | None = None,
    ) -> dict[str, Any]:
        self._require_client()
        if self.airsim is None:
            raise RuntimeError("AirSim module is not available")
        vehicle_names = self.formation_vehicle_names(len(slots))
        if not route:
            raise RuntimeError("formation route is empty")

        control_clients: dict[str, Any] = {}
        for name in vehicle_names:
            control_client = self.airsim.MultirotorClient(ip=self.host, port=self.port, timeout_value=2)
            control_client.confirmConnection()
            control_clients[name] = control_client

        safe_z = -abs(float(altitude_m))
        speed = max(0.5, float(speed_mps))
        executed: list[dict[str, Any]] = []

        def progress(status: str, message: str, **extra: Any) -> None:
            if progress_callback is not None:
                progress_callback(status=status, message=message, **extra)

        def targets_for(leader_x: float, leader_y: float, leader_z: float) -> list[dict[str, Any]]:
            targets: list[dict[str, Any]] = []
            for index, name in enumerate(vehicle_names):
                slot = slots[index] if index < len(slots) else {"x": 0.0, "y": 0.0, "z": 0.0}
                targets.append(
                    {
                        "vehicle": name,
                        "x": leader_x + float(slot.get("x", 0.0)),
                        "y": leader_y + float(slot.get("y", 0.0)),
                        "z": leader_z + float(slot.get("z", 0.0)),
                        "slot": slot,
                    }
                )
            return targets

        def wait_phase(duration_s: float, status: str, message: str, targets: list[dict[str, Any]] | None = None) -> None:
            deadline = time.time() + max(0.0, float(duration_s))
            while time.time() < deadline:
                progress(status, message, formation_targets=targets or [])
                time.sleep(min(0.75, max(0.05, deadline - time.time())))

        progress("arming", f"正在解锁 {len(vehicle_names)} 架编队无人机。")
        for name in vehicle_names:
            self._run_command(
                lambda name=name: (
                    control_clients[name].enableApiControl(True, vehicle_name=name),
                    control_clients[name].armDisarm(True, vehicle_name=name),
                )
            )

        progress("taking_off", "编队起飞指令已发送。")
        for name in vehicle_names:
            def takeoff_command(name: str = name) -> None:
                client = control_clients[name]
                try:
                    client.takeoffAsync(timeout_sec=10, vehicle_name=name)
                except TypeError:
                    client.takeoffAsync(vehicle_name=name)

            self._run_command(takeoff_command)
        wait_phase(4.0, "taking_off", "编队起飞中。")

        progress("climbing", f"编队爬升至 {abs(safe_z):.1f}m。")
        for name in vehicle_names:
            self._run_command(lambda name=name: control_clients[name].moveToZAsync(z=safe_z, velocity=2.0, vehicle_name=name))
        wait_phase(max(4.0, abs(safe_z) / 2.0 + 2.0), "climbing", f"编队爬升至 {abs(safe_z):.1f}m。")

        previous: dict[str, float] | None = None
        for waypoint_index, waypoint in enumerate(route, start=1):
            leader_x = float(waypoint.get("x", 0.0))
            leader_y = float(waypoint.get("y", 0.0))
            leader_z = float(waypoint.get("z", safe_z))
            targets = targets_for(leader_x, leader_y, leader_z)
            progress(
                "forming" if waypoint_index == 1 else "running",
                f"编队航点 {waypoint_index}/{len(route)} 执行中。",
                current_waypoint_index=waypoint_index,
                total_waypoints=len(route),
                formation_targets=targets,
            )
            for target in targets:
                name = str(target["vehicle"])
                self._run_command(
                    lambda name=name, target=target: control_clients[name].moveToPositionAsync(
                        x=float(target["x"]),
                        y=float(target["y"]),
                        z=float(target["z"]),
                        velocity=speed,
                        vehicle_name=name,
                    )
                )
                executed.append({"vehicle": name, "x": target["x"], "y": target["y"], "z": target["z"]})
            leg = math.hypot(leader_x - previous["x"], leader_y - previous["y"]) if previous is not None else 0.0
            wait_phase(
                max(2.0, leg / speed + 1.0),
                "forming" if waypoint_index == 1 else "running",
                f"编队航点 {waypoint_index}/{len(route)} 执行中。",
                targets,
            )
            previous = {"x": leader_x, "y": leader_y, "z": leader_z}

        for name in vehicle_names:
            def hover_command(name: str = name) -> None:
                client = control_clients[name]
                try:
                    client.hoverAsync(vehicle_name=name)
                except TypeError:
                    client.hoverAsync()

            self._run_command(hover_command)
        self.last_success_at = time.time()
        return {"vehicles": list(vehicle_names), "executed": executed, "route": route, "slots": slots}
    def hover(self) -> None:
        client = self._require_client()
        self._stop_position_hold()
        with self._lock:
            state = client.getMultirotorState(vehicle_name=self.vehicle_name)
            target_x = float(state.kinematics_estimated.position.x_val)
            target_y = float(state.kinematics_estimated.position.y_val)
            target_z = float(state.kinematics_estimated.position.z_val)
            try:
                hover_task = client.hoverAsync(vehicle_name=self.vehicle_name)
            except TypeError:
                hover_task = client.hoverAsync()
            self._join_async(hover_task, timeout_s=2.0)
            self.last_success_at = time.time()
        self._start_position_hold(x=target_x, y=target_y, z=target_z)

    def hold_current_position(self, duration_s: float = 2.0) -> None:
        client = self._require_client()
        self._stop_position_hold()
        with self._lock:
            client.enableApiControl(True, vehicle_name=self.vehicle_name)
            client.armDisarm(True, vehicle_name=self.vehicle_name)
            state = client.getMultirotorState(vehicle_name=self.vehicle_name)
            position = state.kinematics_estimated.position
            target_x = float(position.x_val)
            target_y = float(position.y_val)
            target_z = float(position.z_val)
            try:
                hold_task = client.moveToPositionAsync(
                    x=float(position.x_val),
                    y=float(position.y_val),
                    z=float(position.z_val),
                    velocity=1.0,
                    timeout_sec=max(1.0, float(duration_s)),
                    vehicle_name=self.vehicle_name,
                )
            except TypeError:
                hold_task = client.moveToPositionAsync(
                    float(position.x_val),
                    float(position.y_val),
                    float(position.z_val),
                    1.0,
                    vehicle_name=self.vehicle_name,
                )
            self._join_async(hold_task, timeout_s=max(1.5, float(duration_s) + 0.5))
            self.last_success_at = time.time()
        self._start_position_hold(x=target_x, y=target_y, z=target_z)

    def land(self) -> None:
        client = self._require_client()
        self._stop_position_hold()
        with self._lock:
            client.landAsync(vehicle_name=self.vehicle_name)
            self.last_success_at = time.time()

    def stop(self) -> None:
        self.hard_stop()

    def hard_stop(self) -> dict[str, float]:
        client = self._require_client()
        self._stop_position_hold()
        with self._lock:
            return self._hard_stop_locked(client)

    def return_to_launch(self, altitude_m: float = 8.0) -> None:
        client = self._require_client()
        self._stop_position_hold()
        with self._lock:
            try:
                state = client.getMultirotorState(vehicle_name=self.vehicle_name)
                position = state.kinematics_estimated.position
                safe_z = min(float(position.z_val), -abs(float(altitude_m)))
            except Exception:
                safe_z = -abs(float(altitude_m))
            client.moveToZAsync(z=safe_z, velocity=3.0, vehicle_name=self.vehicle_name)
            time.sleep(0.3)
            client.moveToPositionAsync(
                x=0.0,
                y=0.0,
                z=safe_z,
                velocity=4.0,
                vehicle_name=self.vehicle_name,
            )
            self.last_success_at = time.time()

    def goto_local(
        self,
        x: float,
        y: float,
        z: float,
        speed_mps: float = 3.0,
        face_target: bool = True,
    ) -> None:
        client = self._require_client()
        self._stop_position_hold()
        target_x = float(x)
        target_y = float(y)
        target_z = float(z)
        cruise_speed = max(0.5, float(speed_mps))
        control_dt = 0.16
        accel_limit = 1.35
        kp_xy = 0.85
        kp_z = 0.75
        damp = 0.35
        vx_cmd = 0.0
        vy_cmd = 0.0
        vz_cmd = 0.0
        start = time.time()
        with self._lock:
            initial_state = client.getMultirotorState(vehicle_name=self.vehicle_name)
            initial_position = initial_state.kinematics_estimated.position
        initial_dist = math.sqrt(
            (target_x - float(initial_position.x_val)) ** 2
            + (target_y - float(initial_position.y_val)) ** 2
            + (target_z - float(initial_position.z_val)) ** 2
        )
        timeout_s = max(7.0, (initial_dist / max(0.6, cruise_speed)) + 10.0)
        with self._lock:
            client.enableApiControl(True, vehicle_name=self.vehicle_name)
            client.armDisarm(True, vehicle_name=self.vehicle_name)
        while time.time() - start <= timeout_s:
            with self._lock:
                state = client.getMultirotorState(vehicle_name=self.vehicle_name)
                position = state.kinematics_estimated.position
                velocity = state.kinematics_estimated.linear_velocity
            dx = target_x - float(position.x_val)
            dy = target_y - float(position.y_val)
            dz = target_z - float(position.z_val)
            dist_xy = math.hypot(dx, dy)
            speed_now = math.sqrt(
                float(velocity.x_val) ** 2
                + float(velocity.y_val) ** 2
                + float(velocity.z_val) ** 2
            )
            if dist_xy <= 0.6 and abs(dz) <= 0.35 and speed_now <= 0.8:
                break
            desired_vx = kp_xy * dx - damp * float(velocity.x_val)
            desired_vy = kp_xy * dy - damp * float(velocity.y_val)
            desired_vz = kp_z * dz - damp * float(velocity.z_val)
            v_xy = math.hypot(desired_vx, desired_vy)
            if v_xy > cruise_speed:
                scale = cruise_speed / max(1e-6, v_xy)
                desired_vx *= scale
                desired_vy *= scale
            desired_vz = max(-1.6, min(1.6, desired_vz))
            max_delta = accel_limit * control_dt
            vx_cmd += max(-max_delta, min(max_delta, desired_vx - vx_cmd))
            vy_cmd += max(-max_delta, min(max_delta, desired_vy - vy_cmd))
            vz_cmd += max(-max_delta, min(max_delta, desired_vz - vz_cmd))
            yaw_deg = self._yaw_to_world_target(target_x, target_y)
            yaw_mode = self.airsim.YawMode(False, yaw_deg) if (face_target and self.airsim is not None) else (self.airsim.YawMode(True, 0.0) if self.airsim is not None else None)
            try:
                with self._lock:
                    task = client.moveByVelocityAsync(
                        vx=float(vx_cmd),
                        vy=float(vy_cmd),
                        vz=float(vz_cmd),
                        duration=control_dt,
                        drivetrain=self.airsim.DrivetrainType.MaxDegreeOfFreedom if self.airsim is not None else 0,
                        yaw_mode=yaw_mode,
                        vehicle_name=self.vehicle_name,
                    )
                self._join_async(task, timeout_s=control_dt + 0.35)
            except TypeError:
                with self._lock:
                    task = client.moveByVelocityAsync(float(vx_cmd), float(vy_cmd), float(vz_cmd), control_dt, vehicle_name=self.vehicle_name)
                self._join_async(task, timeout_s=control_dt + 0.35)
        self.hold_current_position(duration_s=0.7)
        self.last_success_at = time.time()

    def move_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        duration_s: float = 0.2,
        yaw_rate_deg_s: float = 0.0,
    ) -> None:
        client = self._require_client()
        self._stop_position_hold()
        with self._lock:
            client.enableApiControl(True, vehicle_name=self.vehicle_name)
            yaw_mode = self.airsim.YawMode(True, float(yaw_rate_deg_s)) if self.airsim is not None else None
            task = None
            try:
                task = client.moveByVelocityAsync(
                    vx=float(vx),
                    vy=float(vy),
                    vz=float(vz),
                    duration=max(0.05, float(duration_s)),
                    drivetrain=self.airsim.DrivetrainType.MaxDegreeOfFreedom if self.airsim is not None else 0,
                    yaw_mode=yaw_mode,
                    vehicle_name=self.vehicle_name,
                )
            except TypeError:
                task = client.moveByVelocityAsync(
                    float(vx),
                    float(vy),
                    float(vz),
                    max(0.05, float(duration_s)),
                    vehicle_name=self.vehicle_name,
                )
            self._join_async(task, timeout_s=max(0.5, float(duration_s) + 0.5))
            self.last_success_at = time.time()

    def face_local(self, x: float, y: float) -> None:
        client = self._require_client()
        with self._lock:
            yaw_deg = self._yaw_to_world_target(float(x), float(y))
            try:
                client.rotateToYawAsync(yaw=yaw_deg, timeout_sec=2, vehicle_name=self.vehicle_name)
            except TypeError:
                client.rotateToYawAsync(yaw_deg)
            time.sleep(0.45)
            self.last_success_at = time.time()

    def _yaw_to_world_target(self, target_x: float, target_y: float) -> float:
        if self.client is None:
            return 0.0
        try:
            state = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
            position = state.kinematics_estimated.position
            dx = float(target_x) - float(position.x_val)
            dy = float(target_y) - float(position.y_val)
            if abs(dx) < 0.05 and abs(dy) < 0.05:
                return math.degrees(self._yaw_from_quaternion(state.kinematics_estimated.orientation))
            return math.degrees(math.atan2(dy, dx))
        except Exception:
            return 0.0

    def recover_from_collision(
        self,
        climb_m: float = 8.0,
        backoff_m: float = 10.0,
        force_relocate: bool = True,
    ) -> dict[str, Any]:
        client = self._require_client()
        with self._lock:
            self._cancel_last_task_locked(client)
            state = client.getMultirotorState(vehicle_name=self.vehicle_name)
            position = state.kinematics_estimated.position
            safe_z = float(position.z_val) - abs(float(climb_m))
            retreat_x = float(position.x_val) - abs(float(backoff_m))
            retreat_y = float(position.y_val)
            client.enableApiControl(True, vehicle_name=self.vehicle_name)
            client.moveToZAsync(z=safe_z, velocity=1.5, vehicle_name=self.vehicle_name)
            time.sleep(0.8)
            client.moveToPositionAsync(
                x=retreat_x,
                y=retreat_y,
                z=safe_z,
                velocity=1.5,
                vehicle_name=self.vehicle_name,
            )
            time.sleep(min(3.0, max(1.2, abs(float(backoff_m)) / 4.0)))
            recovery_mode = "motion"
            if force_relocate:
                collision = self.collision_status()
                state_after_motion = client.getMultirotorState(vehicle_name=self.vehicle_name)
                moved_position = state_after_motion.kinematics_estimated.position
                moved_distance = math.sqrt(
                    (float(moved_position.x_val) - float(position.x_val)) ** 2
                    + (float(moved_position.y_val) - float(position.y_val)) ** 2
                    + (float(moved_position.z_val) - float(position.z_val)) ** 2
                )
                if collision.has_collided or moved_distance < 1.0:
                    self._set_vehicle_pose(retreat_x, retreat_y, safe_z, ignore_collision=True)
                    recovery_mode = "relocate"
                    time.sleep(0.2)
            stopped_at = self._hard_stop_locked(client)
            self.last_success_at = time.time()
            return {
                "x": retreat_x,
                "y": retreat_y,
                "z": safe_z,
                "mode": recovery_mode,
                "stopped_at": stopped_at,
            }

    def collision_status(self) -> CollisionStatus:
        if not self.connect() or self.client is None:
            return CollisionStatus()
        try:
            with self._lock:
                info = self.client.simGetCollisionInfo(vehicle_name=self.vehicle_name)
            return CollisionStatus(
                has_collided=bool(getattr(info, "has_collided", False)),
                object_name=str(getattr(info, "object_name", "") or ""),
                object_id=int(getattr(info, "object_id", -1) or -1),
                impact_point=self._vector_to_api(getattr(info, "impact_point", None)),
                normal=self._vector_to_api(getattr(info, "normal", None)),
                time_stamp=int(getattr(info, "time_stamp", 0) or 0),
            )
        except Exception as exc:
            self.last_error = str(exc)
            return CollisionStatus()

    def obstacle_status(
        self,
        sensor_name: str = "LidarSensor1",
        lookahead_m: float = 8.0,
        corridor_width_m: float = 4.0,
        vertical_window_m: float = 2.5,
    ) -> ObstacleStatus:
        if not self.connect() or self.client is None:
            return ObstacleStatus(sensor_name=sensor_name, error="AirSim is not connected")
        try:
            with self._lock:
                lidar = self.client.getLidarData(lidar_name=sensor_name, vehicle_name=self.vehicle_name)
            raw_points = list(getattr(lidar, "point_cloud", []) or [])
            if len(raw_points) < 3:
                return ObstacleStatus(sensor_name=sensor_name, error="No LiDAR points available")

            points: list[tuple[float, float, float, float, float]] = []
            half_width = max(0.5, float(corridor_width_m) / 2.0)
            lookahead = max(1.0, float(lookahead_m))
            vertical_up = max(0.5, float(vertical_window_m))
            vertical_down = 0.5

            for index in range(0, len(raw_points) - 2, 3):
                x = float(raw_points[index])
                y = float(raw_points[index + 1])
                z = float(raw_points[index + 2])
                if x <= 0.6 or x > lookahead or z < -vertical_up or z > vertical_down:
                    continue
                distance = math.sqrt(x * x + y * y + z * z)
                if distance < 0.8:
                    continue
                angle_deg = math.degrees(math.atan2(y, x))
                points.append((x, y, z, distance, angle_deg))

            analysis = self._analyze_obstacle_points(points, lookahead, half_width)
            return ObstacleStatus(
                available=True,
                blocked=analysis["blocked"],
                nearest_distance_m=analysis["nearest_distance_m"],
                front_points=analysis["front_points"],
                left_points=analysis["left_points"],
                right_points=analysis["right_points"],
                recommended_side=analysis["recommended_side"],
                risk_level=analysis["risk_level"],
                sector_clearance=analysis["sector_clearance"],
                sector_points=analysis["sector_points"],
                clusters=analysis["clusters"],
                decision=analysis["decision"],
                sensor_name=sensor_name,
            )
        except Exception as exc:
            self.last_error = str(exc)
            return ObstacleStatus(sensor_name=sensor_name, error=str(exc))

    def distance_sensors_status(
        self,
        emergency_threshold_m: float = 2.5,
        side_emergency_threshold_m: float = 1.5,
    ) -> DistanceSensorsStatus:
        if not self.connect() or self.client is None:
            return DistanceSensorsStatus(error="AirSim is not connected")
        names = {
            "front": "DistanceFront",
            "left": "DistanceLeft",
            "right": "DistanceRight",
        }
        readings: dict[str, float | None] = {}
        errors: list[str] = []
        with self._lock:
            for direction, sensor_name in names.items():
                try:
                    data = self.client.getDistanceSensorData(
                        distance_sensor_name=sensor_name,
                        vehicle_name=self.vehicle_name,
                    )
                    distance = float(getattr(data, "distance", 0.0) or 0.0)
                    if not math.isfinite(distance) or distance <= 0.0:
                        readings[direction] = None
                    else:
                        readings[direction] = round(distance, 2)
                except Exception as exc:
                    readings[direction] = None
                    errors.append(f"{sensor_name}: {exc}")
        valid = {name: value for name, value in readings.items() if value is not None}
        if not valid:
            return DistanceSensorsStatus(error="; ".join(errors[-3:]) if errors else "No distance sensor readings")
        nearest_direction, min_value = min(valid.items(), key=lambda item: float(item[1]))
        front_threshold = max(0.5, float(emergency_threshold_m))
        side_threshold = max(0.3, float(side_emergency_threshold_m))
        front_value = readings.get("front")
        left_value = readings.get("left")
        right_value = readings.get("right")
        emergency_direction = "none"
        if front_value is not None and front_value <= front_threshold:
            emergency_direction = "front"
        elif left_value is not None and left_value <= side_threshold:
            emergency_direction = "left"
        elif right_value is not None and right_value <= side_threshold:
            emergency_direction = "right"
        return DistanceSensorsStatus(
            available=True,
            front_m=readings.get("front"),
            left_m=readings.get("left"),
            right_m=readings.get("right"),
            min_m=min_value,
            nearest_direction=nearest_direction,
            emergency_direction=emergency_direction,
            emergency=emergency_direction != "none",
            error="; ".join(errors[-3:]),
        )

    def lidar_obstacle_points_world(
        self,
        sensor_name: str = "LidarSensor1",
        range_m: float = 45.0,
        vertical_window_m: float = 2.5,
        min_range_m: float = 1.0,
        vertical_down_m: float = 1.4,
        denoise_cell_m: float = 2.0,
        min_points_per_cell: int = 2,
    ) -> list[tuple[float, float]]:
        if not self.connect() or self.client is None:
            return []
        try:
            with self._lock:
                state = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
                lidar = self.client.getLidarData(lidar_name=sensor_name, vehicle_name=self.vehicle_name)
            position = state.kinematics_estimated.position
            orientation = state.kinematics_estimated.orientation
            raw_points = list(getattr(lidar, "point_cloud", []) or [])
            points: list[tuple[float, float]] = []
            max_range = max(2.0, float(range_m))
            min_range = max(0.0, float(min_range_m))
            vertical_up = max(0.5, float(vertical_window_m))
            vertical_down = max(0.2, float(vertical_down_m))
            for index in range(0, len(raw_points) - 2, 3):
                local_x = float(raw_points[index])
                local_y = float(raw_points[index + 1])
                local_z = float(raw_points[index + 2])
                horizontal_distance = math.hypot(local_x, local_y)
                if local_x <= 0.6 or horizontal_distance < min_range or horizontal_distance > max_range:
                    continue
                # AirSim uses NED-style axes. Positive local z is usually below the
                # sensor, so a wide symmetric z window tends to turn the ground into
                # a false obstacle map. Keep only points near the flight layer.
                if local_z < -vertical_up or local_z > vertical_down:
                    continue
                rotated_x, rotated_y, rotated_z = self._rotate_vector_by_quaternion(
                    local_x,
                    local_y,
                    local_z,
                    orientation,
                )
                world_x = float(position.x_val) + rotated_x
                world_y = float(position.y_val) + rotated_y
                points.append((world_x, world_y))
            return self._denoise_world_points(points, cell_m=denoise_cell_m, min_points=min_points_per_cell)
        except Exception as exc:
            self.last_error = str(exc)
            return []

    @staticmethod
    def _denoise_world_points(
        points: list[tuple[float, float]],
        cell_m: float = 2.0,
        min_points: int = 2,
    ) -> list[tuple[float, float]]:
        if not points or min_points <= 1:
            return points
        cell = max(0.5, float(cell_m))
        buckets: dict[tuple[int, int], list[tuple[float, float]]] = {}
        for x, y in points:
            key = (int(math.floor(x / cell)), int(math.floor(y / cell)))
            buckets.setdefault(key, []).append((x, y))
        filtered: list[tuple[float, float]] = []
        for bucket_points in buckets.values():
            if len(bucket_points) >= min_points:
                filtered.extend(bucket_points)
        return filtered

    def _analyze_obstacle_points(
        self,
        points: list[tuple[float, float, float, float, float]],
        lookahead_m: float,
        half_width_m: float,
    ) -> dict[str, Any]:
        sectors = {
            "hard_left": (-90.0, -45.0),
            "left": (-45.0, -15.0),
            "front": (-15.0, 15.0),
            "right": (15.0, 45.0),
            "hard_right": (45.0, 90.0),
        }
        sector_points = {name: 0 for name in sectors}
        sector_clearance: dict[str, float | None] = {name: None for name in sectors}
        nearest: float | None = None
        front_points = 0
        left_points = 0
        right_points = 0

        for x, y, _z, distance, angle in points:
            if nearest is None or distance < nearest:
                nearest = distance
            if abs(y) <= half_width_m:
                front_points += 1
            elif y < -half_width_m:
                left_points += 1
            else:
                right_points += 1
            for name, (low, high) in sectors.items():
                if low <= angle < high:
                    sector_points[name] += 1
                    current = sector_clearance[name]
                    sector_clearance[name] = round(distance, 2) if current is None else min(current, round(distance, 2))
                    break

        clusters = self._cluster_obstacle_points(points)
        blocked = self._is_front_blocked(points, half_width_m, lookahead_m)
        nearest_value = round(nearest, 2) if nearest is not None else None
        risk_level = self._risk_level(nearest_value, front_points, blocked)
        left_score = self._clearance_score(sector_clearance, sector_points, ("hard_left", "left"))
        right_score = self._clearance_score(sector_clearance, sector_points, ("right", "hard_right"))
        recommended_side = "none"
        if blocked:
            recommended_side = "left" if left_score >= right_score else "right"

        return {
            "blocked": blocked,
            "nearest_distance_m": nearest_value,
            "front_points": front_points,
            "left_points": left_points,
            "right_points": right_points,
            "recommended_side": recommended_side,
            "risk_level": risk_level,
            "sector_clearance": sector_clearance,
            "sector_points": sector_points,
            "clusters": clusters[:5],
            "decision": {
                "left_score": round(left_score, 2),
                "right_score": round(right_score, 2),
                "reason": "front corridor occupied" if blocked else "front corridor clear",
            },
        }

    @staticmethod
    def _is_front_blocked(points: list[tuple[float, float, float, float, float]], half_width_m: float, lookahead_m: float) -> bool:
        near_front = 0
        for x, y, _z, distance, _angle in points:
            if x <= lookahead_m and abs(y) <= half_width_m and distance <= lookahead_m:
                near_front += 1
        return near_front >= 6

    @staticmethod
    def _risk_level(nearest_m: float | None, front_points: int, blocked: bool) -> str:
        if nearest_m is None:
            return "clear"
        if blocked and nearest_m <= 4.0:
            return "critical"
        if blocked and nearest_m <= 7.0:
            return "high"
        if front_points > 0:
            return "medium"
        return "low"

    @staticmethod
    def _clearance_score(
        sector_clearance: dict[str, float | None],
        sector_points: dict[str, int],
        sector_names: tuple[str, str],
    ) -> float:
        score = 0.0
        for name in sector_names:
            clearance = sector_clearance.get(name)
            points = sector_points.get(name, 0)
            score += (clearance if clearance is not None else 12.0) - min(points, 30) * 0.15
        return score

    @staticmethod
    def _cluster_obstacle_points(points: list[tuple[float, float, float, float, float]]) -> list[dict[str, Any]]:
        cells: dict[tuple[int, int], list[tuple[float, float, float, float, float]]] = {}
        cell_size = 2.0
        for point in points:
            x, y, _z, _distance, _angle = point
            cell = (int(x // cell_size), int(y // cell_size))
            cells.setdefault(cell, []).append(point)

        clusters: list[dict[str, Any]] = []
        for cell_points in cells.values():
            if len(cell_points) < 3:
                continue
            count = len(cell_points)
            cx = sum(point[0] for point in cell_points) / count
            cy = sum(point[1] for point in cell_points) / count
            cz = sum(point[2] for point in cell_points) / count
            min_distance = min(point[3] for point in cell_points)
            clusters.append(
                {
                    "points": count,
                    "centroid": {"x": round(cx, 2), "y": round(cy, 2), "z": round(cz, 2)},
                    "min_distance_m": round(min_distance, 2),
                }
            )
        clusters.sort(key=lambda item: (float(item["min_distance_m"]), -int(item["points"])))
        return clusters

    def _hard_stop_locked(self, client: Any) -> dict[str, float]:
        self._cancel_last_task_locked(client)
        try:
            client.moveByVelocityAsync(
                vx=0.0,
                vy=0.0,
                vz=0.0,
                duration=0.25,
                vehicle_name=self.vehicle_name,
            )
        except TypeError:
            try:
                client.moveByVelocityAsync(0.0, 0.0, 0.0, 0.25)
            except Exception:
                pass
        except Exception:
            pass
        time.sleep(0.3)

        self._cancel_last_task_locked(client)
        state = client.getMultirotorState(vehicle_name=self.vehicle_name)
        position = state.kinematics_estimated.position
        x = float(position.x_val)
        y = float(position.y_val)
        z = float(position.z_val)
        try:
            client.moveToPositionAsync(
                x=x,
                y=y,
                z=z,
                velocity=0.5,
                timeout_sec=1.0,
                vehicle_name=self.vehicle_name,
            )
        except TypeError:
            client.moveToPositionAsync(x=x, y=y, z=z, velocity=0.5, vehicle_name=self.vehicle_name)
        time.sleep(0.3)

        self._cancel_last_task_locked(client)
        try:
            client.hoverAsync(vehicle_name=self.vehicle_name)
        except TypeError:
            client.hoverAsync()
        self.last_success_at = time.time()
        return {"x": x, "y": y, "z": z}

    def _cancel_last_task_locked(self, client: Any) -> None:
        try:
            client.cancelLastTask(vehicle_name=self.vehicle_name)
        except TypeError:
            client.cancelLastTask()
        except Exception:
            pass

    def _require_client(self) -> Any:
        if not self.connect() or self.client is None:
            raise RuntimeError(f"AirSim is not connected: {self.last_error}")
        return self.client

    def camera_frame(self) -> CameraFrame | None:
        if not self.connect() or self.client is None or self.airsim is None:
            return None
        image_types = (
            int(self.airsim.ImageType.Scene),
        )
        errors: list[str] = []
        camera_names = list(self.camera_candidates)
        for _ in range(2):
            for camera_name in camera_names:
                for image_type in image_types:
                    frame = self._request_camera_frame(camera_name, image_type)
                    if frame is not None:
                        self.last_camera_name = camera_name
                        self.last_image_type = image_type
                        self.active_camera_name = camera_name
                        self.connected = True
                        self.last_success_at = time.time()
                        self.failure_count = 0
                        self.last_error = ""
                        return frame
                    if self.last_error:
                        errors.append(f"{camera_name}:{image_type}: {self.last_error}")
        if errors:
            self.last_error = "; ".join(errors[-4:])
        return None

    def capture_camera_frame(self, camera_name: str | None = None) -> CameraFrame | None:
        if not self.connect() or self.client is None or self.airsim is None:
            return None
        camera = str(camera_name or self.active_camera_name or "front_center").strip()
        if camera not in self.camera_candidates:
            raise ValueError(f"Camera '{camera}' is not available")
        frame = self._request_camera_frame(camera, int(self.airsim.ImageType.Scene))
        if frame is not None:
            self.last_camera_name = camera
            self.last_image_type = frame.image_type
            self.active_camera_name = camera
            self.connected = True
            self.last_success_at = time.time()
            self.failure_count = 0
            self.last_error = ""
        return frame

    def camera_options(self) -> dict[str, Any]:
        return {
            "selected": self.active_camera_name,
            "candidates": list(self.camera_candidates),
            "last_camera_name": self.last_camera_name,
            "last_image_type": self.last_image_type,
        }

    def select_camera(self, camera_name: str) -> dict[str, Any]:
        camera = str(camera_name).strip()
        if camera not in self.camera_candidates:
            raise ValueError(f"Camera '{camera}' is not available")
        with self._lock:
            self.active_camera_name = camera
            self.last_camera_name = ""
            self.last_image_type = -1
        return self.camera_options()

    def _ordered_camera_candidates(self) -> tuple[str, ...]:
        selected = self.active_camera_name
        if selected not in self.camera_candidates:
            return self.camera_candidates
        return (selected,) + tuple(camera for camera in self.camera_candidates if camera != selected)

    def probe_cameras(self) -> list[dict[str, Any]]:
        results: list[dict[str, Any]] = []
        if not self.connect() or self.client is None or self.airsim is None:
            return [{"camera": "-", "image_type": "-", "ok": False, "error": self.last_error}]
        for camera_name in self.camera_candidates:
            image_type = int(self.airsim.ImageType.Scene)
            frame = self._request_camera_frame(camera_name, image_type)
            results.append(
                {
                    "camera": camera_name,
                    "image_type": image_type,
                    "ok": frame is not None,
                    "bytes": len(frame.data) if frame is not None else 0,
                    "content_type": frame.content_type if frame is not None else "",
                    "error": "" if frame is not None else self.last_error,
                }
            )
        return results

    def _request_camera_frame(self, camera_name: str, image_type: int, retries: int = 3) -> CameraFrame | None:
        last_exc: Exception | None = None
        for attempt in range(retries):
            try:
                with self._lock:
                    responses = self.client.simGetImages(
                        [
                            self.airsim.ImageRequest(
                                camera_name,
                                image_type,
                                False,
                                True,
                            )
                        ],
                        vehicle_name=self.vehicle_name,
                    )
                if not responses:
                    continue
                data = responses[0].image_data_uint8
                if not data:
                    self.last_error = "empty image_data_uint8"
                    self._note_failure(soft=True)
                    continue
                raw = bytes(data)
                self.connected = True
                self.last_success_at = time.time()
                self.failure_count = 0
                return CameraFrame(
                    data=raw,
                    content_type=self._guess_image_content_type(raw),
                    camera_name=camera_name,
                    image_type=image_type,
                )
            except Exception as exc:
                last_exc = exc
                self.last_error = str(exc)
                self._note_failure(soft=True)
        if last_exc:
            self.last_error = str(last_exc)
        return None

    def _set_vehicle_pose(self, x: float, y: float, z: float, ignore_collision: bool = True) -> None:
        if self.airsim is None or self.client is None:
            return
        pose = self.airsim.Pose(
            self.airsim.Vector3r(float(x), float(y), float(z)),
            self.airsim.to_quaternion(0.0, 0.0, 0.0),
        )
        self.client.simSetVehiclePose(pose, ignore_collision=bool(ignore_collision), vehicle_name=self.vehicle_name)

    @staticmethod
    def _guess_image_content_type(data: bytes) -> str:
        if data.startswith(b"\xff\xd8"):
            return "image/jpeg"
        if data.startswith(b"\x89PNG\r\n\x1a\n"):
            return "image/png"
        return "application/octet-stream"

    @staticmethod
    def _vector_to_api(value: Any) -> dict[str, float] | None:
        if value is None:
            return None
        try:
            return {
                "x": float(value.x_val),
                "y": float(value.y_val),
                "z": float(value.z_val),
            }
        except Exception:
            return None

    @staticmethod
    def _yaw_from_quaternion(value: Any) -> float:
        try:
            x = float(value.x_val)
            y = float(value.y_val)
            z = float(value.z_val)
            w = float(value.w_val)
            return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        except Exception:
            return 0.0

    @staticmethod
    def _rotate_vector_by_quaternion(vx: float, vy: float, vz: float, quaternion: Any) -> tuple[float, float, float]:
        try:
            qx = float(quaternion.x_val)
            qy = float(quaternion.y_val)
            qz = float(quaternion.z_val)
            qw = float(quaternion.w_val)
        except Exception:
            return vx, vy, vz

        # q * v * q^-1, expanded to avoid allocating temporary objects.
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return (
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        )

    def _note_failure(self, soft: bool = False) -> None:
        self.failure_count += 1
        if not soft and self.failure_count >= 3 and (time.time() - self.last_success_at) > 5.0:
            self.connected = False
