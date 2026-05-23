from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import queue
import threading
import time
from typing import Any

import tornado.ioloop

from config import nav_config

logger = logging.getLogger(__name__)

# ── AirSim msgpack encoding fix ─────────────────────────────────────
# AirSim C++ server sends non-UTF-8 bytes (image/sensor binary) in RPC
# responses. msgpack 0.5.6 with unicode_errors='strict' throws
# UnicodeDecodeError → tornado closes RPC connection → drone falls.
try:
    import msgpack as _msgpack_lib
    import msgpackrpc.transport.tcp as _tcp_transport

    _msgpack_version_str = getattr(_msgpack_lib, '__version__', '0.0')
    _msgpack_version = tuple(int(x) for x in _msgpack_version_str.split(".")[:2])
    if _msgpack_version < (1, 0):
        _orig_base_socket_init = _tcp_transport.BaseSocket.__init__

        def _patched_base_socket_init(self, stream, encodings):
            self._stream = stream
            self._packer = _msgpack_lib.Packer(encoding=encodings[0], default=lambda x: x.to_msgpack())
            self._unpacker = _msgpack_lib.Unpacker(encoding=encodings[1], unicode_errors='replace')

        _tcp_transport.BaseSocket.__init__ = _patched_base_socket_init
        logger.info("AirSim msgpack encoding patch applied (unicode_errors=replace, version=%s)", _msgpack_version_str)
    else:
        logger.info("msgpack >= 1.0 — encoding patch not needed")
except Exception:
    logger.warning("AirSim msgpack encoding patch FAILED", exc_info=True)

# ── msgpackrpc step_timeout fix ─────────────────────────────────────
# Original step_timeout calls loop.stop() then loop.start() inside a
# PeriodicCallback that fires *within* loop.start().  The nested start()
# fails with "IOLoop is already running".  We remove the nested start()
# — Future.join()'s while loop will re-enter start() by itself.
try:
    import msgpackrpc.session
    from msgpackrpc.error import TimeoutError as _MsgpackTimeoutError
    from msgpackrpc.compat import iteritems as _msgpack_iteritems

    _orig_step_timeout = msgpackrpc.session.Session.step_timeout

    def _patched_step_timeout(self: msgpackrpc.session.Session) -> None:
        timeouts = []
        for msgid, future in _msgpack_iteritems(self._request_table):
            if future.step_timeout():
                timeouts.append(msgid)
        if not timeouts:
            return
        self._loop.stop()
        for to_msgid in timeouts:
            to_future = self._request_table.pop(to_msgid)
            to_future.set_error(_MsgpackTimeoutError("Request timed out"))

    msgpackrpc.session.Session.step_timeout = _patched_step_timeout
    logger.info("msgpackrpc step_timeout patch applied (stop only, no restart)")
except Exception:
    logger.warning("msgpackrpc step_timeout patch failed", exc_info=True)


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


class AirSimRpcEngine:
    """Dedicated thread with its own tornado IOLoop for all AirSim RPC calls.

    Creating the AirSim client inside this thread ensures msgpackrpc gets its
    own isolated IOLoop — completely separate from the HTTP server's tornado
    event loop.  All RPC calls are serialized through a command queue so there
    is never concurrent access to the AirSim client.
    """

    def __init__(
        self,
        host: str,
        port: int,
        vehicle_name: str,
        vehicle_names: tuple[str, ...],
        camera_candidates: tuple[str, ...],
    ) -> None:
        self._host = host
        self._port = int(port)
        self._vehicle_name = vehicle_name
        self._vehicle_names = tuple(vehicle_names)
        self._camera_candidates = camera_candidates
        self._active_camera_name = "front_center"

        # Mutable state — only the engine thread writes these, but they are
        # read from other threads for status polls.
        self._connected = False
        self._client: Any = None
        self._airsim: Any = None
        self._last_error = ""
        self._last_success_at = 0.0
        self._failure_count = 0

        # Position-hold state (set from adapter, consumed by engine idle loop).
        self._hold_active = False
        self._hold_x: float | None = None
        self._hold_y: float | None = None
        self._hold_z: float | None = None
        self._hold_stop = threading.Event()

        self._cmd_queue: queue.Queue[CommandJob | None] = queue.Queue()
        self._stop_event = threading.Event()

        # -- fast nav-control mode (zero queue overhead) ------------------
        self._nav_active = False
        self._nav_vx = 0.0
        self._nav_vy = 0.0
        self._nav_vz = 0.0
        self._nav_dur = 0.6
        self._nav_lock = threading.Lock()
        self._nav_cache_cb: Any = None  # set by adapter

        # Shared LiDAR cache — engine captures during nav tick so the
        # agent thread never needs to _exec_rpc for LiDAR (which would
        # block the nav loop for multi-second RPC durations).
        self._shared_lidar_points: list[tuple[float, float, float]] = []
        self._shared_lidar_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._shared_lidar_lock = threading.Lock()
        self._last_lidar_capture_at = 0.0
        self._lidar_capture_interval_s = 0.8

        self._thread = threading.Thread(
            target=self._run, name="aeromind-rpc-engine", daemon=True,
        )
        self._thread.start()

        # Lightweight telemetry poller — own AirSim client, own thread.
        # Decoupled from the engine's nav/command loop so map updates
        # never stall on slow LiDAR or command-queue backpressure.
        self._telemetry_poller_stop = threading.Event()
        self._telemetry_thread = threading.Thread(
            target=self._run_telemetry_poller, name="aeromind-telemetry", daemon=True,
        )
        self._telemetry_thread.start()

    # -- read-only accessors (safe from any thread) ------------------------

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def client(self) -> Any:
        return self._client

    @property
    def airsim(self) -> Any:
        return self._airsim

    @property
    def last_error(self) -> str:
        return self._last_error

    @property
    def last_success_at(self) -> float:
        return self._last_success_at

    @property
    def vehicle_name(self) -> str:
        return self._vehicle_name

    @property
    def vehicle_names(self) -> tuple[str, ...]:
        return self._vehicle_names

    @property
    def active_camera_name(self) -> str:
        return self._active_camera_name

    # -- public API -------------------------------------------------------

    def execute(self, fn: Any, timeout_s: float = 30.0) -> Any:
        """Run *fn(client, airsim)* on the RPC thread and return its result."""
        job = CommandJob(fn)
        self._cmd_queue.put(job)
        if not job.done.wait(timeout=timeout_s):
            raise TimeoutError("AirSim RPC timed out")
        if job.error is not None:
            raise job.error
        return job.result

    def switch_vehicle(self, name: str) -> None:
        self._vehicle_name = str(name).strip()
        self._active_camera_name = "front_center"

    def start_position_hold(self, x: float | None, y: float | None, z: float | None) -> None:
        self._hold_x = float(x) if x is not None else None
        self._hold_y = float(y) if y is not None else None
        self._hold_z = float(z) if z is not None else None
        self._hold_stop.clear()
        self._hold_active = True

    def stop_position_hold(self) -> None:
        self._hold_active = False
        self._hold_stop.set()

    # -- fast nav-control mode ---------------------------------------------
    # While _nav_active is True the engine skips the command queue for
    # velocity + telemetry reads and runs a tight 20 Hz loop directly on
    # the engine thread.  The agent-tools thread sets desired velocity
    # via set_nav_velocity() and reads fresh telemetry through the shared
    # nav-cache callback.

    def set_nav_cache_callback(self, cb: Any) -> None:
        self._nav_cache_cb = cb

    def set_nav_velocity(self, vx: float, vy: float, vz: float, dur: float = 0.6) -> None:
        with self._nav_lock:
            self._nav_vx = float(vx)
            self._nav_vy = float(vy)
            self._nav_vz = float(vz)
            self._nav_dur = max(0.1, float(dur))

    def start_nav_control(self) -> None:
        self._nav_active = True

    def stop_nav_control(self) -> None:
        self._nav_active = False
        with self._nav_lock:
            self._nav_vx = 0.0
            self._nav_vy = 0.0
            self._nav_vz = 0.0

    @staticmethod
    def _rotate_vector_by_quaternion(vx: float, vy: float, vz: float, quaternion: Any) -> tuple[float, float, float]:
        try:
            qx = float(quaternion.x_val)
            qy = float(quaternion.y_val)
            qz = float(quaternion.z_val)
            qw = float(quaternion.w_val)
        except Exception:
            return vx, vy, vz
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return (
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        )

    def capture_lidar_to_shared_cache(
        self, client: Any, vehicle: str, px: float, py: float, pz: float,
        orientation: Any,
    ) -> bool:
        """Capture LiDAR on the engine thread and write to shared cache.

        Uses the telemetry already available in _nav_tick_impl so only a
        single getLidarData RPC is needed — no redundant round-trip.

        Returns True if new points were captured, False on failure.
        """
        try:
            lidar = client.getLidarData(lidar_name="LidarSensor1", vehicle_name=vehicle)
            raw_points = list(getattr(lidar, "point_cloud", []) or [])
            points_3d: list[tuple[float, float, float]] = []
            max_range = 35.0
            for idx in range(0, len(raw_points) - 2, 3):
                lx = float(raw_points[idx])
                ly = float(raw_points[idx + 1])
                lz = float(raw_points[idx + 2])
                if lx <= 0.6 or math.hypot(lx, ly) < 1.0 or math.hypot(lx, ly) > max_range:
                    continue
                if lz < -3.0 or lz > 2.0:
                    continue
                rx, ry, rz = self._rotate_vector_by_quaternion(lx, ly, lz, orientation)
                points_3d.append((px + rx, py + ry, pz + rz))
            with self._shared_lidar_lock:
                self._shared_lidar_points = points_3d
                self._shared_lidar_pos = (px, py, pz)
            self._last_lidar_capture_at = time.time()
            return True
        except Exception:
            logger.warning("capture_lidar_to_shared_cache failed", exc_info=True)
            return False

    def get_shared_lidar(self) -> tuple[list[tuple[float, float, float]], tuple[float, float, float]]:
        """Return a copy of the engine-captured LiDAR points + position.

        Safe to call from any thread.  Returns ([], (0,0,0)) if no data yet.
        """
        with self._shared_lidar_lock:
            return list(self._shared_lidar_points), self._shared_lidar_pos

    # -- fast nav tick (called from _run when _nav_active) ------------------

    def _drain_queue(self, client: Any, airsim_module: Any) -> None:
        """Process at most one pending command-queue job per call.

        Called after each nav tick so that LiDAR reads, depth-camera
        captures, and other _exec_rpc requests from the agent-tools thread
        don't starve while the engine is in nav-control mode.

        Processing is limited to a single job — multi-second LiDAR RPCs
        must not block the nav tick loop and starve the telemetry cache.
        Remaining jobs are drained on subsequent ticks.
        """
        try:
            job = self._cmd_queue.get_nowait()
        except queue.Empty:
            return
        if job is None:  # stop sentinel — re-queue and let main loop handle it
            self._cmd_queue.put(None)
            return
        try:
            job.result = job.fn(client, airsim_module)
            self._last_success_at = time.time()
        except BaseException as exc:
            job.error = exc
            error_str = str(exc).lower()
            if any(kw in error_str for kw in ("timeout", "connection", "closed", "broken pipe", "ioloop")):
                self._connected = False
                self._client = None
                self._airsim = None
                # Nav-mode survival is decided by the agent loop's health
                # check — a single failed queued job should not kill it.
        finally:
            job.done.set()
            self._cmd_queue.task_done()

    def _nav_tick_direct(self, client: Any, airsim_module: Any) -> None:
        """One iteration of the fast nav control loop.  Runs on engine thread
        with zero queue overhead — direct RPC calls only.

        Never raises — a single failed tick must not kill nav mode.
        """
        try:
            self._nav_tick_impl(client, airsim_module)
        except Exception:
            logger.warning("_nav_tick_direct failed", exc_info=True)

    def _nav_tick_impl(self, client: Any, airsim_module: Any) -> None:
        with self._nav_lock:
            vx = self._nav_vx
            vy = self._nav_vy
            vz = self._nav_vz
            dur = self._nav_dur

        vehicle = self._vehicle_name

        # -- velocity command (yaw follows movement direction) --------------
        yaw_deg = math.degrees(math.atan2(vy, vx)) if (abs(vx) > 0.001 or abs(vy) > 0.001) else 0.0
        yaw_mode = airsim_module.YawMode(True, float(yaw_deg)) if airsim_module is not None else None
        try:
            try:
                client.moveByVelocityAsync(
                    vx=float(vx), vy=float(vy), vz=float(vz), duration=float(dur),
                    drivetrain=airsim_module.DrivetrainType.MaxDegreeOfFreedom if airsim_module else 0,
                    yaw_mode=yaw_mode,
                    vehicle_name=vehicle,
                )
            except TypeError:
                client.moveByVelocityAsync(float(vx), float(vy), float(vz), float(dur), vehicle_name=vehicle)
        except Exception:
            logger.warning("_nav_tick_direct velocity command failed", exc_info=True)

        # -- telemetry (always attempt, even if velocity failed) -------------
        px = py = pz = 0.0
        speed = 0.0
        telemetry_ok = False
        orientation: Any = None
        try:
            state = client.getMultirotorState(vehicle_name=vehicle)
            kinematics = state.kinematics_estimated
            position = kinematics.position
            velocity = kinematics.linear_velocity
            orientation = kinematics.orientation
            px = float(position.x_val)
            py = float(position.y_val)
            pz = float(position.z_val)
            speed = math.sqrt(
                float(velocity.x_val) ** 2 + float(velocity.y_val) ** 2 + float(velocity.z_val) ** 2
            )
            telemetry_ok = True
        except Exception:
            logger.warning("_nav_tick_direct telemetry read failed", exc_info=True)
            if self._nav_cache_cb is None:
                return
            # Fall through with px/py/pz = 0 — collision & distance data
            # still get refreshed and pushed to cache even when telemetry
            # is temporarily unavailable.

        # -- collision -----------------------------------------------------
        col_obj = CollisionStatus()
        try:
            col = client.simGetCollisionInfo(vehicle_name=vehicle)
            col_obj = CollisionStatus(
                has_collided=bool(getattr(col, "has_collided", False)),
                object_name=str(getattr(col, "object_name", "") or ""),
                object_id=int(getattr(col, "object_id", -1) or -1),
                time_stamp=int(getattr(col, "time_stamp", 0) or 0),
            )
        except Exception:
            logger.warning("_nav_tick_direct collision read failed", exc_info=True)

        # -- distance sensors ----------------------------------------------
        dists: dict[str, float | None] = {}
        for direction, sname in (
            ("front", "DistanceFront"),
            ("left", "DistanceLeft"),
            ("right", "DistanceRight"),
        ):
            try:
                d = client.getDistanceSensorData(distance_sensor_name=sname, vehicle_name=vehicle)
                val = float(getattr(d, "distance", 0.0) or 0.0)
                dists[direction] = round(val, 2) if (math.isfinite(val) and val > 0.0) else None
            except Exception:
                dists[direction] = None

        # -- populate cache via callback -----------------------------------
        cb = self._nav_cache_cb
        if cb is not None:
            tel = UavTelemetry(
                name=vehicle, mode="AIRSIM-LIVE",
                altitude_m=max(0.0, -pz), speed_mps=speed,
                x=px, y=py, z=pz,
            )
            valid = {k: v for k, v in dists.items() if v is not None}
            nd, mv = ("none", None)
            if valid:
                nd, mv = min(valid.items(), key=lambda item: float(item[1]))
            emergency = False; ed = "none"
            fv, lv, rv = dists.get("front"), dists.get("left"), dists.get("right")
            if fv is not None and fv <= 2.6:
                emergency = True; ed = "front"
            elif lv is not None and lv <= 1.6:
                emergency = True; ed = "left"
            elif rv is not None and rv <= 1.6:
                emergency = True; ed = "right"
            dist = DistanceSensorsStatus(
                available=len(valid) > 0,
                front_m=fv, left_m=lv, right_m=rv,
                min_m=mv, nearest_direction=nd,
                emergency_direction=ed, emergency=emergency,
            )
            cb({
                "telemetry": tel, "collision": col_obj, "distances": dist,
                "cmd_vx": vx, "cmd_vy": vy, "cmd_vz": vz,
            })

            # -- periodic LiDAR capture (no queue, no agent-thread RPC) -----
            now = time.time()
            if telemetry_ok and now - self._last_lidar_capture_at >= self._lidar_capture_interval_s:
                self.capture_lidar_to_shared_cache(
                    client, vehicle, px, py, pz, orientation,
                )

    def stop(self) -> None:
        self._stop_event.set()
        self._telemetry_poller_stop.set()
        self.stop_position_hold()
        self._cmd_queue.put(None)
        self._thread.join(timeout=5.0)
        self._telemetry_thread.join(timeout=3.0)

    # -- telemetry poller (own thread, own AirSim client) -------------------

    def _run_telemetry_poller(self) -> None:
        """Dedicated telemetry thread.

        Own AirSim client, completely independent of the engine's RPC queue.
        Polls getMultirotorState every 100 ms and pushes telemetry into the
        shared nav-cache so the WebSocket map updates at a fixed rate
        regardless of what the engine command loop is doing.
        """
        client: Any = None
        vehicle = self._vehicle_name
        host, port = self._host, self._port

        while not self._telemetry_poller_stop.is_set():
            # -- connect / reconnect -----------------------------------------
            if client is None:
                try:
                    import airsim
                    client = airsim.MultirotorClient(ip=host, port=port, timeout_value=1.5)
                    client.confirmConnection()
                except Exception:
                    client = None
                    self._telemetry_poller_stop.wait(2.0)
                    continue

            # -- read telemetry ----------------------------------------------
            cb = self._nav_cache_cb
            try:
                state = client.getMultirotorState(vehicle_name=vehicle)
                kinematics = state.kinematics_estimated
                pos = kinematics.position
                vel = kinematics.linear_velocity
                px, py, pz = float(pos.x_val), float(pos.y_val), float(pos.z_val)
                speed = math.sqrt(float(vel.x_val)**2 + float(vel.y_val)**2 + float(vel.z_val)**2)
                if cb is not None:
                    tel = UavTelemetry(
                        name=vehicle, mode="AIRSIM-LIVE",
                        altitude_m=max(0.0, -pz), speed_mps=speed,
                        x=px, y=py, z=pz,
                    )
                    cb({"telemetry": tel})
            except Exception:
                logger.warning("telemetry poller read failed", exc_info=True)
                cb = self._nav_cache_cb  # re-read in case it was set mid-flight
                client = None  # force reconnect next iteration
                self._telemetry_poller_stop.wait(0.5)
                continue

            self._telemetry_poller_stop.wait(0.1)

        # cleanup
        if client is not None:
            try:
                client.close()
            except Exception:
                pass

    # -- internal: main loop ----------------------------------------------

    def _run(self) -> None:
        loop = tornado.ioloop.IOLoop()
        loop.make_current()

        client: Any = None
        airsim_module: Any = None

        last_hold_tick = 0.0

        while not self._stop_event.is_set():
            # 1.  Try to connect / reconnect.
            if client is None:
                client, airsim_module = self._try_connect()
                if client is not None:
                    self._client = client
                    self._airsim = airsim_module
                    self._connected = True
                    self._hold_active = False  # reset hold on fresh connection
            if client is None:
                self._stop_event.wait(3.0)
                continue

            # 2.  Fast nav-control mode — tight loop on engine thread with
            #     zero queue overhead.  The agent-tools thread sets desired
            #     velocity via set_nav_velocity() and reads telemetry from
            #     the shared nav-cache callback.
            #     After the tick we drain any pending command-queue jobs
            #     (e.g. LiDAR reads, depth-camera captures) without blocking,
            #     so the agent-tools thread's _exec_rpc calls don't starve.
            if self._nav_active:
                try:
                    self._nav_tick_direct(client, airsim_module)
                    self._last_success_at = time.time()
                except Exception:
                    self._last_error = "nav tick failed"
                    client = None
                    self._client = None
                    self._airsim = None
                    self._connected = False
                    self._nav_active = False
                # Drain pending commands (non-blocking).
                self._drain_queue(client, airsim_module)
                continue

            # 3.  Position-hold idle tick (when active and no command queued).
            now = time.time()
            if self._hold_active and self._cmd_queue.empty() and (now - last_hold_tick) >= 0.35:
                last_hold_tick = now
                try:
                    self._hold_tick(client, airsim_module)
                    self._last_success_at = time.time()
                except Exception:
                    self._last_error = "hold tick failed"
                    client = None
                    self._client = None
                    self._airsim = None
                    self._connected = False
                continue

            # 4.  Wait for next command (with timeout so hold ticks can fire).
            try:
                job = self._cmd_queue.get(timeout=0.35)
            except queue.Empty:
                continue

            if job is None:  # stop sentinel
                self._cmd_queue.task_done()
                return

            # 5.  Execute command.
            self._hold_active = False
            try:
                job.result = job.fn(client, airsim_module)
                self._last_success_at = time.time()
            except BaseException as exc:
                job.error = exc
                error_str = str(exc).lower()
                if any(kw in error_str for kw in ("timeout", "connection", "closed", "broken pipe", "ioloop")):
                    self._connected = False
                    self._client = None
                    self._airsim = None
                    self._hold_active = False
                    client = None
                    airsim_module = None
            finally:
                job.done.set()
                self._cmd_queue.task_done()

        # cleanup
        self._connected = False
        self._client = None
        self._airsim = None
        try:
            loop.stop()
            loop.close()
        except Exception:
            pass

    def _try_connect(self) -> tuple[Any, Any] | tuple[None, None]:
        try:
            import airsim
            client = airsim.MultirotorClient(ip=self._host, port=self._port, timeout_value=1.5)
            client.confirmConnection()

            discovered = self._vehicle_names
            try:
                listed = client.listVehicles()
                listed = [str(n).strip() for n in listed if str(n).strip()]
                if listed:
                    discovered = tuple(listed)
            except Exception:
                pass

            self._vehicle_names = discovered
            if self._vehicle_name not in self._vehicle_names and self._vehicle_names:
                self._vehicle_name = self._vehicle_names[0]

            for name in self._vehicle_names:
                try:
                    client.enableApiControl(True, vehicle_name=name)
                except Exception:
                    pass

            self._last_error = ""
            self._failure_count = 0
            self._last_success_at = time.time()
            logger.info("AirSim RPC engine connected to %s:%s", self._host, self._port)
            return client, airsim
        except Exception as exc:
            self._last_error = str(exc)
            self._failure_count += 1
            return None, None

    def _hold_tick(self, client: Any, airsim_module: Any) -> None:
        """One position-hold control cycle.  Runs inside _run()."""
        client.enableApiControl(True, vehicle_name=self._vehicle_name)
        client.armDisarm(True, vehicle_name=self._vehicle_name)
        state = client.getMultirotorState(vehicle_name=self._vehicle_name)
        position = state.kinematics_estimated.position
        px = float(position.x_val)
        py = float(position.y_val)
        hold_x = self._hold_x if self._hold_x is not None else px
        hold_y = self._hold_y if self._hold_y is not None else py
        target_z = self._hold_z if self._hold_z is not None else float(position.z_val)

        if self._hold_x is None:
            self._hold_x = hold_x
        if self._hold_y is None:
            self._hold_y = hold_y

        vx = max(-0.45, min(0.45, (hold_x - px) * 0.35))
        vy = max(-0.45, min(0.45, (hold_y - py) * 0.35))

        try:
            task = client.moveByVelocityZAsync(
                vx=float(vx), vy=float(vy), z=float(target_z),
                duration=0.5,
                drivetrain=airsim_module.DrivetrainType.MaxDegreeOfFreedom if airsim_module else 0,
                yaw_mode=airsim_module.YawMode(True, 0.0) if airsim_module else None,
                vehicle_name=self._vehicle_name,
            )
        except TypeError:
            task = client.moveByVelocityZAsync(
                float(vx), float(vy), float(target_z), 0.5,
                vehicle_name=self._vehicle_name,
            )
        self._join_async(task, timeout_s=0.8)

    @staticmethod
    def _join_async(task: Any, timeout_s: float | None = None) -> None:
        join = getattr(task, "join", None)
        if not callable(join):
            return
        try:
            join(timeout_s)
        except TypeError:
            join()

    def _hard_stop(self, client: Any, airsim_module: Any, vehicle_name: str) -> dict[str, float]:
        self._cancel_last_task(client, vehicle_name)
        try:
            client.moveByVelocityAsync(
                vx=0.0, vy=0.0, vz=0.0, duration=0.25,
                vehicle_name=vehicle_name,
            )
        except TypeError:
            try:
                client.moveByVelocityAsync(0.0, 0.0, 0.0, 0.25)
            except Exception:
                pass
        except Exception:
            pass
        time.sleep(0.3)

        self._cancel_last_task(client, vehicle_name)
        state = client.getMultirotorState(vehicle_name=vehicle_name)
        position = state.kinematics_estimated.position
        x = float(position.x_val)
        y = float(position.y_val)
        z = float(position.z_val)
        try:
            client.moveToPositionAsync(x=x, y=y, z=z, velocity=0.5, timeout_sec=1.0, vehicle_name=vehicle_name)
        except TypeError:
            client.moveToPositionAsync(x=x, y=y, z=z, velocity=0.5, vehicle_name=vehicle_name)
        time.sleep(0.3)

        self._cancel_last_task(client, vehicle_name)
        try:
            client.hoverAsync(vehicle_name=vehicle_name)
        except TypeError:
            client.hoverAsync()
        return {"x": x, "y": y, "z": z}

    @staticmethod
    def _cancel_last_task(client: Any, vehicle_name: str) -> None:
        try:
            client.cancelLastTask(vehicle_name=vehicle_name)
        except TypeError:
            client.cancelLastTask()
        except Exception:
            pass


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
        self.last_error = ""
        self.last_camera_name = ""
        self.last_image_type = -1
        self.camera_candidates = ("front_center", "bottom_center", "0", "front_left", "front_right")
        self.active_camera_name = "front_center"
        self.last_success_at = 0.0
        self.failure_count = 0

        self._engine = AirSimRpcEngine(
            host=host, port=port, vehicle_name=vehicle_name,
            vehicle_names=vehicle_names, camera_candidates=self.camera_candidates,
        )
        self._engine.set_nav_cache_callback(self.update_nav_cache)
        self._heartbeat_warned = False

        # Shared telemetry cache so WebSocket-ui reads never contend the
        # engine command queue (the nav loop writes, the http thread reads).
        self._nav_cache_lock = threading.Lock()
        self._nav_cache: dict[str, Any] = {
            "telemetry": UavTelemetry(),
            "collision": CollisionStatus(),
            "distances": DistanceSensorsStatus(error="no data yet"),
            "obstacle": ObstacleStatus(error="no data yet"),
            "lidar_points": [],
            "stale": True,
            "updated_at": 0.0,
        }

    # -- nav-cache helpers -------------------------------------------------

    def update_nav_cache(self, data: dict[str, Any]) -> None:
        with self._nav_cache_lock:
            self._nav_cache.update(data)
            self._nav_cache["stale"] = False
            self._nav_cache["updated_at"] = time.time()

    def get_nav_cache(self) -> dict[str, Any]:
        with self._nav_cache_lock:
            return dict(self._nav_cache)

    def get_shared_lidar(self) -> tuple[list[tuple[float, float, float]], tuple[float, float, float]]:
        """Return engine-captured LiDAR points + drone position.

        During nav mode the engine captures LiDAR periodically and writes
        to a shared cache.  The agent thread reads this without any RPC
        or queue round-trip, keeping the nav loop unblocked.
        """
        return self._engine.get_shared_lidar()

    # -- delegation properties --------------------------------------------

    @property
    def vehicle_name(self) -> str:
        return self._engine.vehicle_name

    @vehicle_name.setter
    def vehicle_name(self, value: str) -> None:
        self._engine.switch_vehicle(str(value).strip())

    @property
    def vehicle_names(self) -> tuple[str, ...]:
        return self._engine.vehicle_names

    @vehicle_names.setter
    def vehicle_names(self, value: tuple[str, ...]) -> None:
        pass  # engine owns this; set during _try_connect

    @property
    def connected(self) -> bool:
        return self._engine.connected

    @connected.setter
    def connected(self, value: bool) -> None:
        pass  # no-op; engine owns this state

    @property
    def client(self) -> Any:
        return self._engine.client

    @client.setter
    def client(self, value: Any) -> None:
        pass

    @property
    def airsim(self) -> Any:
        return self._engine.airsim

    @airsim.setter
    def airsim(self, value: Any) -> None:
        pass

    # -- internal helpers -------------------------------------------------

    def _exec_rpc(self, fn, timeout_s: float = 30.0) -> Any:
        """Run *fn(client, airsim)* on the engine thread, return result."""
        return self._engine.execute(fn, timeout_s=timeout_s)

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
        self._engine.stop_position_hold()

    def _start_position_hold(self, x: float | None = None, y: float | None = None, z: float | None = None) -> None:
        self._engine.start_position_hold(x, y, z)

    def stop_reconnect_worker(self) -> None:
        self._stop_position_hold()
        self._engine.stop()

    def switch_vehicle(self, vehicle_name: str) -> None:
        target = str(vehicle_name).strip()
        if target not in self.vehicle_names:
            raise ValueError(f"Vehicle '{target}' is not available")
        self._stop_position_hold()
        self.vehicle_name = target  # triggers engine.switch_vehicle via setter
        self.last_camera_name = ""
        self.last_image_type = -1

    def vehicle_options(self) -> dict[str, Any]:
        return {
            "selected": self.vehicle_name,
            "vehicles": list(self.vehicle_names),
        }

    def connect(self) -> bool:
        if self._engine.connected:
            return True
        # Engine auto-connects on next execute(); probe it.
        try:
            self._exec_rpc(lambda c, a: c.confirmConnection(), timeout_s=5.0)
            self.last_success_at = self._engine.last_success_at
            return True
        except Exception as exc:
            self.last_error = str(exc)
            self._note_failure()
            return False

    def _warmup_camera(self) -> None:
        if not self._engine.connected:
            return
        for _ in range(3):
            self.camera_frame()

    def is_connected(self, grace_s: float = 5.0) -> bool:
        engine_connected = self._engine.connected
        last_ok = self._engine.last_success_at
        return engine_connected or (last_ok > 0 and (time.time() - last_ok) <= grace_s)

    def telemetry(self, vehicle_name: str | None = None) -> UavTelemetry:
        target_name = vehicle_name or self.vehicle_name
        if not self._engine.connected:
            return UavTelemetry(name=target_name)
        try:
            def _do(c: Any, a: Any) -> UavTelemetry:
                state = c.getMultirotorState(vehicle_name=target_name)
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
            result = self._exec_rpc(_do)
            self.last_success_at = self._engine.last_success_at
            return result
        except Exception as exc:
            self._note_failure()
            self.last_error = str(exc)
            return UavTelemetry(name=target_name)

    def nav_telemetry(self) -> dict[str, Any]:
        """Batched read: telemetry + collision + distance sensors in one RPC round-trip.

        Returns a dict with keys ``telemetry``, ``collision``, ``distances``.
        Designed for the hot path in ``_closed_loop_drive_to_goal`` to avoid
        paying queue overhead for every individual sensor read.
        """
        if not self._engine.connected:
            return {
                "telemetry": UavTelemetry(),
                "collision": CollisionStatus(),
                "distances": DistanceSensorsStatus(error="AirSim is not connected"),
            }
        vehicle = self.vehicle_name
        try:
            def _do(c: Any, a: Any) -> dict[str, Any]:
                state = c.getMultirotorState(vehicle_name=vehicle)
                kinematics = state.kinematics_estimated
                position = kinematics.position
                velocity = kinematics.linear_velocity
                speed = math.sqrt(
                    float(velocity.x_val) ** 2
                    + float(velocity.y_val) ** 2
                    + float(velocity.z_val) ** 2
                )
                col = c.simGetCollisionInfo(vehicle_name=vehicle)
                dists: dict[str, float | None] = {}
                for direction, sname in (
                    ("front", "DistanceFront"),
                    ("left", "DistanceLeft"),
                    ("right", "DistanceRight"),
                ):
                    try:
                        d = c.getDistanceSensorData(
                            distance_sensor_name=sname, vehicle_name=vehicle,
                        )
                        val = float(getattr(d, "distance", 0.0) or 0.0)
                        dists[direction] = round(val, 2) if (math.isfinite(val) and val > 0.0) else None
                    except Exception:
                        dists[direction] = None
                return {
                    "px": float(position.x_val),
                    "py": float(position.y_val),
                    "pz": float(position.z_val),
                    "speed_mps": speed,
                    "has_collided": bool(getattr(col, "has_collided", False)),
                    "collision_obj_name": str(getattr(col, "object_name", "") or ""),
                    "collision_obj_id": int(getattr(col, "object_id", -1) or -1),
                    "collision_ts": int(getattr(col, "time_stamp", 0) or 0),
                    "collision_impact": self._vector_to_api(getattr(col, "impact_point", None)),
                    "collision_normal": self._vector_to_api(getattr(col, "normal", None)),
                    "dist_front": dists.get("front"),
                    "dist_left": dists.get("left"),
                    "dist_right": dists.get("right"),
                }

            data = self._exec_rpc(_do)
            self.last_success_at = self._engine.last_success_at

            tel = UavTelemetry(
                name=vehicle,
                mode="AIRSIM-LIVE",
                altitude_m=max(0.0, -data["pz"]),
                speed_mps=data["speed_mps"],
                x=data["px"],
                y=data["py"],
                z=data["pz"],
            )

            col = CollisionStatus(
                has_collided=data["has_collided"],
                object_name=data["collision_obj_name"],
                object_id=data["collision_obj_id"],
                impact_point=data["collision_impact"],
                normal=data["collision_normal"],
                time_stamp=data["collision_ts"],
            )

            valid = {
                k: v
                for k, v in [
                    ("front", data["dist_front"]),
                    ("left", data["dist_left"]),
                    ("right", data["dist_right"]),
                ]
                if v is not None
            }
            if valid:
                nearest_direction, min_val = min(valid.items(), key=lambda item: float(item[1]))
            else:
                nearest_direction, min_val = "none", None

            emergency = False
            emergency_direction = "none"
            front_v = data["dist_front"]
            left_v = data["dist_left"]
            right_v = data["dist_right"]
            if front_v is not None and front_v <= 2.6:
                emergency = True
                emergency_direction = "front"
            elif left_v is not None and left_v <= 1.6:
                emergency = True
                emergency_direction = "left"
            elif right_v is not None and right_v <= 1.6:
                emergency = True
                emergency_direction = "right"

            dist = DistanceSensorsStatus(
                available=len(valid) > 0,
                front_m=data["dist_front"],
                left_m=data["dist_left"],
                right_m=data["dist_right"],
                min_m=min_val,
                nearest_direction=nearest_direction,
                emergency_direction=emergency_direction,
                emergency=emergency,
            )

            result = {"telemetry": tel, "collision": col, "distances": dist}
            self.update_nav_cache(result)
            return result
        except Exception as exc:
            self._note_failure()
            self.last_error = str(exc)
            return {
                "telemetry": UavTelemetry(),
                "collision": CollisionStatus(),
                "distances": DistanceSensorsStatus(error=str(exc)),
            }

    def nav_tick(
        self,
        vx: float,
        vy: float,
        vz: float,
        duration_s: float = 0.5,
    ) -> dict[str, Any]:
        """One combined RPC: apply velocity THEN read telemetry + sensors.

        On the engine thread we first send the velocity command (overriding any
        previous one), then immediately read fresh telemetry, collision, and
        distance-sensor data.  This packs what used to be two separate
        ``_exec_rpc`` calls (``move_velocity`` + ``nav_telemetry``) into one
        round-trip, cutting per-iteration queue overhead in half.

        The result is also written into the shared nav-cache so the WebSocket
        UI never needs to call into the engine at all.
        """
        if not self._engine.connected:
            return {
                "telemetry": UavTelemetry(),
                "collision": CollisionStatus(),
                "distances": DistanceSensorsStatus(error="AirSim is not connected"),
            }
        vehicle = self.vehicle_name
        vx_f = float(vx)
        vy_f = float(vy)
        vz_f = float(vz)
        dur = max(0.1, float(duration_s))

        try:
            def _do(c: Any, a: Any) -> dict[str, Any]:
                # -- velocity command (fire-and-forget, overrides previous) --
                try:
                    c.moveByVelocityAsync(
                        vx=vx_f, vy=vy_f, vz=vz_f, duration=dur,
                        drivetrain=a.DrivetrainType.MaxDegreeOfFreedom,
                        yaw_mode=a.YawMode(True, 0.0),
                        vehicle_name=vehicle,
                    )
                except TypeError:
                    c.moveByVelocityAsync(vx_f, vy_f, vz_f, dur, vehicle_name=vehicle)

                # -- telemetry --
                state = c.getMultirotorState(vehicle_name=vehicle)
                kinematics = state.kinematics_estimated
                position = kinematics.position
                velocity = kinematics.linear_velocity
                speed = math.sqrt(
                    float(velocity.x_val) ** 2
                    + float(velocity.y_val) ** 2
                    + float(velocity.z_val) ** 2
                )

                # -- collision --
                col = c.simGetCollisionInfo(vehicle_name=vehicle)

                # -- distance sensors --
                dists: dict[str, float | None] = {}
                for direction, sname in (
                    ("front", "DistanceFront"),
                    ("left", "DistanceLeft"),
                    ("right", "DistanceRight"),
                ):
                    try:
                        d = c.getDistanceSensorData(
                            distance_sensor_name=sname, vehicle_name=vehicle,
                        )
                        val = float(getattr(d, "distance", 0.0) or 0.0)
                        dists[direction] = round(val, 2) if (math.isfinite(val) and val > 0.0) else None
                    except Exception:
                        dists[direction] = None

                return {
                    "px": float(position.x_val),
                    "py": float(position.y_val),
                    "pz": float(position.z_val),
                    "speed_mps": speed,
                    "has_collided": bool(getattr(col, "has_collided", False)),
                    "collision_obj_name": str(getattr(col, "object_name", "") or ""),
                    "collision_obj_id": int(getattr(col, "object_id", -1) or -1),
                    "collision_ts": int(getattr(col, "time_stamp", 0) or 0),
                    "collision_impact": self._vector_to_api(getattr(col, "impact_point", None)),
                    "collision_normal": self._vector_to_api(getattr(col, "normal", None)),
                    "dist_front": dists.get("front"),
                    "dist_left": dists.get("left"),
                    "dist_right": dists.get("right"),
                }

            data = self._exec_rpc(_do)
            self.last_success_at = self._engine.last_success_at

            tel = UavTelemetry(
                name=vehicle, mode="AIRSIM-LIVE",
                altitude_m=max(0.0, -data["pz"]),
                speed_mps=data["speed_mps"],
                x=data["px"], y=data["py"], z=data["pz"],
            )
            col = CollisionStatus(
                has_collided=data["has_collided"],
                object_name=data["collision_obj_name"],
                object_id=data["collision_obj_id"],
                impact_point=data["collision_impact"],
                normal=data["collision_normal"],
                time_stamp=data["collision_ts"],
            )
            valid = {
                k: v for k, v in [
                    ("front", data["dist_front"]),
                    ("left", data["dist_left"]),
                    ("right", data["dist_right"]),
                ] if v is not None
            }
            nd, mv = ("none", None)
            if valid:
                nd, mv = min(valid.items(), key=lambda item: float(item[1]))
            emergency = False
            ed = "none"
            fv, lv, rv = data["dist_front"], data["dist_left"], data["dist_right"]
            if fv is not None and fv <= 2.6:
                emergency = True; ed = "front"
            elif lv is not None and lv <= 1.6:
                emergency = True; ed = "left"
            elif rv is not None and rv <= 1.6:
                emergency = True; ed = "right"
            dist = DistanceSensorsStatus(
                available=len(valid) > 0,
                front_m=fv, left_m=lv, right_m=rv,
                min_m=mv, nearest_direction=nd,
                emergency_direction=ed, emergency=emergency,
            )

            result: dict[str, Any] = {"telemetry": tel, "collision": col, "distances": dist}
            self.update_nav_cache(result)
            return result
        except Exception as exc:
            self._note_failure()
            self.last_error = str(exc)
            return {
                "telemetry": UavTelemetry(),
                "collision": CollisionStatus(),
                "distances": DistanceSensorsStatus(error=str(exc)),
            }

    def takeoff(self, altitude_m: float = 8.0) -> None:
        target_z = -abs(float(altitude_m))
        self._stop_position_hold()
        vehicle = self.vehicle_name

        def _do(c: Any, a: Any) -> tuple[float, float, float]:
            c.enableApiControl(True, vehicle_name=vehicle)
            c.armDisarm(True, vehicle_name=vehicle)
            try:
                takeoff_task = c.takeoffAsync(timeout_sec=10, vehicle_name=vehicle)
            except TypeError:
                takeoff_task = c.takeoffAsync(vehicle_name=vehicle)
            self._join_async(takeoff_task, timeout_s=12.0)
            climb_task = c.moveToZAsync(z=target_z, velocity=2.0, vehicle_name=vehicle)
            self._join_async(climb_task, timeout_s=max(6.0, abs(target_z) / 2.0 + 4.0))
            settle_deadline = time.time() + max(6.0, abs(target_z) * 1.2)
            while time.time() < settle_deadline:
                state = c.getMultirotorState(vehicle_name=vehicle)
                current_z = float(state.kinematics_estimated.position.z_val)
                if abs(current_z - target_z) <= 0.6:
                    break
                retry_task = c.moveToZAsync(z=target_z, velocity=1.8, vehicle_name=vehicle)
                self._join_async(retry_task, timeout_s=2.0)
                time.sleep(0.12)
            final_state = c.getMultirotorState(vehicle_name=vehicle)
            final_position = final_state.kinematics_estimated.position
            return float(final_position.x_val), float(final_position.y_val), float(target_z)

        fx, fy, fz = self._exec_rpc(_do)
        self.last_success_at = self._engine.last_success_at
        self._start_position_hold(x=fx, y=fy, z=fz)

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
        airsim_module = self._engine.airsim
        if airsim_module is None:
            raise RuntimeError("AirSim module is not available")
        vehicle_names = self.formation_vehicle_names(len(slots))
        if not route:
            raise RuntimeError("formation route is empty")

        # Formation creates per-vehicle clients inline; their msgpackrpc
        # transports use the calling thread's IOLoop.  Formation is executed
        # from agent_tools (not the HTTP handler), so the IOLoop is isolated.
        control_clients: dict[str, Any] = {}
        for name in vehicle_names:
            control_client = airsim_module.MultirotorClient(ip=self.host, port=self.port, timeout_value=2)
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
            control_clients[name].enableApiControl(True, vehicle_name=name)
            control_clients[name].armDisarm(True, vehicle_name=name)

        progress("taking_off", "编队起飞指令已发送。")
        for name in vehicle_names:
            try:
                control_clients[name].takeoffAsync(timeout_sec=10, vehicle_name=name)
            except TypeError:
                control_clients[name].takeoffAsync(vehicle_name=name)
        wait_phase(4.0, "taking_off", "编队起飞中。")

        progress("climbing", f"编队爬升至 {abs(safe_z):.1f}m。")
        for name in vehicle_names:
            control_clients[name].moveToZAsync(z=safe_z, velocity=2.0, vehicle_name=name)
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
                control_clients[name].moveToPositionAsync(
                    x=float(target["x"]), y=float(target["y"]), z=float(target["z"]),
                    velocity=speed, vehicle_name=name,
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
            try:
                control_clients[name].hoverAsync(vehicle_name=name)
            except TypeError:
                control_clients[name].hoverAsync()
        self.last_success_at = time.time()
        return {"vehicles": list(vehicle_names), "executed": executed, "route": route, "slots": slots}

    def hover(self) -> None:
        self._stop_position_hold()
        vehicle = self.vehicle_name

        def _do(c: Any, a: Any) -> tuple[float, float, float]:
            state = c.getMultirotorState(vehicle_name=vehicle)
            target_x = float(state.kinematics_estimated.position.x_val)
            target_y = float(state.kinematics_estimated.position.y_val)
            target_z = float(state.kinematics_estimated.position.z_val)
            try:
                hover_task = c.hoverAsync(vehicle_name=vehicle)
            except TypeError:
                hover_task = c.hoverAsync()
            self._join_async(hover_task, timeout_s=2.0)
            return target_x, target_y, target_z

        tx, ty, tz = self._exec_rpc(_do)
        self.last_success_at = self._engine.last_success_at
        self._start_position_hold(x=tx, y=ty, z=tz)

    def hold_current_position(self, duration_s: float = 2.0) -> None:
        self._stop_position_hold()
        vehicle = self.vehicle_name
        dur = max(1.0, float(duration_s))

        def _do(c: Any, a: Any) -> tuple[float, float, float]:
            c.enableApiControl(True, vehicle_name=vehicle)
            c.armDisarm(True, vehicle_name=vehicle)
            state = c.getMultirotorState(vehicle_name=vehicle)
            position = state.kinematics_estimated.position
            px, py, pz = float(position.x_val), float(position.y_val), float(position.z_val)
            try:
                hold_task = c.moveToPositionAsync(
                    x=px, y=py, z=pz, velocity=1.0,
                    timeout_sec=dur, vehicle_name=vehicle,
                )
            except TypeError:
                hold_task = c.moveToPositionAsync(px, py, pz, 1.0, vehicle_name=vehicle)
            self._join_async(hold_task, timeout_s=max(1.5, dur + 0.5))
            return px, py, pz

        px, py, pz = self._exec_rpc(_do)
        self.last_success_at = self._engine.last_success_at
        self._start_position_hold(x=px, y=py, z=pz)

    def land(self) -> None:
        self._stop_position_hold()
        vehicle = self.vehicle_name
        self._exec_rpc(lambda c, a: c.landAsync(vehicle_name=vehicle))
        self.last_success_at = self._engine.last_success_at

    def stop(self) -> None:
        self.hard_stop()

    def hard_stop(self) -> dict[str, float]:
        self._stop_position_hold()
        vehicle = self.vehicle_name
        result = self._exec_rpc(
            lambda c, a: self._engine._hard_stop(c, a, vehicle)
        )
        self.last_success_at = self._engine.last_success_at
        return result

    def return_to_launch(self, altitude_m: float = 8.0) -> None:
        self._stop_position_hold()
        vehicle = self.vehicle_name
        alt = float(altitude_m)

        def _do(c: Any, a: Any) -> None:
            try:
                state = c.getMultirotorState(vehicle_name=vehicle)
                position = state.kinematics_estimated.position
                safe_z = min(float(position.z_val), -abs(alt))
            except Exception:
                safe_z = -abs(alt)
            c.moveToZAsync(z=safe_z, velocity=3.0, vehicle_name=vehicle)
            time.sleep(0.3)
            c.moveToPositionAsync(x=0.0, y=0.0, z=safe_z, velocity=4.0, vehicle_name=vehicle)

        self._exec_rpc(_do)
        self.last_success_at = self._engine.last_success_at

    def goto_local(
        self,
        x: float,
        y: float,
        z: float,
        speed_mps: float = 3.0,
        face_target: bool = True,
    ) -> None:
        self._stop_position_hold()
        vehicle = self.vehicle_name
        target_x = float(x)
        target_y = float(y)
        target_z = float(z)
        cruise_speed = max(0.5, float(speed_mps))

        def _do(c: Any, a: Any) -> None:
            control_dt = 0.16
            accel_limit = 1.35
            kp_xy = 0.85
            kp_z = 0.75
            damp = 0.35
            vx_cmd = 0.0
            vy_cmd = 0.0
            vz_cmd = 0.0

            initial_state = c.getMultirotorState(vehicle_name=vehicle)
            initial_position = initial_state.kinematics_estimated.position
            initial_dist = math.sqrt(
                (target_x - float(initial_position.x_val)) ** 2
                + (target_y - float(initial_position.y_val)) ** 2
                + (target_z - float(initial_position.z_val)) ** 2
            )
            timeout_s = max(7.0, (initial_dist / max(0.6, cruise_speed)) + 10.0)
            c.enableApiControl(True, vehicle_name=vehicle)
            c.armDisarm(True, vehicle_name=vehicle)

            start = time.time()
            while time.time() - start <= timeout_s:
                state = c.getMultirotorState(vehicle_name=vehicle)
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
                yaw_deg = self._yaw_to_world_target(c, target_x, target_y)
                yaw_mode = a.YawMode(False, yaw_deg) if (face_target and a is not None) else (a.YawMode(True, 0.0) if a is not None else None)
                try:
                    task = c.moveByVelocityAsync(
                        vx=float(vx_cmd), vy=float(vy_cmd), vz=float(vz_cmd),
                        duration=control_dt,
                        drivetrain=a.DrivetrainType.MaxDegreeOfFreedom if a is not None else 0,
                        yaw_mode=yaw_mode,
                        vehicle_name=vehicle,
                    )
                except TypeError:
                    task = c.moveByVelocityAsync(
                        float(vx_cmd), float(vy_cmd), float(vz_cmd), control_dt,
                        vehicle_name=vehicle,
                    )
                self._join_async(task, timeout_s=control_dt + 0.35)

        self._exec_rpc(_do)
        self.hold_current_position(duration_s=0.7)
        self.last_success_at = self._engine.last_success_at

    def move_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        duration_s: float = 0.2,
        yaw_rate_deg_s: float = 0.0,
    ) -> None:
        self._stop_position_hold()
        vehicle = self.vehicle_name
        vx_f = float(vx)
        vy_f = float(vy)
        vz_f = float(vz)
        dur = max(0.05, float(duration_s))
        yaw_rate = float(yaw_rate_deg_s)

        def _do(c: Any, a: Any) -> None:
            c.enableApiControl(True, vehicle_name=vehicle)
            yaw_mode = a.YawMode(True, yaw_rate) if a is not None else None
            try:
                task = c.moveByVelocityAsync(
                    vx=vx_f, vy=vy_f, vz=vz_f, duration=dur,
                    drivetrain=a.DrivetrainType.MaxDegreeOfFreedom if a is not None else 0,
                    yaw_mode=yaw_mode,
                    vehicle_name=vehicle,
                )
            except TypeError:
                task = c.moveByVelocityAsync(vx_f, vy_f, vz_f, dur, vehicle_name=vehicle)
            self._join_async(task, timeout_s=max(0.5, dur + 0.5))

        self._exec_rpc(_do)
        self.last_success_at = self._engine.last_success_at

    def face_local(self, x: float, y: float) -> None:
        vehicle = self.vehicle_name

        def _do(c: Any, a: Any) -> None:
            yaw_deg = self._yaw_to_world_target(float(x), float(y))
            try:
                c.rotateToYawAsync(yaw=yaw_deg, timeout_sec=2, vehicle_name=vehicle)
            except TypeError:
                c.rotateToYawAsync(yaw_deg)
            time.sleep(0.45)

        self._exec_rpc(_do)
        self.last_success_at = self._engine.last_success_at

    def _yaw_to_world_target(self, client: Any, target_x: float, target_y: float) -> float:
        try:
            state = client.getMultirotorState(vehicle_name=self.vehicle_name)
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
        self._stop_position_hold()
        vehicle = self.vehicle_name
        climb = float(climb_m)
        backoff = float(backoff_m)
        relocate = bool(force_relocate)

        def _do(c: Any, a: Any) -> dict[str, Any]:
            self._engine._cancel_last_task(c, vehicle)
            state = c.getMultirotorState(vehicle_name=vehicle)
            position = state.kinematics_estimated.position
            safe_z = float(position.z_val) - abs(climb)
            retreat_x = float(position.x_val) - abs(backoff)
            retreat_y = float(position.y_val)
            c.enableApiControl(True, vehicle_name=vehicle)
            c.moveToZAsync(z=safe_z, velocity=1.5, vehicle_name=vehicle)
            time.sleep(0.8)
            c.moveToPositionAsync(x=retreat_x, y=retreat_y, z=safe_z, velocity=1.5, vehicle_name=vehicle)
            time.sleep(min(3.0, max(1.2, abs(backoff) / 4.0)))
            recovery_mode = "motion"
            if relocate:
                try:
                    collision_info = c.simGetCollisionInfo(vehicle_name=vehicle)
                    has_collided = bool(getattr(collision_info, "has_collided", False))
                except Exception:
                    has_collided = False
                state_after_motion = c.getMultirotorState(vehicle_name=vehicle)
                moved_position = state_after_motion.kinematics_estimated.position
                moved_distance = math.sqrt(
                    (float(moved_position.x_val) - float(position.x_val)) ** 2
                    + (float(moved_position.y_val) - float(position.y_val)) ** 2
                    + (float(moved_position.z_val) - float(position.z_val)) ** 2
                )
                if has_collided or moved_distance < 1.0:
                    pose = a.Pose(a.Vector3r(retreat_x, retreat_y, safe_z), a.to_quaternion(0.0, 0.0, 0.0))
                    c.simSetVehiclePose(pose, ignore_collision=True, vehicle_name=vehicle)
                    recovery_mode = "relocate"
                    time.sleep(0.2)
            stopped_at = self._engine._hard_stop(c, a, vehicle)
            return {"x": retreat_x, "y": retreat_y, "z": safe_z, "mode": recovery_mode, "stopped_at": stopped_at}

        result = self._exec_rpc(_do)
        self.last_success_at = self._engine.last_success_at
        return result

    def collision_status(self) -> CollisionStatus:
        if not self._engine.connected:
            return CollisionStatus()
        vehicle = self.vehicle_name
        try:
            def _do(c: Any, a: Any) -> Any:
                return c.simGetCollisionInfo(vehicle_name=vehicle)
            info = self._exec_rpc(_do)
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
        if not self._engine.connected:
            return ObstacleStatus(sensor_name=sensor_name, error="AirSim is not connected")
        vehicle = self.vehicle_name
        try:
            def _do(c: Any, a: Any) -> Any:
                return c.getLidarData(lidar_name=sensor_name, vehicle_name=vehicle)
            lidar = self._exec_rpc(_do)
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
        if not self._engine.connected:
            return DistanceSensorsStatus(error="AirSim is not connected")
        vehicle = self.vehicle_name
        names = {"front": "DistanceFront", "left": "DistanceLeft", "right": "DistanceRight"}
        readings: dict[str, float | None] = {}
        errors: list[str] = []

        def _do(c: Any, a: Any) -> dict[str, float | None]:
            result: dict[str, float | None] = {}
            for direction, sname in names.items():
                try:
                    data = c.getDistanceSensorData(distance_sensor_name=sname, vehicle_name=vehicle)
                    distance = float(getattr(data, "distance", 0.0) or 0.0)
                    if not math.isfinite(distance) or distance <= 0.0:
                        result[direction] = None
                    else:
                        result[direction] = round(distance, 2)
                except Exception as e:
                    result[direction] = None
                    errors.append(f"{sname}: {e}")
            return result

        try:
            readings = self._exec_rpc(_do)
        except Exception as exc:
            return DistanceSensorsStatus(error=str(exc))

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
        return_3d: bool = False,
        return_position: bool = False,
    ) -> list[tuple[float, float]] | list[tuple[float, float, float]] | tuple[list, tuple[float, float, float]]:
        if not self._engine.connected:
            return ([], (0.0, 0.0, 0.0)) if return_position else []
        vehicle = self.vehicle_name
        try:
            def _do(c: Any, a: Any) -> tuple[Any, Any]:
                state = c.getMultirotorState(vehicle_name=vehicle)
                lidar = c.getLidarData(lidar_name=sensor_name, vehicle_name=vehicle)
                return state, lidar
            state, lidar = self._exec_rpc(_do)

            position = state.kinematics_estimated.position
            orientation = state.kinematics_estimated.orientation
            px = float(position.x_val)
            py = float(position.y_val)
            pz = float(position.z_val)
            raw_points = list(getattr(lidar, "point_cloud", []) or [])
            points_2d: list[tuple[float, float]] = []
            points_3d: list[tuple[float, float, float]] = []
            max_range = max(2.0, float(range_m))
            min_range_val = max(0.0, float(min_range_m))
            vertical_up = max(0.5, float(vertical_window_m))
            vertical_down = max(0.2, float(vertical_down_m))
            for index in range(0, len(raw_points) - 2, 3):
                local_x = float(raw_points[index])
                local_y = float(raw_points[index + 1])
                local_z = float(raw_points[index + 2])
                horizontal_distance = math.hypot(local_x, local_y)
                if local_x <= 0.6 or horizontal_distance < min_range_val or horizontal_distance > max_range:
                    continue
                if local_z < -vertical_up or local_z > vertical_down:
                    continue
                rotated_x, rotated_y, rotated_z = self._rotate_vector_by_quaternion(
                    local_x, local_y, local_z, orientation,
                )
                world_x = px + rotated_x
                world_y = py + rotated_y
                world_z = pz + rotated_z
                if return_3d:
                    points_3d.append((world_x, world_y, world_z))
                else:
                    points_2d.append((world_x, world_y))

            if return_3d:
                result: list = points_3d
            else:
                result = self._denoise_world_points(points_2d, cell_m=denoise_cell_m, min_points=min_points_per_cell)

            if return_position:
                return result, (px, py, pz)
            return result
        except Exception as exc:
            self.last_error = str(exc)
            return ([], (0.0, 0.0, 0.0)) if return_position else []

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

    def _require_client(self) -> Any:
        if not self._engine.connected:
            raise RuntimeError(f"AirSim is not connected: {self.last_error}")
        return self._engine.client

    def camera_frame(self) -> CameraFrame | None:
        if not self._engine.connected:
            return None
        errors: list[str] = []
        camera_names = list(self.camera_candidates)
        for _ in range(2):
            for camera_name in camera_names:
                frame = self._request_camera_frame(camera_name)
                if frame is not None:
                    self.last_camera_name = camera_name
                    self.last_image_type = frame.image_type
                    self.active_camera_name = camera_name
                    self.last_success_at = self._engine.last_success_at
                    self.failure_count = 0
                    self.last_error = ""
                    return frame
                if self.last_error:
                    errors.append(f"{camera_name}: {self.last_error}")
        if errors:
            self.last_error = "; ".join(errors[-4:])
        return None

    def capture_camera_frame(self, camera_name: str | None = None) -> CameraFrame | None:
        if not self._engine.connected:
            return None
        camera = str(camera_name or self.active_camera_name or "front_center").strip()
        if camera not in self.camera_candidates:
            raise ValueError(f"Camera '{camera}' is not available")
        frame = self._request_camera_frame(camera)
        if frame is not None:
            self.last_camera_name = camera
            self.last_image_type = frame.image_type
            self.active_camera_name = camera
            self.last_success_at = self._engine.last_success_at
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
        if not self._engine.connected:
            return [{"camera": "-", "image_type": "-", "ok": False, "error": self.last_error}]
        for camera_name in self.camera_candidates:
            frame = self._request_camera_frame(camera_name)
            results.append(
                {
                    "camera": camera_name,
                    "image_type": 0,
                    "ok": frame is not None,
                    "bytes": len(frame.data) if frame is not None else 0,
                    "content_type": frame.content_type if frame is not None else "",
                    "error": "" if frame is not None else self.last_error,
                }
            )
        return results

    def _request_camera_frame(self, camera_name: str, retries: int = 3) -> CameraFrame | None:
        vehicle = self.vehicle_name
        last_exc: Exception | None = None
        for attempt in range(retries):
            try:
                def _do(c: Any, a: Any) -> Any:
                    responses = c.simGetImages(
                        [a.ImageRequest(camera_name, a.ImageType.Scene, False, True)],
                        vehicle_name=vehicle,
                    )
                    return responses
                responses = self._exec_rpc(_do)
                if not responses:
                    continue
                data = responses[0].image_data_uint8
                if not data:
                    self.last_error = "empty image_data_uint8"
                    self._note_failure(soft=True)
                    continue
                raw = bytes(data)
                self.last_success_at = self._engine.last_success_at
                self.failure_count = 0
                return CameraFrame(
                    data=raw,
                    content_type=self._guess_image_content_type(raw),
                    camera_name=camera_name,
                    image_type=0,
                )
            except Exception as exc:
                last_exc = exc
                self.last_error = str(exc)
                self._note_failure(soft=True)
        if last_exc:
            self.last_error = str(last_exc)
        return None

    def _set_vehicle_pose(self, x: float, y: float, z: float, ignore_collision: bool = True) -> None:
        vehicle = self.vehicle_name
        self._exec_rpc(
            lambda c, a: c.simSetVehiclePose(
                a.Pose(a.Vector3r(float(x), float(y), float(z)), a.to_quaternion(0.0, 0.0, 0.0)),
                ignore_collision=bool(ignore_collision),
                vehicle_name=vehicle,
            )
        )

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
        if not soft and self.failure_count >= 12 and (self.last_success_at == 0 or time.time() - self.last_success_at) > 30.0:
            if not self._heartbeat_warned:
                logger.warning("AirSimAdapter: persistent RPC failures (count=%d)", self.failure_count)
                self._heartbeat_warned = True
