#!/usr/bin/env python3
"""
control_node.py - 飞行控制节点（AirSim / PX4 双模式）

功能：
  - 提供 /control/takeoff（起飞）
  - 提供 /control/land（降落）
  - 提供 /control/arm（解锁/加锁）
  - 订阅 /planning/path，按路径逐航点飞行
  - 发布 /control/drone_state

模式（通过 px4_mode 参数切换）：
  - "airsim"：使用 AirSim Python API 控制
  - "px4"：通过 uXRCE-DDS 向 PX4 发送 VehicleCommand/OffboardControl
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

from aeromind_interfaces.msg import DroneState, Path, Trajectory
from aeromind_interfaces.srv import ArmDrone, Land, ReturnHome, Takeoff

# AirSim API
try:
    import airsim  # type: ignore
    HAS_AIRSIM = True
except ImportError:
    HAS_AIRSIM = False

# PX4 控制器
from aeromind_control.px4_control import PX4Controller, HAS_PX4_MSGS


PROTECTED_FLIGHT_MODES = {"TAKEOFF", "LAND", "RETURN_HOME"}


def interpolate_trajectory(points, elapsed_sec: float):
    """Linearly interpolate a sampled trajectory state at elapsed time."""
    if not points:
        return None
    elapsed = max(0.0, float(elapsed_sec))
    previous = points[0]
    previous_t = _duration_seconds(previous.time_from_start)
    if elapsed <= previous_t:
        return _trajectory_state(previous)
    for current in points[1:]:
        current_t = _duration_seconds(current.time_from_start)
        if elapsed <= current_t:
            span = max(1e-6, current_t - previous_t)
            ratio = max(0.0, min(1.0, (elapsed - previous_t) / span))
            return {
                "position": _lerp_point(previous.position, current.position, ratio),
                "velocity": _lerp_vector(previous.velocity, current.velocity, ratio),
                "acceleration": _lerp_vector(
                    previous.acceleration, current.acceleration, ratio
                ),
                "yaw": _lerp_yaw(float(previous.yaw), float(current.yaw), ratio),
            }
        previous = current
        previous_t = current_t
    return _trajectory_state(points[-1])


def _duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) / 1_000_000_000.0


def _lerp_point(start, end, ratio):
    return tuple(
        float(getattr(start, axis))
        + (float(getattr(end, axis)) - float(getattr(start, axis))) * ratio
        for axis in ("x", "y", "z")
    )


def _lerp_vector(start, end, ratio):
    return _lerp_point(start, end, ratio)


def _lerp_yaw(start, end, ratio):
    if not math.isfinite(start):
        return end
    if not math.isfinite(end):
        return start
    delta = (end - start + math.pi) % (2.0 * math.pi) - math.pi
    return start + delta * ratio


def _trajectory_state(point):
    return {
        "position": tuple(float(getattr(point.position, axis)) for axis in ("x", "y", "z")),
        "velocity": tuple(float(getattr(point.velocity, axis)) for axis in ("x", "y", "z")),
        "acceleration": tuple(
            float(getattr(point.acceleration, axis)) for axis in ("x", "y", "z")
        ),
        "yaw": float(point.yaw),
    }


def px4_gps_fix_to_drone_fix(fix_type: int) -> int:
    """Map PX4 SensorGps fix types to DroneState's compact fix levels."""
    if fix_type >= 5:
        return 4  # RTK float/fixed
    if fix_type == 4:
        return 3  # Differential GNSS
    if fix_type in (3, 8):
        return 2  # 3D or extrapolated 3D
    if fix_type == 2:
        return 1  # 2D
    return 0


def estimator_status_flags_healthy(msg) -> bool:
    """Evaluate EKF alignment and numerical faults from EstimatorStatusFlags."""
    fault_fields = (
        "fs_bad_mag_x",
        "fs_bad_mag_y",
        "fs_bad_mag_z",
        "fs_bad_hdg",
        "fs_bad_mag_decl",
        "fs_bad_airspeed",
        "fs_bad_sideslip",
        "fs_bad_optflow_x",
        "fs_bad_optflow_y",
        "fs_bad_acc_vertical",
        "fs_bad_acc_clipping",
    )
    aligned = bool(getattr(msg, "cs_tilt_align", False)) and bool(
        getattr(msg, "cs_yaw_align", False)
    )
    faulted = any(bool(getattr(msg, name, False)) for name in fault_fields)
    dead_reckoning = bool(getattr(msg, "cs_inertial_dead_reckoning", False))
    return aligned and not faulted and not dead_reckoning


def autonomy_trajectory_action(
    control_mode: str,
    planner_mode: str,
    collision_free: bool,
    has_points: bool,
) -> str:
    """Return how an autonomy trajectory may affect the active flight mode."""
    if control_mode in PROTECTED_FLIGHT_MODES:
        return "ignore_flight_mode"
    if planner_mode == "hover_no_goal":
        return "ignore_no_goal"
    if not collision_free:
        return "ignore_inactive" if control_mode in ("IDLE", "READY") else "hold_unsafe"
    if planner_mode in (
        "goal_reached",
        "safe_hover",
        "blocked_hold",
        "semantic_person_hold",
    ):
        return "hold"
    if not has_points:
        return "ignore_empty"
    return "execute"


def should_accept_replan(has_active: bool, active_age_sec: float, interval_sec: float):
    return not has_active or active_age_sec >= interval_sec


def trajectory_update_mode(active_id: str, incoming_id: str, reset_time: bool) -> str:
    """Classify a trajectory message as a heartbeat or a new time axis."""
    if incoming_id and incoming_id == active_id and not reset_time:
        return "heartbeat"
    return "replace"


def should_accept_trajectory_update(
    reset_time: bool,
    has_active: bool,
    active_age_sec: float,
    interval_sec: float,
) -> bool:
    return bool(reset_time) or should_accept_replan(
        has_active, active_age_sec, interval_sec
    )


def trajectory_handoff_elapsed(points, reference_state, max_elapsed_sec: float):
    """Find a nearby point on a replacement trajectory for a continuous handoff."""
    if not points or reference_state is None:
        return 0.0
    reference_position = reference_state["position"]
    reference_velocity = reference_state["velocity"]
    best_elapsed = 0.0
    best_cost = float("inf")
    maximum = max(0.0, float(max_elapsed_sec))
    for point in points:
        elapsed = _duration_seconds(point.time_from_start)
        if elapsed > maximum:
            break
        state = _trajectory_state(point)
        position_error = sum(
            (state["position"][axis] - reference_position[axis]) ** 2
            for axis in range(3)
        )
        velocity_error = sum(
            (state["velocity"][axis] - reference_velocity[axis]) ** 2
            for axis in range(3)
        )
        cost = position_error + 0.2 * velocity_error
        if cost < best_cost:
            best_cost = cost
            best_elapsed = elapsed
    return best_elapsed


class ControlNode(Node):
    """飞行控制节点"""

    def __init__(self):
        super().__init__("control_node")

        # 声明参数
        self.declare_parameter("px4_mode", "airsim")
        self.declare_parameter("airsim_ip", "192.168.1.100")
        self.declare_parameter("execute_autonomy_trajectory", True)
        self.declare_parameter("autonomy_velocity_limit", 1.8)
        self.declare_parameter("autonomy_accel_limit", 1.0)
        self.declare_parameter("autonomy_min_altitude", 1.0)
        self.declare_parameter("autonomy_log_interval_sec", 2.0)
        self.declare_parameter("autonomy_tracking_rate_hz", 50.0)
        self.declare_parameter("autonomy_replan_accept_interval_sec", 0.6)
        self.declare_parameter("autonomy_trajectory_stale_sec", 1.0)
        self.declare_parameter("autonomy_handoff_max_sec", 0.6)
        self.declare_parameter("px4_telemetry_timeout_sec", 2.0)
        self.declare_parameter("rtl_mode_confirm_timeout_sec", 3.0)
        self._px4_mode = self.get_parameter("px4_mode").value
        self._airsim_ip = self.get_parameter("airsim_ip").value
        self._execute_autonomy_trajectory = bool(
            self.get_parameter("execute_autonomy_trajectory").value
        )
        self._autonomy_velocity_limit = float(self.get_parameter("autonomy_velocity_limit").value)
        self._autonomy_accel_limit = float(self.get_parameter("autonomy_accel_limit").value)
        self._autonomy_min_altitude = float(self.get_parameter("autonomy_min_altitude").value)
        self._autonomy_log_interval_sec = float(self.get_parameter("autonomy_log_interval_sec").value)
        self._autonomy_tracking_rate_hz = max(
            10.0, float(self.get_parameter("autonomy_tracking_rate_hz").value)
        )
        self._autonomy_replan_accept_interval_sec = max(
            0.1,
            float(
                self.get_parameter("autonomy_replan_accept_interval_sec").value
            ),
        )
        self._autonomy_trajectory_stale_sec = max(
            self._autonomy_replan_accept_interval_sec + 0.2,
            float(self.get_parameter("autonomy_trajectory_stale_sec").value),
        )
        self._autonomy_handoff_max_sec = max(
            0.0,
            float(self.get_parameter("autonomy_handoff_max_sec").value),
        )
        self._px4_telemetry_timeout_sec = max(
            0.5, float(self.get_parameter("px4_telemetry_timeout_sec").value)
        )
        self._rtl_mode_confirm_timeout_sec = max(
            1.0, float(self.get_parameter("rtl_mode_confirm_timeout_sec").value)
        )
        self.get_logger().info(f"控制模式: {self._px4_mode}")

        # 创建服务
        self._takeoff_srv = self.create_service(
            Takeoff, "/control/takeoff", self._takeoff_callback
        )
        self._land_srv = self.create_service(
            Land, "/control/land", self._land_callback
        )
        self._return_home_srv = self.create_service(
            ReturnHome, "/control/return_home", self._return_home_callback
        )
        self._arm_srv = self.create_service(
            ArmDrone, "/control/arm", self._arm_callback
        )

        # 订阅规划路径
        self._path_sub = self.create_subscription(
            Path, "/planning/path", self._path_callback, 10
        )
        self._trajectory_sub = self.create_subscription(
            Trajectory, "/autonomy/trajectory", self._trajectory_callback, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/sensor/odometry", self._odom_callback, 10
        )

        # 发布无人机状态
        self._state_pub = self.create_publisher(DroneState, "/control/drone_state", 10)

        # 状态变量
        self._armed = False
        self._latest_vehicle_status = None
        self._latest_vehicle_status_received_at = 0.0
        self._battery_voltage = 0.0
        self._gps_fix = 0
        self._ekf_healthy = None
        self._landed = False
        self._landed_valid = False
        self._latest_odom = None
        self._latest_odom_received_at = 0.0
        self._active_trajectory_until = 0.0
        self._active_trajectory = None
        self._active_trajectory_started_at = 0.0
        self._active_trajectory_updated_at = 0.0
        self._active_trajectory_id = ""
        self._last_autonomy_setpoint = None
        self._last_autonomy_hold_mode = None
        self._last_autonomy_strategy = None
        self._last_autonomy_log_time = 0.0
        self._last_autonomy_velocity = (0.0, 0.0, 0.0)
        self._last_autonomy_velocity_time = time.monotonic()
        self._control_mode = "IDLE"
        self._takeoff_target_altitude = None
        self._rtl_requested_at = 0.0
        self._rtl_mode_confirmed = False

        # 根据模式初始化后端
        self._client = None      # AirSim 客户端
        self._px4_ctrl = None    # PX4 控制器

        if self._px4_mode == "px4":
            self._init_px4()
        else:
            self._init_airsim()

        self._trajectory_tracking_timer = self.create_timer(
            1.0 / self._autonomy_tracking_rate_hz,
            self._trajectory_tracking_callback,
        )
        self._rtl_monitor_timer = self.create_timer(0.2, self._rtl_monitor_callback)

        self.get_logger().info("控制节点已启动")

    # ============================================================
    # AirSim 初始化
    # ============================================================

    def _init_airsim(self):
        if not HAS_AIRSIM:
            self.get_logger().error("airsim 包未安装！请执行 pip install airsim")
            return
        try:
            self._client = airsim.MultirotorClient(ip=self._airsim_ip)
            self._client.confirmConnection()
            self.get_logger().info("已连接到 AirSim")
        except Exception as e:
            self.get_logger().error(f"无法连接 AirSim: {e}")
            self._client = None

    # ============================================================
    # PX4 初始化
    # ============================================================

    def _init_px4(self):
        if not HAS_PX4_MSGS:
            self.get_logger().error(
                "px4_msgs 未安装！请在 src/ 下克隆 px4_msgs 包后重新编译。"
            )
            return

        self._px4_ctrl = PX4Controller(self)

        # 订阅 PX4 状态用于内部跟踪和发布
        from px4_msgs.msg import (
            BatteryStatus,
            EstimatorStatusFlags,
            SensorGps,
            VehicleLandDetected,
            VehicleStatus,
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._px4_status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._px4_status_callback,
            px4_qos,
        )
        # 也订阅 vehicle_status_v4（PX4 可能发这两个之一）
        self._px4_status_v4_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self._px4_status_callback,
            px4_qos,
        )
        self._px4_battery_sub = self.create_subscription(
            BatteryStatus,
            "/fmu/out/battery_status_v1",
            self._px4_battery_callback,
            px4_qos,
        )
        self._px4_gps_sub = self.create_subscription(
            SensorGps,
            "/fmu/out/vehicle_gps_position",
            self._px4_gps_callback,
            px4_qos,
        )
        self._px4_estimator_sub = self.create_subscription(
            EstimatorStatusFlags,
            "/fmu/out/estimator_status_flags",
            self._px4_estimator_callback,
            px4_qos,
        )
        self._px4_land_detected_sub = self.create_subscription(
            VehicleLandDetected,
            "/fmu/out/vehicle_land_detected",
            self._px4_land_detected_callback,
            px4_qos,
        )
        self.get_logger().info(
            "PX4 控制模式已初始化，订阅电池、GPS、EKF 与接地遥测"
        )

    def _px4_battery_callback(self, msg):
        voltage = float(msg.voltage_v)
        self._battery_voltage = (
            voltage
            if bool(msg.connected) and math.isfinite(voltage) and voltage > 0.0
            else 0.0
        )

    def _px4_gps_callback(self, msg):
        self._gps_fix = px4_gps_fix_to_drone_fix(int(msg.fix_type))

    def _px4_estimator_callback(self, msg):
        self._ekf_healthy = estimator_status_flags_healthy(msg)

    def _px4_land_detected_callback(self, msg):
        previous = self._landed if self._landed_valid else None
        self._landed = bool(msg.landed)
        self._landed_valid = True
        if previous is not None and previous != self._landed:
            self.get_logger().info(
                "PX4 接地状态更新: "
                + ("已接地" if self._landed else "已离地")
            )

    def _px4_status_callback(self, msg):
        """PX4 状态更新：同步 armed 和发布 drone_state"""
        self._latest_vehicle_status = msg
        self._latest_vehicle_status_received_at = time.monotonic()
        self._armed = (msg.arming_state == 2)

        if (
            self._control_mode == "RETURN_HOME"
            and int(msg.nav_state) == 5
            and not self._rtl_mode_confirmed
        ):
            self._rtl_mode_confirmed = True
            if self._px4_ctrl is not None:
                self._px4_ctrl.stop_offboard_stream()
            self.get_logger().info(
                "PX4 已确认进入 AUTO_RTL，Offboard 控制已安全移交"
            )
        elif self._control_mode == "RETURN_HOME" and not self._armed:
            self._control_mode = "IDLE"
            self._rtl_requested_at = 0.0

        state = DroneState()
        state.armed = self._armed
        # nav_state → 可读模式名
        nav_state_map = {
            0: "MANUAL", 1: "ALTCTL", 2: "POSCTL",
            3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
            6: "POSITION_SLOW", 10: "ACRO", 14: "OFFBOARD",
            17: "AUTO_TAKEOFF", 18: "AUTO_LAND",
        }
        state.mode = nav_state_map.get(msg.nav_state, f"NAV_{msg.nav_state}")
        state.battery = float(self._battery_voltage)
        state.gps_fix = int(self._gps_fix)
        state.ekf_healthy = (
            bool(self._ekf_healthy)
            if self._ekf_healthy is not None
            else bool(getattr(msg, "pre_flight_checks_pass", False))
        )
        state.landed = bool(self._landed)
        state.landed_valid = bool(self._landed_valid)
        self._state_pub.publish(state)

    def _odom_callback(self, msg: Odometry):
        self._latest_odom = msg
        self._latest_odom_received_at = time.monotonic()
        if self._control_mode != "TAKEOFF" or self._takeoff_target_altitude is None:
            return
        altitude = float(msg.pose.pose.position.z)
        tolerance = max(0.3, min(1.0, self._takeoff_target_altitude * 0.1))
        if altitude >= self._takeoff_target_altitude - tolerance:
            self._control_mode = "READY"
            self.get_logger().info(
                f"起飞高度已确认: {altitude:.2f}m，允许接收后续自主轨迹"
            )

    # ============================================================
    # 起飞服务
    # ============================================================

    def _takeoff_callback(self, request, response):
        altitude = request.altitude
        self.get_logger().info(f"收到起飞请求: 目标高度={altitude}m")
        self._clear_active_trajectory()
        self._control_mode = "TAKEOFF"
        self._takeoff_target_altitude = float(altitude)

        if self._px4_mode == "px4":
            return self._takeoff_px4(altitude, response)
        else:
            return self._takeoff_airsim(altitude, response)

    def _takeoff_airsim(self, altitude: float, response):
        if self._client is None:
            response.success = False
            response.message = "AirSim 未连接"
            return response
        try:
            self._client.enableApiControl(True)
            self._client.armDisarm(True)
            self._armed = True
            self._client.takeoffAsync(timeout_sec=10).join()
            state = self._client.getMultirotorState()
            current_pos = state.kinematics_estimated.position
            self._client.moveToPositionAsync(
                current_pos.x_val, current_pos.y_val, -altitude, 5.0
            ).join()
            response.success = True
            response.message = f"起飞成功，高度={altitude}m"
        except Exception as e:
            response.success = False
            response.message = f"起飞失败: {e}"
        self.get_logger().info(response.message)
        return response

    def _takeoff_px4(self, altitude: float, response):
        if self._px4_ctrl is None:
            response.success = False
            response.message = "PX4 控制器未初始化"
            return response

        try:
            # 使用 Offboard 位置控制起飞：PX4 收到连续 setpoint 后解锁并切 OFFBOARD。
            self._px4_ctrl.offboard_takeoff(altitude)
            self._armed = True
            response.success = True
            response.message = f"PX4 Offboard 起飞已启动，目标高度={altitude}m"
        except Exception as e:
            self._control_mode = "IDLE"
            self._takeoff_target_altitude = None
            response.success = False
            response.message = f"PX4 起飞失败: {e}"

        self.get_logger().info(response.message)
        return response

    # ============================================================
    # 降落服务
    # ============================================================

    def _land_callback(self, request, response):
        self.get_logger().info("收到降落请求")
        self._clear_active_trajectory()
        self._control_mode = "LAND"

        if self._px4_mode == "px4":
            return self._land_px4(response)
        else:
            return self._land_airsim(response)

    def _land_airsim(self, response):
        if self._client is None:
            response.success = False
            response.message = "AirSim 未连接"
            return response
        try:
            self._client.landAsync(timeout_sec=20).join()
            self._armed = False
            response.success = True
            response.message = "降落成功"
        except Exception as e:
            response.success = False
            response.message = f"降落失败: {e}"
        self.get_logger().info(response.message)
        return response

    def _land_px4(self, response):
        if self._px4_ctrl is None:
            response.success = False
            response.message = "PX4 控制器未初始化"
            return response
        try:
            self._px4_ctrl.land()
            self._armed = False
            response.success = True
            response.message = "PX4 降落指令已发送"
        except Exception as e:
            response.success = False
            response.message = f"PX4 降落失败: {e}"
        self.get_logger().info(response.message)
        return response

    # ============================================================
    # 返航服务
    # ============================================================

    def _return_home_callback(self, request, response):
        self.get_logger().info("收到返航请求")

        if self._px4_mode == "px4":
            now = time.monotonic()
            status_age = now - self._latest_vehicle_status_received_at
            odom_age = now - self._latest_odom_received_at
            if (
                self._latest_vehicle_status is None
                or status_age > self._px4_telemetry_timeout_sec
            ):
                response.success = False
                response.message = "拒绝 RTL：飞控状态遥测不可用或已超时"
                self.get_logger().error(response.message)
                return response
            if self._latest_odom is None or odom_age > self._px4_telemetry_timeout_sec:
                response.success = False
                response.message = "拒绝 RTL：里程计不可用或已超时"
                self.get_logger().error(response.message)
                return response
            if not self._armed:
                response.success = False
                response.message = "拒绝 RTL：无人机当前未解锁"
                self.get_logger().warn(response.message)
                return response

            self._clear_active_trajectory()
            position = self._latest_odom.pose.pose.position
            if self._px4_ctrl is not None:
                self._px4_ctrl.set_position(position.x, position.y, position.z)
            self._control_mode = "RETURN_HOME"
            self._rtl_requested_at = now
            self._rtl_mode_confirmed = False
            return self._return_home_px4(response)
        self._clear_active_trajectory()
        self._control_mode = "RETURN_HOME"
        return self._return_home_airsim(response)

    def _return_home_px4(self, response):
        if self._px4_ctrl is None:
            response.success = False
            response.message = "PX4 控制器未初始化"
            return response
        try:
            self._px4_ctrl.return_home()
            response.success = True
            response.message = "PX4 RTL 请求已发送，等待 AUTO_RTL 模式确认"
        except Exception as e:
            response.success = False
            response.message = f"PX4 RTL 返航失败: {e}"
        self.get_logger().info(response.message)
        return response

    def _rtl_monitor_callback(self):
        if self._control_mode != "RETURN_HOME" or self._rtl_requested_at <= 0.0:
            return

        now = time.monotonic()
        status_age = now - self._latest_vehicle_status_received_at
        odom_age = now - self._latest_odom_received_at
        if self._rtl_mode_confirmed:
            if (
                status_age > self._px4_telemetry_timeout_sec
                or odom_age > self._px4_telemetry_timeout_sec
            ):
                self.get_logger().error(
                    "RTL 过程中飞控/里程计遥测中断；无法确认真实位置，"
                    "请立即检查 AirSim-PX4 MAVLink 链路"
                )
                self._rtl_requested_at = 0.0
            return

        if now - self._rtl_requested_at <= self._rtl_mode_confirm_timeout_sec:
            return

        self._control_mode = "READY"
        self._rtl_requested_at = 0.0
        self.get_logger().error(
            "PX4 未在超时时间内确认 AUTO_RTL；保留 Offboard 悬停，返航未接管"
        )

    def _return_home_airsim(self, response):
        if self._client is None:
            response.success = False
            response.message = "AirSim 未连接"
            return response
        try:
            # AirSim SimpleFlight 没有统一 RTL；这里退回到当前位置坐标系原点附近。
            state = self._client.getMultirotorState()
            current_z = state.kinematics_estimated.position.z_val
            self._client.moveToPositionAsync(0.0, 0.0, current_z, 5.0, timeout_sec=30).join()
            response.success = True
            response.message = "AirSim 已移动到局部原点附近（RTL fallback）"
        except Exception as e:
            response.success = False
            response.message = f"AirSim 返航 fallback 失败: {e}"
        self.get_logger().info(response.message)
        return response

    # ============================================================
    # 解锁/加锁服务
    # ============================================================

    def _arm_callback(self, request, response):
        arm = request.arm
        self.get_logger().info(f"收到{'解锁' if arm else '加锁'}请求")
        if not arm:
            self._clear_active_trajectory()

        if self._px4_mode == "px4":
            return self._arm_px4(arm, response)
        else:
            return self._arm_airsim(arm, response)

    def _arm_airsim(self, arm: bool, response):
        if self._client is None:
            response.success = False
            response.message = "AirSim 未连接"
            return response
        try:
            if arm:
                self._client.enableApiControl(True)
                self._client.armDisarm(True)
            else:
                self._client.armDisarm(False)
                self._client.enableApiControl(False)
            self._armed = arm
            response.success = True
            response.message = f"{'解锁' if arm else '加锁'}成功"
        except Exception as e:
            response.success = False
            response.message = f"{'解锁' if arm else '加锁'}失败: {e}"
        self.get_logger().info(response.message)
        return response

    def _arm_px4(self, arm: bool, response):
        if self._px4_ctrl is None:
            response.success = False
            response.message = "PX4 控制器未初始化"
            return response
        try:
            if arm:
                self._px4_ctrl.arm()
            else:
                self._px4_ctrl.disarm()
            self._armed = arm
            response.success = True
            response.message = f"PX4 {'解锁' if arm else '加锁'}指令已发送"
        except Exception as e:
            response.success = False
            response.message = f"PX4 {'解锁' if arm else '加锁'}失败: {e}"
        self.get_logger().info(response.message)
        return response

    # ============================================================
    # 路径跟随
    # ============================================================

    def _path_callback(self, msg: Path):
        """路径回调：逐航点执行飞行"""
        if not self._armed:
            self.get_logger().warn("无法执行路径：未解锁")
            return

        if self._px4_mode == "px4":
            self._follow_path_px4(msg)
        else:
            self._follow_path_airsim(msg)

    def _trajectory_callback(self, msg: Trajectory):
        """执行自主避障平滑轨迹。

        /autonomy/trajectory 已由 autonomy_node 转为 odom/map 世界坐标。
        控制层只负责按时间顺序持续下发 setpoint。
        """
        if not self._execute_autonomy_trajectory:
            return
        if not self._armed:
            self.get_logger().debug("忽略自主轨迹：未解锁")
            return

        action = autonomy_trajectory_action(
            self._control_mode,
            msg.planner_mode,
            bool(msg.collision_free),
            bool(msg.points),
        )
        if action.startswith("ignore"):
            return
        if action == "hold_unsafe":
            self._hold_autonomy("unsafe", f"非安全自主轨迹，切换悬停: {msg.message}")
            return
        if action == "hold":
            self._hold_autonomy(msg.planner_mode, f"自主轨迹结束/悬停: {msg.planner_mode}")
            return

        self._last_autonomy_hold_mode = None
        now = time.monotonic()
        duration = self._trajectory_duration(msg)
        incoming_id = str(getattr(msg, "trajectory_id", "") or "")
        reset_time = bool(getattr(msg, "reset_time", False))
        if trajectory_update_mode(
            self._active_trajectory_id, incoming_id, reset_time
        ) == "heartbeat":
            self._active_trajectory_updated_at = now
            return
        if self._px4_mode == "px4":
            if not should_accept_trajectory_update(
                reset_time,
                self._active_trajectory is not None,
                now - self._active_trajectory_updated_at,
                self._autonomy_replan_accept_interval_sec,
            ):
                return
            handoff_elapsed = trajectory_handoff_elapsed(
                msg.points,
                self._last_autonomy_setpoint,
                self._autonomy_handoff_max_sec,
            )
            self._active_trajectory = msg
            self._active_trajectory_id = incoming_id
            self._active_trajectory_started_at = now - handoff_elapsed
            self._active_trajectory_updated_at = now
            self._control_mode = "AUTONOMY"
            self._log_autonomy_strategy(msg, duration, now)
            return

        if now < self._active_trajectory_until:
            return

        self._active_trajectory_until = now + max(0.05, duration * 0.8)
        self._control_mode = "AUTONOMY"
        self._log_autonomy_strategy(msg, duration, now)
        self._follow_trajectory_airsim(msg)

    def _hold_autonomy(self, mode: str, message: str):
        if self._last_autonomy_hold_mode != mode:
            self.get_logger().info(message)
            self._last_autonomy_hold_mode = mode
        self._clear_active_trajectory()
        self._control_mode = "HOLD"
        self._last_autonomy_velocity = (0.0, 0.0, 0.0)
        self._last_autonomy_velocity_time = time.monotonic()
        if self._px4_mode == "px4":
            if self._px4_ctrl is not None:
                self._px4_ctrl.set_velocity(0.0, 0.0, 0.0)
        elif self._client is not None:
            try:
                self._client.hoverAsync().join()
            except Exception as e:
                self.get_logger().warn(f"AirSim 悬停失败: {e}")

    def _clear_active_trajectory(self):
        self._active_trajectory_until = 0.0
        self._active_trajectory = None
        self._active_trajectory_started_at = 0.0
        self._active_trajectory_updated_at = 0.0
        self._active_trajectory_id = ""
        self._last_autonomy_setpoint = None

    def _trajectory_tracking_callback(self):
        if self._px4_mode != "px4" or self._px4_ctrl is None:
            return
        if not self._armed or self._control_mode != "AUTONOMY":
            return
        trajectory = self._active_trajectory
        if trajectory is None:
            return

        now = time.monotonic()
        update_age = now - self._active_trajectory_updated_at
        if update_age > self._autonomy_trajectory_stale_sec:
            self._hold_autonomy(
                "trajectory_stale",
                f"自主轨迹超过 {self._autonomy_trajectory_stale_sec:.2f}s 未更新，切换悬停",
            )
            return

        trajectory_elapsed = now - self._active_trajectory_started_at
        state = interpolate_trajectory(trajectory.points, trajectory_elapsed)
        if state is None:
            self._hold_autonomy("trajectory_empty", "自主轨迹为空，切换悬停")
            return

        vx, vy, vz = self._clamped_velocity(*state["velocity"])
        guarded_vz = self._apply_altitude_guard(vx, vy, vz, state["position"][2])[2]
        vx, vy, vz = self._smooth_velocity(vx, vy, guarded_vz)
        ax, ay, az = self._clamped_acceleration(*state["acceleration"])
        if guarded_vz != state["velocity"][2] and az < 0.0:
            az = 0.0
        position = (
            state["position"][0],
            state["position"][1],
            max(self._autonomy_min_altitude, state["position"][2]),
        )
        self._px4_ctrl.set_trajectory_state(
            position,
            (vx, vy, vz),
            (ax, ay, az),
            state["yaw"] if math.isfinite(state["yaw"]) else float("nan"),
        )
        self._last_autonomy_setpoint = {
            "position": position,
            "velocity": (vx, vy, vz),
        }

    def _follow_trajectory_airsim(self, msg: Trajectory):
        if self._client is None:
            self.get_logger().warn("无法执行自主轨迹：AirSim 未连接")
            return
        for i, point in enumerate(msg.points):
            pos = point.position
            timeout = max(0.2, self._segment_dt(msg, i) + 0.5)
            speed = max(0.5, math.sqrt(
                point.velocity.x * point.velocity.x
                + point.velocity.y * point.velocity.y
                + point.velocity.z * point.velocity.z
            ))
            try:
                self._client.moveToPositionAsync(
                    pos.x, pos.y, -pos.z, velocity=speed, timeout_sec=timeout
                ).join()
            except Exception as e:
                self.get_logger().error(f"自主轨迹点 {i + 1} 执行失败: {e}")
                break

    def _apply_altitude_guard(self, vx: float, vy: float, vz: float, target_z: float):
        current_z = self._current_altitude()
        min_alt = max(0.0, self._autonomy_min_altitude)
        if current_z is not None and current_z <= min_alt and vz < 0.0:
            self.get_logger().warn(
                f"高度保护触发: 当前高度 {current_z:.2f}m <= {min_alt:.2f}m，禁止继续下降",
                throttle_duration_sec=2.0,
            )
            vz = 0.0
        if math.isfinite(float(target_z)) and target_z < min_alt and vz < 0.0:
            vz = 0.0
        return vx, vy, vz

    def _current_altitude(self):
        if self._latest_odom is None:
            return None
        return float(self._latest_odom.pose.pose.position.z)

    def _smooth_velocity(self, vx: float, vy: float, vz: float):
        limit = max(0.05, self._autonomy_accel_limit)
        now = time.monotonic()
        dt = max(0.02, min(0.5, now - self._last_autonomy_velocity_time))
        max_delta = limit * dt
        prev = self._last_autonomy_velocity

        def clamp_delta(value, old):
            delta = value - old
            if delta > max_delta:
                return old + max_delta
            if delta < -max_delta:
                return old - max_delta
            return value

        smoothed = (
            clamp_delta(vx, prev[0]),
            clamp_delta(vy, prev[1]),
            clamp_delta(vz, prev[2]),
        )
        self._last_autonomy_velocity = smoothed
        self._last_autonomy_velocity_time = now
        return smoothed

    def _log_autonomy_strategy(self, msg: Trajectory, duration: float, now: float):
        should_log = (
            msg.planner_mode != self._last_autonomy_strategy
            or now - self._last_autonomy_log_time >= self._autonomy_log_interval_sec
        )
        if not should_log:
            return
        self.get_logger().info(
            f"执行自主避障轨迹: {len(msg.points)} 点, "
            f"mode={msg.planner_mode}, duration={duration:.2f}s"
        )
        self._last_autonomy_strategy = msg.planner_mode
        self._last_autonomy_log_time = now

    def _clamped_velocity(self, vx: float, vy: float, vz: float):
        limit = max(0.1, self._autonomy_velocity_limit)
        norm = math.sqrt(vx * vx + vy * vy + vz * vz)
        if norm <= limit or norm <= 1e-6:
            return vx, vy, vz
        scale = limit / norm
        return vx * scale, vy * scale, vz * scale

    def _clamped_acceleration(self, ax: float, ay: float, az: float):
        limit = max(0.1, self._autonomy_accel_limit)
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm <= limit or norm <= 1e-6:
            return ax, ay, az
        scale = limit / norm
        return ax * scale, ay * scale, az * scale

    def _trajectory_duration(self, msg: Trajectory):
        if not msg.points:
            return 0.0
        return self._duration_to_sec(msg.points[-1].time_from_start)

    def _segment_dt(self, msg: Trajectory, index: int):
        if index <= 0:
            return self._duration_to_sec(msg.points[index].time_from_start)
        return (
            self._duration_to_sec(msg.points[index].time_from_start)
            - self._duration_to_sec(msg.points[index - 1].time_from_start)
        )

    @staticmethod
    def _duration_to_sec(duration):
        return float(duration.sec) + float(duration.nanosec) / 1_000_000_000.0

    def _follow_path_airsim(self, msg: Path):
        if self._client is None:
            self.get_logger().warn("无法执行路径：AirSim 未连接")
            return

        self.get_logger().info(f"收到路径: {len(msg.waypoints)} 个航点")
        for i, wp in enumerate(msg.waypoints):
            pos = wp.pose.position
            self.get_logger().info(
                f"  航点 {i + 1}/{len(msg.waypoints)}: "
                f"({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
            )
            try:
                self._client.moveToPositionAsync(
                    pos.x, pos.y, -pos.z, velocity=5.0, timeout_sec=30
                ).join()
            except Exception as e:
                self.get_logger().error(f"航点 {i + 1} 执行失败: {e}")
                break

    def _follow_path_px4(self, msg: Path):
        if self._px4_ctrl is None:
            self.get_logger().warn("无法执行路径：PX4 控制器未初始化")
            return

        self.get_logger().info(f"收到路径: {len(msg.waypoints)} 个航点")
        for i, wp in enumerate(msg.waypoints):
            pos = wp.pose.position
            self.get_logger().info(
                f"  航点 {i + 1}/{len(msg.waypoints)}: "
                f"({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
            )
            # 设置目标位置（ENU → PX4 内部转为 NED）
            self._px4_ctrl.set_position(pos.x, pos.y, pos.z)
            # 等待到达（简化：固定 2s）
            time.sleep(2.0)

        self.get_logger().info("PX4: 路径执行完成")


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
