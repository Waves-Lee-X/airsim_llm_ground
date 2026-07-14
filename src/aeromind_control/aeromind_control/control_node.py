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


class ControlNode(Node):
    """飞行控制节点"""

    def __init__(self):
        super().__init__("control_node")

        # 声明参数
        self.declare_parameter("px4_mode", "airsim")
        self.declare_parameter("airsim_ip", "192.168.1.100")
        self.declare_parameter("execute_autonomy_trajectory", True)
        self.declare_parameter("autonomy_velocity_limit", 1.2)
        self.declare_parameter("autonomy_accel_limit", 0.8)
        self.declare_parameter("autonomy_min_altitude", 1.0)
        self.declare_parameter("autonomy_log_interval_sec", 2.0)
        self._px4_mode = self.get_parameter("px4_mode").value
        self._airsim_ip = self.get_parameter("airsim_ip").value
        self._execute_autonomy_trajectory = bool(
            self.get_parameter("execute_autonomy_trajectory").value
        )
        self._autonomy_velocity_limit = float(self.get_parameter("autonomy_velocity_limit").value)
        self._autonomy_accel_limit = float(self.get_parameter("autonomy_accel_limit").value)
        self._autonomy_min_altitude = float(self.get_parameter("autonomy_min_altitude").value)
        self._autonomy_log_interval_sec = float(self.get_parameter("autonomy_log_interval_sec").value)
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
        self._latest_odom = None
        self._active_trajectory_until = 0.0
        self._last_autonomy_hold_mode = None
        self._last_autonomy_strategy = None
        self._last_autonomy_log_time = 0.0
        self._last_autonomy_velocity = (0.0, 0.0, 0.0)
        self._last_autonomy_velocity_time = time.monotonic()
        self._control_mode = "IDLE"

        # 根据模式初始化后端
        self._client = None      # AirSim 客户端
        self._px4_ctrl = None    # PX4 控制器

        if self._px4_mode == "px4":
            self._init_px4()
        else:
            self._init_airsim()

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
        from px4_msgs.msg import VehicleStatus

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
        self.get_logger().info("PX4 控制模式已初始化")

    def _px4_status_callback(self, msg):
        """PX4 状态更新：同步 armed 和发布 drone_state"""
        self._armed = (msg.arming_state == 2)

        state = DroneState()
        state.armed = self._armed
        # nav_state → 可读模式名
        nav_state_map = {
            0: "MANUAL", 1: "ALTCTL", 2: "POSCTL",
            3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
            6: "ACRO", 14: "OFFBOARD", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND",
        }
        state.mode = nav_state_map.get(msg.nav_state, f"NAV_{msg.nav_state}")
        state.battery = 0.0  # VehicleStatus 不含电池
        state.gps_fix = 2
        state.ekf_healthy = True
        self._state_pub.publish(state)

    def _odom_callback(self, msg: Odometry):
        self._latest_odom = msg

    # ============================================================
    # 起飞服务
    # ============================================================

    def _takeoff_callback(self, request, response):
        altitude = request.altitude
        self.get_logger().info(f"收到起飞请求: 目标高度={altitude}m")
        self._control_mode = "TAKEOFF"

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
            response.success = False
            response.message = f"PX4 起飞失败: {e}"

        self.get_logger().info(response.message)
        return response

    # ============================================================
    # 降落服务
    # ============================================================

    def _land_callback(self, request, response):
        self.get_logger().info("收到降落请求")
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
        self._control_mode = "RETURN_HOME"
        self._active_trajectory_until = 0.0

        if self._px4_mode == "px4":
            return self._return_home_px4(response)
        return self._return_home_airsim(response)

    def _return_home_px4(self, response):
        if self._px4_ctrl is None:
            response.success = False
            response.message = "PX4 控制器未初始化"
            return response
        try:
            self._px4_ctrl.return_home()
            response.success = True
            response.message = "PX4 RTL 返航指令已发送"
        except Exception as e:
            response.success = False
            response.message = f"PX4 RTL 返航失败: {e}"
        self.get_logger().info(response.message)
        return response

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
        if self._control_mode in ("LAND", "RETURN_HOME"):
            return
        if not self._armed:
            self.get_logger().debug("忽略自主轨迹：未解锁")
            return
        if not msg.collision_free:
            self._hold_autonomy("unsafe", f"非安全自主轨迹，切换悬停: {msg.message}")
            return
        if msg.planner_mode == "hover_no_goal" and self._control_mode in ("TAKEOFF", "LAND"):
            return
        if msg.planner_mode in ("hover_no_goal", "goal_reached", "safe_hover", "blocked_hold"):
            self._hold_autonomy(msg.planner_mode, f"自主轨迹结束/悬停: {msg.planner_mode}")
            return
        if not msg.points:
            return

        self._last_autonomy_hold_mode = None
        now = time.monotonic()
        if now < self._active_trajectory_until:
            return

        duration = self._trajectory_duration(msg)
        self._active_trajectory_until = now + (0.45 if self._px4_mode == "px4" else max(0.05, duration * 0.8))
        self._control_mode = "AUTONOMY"
        self._log_autonomy_strategy(msg, duration, now)
        if self._px4_mode == "px4":
            self._follow_trajectory_px4(msg)
        else:
            self._follow_trajectory_airsim(msg)

    def _hold_autonomy(self, mode: str, message: str):
        if self._last_autonomy_hold_mode != mode:
            self.get_logger().info(message)
            self._last_autonomy_hold_mode = mode
        self._active_trajectory_until = 0.0
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

    def _follow_trajectory_px4(self, msg: Trajectory):
        if self._px4_ctrl is None:
            self.get_logger().warn("无法执行自主轨迹：PX4 控制器未初始化")
            return
        point = self._trajectory_lookahead_point(msg, lookahead_sec=0.8)
        vx, vy, vz = self._clamped_velocity(
            point.velocity.x,
            point.velocity.y,
            point.velocity.z,
        )
        vx, vy, vz = self._apply_altitude_guard(vx, vy, vz, point.position.z)
        vx, vy, vz = self._smooth_velocity(vx, vy, vz)
        if abs(vx) + abs(vy) + abs(vz) > 0.05:
            self._px4_ctrl.set_velocity(
                vx,
                vy,
                vz,
                point.yaw if math.isfinite(float(point.yaw)) else float("nan"),
            )
        else:
            self._px4_ctrl.set_position(
                point.position.x,
                point.position.y,
                point.position.z,
                point.yaw if math.isfinite(float(point.yaw)) else float("nan"),
            )

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

    def _trajectory_lookahead_point(self, msg: Trajectory, lookahead_sec: float):
        selected = msg.points[-1]
        for point in msg.points:
            if self._duration_to_sec(point.time_from_start) >= lookahead_sec:
                selected = point
                break
        return selected

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
