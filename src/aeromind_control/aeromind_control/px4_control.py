#!/usr/bin/env python3
"""
px4_control.py — PX4 飞控命令构造与 Offboard 控制逻辑

负责：
  - 构造 VehicleCommand（解锁/加锁/起飞/降落/模式切换）
  - 维持 Offboard 模式心跳（>2Hz 发布 OffboardControlMode）
  - 路径跟随：逐航点发布 TrajectorySetpoint

PX4 Offboard 控制流程：
  1. 先发布若干帧 OffboardControlMode + TrajectorySetpoint
  2. 发布 VehicleCommand (ARM)
  3. 发布 VehicleCommand (DO_SET_MODE → OFFBOARD)
  4. 持续发布 OffboardControlMode + TrajectorySetpoint（>2Hz）
"""

import time

# px4_msgs 可选导入
try:
    from px4_msgs.msg import (  # type: ignore
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
    )
    HAS_PX4_MSGS = True
except ImportError:
    HAS_PX4_MSGS = False

# ============================================================
# PX4 命令常量与工具函数（自包含，不依赖 aeromind_bridge）
# ============================================================

# VehicleCommand 命令码
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_NAV_TAKEOFF = 22
VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_DO_SET_MODE = 176
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6

# PX4 nav_state 映射
NAV_STATE_MAP = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL",
    3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
    6: "ACRO", 14: "OFFBOARD", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND",
}


def _enu_to_ned(x: float, y: float, z: float):
    """ROS ENU → PX4 NED 坐标转换"""
    return (y, x, -z)


def _make_arm_command(arm: bool):
    """构造解锁/加锁 VehicleCommand"""
    cmd = VehicleCommand()
    cmd.timestamp = 0
    cmd.param1 = 1.0 if arm else 0.0
    cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM
    cmd.target_system = 1
    cmd.target_component = 1
    cmd.source_system = 1
    cmd.source_component = 1
    cmd.from_external = True
    return cmd


def _make_offboard_mode_command():
    """构造切换到 Offboard 模式的 VehicleCommand"""
    cmd = VehicleCommand()
    cmd.timestamp = 0
    cmd.param1 = 1.0
    cmd.param2 = float(PX4_CUSTOM_MAIN_MODE_OFFBOARD)
    cmd.command = VEHICLE_CMD_DO_SET_MODE
    cmd.target_system = 1
    cmd.target_component = 1
    cmd.source_system = 1
    cmd.source_component = 1
    cmd.from_external = True
    return cmd


def _make_takeoff_command(altitude: float):
    """构造起飞 VehicleCommand"""
    cmd = VehicleCommand()
    cmd.timestamp = 0
    cmd.param7 = altitude
    cmd.command = VEHICLE_CMD_NAV_TAKEOFF
    cmd.target_system = 1
    cmd.target_component = 1
    cmd.source_system = 1
    cmd.source_component = 1
    cmd.from_external = True
    return cmd


def _make_land_command():
    """构造降落 VehicleCommand"""
    cmd = VehicleCommand()
    cmd.timestamp = 0
    cmd.command = VEHICLE_CMD_NAV_LAND
    cmd.target_system = 1
    cmd.target_component = 1
    cmd.source_system = 1
    cmd.source_component = 1
    cmd.from_external = True
    return cmd


def _position_to_trajectory_setpoint(x: float, y: float, z: float, yaw: float = float("nan")):
    """ROS ENU 位置 → PX4 NED TrajectorySetpoint"""
    sp = TrajectorySetpoint()
    sp.timestamp = 0
    px4_x, px4_y, px4_z = _enu_to_ned(x, y, z)
    sp.position[0] = px4_x
    sp.position[1] = px4_y
    sp.position[2] = px4_z
    sp.yaw = yaw
    for i in range(3):
        sp.velocity[i] = float("nan")
        sp.acceleration[i] = float("nan")
    sp.yawspeed = float("nan")
    return sp


class PX4Controller:
    """PX4 Offboard 控制器

    封装所有 PX4 飞控命令的构造、发布逻辑和 Offboard 心跳维护。
    """

    def __init__(self, node):
        """
        Args:
            node: rclpy Node 实例（用于创建 publisher 和 timer）
        """
        self._node = node
        self._logger = node.get_logger()

        if not HAS_PX4_MSGS:
            self._logger.error("px4_msgs 未安装！PX4 控制功能不可用。")
            return

        # PX4 输入话题发布者
        self._offboard_mode_pub = node.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self._trajectory_pub = node.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self._vehicle_cmd_pub = node.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        # Offboard 心跳定时器（10Hz，PX4 要求 >2Hz）
        self._offboard_timer = node.create_timer(0.1, self._offboard_heartbeat)
        self._offboard_active = False
        self._offboard_mode = "position"  # "position" | "velocity"
        self._last_trajectory_setpoint = None

        self._logger.info("PX4 控制器已初始化")

    def _timestamp_us(self) -> int:
        """返回 PX4 消息使用的微秒时间戳"""
        return int(self._node.get_clock().now().nanoseconds / 1000)

    def _publish_vehicle_command(self, cmd):
        cmd.timestamp = self._timestamp_us()
        self._vehicle_cmd_pub.publish(cmd)

    def _publish_offboard_control_mode(self):
        mode = OffboardControlMode()
        mode.timestamp = self._timestamp_us()
        if self._offboard_mode == "position":
            mode.position = True
            mode.velocity = False
        else:
            mode.position = False
            mode.velocity = True
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self._offboard_mode_pub.publish(mode)

    def _publish_last_trajectory_setpoint(self):
        if self._last_trajectory_setpoint is None:
            return
        self._last_trajectory_setpoint.timestamp = self._timestamp_us()
        self._trajectory_pub.publish(self._last_trajectory_setpoint)

    # ============================================================
    # 基础飞控命令
    # ============================================================

    def arm(self) -> bool:
        """解锁无人机"""
        if not HAS_PX4_MSGS:
            return False
        cmd = _make_arm_command(True)
        self._publish_vehicle_command(cmd)
        self._logger.info("PX4: 解锁指令已发送")
        return True

    def disarm(self) -> bool:
        """加锁无人机"""
        if not HAS_PX4_MSGS:
            return False
        self._offboard_active = False
        cmd = _make_arm_command(False)
        self._publish_vehicle_command(cmd)
        self._logger.info("PX4: 加锁指令已发送")
        return True

    def set_offboard_mode(self) -> bool:
        """切换到 Offboard 模式并启动心跳"""
        if not HAS_PX4_MSGS:
            return False
        self._offboard_active = True
        cmd = _make_offboard_mode_command()
        self._publish_vehicle_command(cmd)
        self._logger.info("PX4: Offboard 模式已激活")
        return True

    def takeoff(self, altitude: float) -> bool:
        """起飞到指定高度（NED 坐标系，单位 m）

        Args:
            altitude: 目标高度 (m, 正值向上)
        """
        if not HAS_PX4_MSGS:
            return False
        cmd = _make_takeoff_command(altitude)
        self._publish_vehicle_command(cmd)
        self._logger.info(f"PX4: 起飞指令已发送 (高度={altitude}m)")
        return True

    def offboard_takeoff(self, altitude: float, x: float = 0.0, y: float = 0.0) -> bool:
        """使用 Offboard 位置控制起飞到指定本地高度。

        Args:
            altitude: 目标高度 (ROS ENU, m, 正值向上)
            x, y: 起飞时保持的本地水平位置 (ROS ENU, m)
        """
        if not HAS_PX4_MSGS:
            return False

        self._offboard_mode = "position"
        self._last_trajectory_setpoint = _position_to_trajectory_setpoint(x, y, altitude)

        # PX4 要求切 Offboard 前已经收到若干帧 setpoint。
        for _ in range(10):
            self._publish_offboard_control_mode()
            self._publish_last_trajectory_setpoint()
            time.sleep(0.1)

        self.arm()
        time.sleep(0.2)
        self.set_offboard_mode()
        self._offboard_active = True
        self._logger.info(
            f"PX4: Offboard 起飞目标已设置 (x={x:.2f}, y={y:.2f}, 高度={altitude:.2f}m)"
        )
        return True

    def land(self) -> bool:
        """降落"""
        if not HAS_PX4_MSGS:
            return False
        cmd = _make_land_command()
        self._publish_vehicle_command(cmd)
        self._offboard_active = False
        self._logger.info("PX4: 降落指令已发送")
        return True

    # ============================================================
    # 位置控制（路径跟随）
    # ============================================================

    def set_position(self, x: float, y: float, z: float, yaw: float = float("nan")):
        """设置目标位置（ROS ENU 坐标系）

        Args:
            x, y, z: 目标位置 (ENU, m, z 向上为正)
            yaw: 目标偏航角 (rad)
        """
        if not HAS_PX4_MSGS:
            return
        self._offboard_mode = "position"
        sp = _position_to_trajectory_setpoint(x, y, z, yaw)
        sp.timestamp = self._timestamp_us()
        self._last_trajectory_setpoint = sp
        self._trajectory_pub.publish(sp)

    def follow_path(self, waypoints: list, velocity: float = 5.0, timeout_per_wp: float = 30.0):
        """按路径逐航点飞行（同步阻塞）

        Args:
            waypoints: geometry_msgs/PoseStamped 列表
            velocity: 飞行速度 (m/s)，暂无实际效果（PX4 内部处理）
            timeout_per_wp: 每个航点的超时 (s)
        """
        if not HAS_PX4_MSGS or not waypoints:
            return

        self._logger.info(f"PX4: 开始路径跟随 ({len(waypoints)} 个航点)")

        for i, wp in enumerate(waypoints):
            pos = wp.pose.position
            self._logger.info(
                f"  航点 {i + 1}/{len(waypoints)}: "
                f"({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
            )
            self.set_position(pos.x, pos.y, pos.z)

            # 等待到达航点（简化：固定等待时间）
            # TODO: 实际应通过 vehicle_local_position 判断是否到达
            time.sleep(2.0)

        self._logger.info("PX4: 路径跟随完成")

    # ============================================================
    # Offboard 心跳
    # ============================================================

    def _offboard_heartbeat(self):
        """Offboard 模式心跳：持续发布控制模式和目标点以维持 Offboard"""
        if not self._offboard_active or not HAS_PX4_MSGS:
            return

        self._publish_offboard_control_mode()
        self._publish_last_trajectory_setpoint()

    @property
    def available(self) -> bool:
        """PX4 控制是否可用"""
        return HAS_PX4_MSGS
