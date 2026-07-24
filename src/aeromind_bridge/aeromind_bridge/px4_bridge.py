#!/usr/bin/env python3
"""
px4_bridge.py — PX4 uXRCE-DDS 话题与 ROS 2 标准消息之间的转换工具

负责：
  - PX4 NED 坐标系 ↔ ROS ENU 坐标系的转换
  - px4_msgs 传感器话题 → nav_msgs/Odometry, sensor_msgs/Imu
  - px4_msgs 状态话题 → aeromind_interfaces/DroneState
  - geometry_msgs/Twist → px4_msgs OffboardControlMode + TrajectorySetpoint

坐标系转换规则：
  PX4 NED (x=北, y=东, z=下) → ROS ENU (x=东, y=北, z=上)
    ros.x = px4.y
    ros.y = px4.x
    ros.z = -px4.z
"""

import math
from typing import Optional, Tuple

from geometry_msgs.msg import Quaternion, Vector3, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as RosTime

from aeromind_interfaces.msg import DroneState

# px4_msgs 可选导入
try:
    from px4_msgs.msg import (  # type: ignore
        VehicleOdometry,
        VehicleAngularVelocity,
        VehicleAcceleration,
        VehicleAttitude,
        SensorCombined,
        VehicleLocalPosition,
        VehicleStatus,
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
    )
    HAS_PX4_MSGS = True
except ImportError:
    HAS_PX4_MSGS = False
    VehicleOdometry = None  # type: ignore
    VehicleAngularVelocity = None  # type: ignore
    VehicleAcceleration = None  # type: ignore
    VehicleAttitude = None  # type: ignore
    SensorCombined = None  # type: ignore
    VehicleLocalPosition = None  # type: ignore
    VehicleStatus = None  # type: ignore
    OffboardControlMode = None  # type: ignore
    TrajectorySetpoint = None  # type: ignore
    VehicleCommand = None  # type: ignore


# ============================================================
# PX4 arming_state → 描述字符串 (VehicleStatus v4)
# ============================================================
# 注意: 新版 PX4 VehicleStatus 中 arming_state 含义:
#   1=DISARMED, 2=ARMED
# 旧版旧定义(1=INIT,2=STANDBY,3=ARMED)已废弃
ARMING_STATE_MAP = {
    1: "DISARMED",
    2: "ARMED",
}

# PX4 nav_state → 描述字符串 (对齐 PX4 v1.14+ 官方定义)
NAV_STATE_MAP = {
    0:  "MANUAL",
    1:  "ALTCTL",
    2:  "POSCTL",
    3:  "AUTO_MISSION",
    4:  "AUTO_LOITER",
    5:  "AUTO_RTL",
    6:  "POSITION_SLOW",
    10: "ACRO",
    12: "DESCEND",
    13: "TERMINATION",
    14: "OFFBOARD",
    15: "STAB",
    17: "AUTO_TAKEOFF",
    18: "AUTO_LAND",
    19: "AUTO_FOLLOW_TARGET",
    20: "AUTO_PRECLAND",
    21: "ORBIT",
    22: "AUTO_VTOL_TAKEOFF",
}

# PX4 VehicleCommand 命令码
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_NAV_TAKEOFF = 22
VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_DO_SET_MODE = 176

# PX4 自定义模式值
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


def ned_to_enu_position(px4_x, px4_y, px4_z) -> Tuple[float, float, float]:
    """PX4 NED 位置 → ROS ENU 位置"""
    return (float(px4_y), float(px4_x), float(-px4_z))


def ned_to_enu_velocity(px4_vx, px4_vy, px4_vz) -> Tuple[float, float, float]:
    """PX4 NED 速度 → ROS ENU 速度"""
    return (float(px4_vy), float(px4_vx), float(-px4_vz))


def ned_to_enu_quaternion(px4_q: Quaternion) -> Quaternion:
    """PX4 NED 四元数 → ROS ENU 四元数"""
    q = Quaternion()
    q.x = float(px4_q.y)
    q.y = float(px4_q.x)
    q.z = float(-px4_q.z)
    q.w = float(px4_q.w)
    return q


def enu_to_ned_position(ros_x, ros_y, ros_z) -> Tuple[float, float, float]:
    """ROS ENU 位置 → PX4 NED 位置"""
    return (float(ros_y), float(ros_x), float(-ros_z))


def px4_timestamp_to_ros(px4_timestamp_us: int) -> RosTime:
    """PX4 微秒时间戳 → ROS Time 消息"""
    sec = int(px4_timestamp_us // 1_000_000)
    nanosec = int((px4_timestamp_us % 1_000_000) * 1000)
    return RosTime(sec=sec, nanosec=nanosec)


def vehicle_odometry_to_ros(px4_msg) -> Odometry:
    """px4_msgs/VehicleOdometry → nav_msgs/Odometry"""
    odom = Odometry()
    odom.header = Header(
        stamp=px4_timestamp_to_ros(px4_msg.timestamp),
        frame_id="odom",
    )
    odom.child_frame_id = "base_link"

    # 位置: NED → ENU
    ros_x, ros_y, ros_z = ned_to_enu_position(
        px4_msg.position[0], px4_msg.position[1], px4_msg.position[2]
    )
    odom.pose.pose.position.x = ros_x
    odom.pose.pose.position.y = ros_y
    odom.pose.pose.position.z = ros_z

    # 姿态: NED → ENU
    px4_q = Quaternion(
        x=float(px4_msg.q[1]), y=float(px4_msg.q[2]),
        z=float(px4_msg.q[3]), w=float(px4_msg.q[0]),
    )
    enu_q = ned_to_enu_quaternion(px4_q)
    odom.pose.pose.orientation = enu_q

    # 速度: NED → ENU
    ros_vx, ros_vy, ros_vz = ned_to_enu_velocity(
        px4_msg.velocity[0], px4_msg.velocity[1], px4_msg.velocity[2]
    )
    odom.twist.twist.linear.x = ros_vx
    odom.twist.twist.linear.y = ros_vy
    odom.twist.twist.linear.z = ros_vz

    # 角速度: FRD → ENU (同 NED 转换)
    ros_wx, ros_wy, ros_wz = ned_to_enu_velocity(
        px4_msg.angular_velocity[0],
        px4_msg.angular_velocity[1],
        px4_msg.angular_velocity[2],
    )
    odom.twist.twist.angular.x = ros_wx
    odom.twist.twist.angular.y = ros_wy
    odom.twist.twist.angular.z = ros_wz

    return odom


def vehicle_attitude_to_ros(px4_msg) -> Quaternion:
    """px4_msgs/VehicleAttitude → geometry_msgs/Quaternion (ENU)"""
    px4_q = Quaternion(
        x=float(px4_msg.q[1]), y=float(px4_msg.q[2]),
        z=float(px4_msg.q[3]), w=float(px4_msg.q[0]),
    )
    return ned_to_enu_quaternion(px4_q)


def vehicle_local_position_to_ros(px4_msg, attitude_msg=None) -> Odometry:
    """px4_msgs/VehicleLocalPosition + VehicleAttitude → nav_msgs/Odometry"""
    odom = Odometry()
    odom.header = Header(
        stamp=px4_timestamp_to_ros(px4_msg.timestamp),
        frame_id="odom",
    )
    odom.child_frame_id = "base_link"

    # 位置: NED → ENU
    ros_x, ros_y, ros_z = ned_to_enu_position(px4_msg.x, px4_msg.y, px4_msg.z)
    odom.pose.pose.position.x = ros_x
    odom.pose.pose.position.y = ros_y
    odom.pose.pose.position.z = ros_z

    # 姿态：使用 vehicle_attitude
    if attitude_msg is not None:
        odom.pose.pose.orientation = vehicle_attitude_to_ros(attitude_msg)
    else:
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    # 速度: NED → ENU
    ros_vx, ros_vy, ros_vz = ned_to_enu_velocity(px4_msg.vx, px4_msg.vy, px4_msg.vz)
    odom.twist.twist.linear.x = ros_vx
    odom.twist.twist.linear.y = ros_vy
    odom.twist.twist.linear.z = ros_vz

    # 角速度（VehicleLocalPosition 不含，填 0）
    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = 0.0

    return odom


def build_imu_from_sensor_combined(sensor_msg, attitude_msg=None) -> Imu:
    """px4_msgs/SensorCombined + VehicleAttitude → sensor_msgs/Imu"""
    imu = Imu()
    imu.header = Header(
        stamp=px4_timestamp_to_ros(sensor_msg.timestamp),
        frame_id="imu_link",
    )

    # 姿态
    if attitude_msg is not None:
        imu.orientation = vehicle_attitude_to_ros(attitude_msg)
    else:
        imu.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    # 角速度: SensorCombined.gyro_rad (FRD → ENU)
    gx, gy, gz = sensor_msg.gyro_rad[0], sensor_msg.gyro_rad[1], sensor_msg.gyro_rad[2]
    ros_wx, ros_wy, ros_wz = ned_to_enu_velocity(gx, gy, gz)
    imu.angular_velocity.x = ros_wx
    imu.angular_velocity.y = ros_wy
    imu.angular_velocity.z = ros_wz

    # 加速度: SensorCombined.accelerometer_m_s2 (FRD → ENU)
    ax, ay, az = sensor_msg.accelerometer_m_s2[0], sensor_msg.accelerometer_m_s2[1], sensor_msg.accelerometer_m_s2[2]
    ros_ax, ros_ay, ros_az = ned_to_enu_velocity(ax, ay, az)
    imu.linear_acceleration.x = ros_ax
    imu.linear_acceleration.y = ros_ay
    imu.linear_acceleration.z = ros_az

    return imu


def build_imu_from_px4(
    attitude_msg,
    angular_vel_msg,
    accel_msg,
) -> Imu:
    """从 PX4 姿态、角速度、加速度消息合成 sensor_msgs/Imu"""
    imu = Imu()
    # 使用加速度消息的时间戳
    imu.header = Header(
        stamp=px4_timestamp_to_ros(accel_msg.timestamp),
        frame_id="imu_link",
    )

    # 姿态
    imu.orientation = vehicle_attitude_to_ros(attitude_msg)

    # 角速度: FRD → ENU
    wx = angular_vel_msg.xyz[0]
    wy = angular_vel_msg.xyz[1]
    wz = angular_vel_msg.xyz[2]
    ros_wx, ros_wy, ros_wz = ned_to_enu_velocity(wx, wy, wz)
    imu.angular_velocity.x = ros_wx
    imu.angular_velocity.y = ros_wy
    imu.angular_velocity.z = ros_wz

    # 加速度: FRD → ENU
    ax = accel_msg.xyz[0]
    ay = accel_msg.xyz[1]
    az = accel_msg.xyz[2]
    ros_ax, ros_ay, ros_az = ned_to_enu_velocity(ax, ay, az)
    imu.linear_acceleration.x = ros_ax
    imu.linear_acceleration.y = ros_ay
    imu.linear_acceleration.z = ros_az

    return imu


def vehicle_status_to_drone_state(px4_msg) -> DroneState:
    """px4_msgs/VehicleStatus → aeromind_interfaces/DroneState"""
    state = DroneState()
    state.armed = (px4_msg.arming_state == 2)  # 2 = ARMED (PX4 v1.14+)
    state.mode = NAV_STATE_MAP.get(px4_msg.nav_state, f"UNKNOWN({px4_msg.nav_state})")
    state.battery = 0.0   # VehicleStatus 不含电池信息
    state.gps_fix = 2     # 默认值，GPS 信息需从 SensorGps 话题获取
    state.ekf_healthy = (px4_msg.pre_flight_checks_pass if hasattr(px4_msg, 'pre_flight_checks_pass') else True)
    state.preflight_ok = bool(
        getattr(px4_msg, "pre_flight_checks_pass", False)
    )
    state.preflight_valid = hasattr(px4_msg, "pre_flight_checks_pass")
    state.landed = False
    state.landed_valid = False
    return state


def cmd_vel_to_offboard_control(cmd_vel: Twist) -> OffboardControlMode:
    """将 Twist 速度指令转为 PX4 速度控制模式"""
    mode = OffboardControlMode()
    mode.timestamp = 0  # PX4 自动填充
    mode.position = False
    mode.velocity = True
    mode.acceleration = False
    mode.attitude = False
    mode.body_rate = False
    return mode


def cmd_vel_to_trajectory_setpoint(cmd_vel: Twist) -> TrajectorySetpoint:
    """将 Twist 速度指令转为 PX4 轨迹设定点（ROS ENU → PX4 NED）"""
    sp = TrajectorySetpoint()
    sp.timestamp = 0
    # 速度: ROS ENU → PX4 NED
    px4_vx, px4_vy, px4_vz = enu_to_ned_position(
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z
    )
    sp.velocity[0] = px4_vx
    sp.velocity[1] = px4_vy
    sp.velocity[2] = px4_vz
    # 偏航角速度
    sp.yawspeed = -cmd_vel.angular.z  # ROS ENU → PX4 NED (符号取反)
    # 位置/加速度不设
    for i in range(3):
        sp.position[i] = float("nan")
        sp.acceleration[i] = float("nan")
    sp.yaw = float("nan")
    return sp


def position_to_trajectory_setpoint(x: float, y: float, z: float, yaw: float = float("nan")) -> TrajectorySetpoint:
    """ROS ENU 位置 → PX4 NED 轨迹设定点"""
    sp = TrajectorySetpoint()
    sp.timestamp = 0
    px4_x, px4_y, px4_z = enu_to_ned_position(x, y, z)
    sp.position[0] = px4_x
    sp.position[1] = px4_y
    sp.position[2] = px4_z
    sp.yaw = yaw
    # 速度/加速度不设（设为 NaN 表示不控制该量）
    for i in range(3):
        sp.velocity[i] = float("nan")
        sp.acceleration[i] = float("nan")
    sp.yawspeed = float("nan")
    return sp


def position_offboard_mode() -> OffboardControlMode:
    """位置控制 Offboard 模式"""
    mode = OffboardControlMode()
    mode.timestamp = 0
    mode.position = True
    mode.velocity = False
    mode.acceleration = False
    mode.attitude = False
    mode.body_rate = False
    return mode


def make_arm_command(arm: bool) -> VehicleCommand:
    """构造解锁/加锁指令"""
    cmd = VehicleCommand()
    cmd.timestamp = 0
    cmd.param1 = 1.0 if arm else 0.0
    cmd.param2 = 0.0
    cmd.param3 = 0.0
    cmd.param4 = 0.0
    cmd.param5 = 0.0
    cmd.param6 = 0.0
    cmd.param7 = 0.0
    cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM
    cmd.target_system = 1
    cmd.target_component = 1
    cmd.source_system = 1
    cmd.source_component = 1
    cmd.confirmation = 0
    cmd.from_external = True
    return cmd


def make_offboard_mode_command() -> VehicleCommand:
    """构造切换到 Offboard 模式的指令"""
    cmd = VehicleCommand()
    cmd.timestamp = 0
    cmd.param1 = 1.0  # 主模式: custom
    cmd.param2 = float(PX4_CUSTOM_MAIN_MODE_OFFBOARD)  # 子模式: offboard
    cmd.param3 = 0.0
    cmd.param4 = 0.0
    cmd.param5 = 0.0
    cmd.param6 = 0.0
    cmd.param7 = 0.0
    cmd.command = VEHICLE_CMD_DO_SET_MODE
    cmd.target_system = 1
    cmd.target_component = 1
    cmd.source_system = 1
    cmd.source_component = 1
    cmd.confirmation = 0
    cmd.from_external = True
    return cmd


def make_takeoff_command(altitude: float) -> VehicleCommand:
    """构造起飞指令（NED 坐标系，z 向下为正，起飞取负）"""
    cmd = VehicleCommand()
    cmd.timestamp = 0
    cmd.param1 = 0.0   # pitch
    cmd.param2 = 0.0   # empty
    cmd.param3 = 0.0   # empty
    cmd.param4 = 0.0   # yaw
    cmd.param5 = 0.0   # latitude (不设)
    cmd.param6 = 0.0   # longitude (不设)
    cmd.param7 = altitude  # 起飞高度 (m)
    cmd.command = VEHICLE_CMD_NAV_TAKEOFF
    cmd.target_system = 1
    cmd.target_component = 1
    cmd.source_system = 1
    cmd.source_component = 1
    cmd.confirmation = 0
    cmd.from_external = True
    return cmd


def make_land_command() -> VehicleCommand:
    """构造降落指令"""
    cmd = VehicleCommand()
    cmd.timestamp = 0
    cmd.param1 = 0.0
    cmd.param2 = 0.0
    cmd.param3 = 0.0
    cmd.param4 = 0.0
    cmd.param5 = 0.0
    cmd.param6 = 0.0
    cmd.param7 = 0.0
    cmd.command = VEHICLE_CMD_NAV_LAND
    cmd.target_system = 1
    cmd.target_component = 1
    cmd.source_system = 1
    cmd.source_component = 1
    cmd.confirmation = 0
    cmd.from_external = True
    return cmd
