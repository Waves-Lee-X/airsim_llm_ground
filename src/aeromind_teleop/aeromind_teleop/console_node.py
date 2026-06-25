#!/usr/bin/env python3
"""
console_node.py — AeroMind 终端控制台

实时显示:
  - 飞控状态（解锁/模式/故障/预检）
  - 位置、速度（ENU 坐标系）
  - 传感器状态（GPS/EKF）
  - 键盘控制面板

用法:
  ros2 run aeromind_teleop console_node
"""

import os
import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from aeromind_interfaces.msg import DroneState


class ConsoleNode(Node):
    """终端仪表盘节点"""

    def __init__(self):
        super().__init__("console_node")

        # 缓存最新数据
        self._odom = None
        self._state = None
        self._frame_count = 0

        # 订阅
        self._odom_sub = self.create_subscription(
            Odometry, "/sensor/odometry", self._odom_callback, 10
        )
        self._state_sub = self.create_subscription(
            DroneState, "/control/drone_state", self._state_callback, 10
        )

        # 2Hz 刷新
        self._timer = self.create_timer(0.5, self._render)

    def _odom_callback(self, msg):
        self._odom = msg

    def _state_callback(self, msg):
        self._state = msg

    def _render(self):
        self._frame_count += 1
        # 清屏 + 光标归位
        sys.stdout.write("\033[2J\033[H")

        self._render_header()
        self._render_status()
        self._render_position()
        self._render_controls()

    def _render_header(self):
        print("=" * 60)
        print("   AeroMind 飞控控制台")
        print("=" * 60)

    def _render_status(self):
        state = self._state
        if state is None:
            print("\n  [等待飞控数据...]")
            return

        arm_icon = "\033[32m已解锁\033[0m" if state.armed else "\033[31m已加锁\033[0m"
        ekf_icon = "\033[32m正常\033[0m" if state.ekf_healthy else "\033[31m异常\033[0m"

        gps_map = {0: "无", 1: "2D", 2: "3D", 3: "DGPS", 4: "RTK"}
        gps_str = gps_map.get(state.gps_fix, "?")

        print(f"\n  状态: {arm_icon}  |  模式: {state.mode}")
        print(f"  GPS: {gps_str}  |  EKF: {ekf_icon}  |  电池: {state.battery:.1f}V")

    def _render_position(self):
        odom = self._odom
        if odom is None:
            print("\n  [等待位置数据...]")
            return

        pos = odom.pose.pose.position
        vel = odom.twist.twist.linear
        ori = odom.pose.pose.orientation

        # 四元数 → 欧拉角
        yaw = math.atan2(
            2 * (ori.w * ori.z + ori.x * ori.y),
            1 - 2 * (ori.y * ori.y + ori.z * ori.z),
        )
        yaw_deg = math.degrees(yaw) % 360

        alt = -pos.z  # NED z → 高度

        print(f"\n  ┌─ 位置 (ENU) ─────────────────────────────┐")
        print(f"  │  X (东): {pos.x:>10.2f} m    Y (北): {pos.y:>10.2f} m  │")
        print(f"  │  高度:    {alt:>10.2f} m    航向:   {yaw_deg:>7.1f}°  │")
        print(f"  ├─ 速度 ────────────────────────────────────┤")
        print(f"  │  Vx: {vel.x:>8.2f}  Vy: {vel.y:>8.2f}  Vz: {vel.z:>8.2f} │")
        print(f"  └────────────────────────────────────────────┘")

    def _render_controls(self):
        print(f"""
  ╔══════════════════════════════════════════════╗
  ║           键盘控制面板                        ║
  ╠══════════════════════════════════════════════╣
  ║  W/S    前进/后退     I/K    上升/下降       ║
  ║  A/D    左移/右移     J/L    左转/右转       ║
  ║  H      急停/悬停     ESC    退出            ║
  ╠══════════════════════════════════════════════╣
  ║  另开终端运行:                                ║
  ║  ros2 run aeromind_teleop teleop_node        ║
  ╚══════════════════════════════════════════════╝
""")


def main(args=None):
    rclpy.init(args=args)
    node = ConsoleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\033[2J\033[H")  # 清屏
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
