#!/usr/bin/env python3
"""
teleop_node.py — 键盘遥控节点（位置 + 速度双模式）

整合自 hw_insight keyboard_position.py + keyboard_velocity.py
功能：
  - 位置模式：WASD/IK/JL 逐步移动目标位置，H 悬停
  - 速度模式：WASD/IK/JL 逐步增减速度（带归零渐变），H 急停
  - ESC 退出，非阻塞键盘读取

用法:
  ros2 run aeromind_teleop teleop_node --ros-args -p mode:=velocity
  ros2 run aeromind_teleop teleop_node --ros-args -p mode:=position

按键说明:
  W/S    前进/后退  (ENU Y轴: 北/南)
  A/D    左移/右移  (ENU X轴: 西/东)
  I/K    上升/下降  (ENU Z轴: 上/下)
  J/L    左转/右转
  H      停止（位置模式=悬停，速度模式=归零）
  ESC    退出

坐标系: ROS ENU (x=东, y=北, z=上)
"""

import sys
import tty
import termios
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ENU 坐标系步长配置
XY_STEP = 5.0       # 位置模式: 水平步长 (m) / 速度模式: 速度增量 (m/s)
XY_MAX = 20.0        # 位置模式: 最远距离 / 速度模式: 最大速度
Z_STEP = 2.0
Z_MAX = 10.0
YAW_STEP = 0.3
YAW_MAX = 2.0


def getch_nonblock(timeout: float = 0.1):
    """非阻塞读取单字符, 超时返回 None"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if rlist else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class TeleopNode(Node):
    """键盘遥控节点 — 位置/速度双模式 (ENU 坐标系)"""

    def __init__(self):
        super().__init__("teleop_node")

        self.declare_parameter("mode", "velocity")
        self._mode = self.get_parameter("mode").value
        self.get_logger().info(f"遥控模式: {self._mode}  (ENU 坐标系)")

        # 发布速度/位置指令
        self._cmd_pub = self.create_publisher(Twist, "/control/cmd_vel", 10)

        # 当前指令值（速度模式下累积，位置模式下直接设）
        self._vx = 0.0   # ENU X (东)
        self._vy = 0.0   # ENU Y (北)
        self._vz = 0.0   # ENU Z (上)
        self._vyaw = 0.0

        # 定时器: 10Hz 读取键盘 + 发布
        self._timer = self.create_timer(0.1, self._loop)

        self._print_help()

    def _print_help(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("键盘遥控已启动 (ENU 坐标系)")
        self.get_logger().info("  W/S: 前后(北/南)  A/D: 左右(西/东)")
        self.get_logger().info("  I/K: 上下(上/下)  J/L: 左右转")
        self.get_logger().info("  H: 停止  ESC: 退出")
        self.get_logger().info(f"  模式: {self._mode}")
        self.get_logger().info("=" * 50)

    def _velocity_to_zero(self, x: bool, y: bool, z: bool, yaw: bool):
        """速度渐变归零：每一步向零靠近一个步长"""
        if x:
            self._vx = (self._vx - XY_STEP if self._vx > 0
                        else self._vx + XY_STEP if self._vx < 0 else 0.0)
        if y:
            self._vy = (self._vy - XY_STEP if self._vy > 0
                        else self._vy + XY_STEP if self._vy < 0 else 0.0)
        if z:
            self._vz = (self._vz - Z_STEP if self._vz > 0
                        else self._vz + Z_STEP if self._vz < 0 else 0.0)
        if yaw:
            self._vyaw = (self._vyaw - YAW_STEP if self._vyaw > 0
                          else self._vyaw + YAW_STEP if self._vyaw < 0 else 0.0)

    def _loop(self):
        key = getch_nonblock(0.1)

        if key is not None:
            self._handle_key(key)
        elif self._mode == "velocity":
            # 无按键 → 所有方向渐变归零
            self._velocity_to_zero(True, True, True, True)

        msg = Twist()
        if self._mode == "velocity":
            msg.linear.x = float(self._vx)
            msg.linear.y = float(self._vy)
            msg.linear.z = float(self._vz)
            msg.angular.z = float(self._vyaw)
        else:
            # 位置模式：直接发送位置增量
            msg.linear.x = float(self._vx)
            msg.linear.y = float(self._vy)
            msg.linear.z = float(self._vz)
            msg.angular.z = float(self._vyaw)
            # 位置模式下每次发布后归零（单次指令）
            self._vx = self._vy = self._vz = self._vyaw = 0.0

        self._cmd_pub.publish(msg)

    def _handle_key(self, key):
        if self._mode == "velocity":
            self._handle_velocity_key(key)
        else:
            self._handle_position_key(key)

    # ============================================================
    # 位置模式 (ENU: X=东, Y=北, Z=上)
    # ============================================================

    def _handle_position_key(self, key):
        """位置模式：设置单次目标位移"""
        if key in ('w', 'W'):
            self._vy = XY_STEP          # 前进 = 北 (ENU +Y)
            self.get_logger().info("前进 (北)")
        elif key in ('s', 'S'):
            self._vy = -XY_STEP         # 后退 = 南 (ENU -Y)
            self.get_logger().info("后退 (南)")
        elif key in ('d', 'D'):
            self._vx = XY_STEP          # 右移 = 东 (ENU +X)
            self.get_logger().info("右移 (东)")
        elif key in ('a', 'A'):
            self._vx = -XY_STEP         # 左移 = 西 (ENU -X)
            self.get_logger().info("左移 (西)")
        elif key in ('i', 'I'):
            self._vz = Z_STEP           # 上升 (ENU +Z)
            self.get_logger().info("上升")
        elif key in ('k', 'K'):
            self._vz = -Z_STEP          # 下降 (ENU -Z)
            self.get_logger().info("下降")
        elif key in ('j', 'J'):
            self._vyaw = -YAW_STEP
            self.get_logger().info("左转")
        elif key in ('l', 'L'):
            self._vyaw = YAW_STEP
            self.get_logger().info("右转")
        elif key in ('h', 'H'):
            self.get_logger().info("悬停")
        elif ord(key) in (27, 3):
            self.get_logger().info("退出")
            raise SystemExit(0)

    # ============================================================
    # 速度模式 (ENU: X=东, Y=北, Z=上)
    # ============================================================

    def _handle_velocity_key(self, key):
        """速度模式：累积速度 + 非按下方向渐变归零"""
        if key in ('w', 'W'):
            # 前进 = 北 (ENU +Y)
            self._vy = min(self._vy + XY_STEP, XY_MAX)
            self._velocity_to_zero(True, False, True, True)  # 归零 X, Z, yaw
            self.get_logger().info("前进 (北)")
        elif key in ('s', 'S'):
            # 后退 = 南 (ENU -Y)
            self._vy = max(self._vy - XY_STEP, -XY_MAX)
            self._velocity_to_zero(True, False, True, True)
            self.get_logger().info("后退 (南)")
        elif key in ('d', 'D'):
            # 右移 = 东 (ENU +X)
            self._vx = min(self._vx + XY_STEP, XY_MAX)
            self._velocity_to_zero(False, True, True, True)  # 归零 Y, Z, yaw
            self.get_logger().info("右移 (东)")
        elif key in ('a', 'A'):
            # 左移 = 西 (ENU -X)
            self._vx = max(self._vx - XY_STEP, -XY_MAX)
            self._velocity_to_zero(False, True, True, True)
            self.get_logger().info("左移 (西)")
        elif key in ('i', 'I'):
            # 上升 (ENU +Z)
            self._vz = min(self._vz + Z_STEP, Z_MAX)
            self._velocity_to_zero(True, True, False, True)  # 归零 X, Y, yaw
            self.get_logger().info("上升")
        elif key in ('k', 'K'):
            # 下降 (ENU -Z)
            self._vz = max(self._vz - Z_STEP, -Z_MAX)
            self._velocity_to_zero(True, True, False, True)
            self.get_logger().info("下降")
        elif key in ('j', 'J'):
            self._vyaw = max(self._vyaw - YAW_STEP, -YAW_MAX)
            self._velocity_to_zero(True, True, True, False)  # 归零 X, Y, Z
            self.get_logger().info("左转")
        elif key in ('l', 'L'):
            self._vyaw = min(self._vyaw + YAW_STEP, YAW_MAX)
            self._velocity_to_zero(True, True, True, False)
            self.get_logger().info("右转")
        elif key in ('h', 'H'):
            self._vx = self._vy = self._vz = self._vyaw = 0.0
            self.get_logger().info("急停")
        elif ord(key) in (27, 3):
            self.get_logger().info("退出")
            raise SystemExit(0)
        # 注意: 不再有 else 分支 — 无按键时的衰减已移到 _loop 中处理


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
