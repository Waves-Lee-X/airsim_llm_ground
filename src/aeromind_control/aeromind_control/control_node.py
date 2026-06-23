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

from aeromind_interfaces.msg import Path, DroneState
from aeromind_interfaces.srv import Takeoff, Land, ArmDrone

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
        self._px4_mode = self.get_parameter("px4_mode").value
        self.get_logger().info(f"控制模式: {self._px4_mode}")

        # 创建服务
        self._takeoff_srv = self.create_service(
            Takeoff, "/control/takeoff", self._takeoff_callback
        )
        self._land_srv = self.create_service(
            Land, "/control/land", self._land_callback
        )
        self._arm_srv = self.create_service(
            ArmDrone, "/control/arm", self._arm_callback
        )

        # 订阅规划路径
        self._path_sub = self.create_subscription(
            Path, "/planning/path", self._path_callback, 10
        )

        # 发布无人机状态
        self._state_pub = self.create_publisher(DroneState, "/control/drone_state", 10)

        # 状态变量
        self._armed = False

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
            self._client = airsim.MultirotorClient()
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
        self._armed = (msg.arming_state == 3)

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

    # ============================================================
    # 起飞服务
    # ============================================================

    def _takeoff_callback(self, request, response):
        altitude = request.altitude
        self.get_logger().info(f"收到起飞请求: 目标高度={altitude}m")

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
            # Step 1: 解锁
            if not self._armed:
                self._px4_ctrl.arm()
                time.sleep(1.0)

            # Step 2: 切换到 Offboard 模式
            self._px4_ctrl.set_offboard_mode()
            time.sleep(0.5)

            # Step 3: 设置当前位置悬停（不漂移）
            self._px4_ctrl.set_position(0.0, 0.0, -10.0)  # ENU, z 向上
            time.sleep(0.5)

            # Step 4: 发送起飞指令
            self._px4_ctrl.takeoff(altitude)

            self._armed = True
            response.success = True
            response.message = f"PX4 起飞指令已发送，目标高度={altitude}m"
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
