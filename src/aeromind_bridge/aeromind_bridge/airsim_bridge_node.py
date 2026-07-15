#!/usr/bin/env python3
"""
airsim_bridge_node.py - ROS 2 桥接节点（AirSim / PX4 双模式）

模式（通过 mode 参数切换）：
  - "airsim"：直接连接 AirSim API，定时读取传感器数据
  - "px4"：订阅 PX4 uXRCE-DDS 话题，转换为 ROS 标准消息

AirSim 模式功能：
  - 定时发布 /sensor/odometry（10Hz）
  - 定时发布 /sensor/imu（10Hz）
  - 定时发布 /control/drone_state（10Hz）
  - 定时发布相机图像 /sensor/camera/*（1Hz）
  - 定时发布 LiDAR 点云 /sensor/lidar/points（1Hz）
  - 订阅 /control/cmd_vel → 转发给 AirSim

PX4 模式功能：
  - 订阅 PX4 传感器话题 → 转换发布 /sensor/odometry, /sensor/imu
  - 订阅 PX4 状态话题 → 转换发布 /control/drone_state
  - 订阅 /control/cmd_vel → 转换为 PX4 OffboardControl 指令
"""

import math
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS 2 标准消息
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Twist, Quaternion, Vector3
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

# 自定义消息
from aeromind_interfaces.msg import DroneState
from aeromind_bridge.camera_bridge import CameraBridge

# PX4 转换工具
from aeromind_bridge.px4_bridge import (
    HAS_PX4_MSGS,
    vehicle_local_position_to_ros,
    build_imu_from_sensor_combined,
    vehicle_status_to_drone_state,
    cmd_vel_to_offboard_control,
    cmd_vel_to_trajectory_setpoint,
    ned_to_enu_position,
    ned_to_enu_quaternion,
    ned_to_enu_velocity,
)

# AirSim API
try:
    import airsim  # type: ignore
    HAS_AIRSIM = True
except ImportError:
    HAS_AIRSIM = False


class AirSimBridgeNode(Node):
    """AirSim / PX4 桥接节点"""

    def __init__(self):
        super().__init__("airsim_bridge_node")

        # 声明参数
        self.declare_parameter("mode", "airsim")
        self.declare_parameter("airsim_ip", "192.168.1.100")
        self.declare_parameter("publish_drone_state", True)
        self._mode = self.get_parameter("mode").value
        self._airsim_ip = self.get_parameter("airsim_ip").value
        self._publish_drone_state = bool(
            self.get_parameter("publish_drone_state").value
        )

        if self._mode not in ("airsim", "px4"):
            self.get_logger().error(f"未知模式 '{self._mode}'，回退到 airsim")
            self._mode = "airsim"

        self.get_logger().info(f"桥接节点启动，模式: {self._mode}")

        # AirSim 图像 RPC 可能阻塞一秒以上，必须与飞控遥测分离执行。
        self._flight_callback_group = MutuallyExclusiveCallbackGroup()
        self._camera_callback_group = MutuallyExclusiveCallbackGroup()

        # 创建发布者（两种模式共用）
        self._odom_pub = self.create_publisher(Odometry, "/sensor/odometry", 10)
        self._imu_pub = self.create_publisher(Imu, "/sensor/imu", 10)
        self._state_pub = self.create_publisher(DroneState, "/control/drone_state", 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # 创建订阅者：接收速度指令（两种模式共用）
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            "/control/cmd_vel",
            self._cmd_vel_callback,
            10,
            callback_group=self._flight_callback_group,
        )

        # 根据模式初始化
        self._client = None  # AirSim 客户端（sensor 数据用）
        if self._mode == "airsim":
            self._init_airsim()
        else:
            self._init_px4()
            # PX4 模式下也连接 AirSim 以获取相机/LiDAR
            self._init_airsim_camera_only()

        self.get_logger().info("桥接节点初始化完成")

    # ============================================================
    # AirSim 模式
    # ============================================================

    def _init_airsim(self):
        """初始化 AirSim 连接和定时器"""
        if not HAS_AIRSIM:
            self.get_logger().error("airsim 包未安装！请执行 pip install airsim")
            return

        try:
            self._client = airsim.MultirotorClient(ip=self._airsim_ip)
            self._client.confirmConnection()
            self.get_logger().info(f"已连接到 AirSim ({self._airsim_ip})")
        except Exception as e:
            self.get_logger().error(f"无法连接 AirSim: {e}")
            self._client = None

        # 定时器：10Hz 读取传感器
        self._airsim_timer = self.create_timer(
            0.1,
            self._airsim_timer_callback,
            callback_group=self._flight_callback_group,
        )

        # 定时器：1Hz 发布相机图像和 LiDAR
        self._camera_bridge = CameraBridge(self, self._client)
        self._camera_timer = self.create_timer(
            1.0,
            self._camera_timer_callback,
            callback_group=self._camera_callback_group,
        )

        # 缓存 IMU 所需的最新状态
        self._latest_airsim_state = None

    def _init_airsim_camera_only(self):
        """PX4 模式下仅连接 AirSim 获取相机/LiDAR（不接飞控）"""
        if not HAS_AIRSIM:
            return
        try:
            self._client = airsim.MultirotorClient(ip=self._airsim_ip)
            self._client.confirmConnection()
            self.get_logger().info(f"已连接 AirSim 相机/LiDAR ({self._airsim_ip})")
        except Exception as e:
            self.get_logger().warn(f"AirSim 相机连接失败（不影响 PX4 飞控）: {e}")
            self._client = None
            return

        self._camera_bridge = CameraBridge(self, self._client)
        self._camera_timer = self.create_timer(
            1.0,
            self._camera_timer_callback,
            callback_group=self._camera_callback_group,
        )

    def _airsim_timer_callback(self):
        """AirSim 定时器：读取状态并发布 ROS 消息"""
        now = self.get_clock().now().to_msg()
        if self._client is None:
            self._publish_empty_all(now)
            return

        try:
            state = self._client.getMultirotorState()
            self._publish_airsim_odometry(now, state)
            self._publish_airsim_imu(now, state)
            self._publish_airsim_state(now)
        except Exception as e:
            self.get_logger().warn(f"获取 AirSim 状态失败: {e}")

    def _publish_airsim_odometry(self, now, state):
        odom = Odometry()
        odom.header = Header(stamp=now, frame_id="odom")
        odom.child_frame_id = "base_link"

        pos = state.kinematics_estimated.position
        enu_position = ned_to_enu_position(pos.x_val, pos.y_val, pos.z_val)
        odom.pose.pose.position.x = enu_position[0]
        odom.pose.pose.position.y = enu_position[1]
        odom.pose.pose.position.z = enu_position[2]

        orient = state.kinematics_estimated.orientation
        odom.pose.pose.orientation = ned_to_enu_quaternion(
            Quaternion(
                x=float(orient.x_val),
                y=float(orient.y_val),
                z=float(orient.z_val),
                w=float(orient.w_val),
            )
        )

        vel = state.kinematics_estimated.linear_velocity
        enu_velocity = ned_to_enu_velocity(vel.x_val, vel.y_val, vel.z_val)
        odom.twist.twist.linear.x = enu_velocity[0]
        odom.twist.twist.linear.y = enu_velocity[1]
        odom.twist.twist.linear.z = enu_velocity[2]

        ang = state.kinematics_estimated.angular_velocity
        enu_angular = ned_to_enu_velocity(ang.x_val, ang.y_val, ang.z_val)
        odom.twist.twist.angular.x = enu_angular[0]
        odom.twist.twist.angular.y = enu_angular[1]
        odom.twist.twist.angular.z = enu_angular[2]

        self._publish_odometry(odom)

    def _publish_airsim_imu(self, now, state):
        imu = Imu()
        imu.header = Header(stamp=now, frame_id="imu_link")

        accel = state.kinematics_estimated.linear_acceleration
        imu.linear_acceleration.x = accel.x_val
        imu.linear_acceleration.y = accel.y_val
        imu.linear_acceleration.z = accel.z_val

        ang = state.kinematics_estimated.angular_velocity
        imu.angular_velocity.x = ang.x_val
        imu.angular_velocity.y = ang.y_val
        imu.angular_velocity.z = ang.z_val

        orient = state.kinematics_estimated.orientation
        imu.orientation.x = orient.x_val
        imu.orientation.y = orient.y_val
        imu.orientation.z = orient.z_val
        imu.orientation.w = orient.w_val

        self._imu_pub.publish(imu)

    def _publish_airsim_state(self, now):
        drone_state = DroneState()
        drone_state.armed = True
        drone_state.mode = "GUIDED"
        drone_state.battery = 11.1
        drone_state.gps_fix = 3
        drone_state.ekf_healthy = True
        self._state_pub.publish(drone_state)

    def _camera_timer_callback(self):
        """1Hz 定时器：发布相机图像和 LiDAR 点云"""
        if self._camera_bridge is not None:
            self._camera_bridge.publish_all()

    # ============================================================
    # PX4 模式
    # ============================================================

    def _init_px4(self):
        """初始化 PX4 uXRCE-DDS 话题订阅"""
        if not HAS_PX4_MSGS:
            self.get_logger().error(
                "px4_msgs 未安装！请在 src/ 下克隆 px4_msgs 包后重新编译。"
            )
            return

        # PX4 uXRCE-DDS 使用 BEST_EFFORT QoS，必须匹配
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 缓存 PX4 消息
        self._latest_attitude = None
        self._latest_sensor_combined = None
        self._latest_local_pos = None
        self._latest_vehicle_status = None
        self._px4_offboard_active = False
        self._latest_cmd_vel_mode = None
        self._latest_cmd_vel_setpoint = None

        # 导入消息类型
        from px4_msgs.msg import (  # type: ignore
            VehicleAttitude,
            SensorCombined,
            VehicleLocalPosition,
            VehicleStatus,
        )

        # 姿态（必要，用于 IMU 和 Odometry 的朝向）
        self._attitude_sub = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self._px4_attitude_callback,
            px4_qos,
            callback_group=self._flight_callback_group,
        )

        # 传感器数据（IMU 数据来源）
        self._sensor_sub = self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            self._px4_sensor_callback,
            px4_qos,
            callback_group=self._flight_callback_group,
        )

        # 本地位置（里程计数据来源）
        self._local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._px4_local_pos_callback,
            px4_qos,
            callback_group=self._flight_callback_group,
        )

        # 状态（同时订阅 vehicle_status 和 vehicle_status_v4，谁发数据就用谁）
        self._status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._px4_status_callback,
            px4_qos,
            callback_group=self._flight_callback_group,
        )
        self._status_v4_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self._px4_status_callback,
            px4_qos,
            callback_group=self._flight_callback_group,
        )

        # 定时器：10Hz 合成并发布 IMU 和 Odometry
        self._px4_timer = self.create_timer(
            0.1,
            self._px4_timer_callback,
            callback_group=self._flight_callback_group,
        )

        # PX4 模式下的 cmd_vel 发布者
        from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand as VCmd
        self._px4_offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self._px4_trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self._px4_vehicle_cmd_pub = self.create_publisher(
            VCmd, "/fmu/in/vehicle_command", 10
        )
        self._px4_cmd_vel_timer = self.create_timer(
            0.1,
            self._px4_cmd_vel_timer_callback,
            callback_group=self._flight_callback_group,
        )

        self.get_logger().info("PX4 模式已初始化，等待 uXRCE-DDS 话题数据...")

    def _px4_timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _publish_px4_vehicle_cmd(self, cmd):
        cmd.timestamp = self._px4_timestamp_us()
        self._px4_vehicle_cmd_pub.publish(cmd)

    def _publish_px4_cmd_vel_setpoint(self):
        if self._latest_cmd_vel_mode is None or self._latest_cmd_vel_setpoint is None:
            return
        self._latest_cmd_vel_mode.timestamp = self._px4_timestamp_us()
        self._latest_cmd_vel_setpoint.timestamp = self._px4_timestamp_us()
        self._px4_offboard_mode_pub.publish(self._latest_cmd_vel_mode)
        self._px4_trajectory_pub.publish(self._latest_cmd_vel_setpoint)

    def _px4_cmd_vel_timer_callback(self):
        if self._px4_offboard_active:
            self._publish_px4_cmd_vel_setpoint()

    def _px4_attitude_callback(self, msg):
        self._latest_attitude = msg

    def _px4_sensor_callback(self, msg):
        self._latest_sensor_combined = msg

    def _px4_local_pos_callback(self, msg):
        self._latest_local_pos = msg

    def _px4_status_callback(self, msg):
        """PX4 VehicleStatus → /control/drone_state + 缓存状态"""
        self._latest_vehicle_status = msg  # 缓存用于 cmd_vel 自动 arm
        if not self._publish_drone_state:
            return
        try:
            state = vehicle_status_to_drone_state(msg)
            self._state_pub.publish(state)
        except Exception as e:
            self.get_logger().warn(f"PX4 状态转换失败: {e}")

    def _px4_timer_callback(self):
        """10Hz 定时器：合成并发布 IMU 和 Odometry"""
        if self._latest_sensor_combined is not None and self._latest_attitude is not None:
            try:
                imu = build_imu_from_sensor_combined(
                    self._latest_sensor_combined,
                    self._latest_attitude,
                )
                self._imu_pub.publish(imu)
            except Exception as e:
                self.get_logger().warn(f"PX4 IMU 合成失败: {e}")

        if self._latest_local_pos is not None:
            try:
                odom = vehicle_local_position_to_ros(
                    self._latest_local_pos,
                    self._latest_attitude,
                )
                self._publish_odometry(odom)
            except Exception as e:
                self.get_logger().warn(f"PX4 Odometry 合成失败: {e}")

    # ============================================================
    # cmd_vel 处理（两种模式）
    # ============================================================

    def _cmd_vel_callback(self, msg: Twist):
        """速度指令：转发到对应后端"""
        if self._mode == "airsim":
            self.get_logger().info(
                f"cmd_vel: vx={msg.linear.x:.1f} vy={msg.linear.y:.1f} "
                f"vz={msg.linear.z:.1f} yaw={msg.angular.z:.1f}"
            )
            self._cmd_vel_to_airsim(msg)
        else:
            self._cmd_vel_to_px4(msg)

    def _publish_odometry(self, odom: Odometry):
        self._odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id or "base_link"
        transform.transform.translation.x = float(odom.pose.pose.position.x)
        transform.transform.translation.y = float(odom.pose.pose.position.y)
        transform.transform.translation.z = float(odom.pose.pose.position.z)
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

    def _cmd_vel_to_airsim(self, msg: Twist):
        """速度指令 → AirSim moveByVelocityAsync (ENU → NED 转换)"""
        if self._client is None:
            return
        try:
            # AirSim API 使用 NED 坐标系，ROS Twist 使用 ENU
            # ENU (x=东, y=北, z=上) → NED (x=北, y=东, z=下)
            self._client.moveByVelocityAsync(
                msg.linear.y, msg.linear.x, -msg.linear.z,
                duration=0.1,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(
                    is_rate=True, yaw_or_rate=msg.angular.z
                ),
            )
        except Exception as e:
            self.get_logger().warn(f"AirSim 速度指令发送失败: {e}")

    def _cmd_vel_to_px4(self, msg: Twist):
        """速度指令 → PX4 OffboardControlMode + TrajectorySetpoint

        模仿 lesson3 move_velocity.py 逻辑：
        - 未解锁 + 上升指令 → 自动 arm + offboard
        - 已 offboard → 发布速度设定点
        """
        if not HAS_PX4_MSGS:
            return

        # 检查是否需要自动 arm
        status = getattr(self, '_latest_vehicle_status', None)
        if status is not None:
            # arming_state: 1=DISARMED, 2=ARMED; nav_state: 14=OFFBOARD
            if status.arming_state == 1:  # 未解锁
                if msg.linear.z > 1.0 or msg.linear.y > 1.0:  # 上升(ENU +Z)或前进(ENU +Y)
                    self.get_logger().info("自动解锁 + 切换 Offboard 模式")
                    self._prepare_cmd_vel_offboard(msg)
                    self._publish_px4_vehicle_cmd(self._make_arm_cmd(True))
                    time.sleep(0.2)
                    self._publish_px4_vehicle_cmd(self._make_offboard_cmd())
                    self._px4_offboard_active = True
                    return  # 等下一帧再发速度
                return  # 未解锁且无上升指令，不发
            if status.nav_state != 14 and self._has_motion_command(msg):
                self.get_logger().info("切换 Offboard 模式用于键盘控制")
                self._prepare_cmd_vel_offboard(msg)
                self._publish_px4_vehicle_cmd(self._make_offboard_cmd())
                self._px4_offboard_active = True
                return

        try:
            mode = cmd_vel_to_offboard_control(msg)
            sp = cmd_vel_to_trajectory_setpoint(msg)
            self._latest_cmd_vel_mode = mode
            self._latest_cmd_vel_setpoint = sp
            self._px4_offboard_active = True
            self._publish_px4_cmd_vel_setpoint()
        except Exception as e:
            self.get_logger().warn(f"PX4 速度指令发送失败: {e}")

    @staticmethod
    def _has_motion_command(msg: Twist) -> bool:
        return (
            abs(msg.linear.x) > 0.01
            or abs(msg.linear.y) > 0.01
            or abs(msg.linear.z) > 0.01
            or abs(msg.angular.z) > 0.01
        )

    def _prepare_cmd_vel_offboard(self, msg: Twist):
        self._latest_cmd_vel_mode = cmd_vel_to_offboard_control(msg)
        self._latest_cmd_vel_setpoint = cmd_vel_to_trajectory_setpoint(msg)
        for _ in range(10):
            self._publish_px4_cmd_vel_setpoint()
            time.sleep(0.1)

    @staticmethod
    def _make_arm_cmd(arm: bool):
        """构造解锁 VehicleCommand"""
        from px4_msgs.msg import VehicleCommand as VCmd
        cmd = VCmd()
        cmd.timestamp = 0
        cmd.param1 = 1.0 if arm else 0.0
        cmd.command = 400  # VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        return cmd

    @staticmethod
    def _make_offboard_cmd():
        """构造切换 Offboard 模式 VehicleCommand"""
        from px4_msgs.msg import VehicleCommand as VCmd
        cmd = VCmd()
        cmd.timestamp = 0
        cmd.param1 = 1.0
        cmd.param2 = 6.0
        cmd.command = 176  # VEHICLE_CMD_DO_SET_MODE
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        return cmd

    # ============================================================
    # 空消息（AirSim 未连接时的降级输出）
    # ============================================================

    def _publish_empty_all(self, now):
        odom = Odometry()
        odom.header = Header(stamp=now, frame_id="odom")
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        self._publish_odometry(odom)

        imu = Imu()
        imu.header = Header(stamp=now, frame_id="imu_link")
        imu.orientation.w = 1.0
        self._imu_pub.publish(imu)

        drone_state = DroneState()
        drone_state.armed = False
        drone_state.mode = "UNKNOWN"
        drone_state.battery = 0.0
        drone_state.gps_fix = 0
        drone_state.ekf_healthy = False
        self._state_pub.publish(drone_state)


def main(args=None):
    rclpy.init(args=args)
    node = AirSimBridgeNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
