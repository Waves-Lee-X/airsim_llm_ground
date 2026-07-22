#!/usr/bin/env python3
"""Real-time local autonomy node.

Pipeline:
  depth/odom/goal -> local voxel ESDF query -> kinodynamic replanning
  -> quintic smooth trajectory -> optional /control/cmd_vel safety command.

This is the first working foundation for the intended VIO/SLAM + ESDF +
Kinodynamic Replanning + Minimum Snap stack. The mapping and optimizer are
kept behind small modules so they can later be replaced by Voxblox/FIESTA and
a full minimum-snap solver without changing ROS-facing contracts.
"""

from __future__ import annotations

import json
import math
import struct
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from aeromind_interfaces.action import FollowWaypoints
from aeromind_interfaces.msg import (
    AutonomyStatus,
    Trajectory,
    TrajectoryPoint,
    WorldModelEvent,
)

from .frame_transform import body_to_world
from .kinodynamic_replanner import KinodynamicReplanner, ReplanResult
from .local_esdf import LocalEsdfMap
from .minimum_snap import sample_minimum_snap_waypoints, splice_replanned_trajectory
from .odometry_quality import odometry_quality_issue
from .semantic_guard import SemanticTrajectoryGuard


class AutonomyNode(Node):
    def __init__(self):
        super().__init__("autonomy_node")

        self.declare_parameter("enabled", True)
        self.declare_parameter("publish_control_cmd", False)
        self.declare_parameter("depth_topic", "/sensor/camera/depth/front_center")
        self.declare_parameter("depth_info_topic", "/sensor/camera/depth/camera_info")
        self.declare_parameter("require_camera_info", True)
        self.declare_parameter("odom_topic", "/sensor/odometry")
        self.declare_parameter("goal_topic", "/autonomy/goal")
        self.declare_parameter("world_cloud_topic", "")
        self.declare_parameter("world_cloud_stride", 3)
        self.declare_parameter("require_depth_tf", True)
        self.declare_parameter("map_publish_interval_sec", 0.5)
        self.declare_parameter("control_topic", "/control/cmd_vel")
        self.declare_parameter("map_resolution", 0.35)
        self.declare_parameter("map_rolling_radius", 24.0)
        self.declare_parameter("map_decay_sec", 4.0)
        self.declare_parameter("safety_radius", 1.2)
        self.declare_parameter("max_speed", 2.0)
        self.declare_parameter("max_acceleration", 1.5)
        self.declare_parameter("strategy_switch_penalty", 0.8)
        self.declare_parameter("min_flight_altitude", 1.0)
        self.declare_parameter("horizon_sec", 2.5)
        self.declare_parameter("depth_fov_deg", 95.0)
        self.declare_parameter("depth_stride", 8)
        self.declare_parameter("min_depth_m", 0.7)
        self.declare_parameter("max_depth_m", 18.0)
        self.declare_parameter("depth_lateral_limit_m", 8.0)
        self.declare_parameter("depth_vertical_limit_m", 4.0)
        self.declare_parameter("obstacle_vertical_corridor_m", 2.0)
        self.declare_parameter("arrival_distance_m", 0.6)
        self.declare_parameter("arrival_speed_mps", 0.3)
        self.declare_parameter("blocked_exit_clearance_m", 1.6)
        self.declare_parameter("recovery_lateral_m", 5.0)
        self.declare_parameter("recovery_climb_m", 3.0)
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", 0.0)
        self.declare_parameter("use_param_target", False)
        self.declare_parameter("blocked_timeout_sec", 3.0)
        self.declare_parameter("odom_timeout_sec", 1.0)
        self.declare_parameter("depth_timeout_sec", 3.5)
        self.declare_parameter("require_depth_for_flight", True)
        self.declare_parameter("odom_jump_reset_m", 3.0)
        self.declare_parameter("max_position_variance", 2.0)
        self.declare_parameter("max_orientation_variance", 0.5)
        self.declare_parameter("waypoint_sample_dt", 0.1)
        self.declare_parameter("waypoint_max_count", 64)
        self.declare_parameter("waypoint_timeout_margin_sec", 20.0)
        self.declare_parameter("waypoint_collision_lookahead_sec", 3.0)
        self.declare_parameter("waypoint_max_replans", 12)
        self.declare_parameter("semantic_guard_enabled", True)
        self.declare_parameter("semantic_guard_clear_dwell_sec", 1.5)
        self.declare_parameter("semantic_guard_hold_timeout_sec", 30.0)
        self.declare_parameter("takeoff_zone_radius_m", 2.0)
        self.declare_parameter("takeoff_ground_exclusion_m", 0.35)
        self.declare_parameter("takeoff_check_height_m", 5.0)
        self.declare_parameter("takeoff_clearance_max_m", 12.0)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._publish_control_cmd = bool(self.get_parameter("publish_control_cmd").value)
        self._depth_fov_deg = float(self.get_parameter("depth_fov_deg").value)
        self._depth_stride = max(1, int(self.get_parameter("depth_stride").value))
        self._min_depth_m = float(self.get_parameter("min_depth_m").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)
        self._depth_lateral_limit_m = float(self.get_parameter("depth_lateral_limit_m").value)
        self._depth_vertical_limit_m = float(self.get_parameter("depth_vertical_limit_m").value)
        self._obstacle_vertical_corridor_m = max(
            0.5,
            float(self.get_parameter("obstacle_vertical_corridor_m").value),
        )
        self._blocked_timeout_sec = float(self.get_parameter("blocked_timeout_sec").value)
        self._odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self._depth_timeout_sec = float(self.get_parameter("depth_timeout_sec").value)
        self._require_depth = bool(self.get_parameter("require_depth_for_flight").value)
        self._takeoff_zone_radius = float(self.get_parameter("takeoff_zone_radius_m").value)
        self._takeoff_ground_exclusion = float(self.get_parameter("takeoff_ground_exclusion_m").value)
        self._takeoff_check_height = float(self.get_parameter("takeoff_check_height_m").value)
        self._takeoff_clearance_max = float(self.get_parameter("takeoff_clearance_max_m").value)
        self._odom_jump_reset_m = float(self.get_parameter("odom_jump_reset_m").value)
        self._recovery_lateral_m = float(self.get_parameter("recovery_lateral_m").value)
        self._recovery_climb_m = float(self.get_parameter("recovery_climb_m").value)
        self._goal_world = None
        self._primary_goal_world = None
        self._latest_odom = None
        self._latest_depth_stamp = None
        self._latest_depth_info = None
        self._latest_odom_received = None
        self._latest_depth_received = None
        self._previous_odom_position = None
        self._blocked_since = None
        self._last_terminal_state = None
        self._recovery_attempt = 0
        self._recovering = False
        self._recovery_strategy = ""
        self._latest_image_analysis = None
        self._last_map_publish = 0.0
        self._action_lock = threading.RLock()
        self._map_lock = threading.RLock()
        self._active_waypoint_action = None
        self._waypoint_goal_reserved = False
        self._semantic_guard_enabled = bool(
            self.get_parameter("semantic_guard_enabled").value
        )
        self._semantic_guard = SemanticTrajectoryGuard(
            clear_dwell_sec=float(
                self.get_parameter("semantic_guard_clear_dwell_sec").value
            ),
            hold_timeout_sec=float(
                self.get_parameter("semantic_guard_hold_timeout_sec").value
            ),
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        resolution = float(self.get_parameter("map_resolution").value)
        safety_radius = float(self.get_parameter("safety_radius").value)
        max_speed = float(self.get_parameter("max_speed").value)
        horizon_sec = float(self.get_parameter("horizon_sec").value)
        self._esdf = LocalEsdfMap(
            resolution=resolution,
            rolling_radius=float(self.get_parameter("map_rolling_radius").value),
            decay_sec=float(self.get_parameter("map_decay_sec").value),
        )
        self._replanner = KinodynamicReplanner(
            safety_radius=safety_radius,
            max_speed=max_speed,
            horizon_sec=horizon_sec,
            arrival_distance=float(self.get_parameter("arrival_distance_m").value),
            arrival_speed=float(self.get_parameter("arrival_speed_mps").value),
            blocked_exit_clearance=float(self.get_parameter("blocked_exit_clearance_m").value),
            max_acceleration=float(self.get_parameter("max_acceleration").value),
            min_altitude=float(self.get_parameter("min_flight_altitude").value),
            strategy_switch_penalty=float(
                self.get_parameter("strategy_switch_penalty").value
            ),
        )

        if bool(self.get_parameter("use_param_target").value):
            self._goal_world = (
                float(self.get_parameter("target_x").value),
                float(self.get_parameter("target_y").value),
                float(self.get_parameter("target_z").value),
            )
            self._primary_goal_world = self._goal_world

        depth_topic = self.get_parameter("depth_topic").value
        depth_info_topic = self.get_parameter("depth_info_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        goal_topic = self.get_parameter("goal_topic").value
        control_topic = self.get_parameter("control_topic").value

        self.create_subscription(Image, depth_topic, self._depth_callback, 10)
        self.create_subscription(
            CameraInfo,
            depth_info_topic,
            self._depth_info_callback,
            10,
        )
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.create_subscription(PoseStamped, goal_topic, self._goal_callback, 10)
        self.create_subscription(Empty, "/autonomy/cancel", self._cancel_callback, 10)
        self.create_subscription(String, "/perception/image_analysis", self._image_analysis_callback, 10)
        self.create_subscription(
            WorldModelEvent,
            "/world_model/events",
            self._world_event_callback,
            10,
        )
        world_cloud_topic = str(self.get_parameter("world_cloud_topic").value).strip()
        self._world_cloud_sub = None
        if world_cloud_topic:
            self._world_cloud_sub = self.create_subscription(
                PointCloud2,
                world_cloud_topic,
                self._world_cloud_callback,
                10,
            )

        self._trajectory_pub = self.create_publisher(Trajectory, "/autonomy/trajectory", 10)
        self._status_pub = self.create_publisher(AutonomyStatus, "/autonomy/status", 10)
        self._map_pub = self.create_publisher(
            PointCloud2,
            "/autonomy/esdf_obstacles",
            10,
        )
        self._cmd_pub = self.create_publisher(Twist, control_topic, 10)
        self._waypoint_action_server = ActionServer(
            self,
            FollowWaypoints,
            "/autonomy/follow_waypoints",
            execute_callback=self._execute_waypoints,
            goal_callback=self._waypoint_goal_callback,
            cancel_callback=self._waypoint_cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self._timer = self.create_timer(0.1, self._timer_callback)
        self.get_logger().info(
            "自主避障节点已启动: "
            f"enabled={self._enabled}, publish_control_cmd={self._publish_control_cmd}, "
            f"semantic_guard={self._semantic_guard_enabled}"
        )

    def _world_event_callback(self, msg: WorldModelEvent):
        if not self._semantic_guard_enabled:
            return
        if msg.event_type not in {"person_entered_path", "person_cleared_path"}:
            return
        now = time.monotonic()
        with self._action_lock:
            self._semantic_guard.update_event(
                msg.event_type,
                bool(msg.active),
                list(msg.object_ids),
                now,
            )
        if msg.event_type == "person_entered_path" and msg.active:
            self.get_logger().warn(
                "语义保护触发：人员进入剩余航迹，连续航点任务保持悬停"
            )

    def _odom_callback(self, msg: Odometry):
        quality_issue = odometry_quality_issue(
            msg,
            max_position_variance=float(
                self.get_parameter("max_position_variance").value
            ),
            max_orientation_variance=float(
                self.get_parameter("max_orientation_variance").value
            ),
        )
        if quality_issue:
            self.get_logger().warn(
                f"拒绝低质量里程计: {quality_issue}",
                throttle_duration_sec=2.0,
            )
            return
        position = self._odom_position(msg)
        if (
            self._previous_odom_position is not None
            and math.dist(position, self._previous_odom_position) > self._odom_jump_reset_m
        ):
            with self._map_lock:
                self._esdf.clear()
            self.get_logger().warn("检测到里程计位姿跳变，已清空局部障碍地图")
        self._latest_odom = msg
        self._latest_odom_received = time.monotonic()
        self._previous_odom_position = position

    def _goal_callback(self, msg: PoseStamped):
        self._request_waypoint_action_stop("新的单点目标已接管控制")
        self._goal_world = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )
        self._primary_goal_world = self._goal_world
        self._blocked_since = None
        self._last_terminal_state = None
        self._recovery_attempt = 0
        self._recovering = False
        self._recovery_strategy = ""
        self._reset_replanner_blocked()
        self.get_logger().info(
            f"收到自主飞行目标: ({self._goal_world[0]:.2f}, "
            f"{self._goal_world[1]:.2f}, {self._goal_world[2]:.2f})"
        )

    def _cancel_callback(self, _msg: Empty):
        self._request_waypoint_action_stop("收到 /autonomy/cancel")
        self._goal_world = None
        self._primary_goal_world = None
        self._blocked_since = None
        self._last_terminal_state = "CANCELED"
        self._recovery_attempt = 0
        self._recovering = False
        self._recovery_strategy = ""
        self._reset_replanner_blocked()
        self.get_logger().info("收到自主任务取消请求，已清空目标并保持悬停")

    def _depth_callback(self, msg: Image):
        if self._latest_odom is None:
            return
        if (
            bool(self.get_parameter("require_camera_info").value)
            and self._latest_depth_info is None
        ):
            self.get_logger().warn(
                "等待深度相机 CameraInfo，暂不融合深度图",
                throttle_duration_sec=5.0,
            )
            return
        optical_points = self._depth_to_optical_points(msg)
        transform = self._lookup_depth_transform(msg)
        if transform is not None:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            sensor_origin = (
                float(translation.x),
                float(translation.y),
                float(translation.z),
            )
            orientation = (
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            )
            world_points = [
                body_to_world(point, sensor_origin, orientation)
                for point in optical_points
            ]
        elif bool(self.get_parameter("require_depth_tf").value):
            return
        else:
            sensor_origin = self._odom_position(self._latest_odom)
            orientation = self._odom_orientation(self._latest_odom)
            body_points = [(point[2], -point[0], -point[1]) for point in optical_points]
            world_points = [
                body_to_world(point, sensor_origin, orientation)
                for point in body_points
            ]
        vertical_band = self._active_vertical_band()
        if vertical_band is not None:
            minimum_z, maximum_z = vertical_band
            world_points = [
                point
                for point in world_points
                if minimum_z <= point[2] <= maximum_z
            ]
        received = time.monotonic()
        with self._map_lock:
            self._esdf.insert_points(
                world_points,
                stamp=received,
                sensor_origin=sensor_origin,
            )
        self._latest_depth_stamp = msg.header.stamp
        self._latest_depth_received = received

    def _depth_info_callback(self, msg: CameraInfo):
        if msg.width > 0 and msg.height > 0 and len(msg.k) == 9 and msg.k[0] > 0.0:
            self._latest_depth_info = msg

    def _image_analysis_callback(self, msg: String):
        try:
            self._latest_image_analysis = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("收到无法解析的 /perception/image_analysis")

    def _world_cloud_callback(self, msg: PointCloud2):
        if self._latest_odom is None:
            return
        odom_frame = str(self._latest_odom.header.frame_id).lstrip("/")
        cloud_frame = str(msg.header.frame_id).lstrip("/")
        if not cloud_frame or cloud_frame != odom_frame:
            self.get_logger().warn(
                f"忽略世界点云：frame_id={cloud_frame or '--'}，"
                f"里程计 frame_id={odom_frame or '--'}",
                throttle_duration_sec=5.0,
            )
            return
        stride = max(1, int(self.get_parameter("world_cloud_stride").value))
        points = []
        for index, point in enumerate(
            point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        ):
            if index % stride:
                continue
            points.append((float(point[0]), float(point[1]), float(point[2])))
            if len(points) >= self._esdf.max_points:
                break
        vertical_band = self._active_vertical_band()
        if vertical_band is not None:
            minimum_z, maximum_z = vertical_band
            points = [
                point for point in points if minimum_z <= point[2] <= maximum_z
            ]
        received = time.monotonic()
        with self._map_lock:
            self._esdf.insert_points(points, stamp=received)
        self._latest_depth_received = received

    def _timer_callback(self):
        position = self._world_position()
        velocity = self._local_velocity()
        goal = self._goal_world
        vertical_band = self._active_vertical_band(position, goal)
        with self._map_lock:
            self._esdf.prune(position, stamp=time.monotonic())
            if vertical_band is not None:
                self._esdf.prune_height_band(*vertical_band)

        with self._action_lock:
            waypoint_active = (
                self._active_waypoint_action is not None
                or self._waypoint_goal_reserved
            )
        if waypoint_active:
            self._publish_map_if_due()
            return

        if not self._enabled:
            with self._map_lock:
                result = self._replanner.replan(self._esdf, position, velocity, None)
            result.state = "DISABLED"
            result.message = "自主避障未启用，仅发布状态"
        elif not self._odom_fresh():
            with self._map_lock:
                result = self._sensor_wait_result(
                    position, velocity, "里程计不可用或已超时，保持悬停"
                )
        elif goal is not None and self._require_depth and not self._depth_fresh():
            with self._map_lock:
                result = self._sensor_wait_result(
                    position, velocity, "深度数据不可用或已超时，禁止盲飞"
                )
        else:
            with self._map_lock:
                result = self._replanner.replan(self._esdf, position, velocity, goal)

        result = self._apply_task_lifecycle(result)
        self._publish_trajectory(result)
        self._publish_status(result)
        self._publish_map_if_due()

        if self._enabled and self._publish_control_cmd:
            self._publish_cmd_vel(result.command_velocity)

    def _local_velocity(self):
        if self._latest_odom is None:
            return (0.0, 0.0, 0.0)
        vel = self._latest_odom.twist.twist.linear
        return (float(vel.x), float(vel.y), float(vel.z))

    def _active_vertical_band(self, position=None, goal=None):
        goal = self._goal_world if goal is None else goal
        if goal is None:
            return None
        position = self._world_position() if position is None else position
        margin = self._obstacle_vertical_corridor_m
        return (
            min(position[2], goal[2]) - margin,
            max(position[2], goal[2]) + margin,
        )

    def _world_position(self):
        if self._latest_odom is None:
            return (0.0, 0.0, 0.0)
        return self._odom_position(self._latest_odom)

    def _odom_fresh(self):
        return (
            self._latest_odom_received is not None
            and time.monotonic() - self._latest_odom_received <= self._odom_timeout_sec
        )

    def _depth_fresh(self):
        return (
            self._latest_depth_received is not None
            and time.monotonic() - self._latest_depth_received <= self._depth_timeout_sec
        )

    def _sensor_wait_result(self, position, velocity, message):
        result = self._replanner.replan(self._esdf, position, velocity, None)
        result.state = "SENSOR_WAIT"
        result.strategy = "sensor_timeout_hold"
        result.collision_free = False
        result.message = message
        if self._goal_world is not None:
            result.target_distance = math.dist(position, self._goal_world)
        return result

    def _apply_task_lifecycle(self, result):
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if result.strategy == "goal_reached":
            if self._recovering and self._primary_goal_world is not None:
                self._goal_world = self._primary_goal_world
                self._recovering = False
                self._blocked_since = None
                self._reset_replanner_blocked()
                result.state = "RECOVERY"
                result.strategy = "recovery_resume"
                result.message = "恢复点已到达，继续前往原始目标"
                return result
            self._goal_world = None
            self._primary_goal_world = None
            self._blocked_since = None
            self._last_terminal_state = "ARRIVED"
            self._recovery_attempt = 0
            self._recovering = False
            self._recovery_strategy = ""
            result.message = "自主任务完成：已到达目标附近，目标已清空"
            return result

        if result.strategy == "safe_hover" or result.state == "BLOCKED":
            if self._blocked_since is None:
                self._blocked_since = now
            elif now - self._blocked_since >= self._blocked_timeout_sec:
                recovery = self._next_recovery_goal()
                if recovery is not None:
                    self._goal_world = recovery["goal"]
                    self._blocked_since = None
                    self._recovering = True
                    self._recovery_strategy = recovery["strategy"]
                    self._reset_replanner_blocked()
                    result.state = "RECOVERY"
                    result.strategy = recovery["strategy"]
                    result.collision_free = True
                    result.message = recovery["message"]
                    self.get_logger().warn(recovery["message"])
                    return result
                self._goal_world = None
                self._primary_goal_world = None
                self._last_terminal_state = "BLOCKED_HOLD"
                self._recovering = False
                self._recovery_strategy = ""
                result.state = "BLOCKED_HOLD"
                result.strategy = "blocked_hold"
                result.collision_free = False
                result.message = "自主任务阻塞：恢复策略已用尽，已清空目标并保持悬停"
            return result

        self._blocked_since = None
        return result

    def _next_recovery_goal(self):
        if self._latest_odom is None or self._primary_goal_world is None:
            return None
        pos = self._latest_odom.pose.pose.position
        current = (float(pos.x), float(pos.y), float(pos.z))
        goal = self._primary_goal_world
        forward_x = goal[0] - current[0]
        forward_y = goal[1] - current[1]
        norm = math.hypot(forward_x, forward_y)
        if norm < 0.1:
            forward_x, forward_y, norm = 0.0, 1.0, 1.0
        forward = (forward_x / norm, forward_y / norm)
        left = (-forward[1], forward[0])
        right = (forward[1], -forward[0])
        strategies = self._ordered_recovery_strategies([
            {
                "strategy": "recovery_right",
                "goal": (
                    current[0] + right[0] * self._recovery_lateral_m,
                    current[1] + right[1] * self._recovery_lateral_m,
                    current[2],
                ),
                "message": f"自主任务阻塞：尝试右绕 {self._recovery_lateral_m:.1f} m",
                "keywords": ("右", "右侧", "右绕", "right"),
            },
            {
                "strategy": "recovery_left",
                "goal": (
                    current[0] + left[0] * self._recovery_lateral_m,
                    current[1] + left[1] * self._recovery_lateral_m,
                    current[2],
                ),
                "message": f"自主任务阻塞：尝试左绕 {self._recovery_lateral_m:.1f} m",
                "keywords": ("左", "左侧", "左绕", "left"),
            },
            {
                "strategy": "recovery_climb",
                "goal": (
                    current[0],
                    current[1],
                    current[2] + self._recovery_climb_m,
                ),
                "message": f"自主任务阻塞：尝试上升 {self._recovery_climb_m:.1f} m",
                "keywords": ("上", "上方", "上升", "爬升", "越过", "climb", "up"),
            },
        ])
        if self._recovery_attempt >= len(strategies):
            return None
        item = strategies[self._recovery_attempt]
        self._recovery_attempt += 1
        return {
            "strategy": item["strategy"],
            "goal": item["goal"],
            "message": self._semantic_recovery_message(item["message"]),
        }

    def _ordered_recovery_strategies(self, strategies):
        text = self._semantic_recovery_text()
        if not text:
            return strategies
        for index, item in enumerate(strategies):
            if any(keyword.lower() in text for keyword in item["keywords"]):
                return [item] + strategies[:index] + strategies[index + 1 :]
        return strategies

    def _semantic_recovery_text(self):
        analysis = self._latest_image_analysis or {}
        parts = [
            str(analysis.get("suggestion", "")),
            str(analysis.get("scene", "")),
            str(analysis.get("message", "")),
        ]
        return " ".join(parts).lower()

    def _semantic_recovery_message(self, base_message: str):
        analysis = self._latest_image_analysis or {}
        suggestion = str(analysis.get("suggestion", "")).strip()
        if suggestion:
            return f"{base_message}（参考图像语义建议：{suggestion}）"
        return base_message

    def _reset_replanner_blocked(self):
        if hasattr(self._replanner, "_blocked_latched"):
            self._replanner._blocked_latched = False

    def _publish_cmd_vel(self, velocity):
        msg = Twist()
        msg.linear.x = float(velocity[0])
        msg.linear.y = float(velocity[1])
        msg.linear.z = float(velocity[2])
        msg.angular.z = 0.0
        self._cmd_pub.publish(msg)

    def _publish_trajectory(self, result, trajectory_id="", reset_time=False):
        msg = Trajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._latest_odom.header.frame_id if self._latest_odom else "map"
        msg.collision_free = bool(result.collision_free)
        msg.planner_mode = result.strategy
        msg.message = result.message
        msg.trajectory_id = str(trajectory_id)
        msg.reset_time = bool(reset_time)
        for sample in result.trajectory:
            point = TrajectoryPoint()
            sec = int(sample["t"])
            nanosec = int((sample["t"] - sec) * 1e9)
            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nanosec
            world_position = sample["position"]
            point.position.x = world_position[0]
            point.position.y = world_position[1]
            point.position.z = world_position[2]
            point.velocity = self._vec3(sample["velocity"])
            point.acceleration = self._vec3(sample["acceleration"])
            point.yaw = 0.0
            msg.points.append(point)
        self._trajectory_pub.publish(msg)

    def _waypoint_goal_callback(self, goal_request):
        waypoint_count = len(goal_request.relative_waypoints)
        maximum = int(self.get_parameter("waypoint_max_count").value)
        with self._action_lock:
            busy = self._active_waypoint_action is not None or self._waypoint_goal_reserved
        if busy or waypoint_count < 1 or waypoint_count > maximum:
            return GoalResponse.REJECT
        if not math.isfinite(float(goal_request.cruise_speed)) or goal_request.cruise_speed <= 0.0:
            return GoalResponse.REJECT
        for waypoint in goal_request.relative_waypoints:
            if not all(math.isfinite(value) for value in (waypoint.x, waypoint.y, waypoint.z)):
                return GoalResponse.REJECT
        with self._action_lock:
            self._waypoint_goal_reserved = True
        return GoalResponse.ACCEPT

    def _waypoint_cancel_callback(self, _goal_handle):
        self._request_waypoint_action_stop("Action 客户端请求取消")
        return CancelResponse.ACCEPT

    def _request_waypoint_action_stop(self, reason):
        with self._action_lock:
            if self._active_waypoint_action is not None:
                self._active_waypoint_action["stop_reason"] = str(reason)

    def _execute_waypoints(self, goal_handle):
        result = FollowWaypoints.Result()
        if not self._enabled or not self._odom_fresh():
            result.message = "自主系统未启用或里程计不可用"
            self._release_waypoint_reservation()
            goal_handle.abort()
            return result
        if self._require_depth and not self._depth_fresh():
            result.message = "深度数据不可用，拒绝连续多航点盲飞"
            self._release_waypoint_reservation()
            goal_handle.abort()
            return result

        start = self._world_position()
        yaw = self._odom_yaw(self._latest_odom)
        world_waypoints = self._relative_waypoints_to_world(
            start, yaw, goal_handle.request.relative_waypoints
        )
        try:
            samples, waypoint_times = sample_minimum_snap_waypoints(
                start,
                world_waypoints,
                min(float(goal_handle.request.cruise_speed), self._replanner.max_speed),
                sample_dt=float(self.get_parameter("waypoint_sample_dt").value),
                start_velocity=self._local_velocity(),
            )
        except ValueError as exc:
            result.message = str(exc)
            self._release_waypoint_reservation()
            goal_handle.abort()
            return result

        trajectory_id = f"waypoints-{uuid.uuid4().hex}"
        started_at = time.monotonic()
        planned_duration = waypoint_times[-1]
        timeout_margin = float(self.get_parameter("waypoint_timeout_margin_sec").value)
        mission_deadline = started_at + max(
            planned_duration * 3.0,
            planned_duration + timeout_margin,
        )
        state = {
            "id": trajectory_id,
            "stop_reason": "",
            "samples": samples,
            "waypoint_times": waypoint_times,
            "world_waypoints": world_waypoints,
            "cruise_speed": min(
                float(goal_handle.request.cruise_speed), self._replanner.max_speed
            ),
            "started_at": started_at,
            "mission_deadline": mission_deadline,
            "completed": 0,
            "timeline_completed": 0,
            "progress": 0.0,
            "replan_count": 0,
            "strategy": "minimum_snap_waypoints",
            "reset_time": True,
        }
        with self._action_lock:
            self._active_waypoint_action = state
            self._waypoint_goal_reserved = False
            self._semantic_guard.begin_mission(started_at)
        self._goal_world = world_waypoints[-1]
        self._primary_goal_world = world_waypoints[-1]
        self.get_logger().info(
            f"开始连续多航点 Action: {len(world_waypoints)} 点, "
            f"duration={waypoint_times[-1]:.2f}s, id={trajectory_id}"
        )

        try:
            while rclpy.ok():
                now = time.monotonic()
                elapsed = now - state["started_at"]
                samples = state["samples"]
                with self._action_lock:
                    stop_reason = state["stop_reason"]
                if goal_handle.is_cancel_requested or stop_reason:
                    result.message = stop_reason or "连续多航点任务已取消"
                    result.completed_waypoints = state["completed"]
                    self._set_result_position(result, self._world_position())
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    return result
                if not self._odom_fresh() or (self._require_depth and not self._depth_fresh()):
                    result.message = "执行中传感器数据超时，已切换悬停"
                    result.completed_waypoints = state["completed"]
                    self._set_result_position(result, self._world_position())
                    goal_handle.abort()
                    return result

                position = self._world_position()
                self._update_completed_waypoints(state, position)
                semantic_decision = self._semantic_guard_decision(now)
                if semantic_decision in {"hold", "clearing"}:
                    state.setdefault("semantic_hold_elapsed", elapsed)
                    clearance = self._trajectory_clearance_window(
                        samples, position, state["semantic_hold_elapsed"]
                    )
                    self._publish_waypoint_hold(
                        state, trajectory_id, clearance, semantic_decision
                    )
                    self._publish_waypoint_feedback(
                        goal_handle,
                        state,
                        clearance,
                        "SEMANTIC_HOLD",
                    )
                    time.sleep(0.2)
                    continue
                if semantic_decision == "timeout":
                    result.message = "人员持续占用航迹超过语义保护时限，任务已悬停终止"
                    result.completed_waypoints = state["completed"]
                    self._set_result_position(result, position)
                    goal_handle.abort()
                    return result
                if semantic_decision == "resume":
                    with self._action_lock:
                        hold_duration = self._semantic_guard.consume_resume(now)
                    state["mission_deadline"] += hold_duration
                    state.pop("semantic_hold_elapsed", None)
                    resumed, message, new_clearance = self._replan_waypoint_route(state)
                    if not resumed:
                        result.message = f"语义风险解除，但剩余轨迹恢复失败：{message}"
                        result.completed_waypoints = state["completed"]
                        self._set_result_position(result, position)
                        goal_handle.abort()
                        return result
                    self.get_logger().info(
                        f"人员已离开航迹并稳定 {self._semantic_guard.clear_dwell_sec:.1f}s，"
                        "已重拼接剩余 Minimum Snap 轨迹"
                    )
                    self._publish_waypoint_trajectory(
                        state["samples"],
                        trajectory_id,
                        reset_time=True,
                        clearance=new_clearance,
                        strategy=state["strategy"],
                    )
                    state["reset_time"] = False
                    continue
                clearance = self._trajectory_clearance_window(samples, position, elapsed)
                if clearance < self._replanner.safety_radius:
                    maximum = int(self.get_parameter("waypoint_max_replans").value)
                    if state["replan_count"] >= maximum:
                        result.message = f"局部重规划已达到上限 {maximum} 次，已悬停"
                        result.completed_waypoints = state["completed"]
                        self._set_result_position(result, position)
                        goal_handle.abort()
                        return result
                    self._publish_waypoint_feedback(
                        goal_handle,
                        state,
                        clearance,
                        "REPLANNING",
                    )
                    self._publish_status(
                        self._waypoint_status_result(
                            state,
                            clearance,
                            math.dist(position, world_waypoints[-1]),
                            "REPLANNING",
                        )
                    )
                    replanned, message, new_clearance = self._replan_waypoint_route(state)
                    if not replanned:
                        result.message = message
                        result.completed_waypoints = state["completed"]
                        self._set_result_position(result, position)
                        goal_handle.abort()
                        return result
                    self._publish_waypoint_trajectory(
                        state["samples"],
                        trajectory_id,
                        reset_time=True,
                        clearance=new_clearance,
                        strategy=state["strategy"],
                    )
                    state["reset_time"] = False
                    continue

                self._publish_waypoint_trajectory(
                    (
                        samples
                        if state["reset_time"]
                        else self._remaining_trajectory_samples(samples, elapsed)
                    ),
                    trajectory_id,
                    reset_time=state["reset_time"],
                    clearance=clearance,
                    strategy=state["strategy"],
                )
                state["reset_time"] = False
                final_distance = math.dist(self._world_position(), world_waypoints[-1])
                self._publish_waypoint_feedback(
                    goal_handle, state, clearance, "TRACKING", final_distance
                )

                status = self._waypoint_status_result(
                    state, clearance, final_distance, "TRACKING"
                )
                self._publish_status(status)

                speed = math.sqrt(sum(value * value for value in self._local_velocity()))
                if (
                    state["completed"] >= len(world_waypoints)
                    and final_distance <= self._replanner.arrival_distance
                    and speed <= self._replanner.arrival_speed
                ):
                    result.success = True
                    result.message = f"连续多航点任务完成，共 {len(world_waypoints)} 个航点"
                    result.completed_waypoints = len(world_waypoints)
                    self._set_result_position(result, self._world_position())
                    goal_handle.succeed()
                    return result
                if now > state["mission_deadline"]:
                    result.message = "连续多航点任务超过执行时限"
                    result.completed_waypoints = state["completed"]
                    self._set_result_position(result, self._world_position())
                    goal_handle.abort()
                    return result
                time.sleep(0.2)
        finally:
            with self._action_lock:
                if self._active_waypoint_action is state:
                    self._active_waypoint_action = None
                self._waypoint_goal_reserved = False
                self._semantic_guard.end_mission()
            self._goal_world = None
            self._primary_goal_world = None

    def _publish_waypoint_trajectory(
        self, samples, trajectory_id, reset_time, clearance, strategy
    ):
        result = ReplanResult(
            state="TRACKING",
            strategy=strategy,
            collision_free=True,
            target_distance=math.dist(self._world_position(), self._goal_world),
            nearest_obstacle=clearance,
            command_velocity=(0.0, 0.0, 0.0),
            trajectory=samples,
            message="连续多航点 Minimum Snap 轨迹",
        )
        self._publish_trajectory(result, trajectory_id, reset_time)

    def _publish_waypoint_hold(self, state, trajectory_id, clearance, decision):
        object_count = len(self._semantic_guard.active_object_ids)
        result = ReplanResult(
            state="SEMANTIC_HOLD",
            strategy="semantic_person_hold",
            collision_free=True,
            target_distance=math.dist(self._world_position(), self._goal_world),
            nearest_obstacle=clearance,
            command_velocity=(0.0, 0.0, 0.0),
            trajectory=self._remaining_trajectory_samples(
                state["samples"], state.get("semantic_hold_elapsed", 0.0)
            ),
            message=(
                f"人员占用航迹，保持悬停（{object_count} 个活动目标）"
                if decision == "hold"
                else "人员已离开，等待稳定后恢复剩余轨迹"
            ),
        )
        self._publish_trajectory(result, trajectory_id, reset_time=False)
        self._publish_status(result)

    def _semantic_guard_decision(self, now):
        if not self._semantic_guard_enabled:
            return "tracking"
        with self._action_lock:
            return self._semantic_guard.decision(now)

    @staticmethod
    def _remaining_trajectory_samples(samples, elapsed):
        if len(samples) <= 2:
            return list(samples)
        start_index = 0
        for index, sample in enumerate(samples):
            if float(sample["t"]) >= max(0.0, float(elapsed) - 0.2):
                start_index = max(0, index - 1)
                break
        else:
            start_index = len(samples) - 2
        return list(samples[start_index:])

    def _replan_waypoint_route(self, state):
        completed = state["completed"]
        remaining_waypoints = state["world_waypoints"][completed:]
        if not remaining_waypoints:
            return False, "没有可重规划的剩余航点", 0.0
        position = self._world_position()
        velocity = self._local_velocity()
        with self._map_lock:
            local_result = self._replanner.replan(
                self._esdf, position, velocity, remaining_waypoints[0]
            )
        if local_result.state != "TRACKING" or not local_result.collision_free:
            return (
                False,
                f"局部重规划未找到安全绕行轨迹：{local_result.message}",
                float(local_result.nearest_obstacle),
            )
        try:
            samples, waypoint_times = splice_replanned_trajectory(
                local_result.trajectory,
                remaining_waypoints,
                state["cruise_speed"],
                sample_dt=float(self.get_parameter("waypoint_sample_dt").value),
                reached_tolerance=max(0.15, self._replanner.arrival_distance * 0.5),
            )
        except ValueError as exc:
            return False, f"重拼接轨迹失败：{exc}", 0.0
        clearance = self._trajectory_clearance_window(samples, position, 0.0)
        if clearance < self._replanner.safety_radius:
            return (
                False,
                f"重拼接轨迹安全距离仍只有 {clearance:.2f} m，已悬停",
                clearance,
            )
        state["samples"] = samples
        state["waypoint_times"] = waypoint_times
        state["started_at"] = time.monotonic()
        state["timeline_completed"] = completed
        state["replan_count"] += 1
        state["strategy"] = f"minimum_snap_replan:{local_result.strategy}"
        state["reset_time"] = True
        self.get_logger().warn(
            f"连续航点局部重规划 #{state['replan_count']}: "
            f"{local_result.strategy}, 剩余原始航点={len(remaining_waypoints)}, "
            f"clearance={clearance:.2f}m"
        )
        return True, local_result.message, clearance

    def _update_completed_waypoints(self, state, position):
        tolerance = max(0.8, self._replanner.arrival_distance * 1.5)
        waypoints = state["world_waypoints"]
        while (
            state["completed"] < len(waypoints)
            and math.dist(position, waypoints[state["completed"]]) <= tolerance
        ):
            state["completed"] += 1

    def _publish_waypoint_feedback(
        self, goal_handle, state, clearance, feedback_state, final_distance=None
    ):
        total = len(state["world_waypoints"])
        elapsed = max(
            0.0,
            float(
                state.get(
                    "semantic_hold_elapsed",
                    time.monotonic() - state["started_at"],
                )
            ),
        )
        duration = max(0.1, float(state["samples"][-1]["t"]))
        base = state["timeline_completed"]
        candidate = (base + min(1.0, elapsed / duration) * (total - base)) / total
        state["progress"] = max(state["progress"], min(1.0, candidate))
        feedback = FollowWaypoints.Feedback()
        feedback.current_waypoint = min(state["completed"] + 1, total)
        feedback.remaining_distance = float(
            final_distance
            if final_distance is not None
            else math.dist(self._world_position(), state["world_waypoints"][-1])
        )
        feedback.progress = float(state["progress"])
        feedback.state = feedback_state
        feedback.nearest_obstacle = float(clearance)
        goal_handle.publish_feedback(feedback)

    def _waypoint_status_result(self, state, clearance, final_distance, status_state):
        return ReplanResult(
            state=status_state,
            strategy=state["strategy"],
            collision_free=True,
            target_distance=final_distance,
            nearest_obstacle=clearance,
            command_velocity=(0.0, 0.0, 0.0),
            trajectory=state["samples"],
            message=(
                f"连续多航点 {state['completed']}/{len(state['world_waypoints'])}，"
                f"局部重规划 {state['replan_count']} 次"
            ),
        )

    def _release_waypoint_reservation(self):
        with self._action_lock:
            self._waypoint_goal_reserved = False

    def _trajectory_clearance_window(self, samples, position, elapsed):
        lookahead = max(
            0.5,
            float(self.get_parameter("waypoint_collision_lookahead_sec").value),
        )
        points = [
            sample["position"]
            for sample in samples
            if elapsed - 0.1 <= float(sample["t"]) <= elapsed + lookahead
            if math.dist(sample["position"], position) >= self._replanner.start_ignore_radius
        ]
        with self._map_lock:
            return self._esdf.trajectory_clearance(
                points or [position], max_radius=max(4.0, self._replanner.safety_radius * 2.5)
            )

    @staticmethod
    def _relative_waypoints_to_world(start, yaw, waypoints):
        current = tuple(start)
        result = []
        forward = (-math.sin(yaw), math.cos(yaw))
        right = (math.cos(yaw), math.sin(yaw))
        for point in waypoints:
            current = (
                current[0] + forward[0] * point.x + right[0] * point.y,
                current[1] + forward[1] * point.x + right[1] * point.y,
                current[2] + point.z,
            )
            result.append(current)
        return result

    @staticmethod
    def _odom_yaw(msg):
        orientation = msg.pose.pose.orientation
        siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def _set_result_position(result, position):
        result.final_position.x = float(position[0])
        result.final_position.y = float(position[1])
        result.final_position.z = float(position[2])

    def _publish_status(self, result):
        msg = AutonomyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.enabled = self._enabled
        msg.state = result.state
        msg.replanning = result.state in (
            "TRACKING",
            "REPLANNING",
            "BLOCKED",
            "RECOVERY",
        )
        msg.nearest_obstacle_m = float(result.nearest_obstacle)
        takeoff_valid = self._odom_fresh() and self._depth_fresh()
        if takeoff_valid:
            position = self._world_position()
            with self._map_lock:
                takeoff_clearance = self._esdf.takeoff_zone_clearance(
                    position,
                    horizontal_radius=self._takeoff_zone_radius,
                    ground_exclusion=self._takeoff_ground_exclusion,
                    check_height=self._takeoff_check_height,
                    max_distance=self._takeoff_clearance_max,
                )
            msg.takeoff_clearance_valid = True
            msg.takeoff_clearance_m = float(takeoff_clearance)
            msg.takeoff_clearance_source = "front_depth_esdf_ground_filtered"
        else:
            msg.takeoff_clearance_valid = False
            msg.takeoff_clearance_m = 0.0
            msg.takeoff_clearance_source = "unavailable"
        msg.target_distance_m = float(result.target_distance)
        msg.active_strategy = result.strategy
        msg.message = result.message
        self._status_pub.publish(msg)

    def _publish_map_if_due(self):
        now = time.monotonic()
        interval = max(
            0.1,
            float(self.get_parameter("map_publish_interval_sec").value),
        )
        if now - self._last_map_publish < interval:
            return
        self._last_map_publish = now
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = (
            self._latest_odom.header.frame_id if self._latest_odom else "odom"
        )
        with self._map_lock:
            points = self._esdf.points()
        self._map_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    def _lookup_depth_transform(self, msg: Image):
        target_frame = str(self._latest_odom.header.frame_id).strip()
        source_frame = str(msg.header.frame_id).strip()
        if not target_frame or not source_frame:
            self.get_logger().warn(
                "深度图或里程计缺少 frame_id，无法执行 TF 融合",
                throttle_duration_sec=5.0,
            )
            return None
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"等待深度 TF {target_frame} <- {source_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

    def _depth_to_optical_points(self, msg: Image):
        width = int(msg.width)
        height = int(msg.height)
        if width <= 0 or height <= 0 or not msg.data:
            return []

        encoding = msg.encoding.lower()
        if encoding in ("32fc1", "32fc"):
            fmt = "<f"
            byte_width = 4
            convert = float
        elif encoding in ("16uc1", "mono16"):
            fmt = "<H"
            byte_width = 2
            convert = lambda value: float(value) / 1000.0
        else:
            self.get_logger().warn(f"暂不支持深度编码: {msg.encoding}", throttle_duration_sec=5.0)
            return []

        step = int(msg.step) or width * byte_width
        info = self._latest_depth_info
        if info is not None and int(info.width) == width and int(info.height) == height:
            fx = float(info.k[0])
            fy = float(info.k[4])
            cx = float(info.k[2])
            cy = float(info.k[5])
        else:
            fov = math.radians(self._depth_fov_deg)
            fx = width / (2.0 * math.tan(fov / 2.0))
            fy = fx
            cx = (width - 1) / 2.0
            cy = (height - 1) / 2.0
        points = []
        for v in range(0, height, self._depth_stride):
            for u in range(0, width, self._depth_stride):
                offset = v * step + u * byte_width
                if offset + byte_width > len(msg.data):
                    continue
                try:
                    depth = convert(struct.unpack_from(fmt, msg.data, offset)[0])
                except (struct.error, ValueError):
                    continue
                if not math.isfinite(depth) or depth < self._min_depth_m or depth > self._max_depth_m:
                    continue
                x = (u - cx) * depth / fx
                y = (v - cy) * depth / fy
                z = depth
                if abs(x) > self._depth_lateral_limit_m or abs(y) > self._depth_vertical_limit_m:
                    continue
                points.append((x, y, z))
        return points

    @staticmethod
    def _odom_position(msg: Odometry):
        point = msg.pose.pose.position
        return (float(point.x), float(point.y), float(point.z))

    @staticmethod
    def _odom_orientation(msg: Odometry):
        value = msg.pose.pose.orientation
        return (float(value.x), float(value.y), float(value.z), float(value.w))

    @staticmethod
    def _vec3(value):
        msg = Vector3()
        msg.x = float(value[0])
        msg.y = float(value[1])
        msg.z = float(value[2])
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
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
