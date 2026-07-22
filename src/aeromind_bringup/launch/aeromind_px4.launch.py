# -*- coding: utf-8 -*-
"""
aeromind_px4.launch.py — AeroMind PX4 模式启动文件

启动所有节点，bridge 和 control 配置为 PX4 模式。
需要先启动：
  1. PX4 SITL（已连接 AirSim TCP:4560）
  2. MicroXRCE-DDS Agent（桥接 PX4 uORB ↔ ROS 2 DDS）

用法:
  ros2 launch aeromind_px4.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _env(name, default=""):
    return os.environ.get(name, "").strip() or default


def _get_windows_ip():
    """自动获取 Windows 宿主机 IP（WSL2 默认网关）"""
    import socket
    import subprocess

    try:
        result = subprocess.run(
            ["ip", "route", "show", "default"],
            capture_output=True, text=True, timeout=5
        )
        for line in result.stdout.splitlines():
            parts = line.split()
            if "via" in parts:
                idx = parts.index("via")
                ip = parts[idx + 1]
                socket.inet_aton(ip)
                return ip
    except Exception:
        pass
    return "192.168.1.100"


def generate_launch_description():
    """生成 PX4 模式 LaunchDescription"""

    default_ip = _get_windows_ip()
    airsim_ip_arg = DeclareLaunchArgument(
        "airsim_ip",
        default_value=default_ip,
        description="AirSim 宿主机 IP 地址（默认自动检测 WSL 网关）",
    )
    airsim_ip = LaunchConfiguration("airsim_ip")
    primary_rgbd_period_arg = DeclareLaunchArgument(
        "primary_rgbd_period_sec",
        default_value="0.1",
        description="AirSim 前视 RGB-D 批量发布周期",
    )
    lidar_period_arg = DeclareLaunchArgument(
        "lidar_period_sec",
        default_value="0.2",
        description="AirSim LiDAR 发布周期",
    )
    publish_aux_cameras_arg = DeclareLaunchArgument(
        "publish_aux_cameras",
        default_value="true",
        description="是否低频发布底视和左右辅助相机",
    )
    llm_enabled_arg = DeclareLaunchArgument(
        "llm_enabled",
        default_value="true",
        description="是否启用大模型解析（失败会自动回退规则解析）",
    )
    llm_api_url_arg = DeclareLaunchArgument(
        "llm_api_url",
        default_value=_env("AEROMIND_LLM_API_URL", "https://api.deepseek.com/v1"),
        description="LLM API 地址；默认读取 AEROMIND_LLM_API_URL",
    )
    llm_model_arg = DeclareLaunchArgument(
        "llm_model",
        default_value=_env("AEROMIND_LLM_MODEL", "deepseek-chat"),
        description="LLM 模型名；默认读取 AEROMIND_LLM_MODEL",
    )
    llm_api_key_arg = DeclareLaunchArgument(
        "llm_api_key",
        default_value="",
        description="可选的临时 LLM Key；留空时节点从环境变量安全读取",
    )
    llm_timeout_arg = DeclareLaunchArgument(
        "llm_timeout_sec",
        default_value="3.0",
        description="LLM 解析超时时间，单位秒",
    )
    autonomy_enabled_arg = DeclareLaunchArgument(
        "autonomy_enabled",
        default_value="true",
        description="是否启用底层自主避障重规划状态机",
    )
    autonomy_control_arg = DeclareLaunchArgument(
        "autonomy_control",
        default_value="false",
        description="是否允许自主避障节点直接发布 /control/cmd_vel 接管速度控制",
    )
    autonomy_trajectory_execution_arg = DeclareLaunchArgument(
        "autonomy_trajectory_execution",
        default_value="true",
        description="是否允许 control_node 执行 /autonomy/trajectory 平滑轨迹",
    )
    autonomy_velocity_limit_arg = DeclareLaunchArgument(
        "autonomy_velocity_limit",
        default_value="1.8",
        description="执行自主避障轨迹时的速度上限，单位 m/s",
    )
    autonomy_accel_limit_arg = DeclareLaunchArgument(
        "autonomy_accel_limit",
        default_value="1.0",
        description="执行自主避障轨迹时的加速度变化上限，单位 m/s^2",
    )
    autonomy_min_altitude_arg = DeclareLaunchArgument(
        "autonomy_min_altitude",
        default_value="1.0",
        description="自主轨迹执行时的最低高度保护，单位 m",
    )
    autonomy_blocked_timeout_arg = DeclareLaunchArgument(
        "autonomy_blocked_timeout_sec",
        default_value="3.0",
        description="连续阻塞多久后清空自主目标并结束任务",
    )
    autonomy_min_depth_arg = DeclareLaunchArgument(
        "autonomy_min_depth_m",
        default_value="0.7",
        description="深度避障最小有效距离，低于该值视为噪声",
    )
    autonomy_max_depth_arg = DeclareLaunchArgument(
        "autonomy_max_depth_m",
        default_value="18.0",
        description="深度避障最大有效距离",
    )
    autonomy_map_radius_arg = DeclareLaunchArgument(
        "autonomy_map_radius_m",
        default_value="24.0",
        description="世界坐标滚动障碍地图半径",
    )
    autonomy_map_decay_arg = DeclareLaunchArgument(
        "autonomy_map_decay_sec",
        default_value="4.0",
        description="未再次观测障碍体素的保留时间",
    )
    autonomy_odom_timeout_arg = DeclareLaunchArgument(
        "autonomy_odom_timeout_sec",
        default_value="1.0",
        description="里程计超时后进入悬停的时间阈值",
    )
    autonomy_depth_timeout_arg = DeclareLaunchArgument(
        "autonomy_depth_timeout_sec",
        default_value="3.5",
        description="深度数据超时后禁止继续自主飞行的时间阈值",
    )
    autonomy_waypoint_lookahead_arg = DeclareLaunchArgument(
        "autonomy_waypoint_lookahead_sec",
        default_value="3.0",
        description="连续多航点轨迹用于触发局部重规划的前视时间窗",
    )
    autonomy_waypoint_max_replans_arg = DeclareLaunchArgument(
        "autonomy_waypoint_max_replans",
        default_value="12",
        description="单次连续多航点任务允许的最大局部重规划次数",
    )
    autonomy_semantic_guard_arg = DeclareLaunchArgument(
        "autonomy_semantic_guard_enabled",
        default_value="true",
        description="连续航点任务是否响应人员进入航迹的语义事件",
    )
    autonomy_semantic_clear_dwell_arg = DeclareLaunchArgument(
        "autonomy_semantic_clear_dwell_sec",
        default_value="1.5",
        description="人员离开航迹后恢复飞行前的稳定等待时间",
    )
    autonomy_semantic_hold_timeout_arg = DeclareLaunchArgument(
        "autonomy_semantic_hold_timeout_sec",
        default_value="30.0",
        description="人员持续占用航迹时悬停等待的最长时间",
    )
    autonomy_odom_topic_arg = DeclareLaunchArgument(
        "autonomy_odom_topic",
        default_value="/sensor/odometry",
        description="PX4、VIO 或 SLAM 提供的 nav_msgs/Odometry 话题",
    )
    autonomy_world_cloud_topic_arg = DeclareLaunchArgument(
        "autonomy_world_cloud_topic",
        default_value="",
        description="可选的世界坐标注册 PointCloud2 话题",
    )
    autonomy_max_position_variance_arg = DeclareLaunchArgument(
        "autonomy_max_position_variance",
        default_value="2.0",
        description="VIO/SLAM 位置协方差拒绝阈值",
    )
    autonomy_max_orientation_variance_arg = DeclareLaunchArgument(
        "autonomy_max_orientation_variance",
        default_value="0.5",
        description="VIO/SLAM 姿态协方差拒绝阈值",
    )
    yolo_enabled_arg = DeclareLaunchArgument(
        "yolo_enabled",
        default_value="false",
        description="是否启动 YOLO 目标检测节点",
    )
    yolo_model_arg = DeclareLaunchArgument(
        "yolo_model",
        default_value="yolov8n.pt",
        description="YOLO 模型路径或模型名",
    )
    yolo_confidence_arg = DeclareLaunchArgument(
        "yolo_confidence",
        default_value="0.35",
        description="YOLO 检测置信度阈值",
    )
    semantic_world_enabled_arg = DeclareLaunchArgument(
        "semantic_world_enabled",
        default_value="true",
        description="是否启动检测+深度+TF 三维语义融合节点",
    )
    semantic_world_frame_arg = DeclareLaunchArgument(
        "semantic_world_frame",
        default_value="odom",
        description="语义世界对象的目标坐标系；SLAM 模式使用 map",
    )
    semantic_history_enabled_arg = DeclareLaunchArgument(
        "semantic_history_enabled",
        default_value="true",
        description="是否将语义对象观测持久化到 SQLite",
    )
    semantic_history_db_path_arg = DeclareLaunchArgument(
        "semantic_history_db_path",
        default_value="~/.aeromind/world_model.db",
        description="语义对象历史 SQLite 路径",
    )
    semantic_history_retention_hours_arg = DeclareLaunchArgument(
        "semantic_history_retention_hours",
        default_value="24.0",
        description="语义对象历史保留时长",
    )
    vlm_enabled_arg = DeclareLaunchArgument(
        "vlm_enabled",
        default_value="false",
        description="是否启用视觉语言模型图像语义分析",
    )
    vlm_api_url_arg = DeclareLaunchArgument(
        "vlm_api_url",
        default_value=_env("AEROMIND_VLM_API_URL"),
        description="VLM API 地址；默认读取 AEROMIND_VLM_API_URL",
    )
    vlm_model_arg = DeclareLaunchArgument(
        "vlm_model",
        default_value=_env("AEROMIND_VLM_MODEL"),
        description="视觉语言模型名；默认读取 AEROMIND_VLM_MODEL",
    )
    vlm_api_key_arg = DeclareLaunchArgument(
        "vlm_api_key",
        default_value="",
        description="可选的临时 VLM Key；留空时节点从环境变量安全读取",
    )
    auto_analyze_enabled_arg = DeclareLaunchArgument(
        "auto_analyze_enabled",
        default_value="false",
        description="是否启用 RGB 图像自动低频语义分析",
    )
    auto_analyze_interval_arg = DeclareLaunchArgument(
        "auto_analyze_interval_sec",
        default_value="8.0",
        description="自动图像语义分析间隔，单位秒",
    )
    auto_analyze_use_vlm_arg = DeclareLaunchArgument(
        "auto_analyze_use_vlm",
        default_value="false",
        description="自动低频分析是否调用 VLM；false 时只用规则和 detection 摘要",
    )

    # AirSim/PX4 桥接节点（PX4 模式）
    bridge_node = Node(
        package="aeromind_bridge",
        executable="airsim_bridge_node",
        name="airsim_bridge_node",
        output="screen",
        parameters=[
            {"mode": "px4"},
            {"airsim_ip": airsim_ip},
            {"publish_drone_state": False},
            {"primary_rgbd_period_sec": ParameterValue(LaunchConfiguration("primary_rgbd_period_sec"), value_type=float)},
            {"lidar_period_sec": ParameterValue(LaunchConfiguration("lidar_period_sec"), value_type=float)},
            {"publish_aux_cameras": ParameterValue(LaunchConfiguration("publish_aux_cameras"), value_type=bool)},
        ],
    )

    # 感知节点
    perception_node = Node(
        package="aeromind_perception",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[
            {"yolo_enabled": ParameterValue(LaunchConfiguration("yolo_enabled"), value_type=bool)},
            {"vlm_enabled": ParameterValue(LaunchConfiguration("vlm_enabled"), value_type=bool)},
            {"vlm_api_url": LaunchConfiguration("vlm_api_url")},
            {"vlm_model": LaunchConfiguration("vlm_model")},
            {"vlm_api_key": LaunchConfiguration("vlm_api_key")},
            {"auto_analyze_enabled": ParameterValue(LaunchConfiguration("auto_analyze_enabled"), value_type=bool)},
            {"auto_analyze_interval_sec": ParameterValue(LaunchConfiguration("auto_analyze_interval_sec"), value_type=float)},
            {"auto_analyze_use_vlm": ParameterValue(LaunchConfiguration("auto_analyze_use_vlm"), value_type=bool)},
        ],
    )
    yolo_detection_node = Node(
        package="aeromind_perception",
        executable="yolo_detection_node",
        name="yolo_detection_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("yolo_enabled")),
        parameters=[
            {"model": LaunchConfiguration("yolo_model")},
            {"confidence_threshold": ParameterValue(LaunchConfiguration("yolo_confidence"), value_type=float)},
        ],
    )
    semantic_fusion_node = Node(
        package="aeromind_perception",
        executable="semantic_fusion_node",
        name="semantic_fusion_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("semantic_world_enabled")),
        parameters=[{"world_frame": LaunchConfiguration("semantic_world_frame")}],
    )
    object_tracker_node = Node(
        package="aeromind_perception",
        executable="object_tracker_node",
        name="object_tracker_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("semantic_world_enabled")),
        parameters=[
            {"history_enabled": ParameterValue(LaunchConfiguration("semantic_history_enabled"), value_type=bool)},
            {"history_db_path": LaunchConfiguration("semantic_history_db_path")},
            {"history_retention_hours": ParameterValue(LaunchConfiguration("semantic_history_retention_hours"), value_type=float)},
        ],
    )

    # 规划节点
    planning_node = Node(
        package="aeromind_planning",
        executable="planning_node",
        name="planning_node",
        output="screen",
    )

    # 底层实时自主避障节点：ESDF-style local map + kinodynamic replanning
    autonomy_node = Node(
        package="aeromind_autonomy",
        executable="autonomy_node",
        name="autonomy_node",
        output="screen",
        parameters=[
            {"enabled": ParameterValue(LaunchConfiguration("autonomy_enabled"), value_type=bool)},
            {"publish_control_cmd": ParameterValue(LaunchConfiguration("autonomy_control"), value_type=bool)},
            {"blocked_timeout_sec": ParameterValue(LaunchConfiguration("autonomy_blocked_timeout_sec"), value_type=float)},
            {"min_depth_m": ParameterValue(LaunchConfiguration("autonomy_min_depth_m"), value_type=float)},
            {"max_depth_m": ParameterValue(LaunchConfiguration("autonomy_max_depth_m"), value_type=float)},
            {"map_rolling_radius": ParameterValue(LaunchConfiguration("autonomy_map_radius_m"), value_type=float)},
            {"map_decay_sec": ParameterValue(LaunchConfiguration("autonomy_map_decay_sec"), value_type=float)},
            {"odom_timeout_sec": ParameterValue(LaunchConfiguration("autonomy_odom_timeout_sec"), value_type=float)},
            {"depth_timeout_sec": ParameterValue(LaunchConfiguration("autonomy_depth_timeout_sec"), value_type=float)},
            {"waypoint_collision_lookahead_sec": ParameterValue(LaunchConfiguration("autonomy_waypoint_lookahead_sec"), value_type=float)},
            {"waypoint_max_replans": ParameterValue(LaunchConfiguration("autonomy_waypoint_max_replans"), value_type=int)},
            {"semantic_guard_enabled": ParameterValue(LaunchConfiguration("autonomy_semantic_guard_enabled"), value_type=bool)},
            {"semantic_guard_clear_dwell_sec": ParameterValue(LaunchConfiguration("autonomy_semantic_clear_dwell_sec"), value_type=float)},
            {"semantic_guard_hold_timeout_sec": ParameterValue(LaunchConfiguration("autonomy_semantic_hold_timeout_sec"), value_type=float)},
            {"max_speed": ParameterValue(LaunchConfiguration("autonomy_velocity_limit"), value_type=float)},
            {"max_acceleration": ParameterValue(LaunchConfiguration("autonomy_accel_limit"), value_type=float)},
            {"min_flight_altitude": ParameterValue(LaunchConfiguration("autonomy_min_altitude"), value_type=float)},
            {"odom_topic": LaunchConfiguration("autonomy_odom_topic")},
            {"world_cloud_topic": LaunchConfiguration("autonomy_world_cloud_topic")},
            {"max_position_variance": ParameterValue(LaunchConfiguration("autonomy_max_position_variance"), value_type=float)},
            {"max_orientation_variance": ParameterValue(LaunchConfiguration("autonomy_max_orientation_variance"), value_type=float)},
        ],
    )

    # 控制节点（PX4 模式）
    control_node = Node(
        package="aeromind_control",
        executable="control_node",
        name="control_node",
        output="screen",
        parameters=[
            {"px4_mode": "px4"},
            {
                "execute_autonomy_trajectory": ParameterValue(
                    LaunchConfiguration("autonomy_trajectory_execution"),
                    value_type=bool,
                )
            },
            {
                "autonomy_velocity_limit": ParameterValue(
                    LaunchConfiguration("autonomy_velocity_limit"),
                    value_type=float,
                )
            },
            {
                "autonomy_accel_limit": ParameterValue(
                    LaunchConfiguration("autonomy_accel_limit"),
                    value_type=float,
                )
            },
            {
                "autonomy_min_altitude": ParameterValue(
                    LaunchConfiguration("autonomy_min_altitude"),
                    value_type=float,
                )
            },
        ],
    )

    # 键盘遥控节点（需要独立终端运行，不在此启动）
    # 使用: ros2 run aeromind_teleop teleop_node
    # teleop_node = Node(
    #     package="aeromind_teleop",
    #     executable="teleop_node",
    #     name="teleop_node",
    #     output="screen",
    # )

    # Agent 节点
    agent_node = Node(
        package="aeromind_agent",
        executable="agent_node",
        name="agent_node",
        output="screen",
        parameters=[
            {"llm_enabled": ParameterValue(LaunchConfiguration("llm_enabled"), value_type=bool)},
            {"llm_api_url": LaunchConfiguration("llm_api_url")},
            {"llm_model": LaunchConfiguration("llm_model")},
            {"llm_api_key": LaunchConfiguration("llm_api_key")},
            {"llm_timeout_sec": ParameterValue(LaunchConfiguration("llm_timeout_sec"), value_type=float)},
        ],
    )

    return LaunchDescription([
        airsim_ip_arg,
        primary_rgbd_period_arg,
        lidar_period_arg,
        publish_aux_cameras_arg,
        llm_enabled_arg,
        llm_api_url_arg,
        llm_model_arg,
        llm_api_key_arg,
        llm_timeout_arg,
        autonomy_enabled_arg,
        autonomy_control_arg,
        autonomy_trajectory_execution_arg,
        autonomy_velocity_limit_arg,
        autonomy_accel_limit_arg,
        autonomy_min_altitude_arg,
        autonomy_blocked_timeout_arg,
        autonomy_min_depth_arg,
        autonomy_max_depth_arg,
        autonomy_map_radius_arg,
        autonomy_map_decay_arg,
        autonomy_odom_timeout_arg,
        autonomy_depth_timeout_arg,
        autonomy_waypoint_lookahead_arg,
        autonomy_waypoint_max_replans_arg,
        autonomy_semantic_guard_arg,
        autonomy_semantic_clear_dwell_arg,
        autonomy_semantic_hold_timeout_arg,
        autonomy_odom_topic_arg,
        autonomy_world_cloud_topic_arg,
        autonomy_max_position_variance_arg,
        autonomy_max_orientation_variance_arg,
        yolo_enabled_arg,
        yolo_model_arg,
        yolo_confidence_arg,
        semantic_world_enabled_arg,
        semantic_world_frame_arg,
        semantic_history_enabled_arg,
        semantic_history_db_path_arg,
        semantic_history_retention_hours_arg,
        vlm_enabled_arg,
        vlm_api_url_arg,
        vlm_model_arg,
        vlm_api_key_arg,
        auto_analyze_enabled_arg,
        auto_analyze_interval_arg,
        auto_analyze_use_vlm_arg,
        bridge_node,
        perception_node,
        yolo_detection_node,
        semantic_fusion_node,
        object_tracker_node,
        planning_node,
        autonomy_node,
        control_node,
        agent_node,
    ])
