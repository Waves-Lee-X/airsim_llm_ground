# -*- coding: utf-8 -*-
"""Start the contest demonstration stack after PX4, DDS Agent and AirSim."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _env(name: str, default: str = "") -> str:
    return os.environ.get(name, "").strip() or default


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("yolo_enabled", default_value="true"),
        DeclareLaunchArgument("yolo_model", default_value="yolov8n.pt"),
        DeclareLaunchArgument("yolo_confidence", default_value="0.35"),
        DeclareLaunchArgument("vlm_enabled", default_value="true"),
        DeclareLaunchArgument("auto_analyze_enabled", default_value="false"),
        DeclareLaunchArgument("airsim_vehicle_name", default_value=""),
        DeclareLaunchArgument("airsim_lidar_name", default_value=""),
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("agent_port", default_value="8090"),
        DeclareLaunchArgument("agent_provider", default_value=_env("AEROMIND_AGENT_PROVIDER", "deepseek")),
        DeclareLaunchArgument("agent_model", default_value=_env("AEROMIND_AGENT_MODEL", "deepseek-chat")),
    ]

    px4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aeromind_bringup"), "launch", "aeromind_px4.launch.py"
            ])
        ),
        launch_arguments={
            # The Gateway owns open-ended model planning; the legacy ROS Agent keeps
            # its deterministic parser and execution services without a second LLM call.
            "llm_enabled": "false",
            "yolo_enabled": LaunchConfiguration("yolo_enabled"),
            "yolo_model": LaunchConfiguration("yolo_model"),
            "yolo_confidence": LaunchConfiguration("yolo_confidence"),
            "vlm_enabled": LaunchConfiguration("vlm_enabled"),
            "auto_analyze_enabled": LaunchConfiguration("auto_analyze_enabled"),
            "auto_analyze_use_vlm": "false",
            "airsim_vehicle_name": LaunchConfiguration("airsim_vehicle_name"),
            "airsim_lidar_name": LaunchConfiguration("airsim_lidar_name"),
        }.items(),
    )

    web_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aeromind_bringup"), "launch", "aeromind_web.launch.py"
            ])
        ),
        launch_arguments={
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("web_port"),
            "agent_port": LaunchConfiguration("agent_port"),
            "agent_provider": LaunchConfiguration("agent_provider"),
            "agent_model": LaunchConfiguration("agent_model"),
        }.items(),
    )

    return LaunchDescription([*arguments, px4_launch, web_launch])
