# -*- coding: utf-8 -*-
"""
aeromind_web.launch.py — AeroMind Web 控制台

用法:
  ros2 launch aeromind_bringup aeromind_web.launch.py
  ros2 launch aeromind_bringup aeromind_web.launch.py port:=8081
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_arg = DeclareLaunchArgument(
        "host",
        default_value="0.0.0.0",
        description="Web 控制台监听地址",
    )
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8080",
        description="Web 控制台端口",
    )
    agent_gateway_enabled_arg = DeclareLaunchArgument(
        "agent_gateway_enabled",
        default_value="true",
        description="是否启动多 Provider Agent 流式会话网关",
    )
    agent_port_arg = DeclareLaunchArgument(
        "agent_port",
        default_value="8090",
        description="Agent Gateway 端口",
    )
    agent_provider_arg = DeclareLaunchArgument(
        "agent_provider",
        default_value="claude",
        description="默认 Agent Provider，例如 claude、deepseek 或 qwen",
    )
    agent_model_arg = DeclareLaunchArgument(
        "agent_model",
        default_value="sonnet",
        description="默认模型或别名",
    )
    agent_db_arg = DeclareLaunchArgument(
        "agent_db",
        default_value="~/.aeromind/agent_gateway.db",
        description="Agent 会话 SQLite 数据库路径",
    )

    web_node = Node(
        package="aeromind_web",
        executable="web_console_node",
        name="web_console_node",
        output="screen",
        parameters=[
            {"host": LaunchConfiguration("host")},
            {"port": LaunchConfiguration("port")},
        ],
    )
    agent_gateway_node = Node(
        package="aeromind_agent_gateway",
        executable="agent_gateway",
        name="agent_gateway",
        output="screen",
        condition=IfCondition(LaunchConfiguration("agent_gateway_enabled")),
        additional_env={
            "AEROMIND_AGENT_HOST": LaunchConfiguration("host"),
            "AEROMIND_AGENT_PORT": LaunchConfiguration("agent_port"),
            "AEROMIND_AGENT_DB": LaunchConfiguration("agent_db"),
            "AEROMIND_AGENT_PROVIDER": LaunchConfiguration("agent_provider"),
            "AEROMIND_AGENT_MODEL": LaunchConfiguration("agent_model"),
        },
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        agent_gateway_enabled_arg,
        agent_port_arg,
        agent_provider_arg,
        agent_model_arg,
        agent_db_arg,
        web_node,
        agent_gateway_node,
    ])
