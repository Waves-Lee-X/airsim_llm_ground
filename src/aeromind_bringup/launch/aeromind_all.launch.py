# -*- coding: utf-8 -*-
"""
aeromind_all.launch.py — AeroMind 全系统启动文件（AirSim 模式）

启动所有 5 个节点，bridge 直连 AirSim API。
airsim_ip 默认自动读取 /etc/resolv.conf 获取 Windows 宿主 IP。

用法:
  ros2 launch aeromind_bringup aeromind_all.launch.py
  ros2 launch aeromind_bringup aeromind_all.launch.py airsim_ip:=192.168.1.100
"""

import socket
import struct

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_windows_ip():
    """自动获取 Windows 宿主机 IP（WSL2 默认网关）"""
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
    """生成 LaunchDescription"""

    default_ip = _get_windows_ip()

    airsim_ip_arg = DeclareLaunchArgument(
        "airsim_ip",
        default_value=default_ip,
        description="AirSim 宿主机 IP 地址（默认自动检测 WSL 网关）",
    )
    airsim_ip = LaunchConfiguration("airsim_ip")

    bridge_node = Node(
        package="aeromind_bridge",
        executable="airsim_bridge_node",
        name="airsim_bridge_node",
        output="screen",
        parameters=[{"airsim_ip": airsim_ip}],
    )

    perception_node = Node(
        package="aeromind_perception",
        executable="perception_node",
        name="perception_node",
        output="screen",
    )

    planning_node = Node(
        package="aeromind_planning",
        executable="planning_node",
        name="planning_node",
        output="screen",
    )

    control_node = Node(
        package="aeromind_control",
        executable="control_node",
        name="control_node",
        output="screen",
        parameters=[{"airsim_ip": airsim_ip}],
    )

    agent_node = Node(
        package="aeromind_agent",
        executable="agent_node",
        name="agent_node",
        output="screen",
    )

    return LaunchDescription([
        airsim_ip_arg,
        bridge_node,
        perception_node,
        planning_node,
        control_node,
        agent_node,
    ])
