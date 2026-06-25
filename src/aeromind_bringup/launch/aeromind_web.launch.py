# -*- coding: utf-8 -*-
"""
aeromind_web.launch.py — AeroMind Web 控制台

用法:
  ros2 launch aeromind_bringup aeromind_web.launch.py
  ros2 launch aeromind_bringup aeromind_web.launch.py port:=8081
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription([
        host_arg,
        port_arg,
        web_node,
    ])
