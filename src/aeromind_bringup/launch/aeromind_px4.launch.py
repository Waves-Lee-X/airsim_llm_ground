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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成 PX4 模式 LaunchDescription"""

    # AirSim/PX4 桥接节点（PX4 模式）
    bridge_node = Node(
        package="aeromind_bridge",
        executable="airsim_bridge_node",
        name="airsim_bridge_node",
        output="screen",
        parameters=[{"mode": "px4"}],
    )

    # 感知节点
    perception_node = Node(
        package="aeromind_perception",
        executable="perception_node",
        name="perception_node",
        output="screen",
    )

    # 规划节点
    planning_node = Node(
        package="aeromind_planning",
        executable="planning_node",
        name="planning_node",
        output="screen",
    )

    # 控制节点（PX4 模式）
    control_node = Node(
        package="aeromind_control",
        executable="control_node",
        name="control_node",
        output="screen",
        parameters=[{"px4_mode": "px4"}],
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
    )

    return LaunchDescription([
        bridge_node,
        perception_node,
        planning_node,
        control_node,
        agent_node,
    ])
