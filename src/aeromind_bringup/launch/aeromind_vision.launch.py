# -*- coding: utf-8 -*-
"""
aeromind_vision.launch.py — 相机 + RViz2 可视化启动文件

功能等价于 hw_insight lesson3.launch.py：
  - 相机图像转发
  - 深度点云 RViz 窗口
  - 图像+激光雷达 RViz 窗口

相机话题（由 aeromind_bridge AirSim 模式发布）：
  - /sensor/camera/rgb/front_center      (Scene)
  - /sensor/camera/depth/front_center    (Depth)
  - /sensor/camera/rgb/bottom_center     (下视)
  - /sensor/camera/rgb/front_left        (左前)
  - /sensor/camera/rgb/front_right       (右前)
  - /sensor/lidar/points                 (PointCloud2)

⚠️ 依赖: 需要先启动 bridge 节点（aeromind_all 或 aeromind_px4 launch），
   否则没有相机/LiDAR 数据源，RViz 窗口将是空的。

用法:
  # 先启动主系统（提供数据源）
  ros2 launch aeromind_bringup aeromind_px4.launch.py
  # 再启动可视化
  ros2 launch aeromind_bringup aeromind_vision.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 Vision LaunchDescription"""

    # === RGB 图压缩转发 ===
    rgb_relay = Node(
        package="image_transport",
        executable="republish",
        name="rgb_relay",
        arguments=["raw", "compressed"],
        remappings=[
            ("in", "/sensor/camera/rgb/front_center"),
            ("out/compressed", "/sensor/camera/rgb/front_center/compressed"),
        ],
    )

    # === RViz2：深度点云 ===
    pkg_share = get_package_share_directory("aeromind_bringup")
    depth_rviz_path = os.path.join(pkg_share, "launch", "depth_cloud.rviz")
    image_lidar_rviz_path = os.path.join(pkg_share, "launch", "image_lidar.rviz")

    depth_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="depth_rviz2",
        arguments=["-d", depth_rviz_path],
    )

    # === RViz2：图像 + 激光雷达 ===
    image_lidar_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="image_lidar_rviz2",
        arguments=["-d", image_lidar_rviz_path],
    )

    return LaunchDescription([
        rgb_relay,
        depth_rviz_node,
        image_lidar_rviz_node,
    ])
