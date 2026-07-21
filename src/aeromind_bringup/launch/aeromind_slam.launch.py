# -*- coding: utf-8 -*-
"""PX4/AirSim system with RTAB-Map RGB-D SLAM in safe shadow mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    yolo_enabled_arg = DeclareLaunchArgument(
        "yolo_enabled", default_value="true", description="是否启动 YOLO"
    )
    database_path_arg = DeclareLaunchArgument(
        "slam_database_path",
        default_value=os.path.expanduser("~/.ros/aeromind_rtabmap.db"),
        description="RTAB-Map 地图数据库路径",
    )
    rtabmap_args_arg = DeclareLaunchArgument(
        "rtabmap_args",
        default_value="-d",
        description="RTAB-Map 命令行参数；-d 表示启动时删除旧库",
    )
    detection_rate_arg = DeclareLaunchArgument(
        "slam_detection_rate",
        default_value="5.0",
        description="RTAB-Map 回环/建图检测频率",
    )

    px4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("aeromind_bringup"),
                "launch",
                "aeromind_px4.launch.py",
            )
        ),
        launch_arguments={
            "yolo_enabled": LaunchConfiguration("yolo_enabled"),
            "semantic_world_enabled": "true",
            "semantic_world_frame": "map",
            "primary_rgbd_period_sec": "0.1",
            "lidar_period_sec": "0.2",
            "publish_aux_cameras": "false",
            # Shadow mode: planning/control stays in PX4 local odom until map-frame
            # trajectories are transformed back and flight-tested.
            "autonomy_odom_topic": "/sensor/odometry",
        }.items(),
    )

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        output="screen",
        arguments=[LaunchConfiguration("rtabmap_args")],
        parameters=[
            {
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "",
                "publish_tf": True,
                "subscribe_rgb": True,
                "subscribe_depth": True,
                "subscribe_rgbd": False,
                "subscribe_odom_info": False,
                "approx_sync": True,
                "queue_size": 30,
                "topic_queue_size": 30,
                "wait_for_transform_duration": 0.2,
                "database_path": LaunchConfiguration("slam_database_path"),
                "Rtabmap/DetectionRate": LaunchConfiguration("slam_detection_rate"),
                "Mem/IncrementalMemory": "true",
                "RGBD/CreateOccupancyGrid": "true",
                "Grid/3D": "true",
                "Grid/RayTracing": "true",
            }
        ],
        remappings=[
            ("rgb/image", "/sensor/camera/rgb/front_center"),
            ("depth/image", "/sensor/camera/depth/front_center"),
            ("rgb/camera_info", "/sensor/camera/rgb/front_center/camera_info"),
            ("odom", "/sensor/odometry"),
        ],
    )

    slam_pose_node = Node(
        package="aeromind_bridge",
        executable="slam_pose_node",
        name="slam_pose_node",
        output="screen",
        parameters=[
            {"input_odom_topic": "/sensor/odometry"},
            {"output_odom_topic": "/localization/odometry"},
            {"map_frame": "map"},
            {"odom_frame": "odom"},
            {"base_frame": "base_link"},
        ],
    )

    return LaunchDescription(
        [
            yolo_enabled_arg,
            database_path_arg,
            rtabmap_args_arg,
            detection_rate_arg,
            px4_launch,
            rtabmap_node,
            slam_pose_node,
        ]
    )
