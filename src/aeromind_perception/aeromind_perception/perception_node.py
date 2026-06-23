#!/usr/bin/env python3
"""
perception_node.py - 感知节点（占位）

后续将挂载：
  - YOLO 目标检测 → /perception/detection
  - OctoMap 建图 → /perception/obstacle_map
  - 相机图像处理 → 订阅 /sensor/camera/image

当前仅定时发送空消息，供上下游联调。
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from aeromind_interfaces.msg import Detection, ObstacleMap


class PerceptionNode(Node):
    """感知节点"""

    def __init__(self):
        super().__init__("perception_node")

        # 发布者
        self._detection_pub = self.create_publisher(
            Detection, "/perception/detection", 10
        )
        self._obstacle_pub = self.create_publisher(
            ObstacleMap, "/perception/obstacle_map", 10
        )

        # 订阅相机图像（当前不处理，占位接口）
        self._image_sub = self.create_subscription(
            Image, "/sensor/camera/image", self._image_callback, 10
        )

        # 定时器：1Hz 发送占位消息
        self._timer = self.create_timer(1.0, self._timer_callback)

        self.get_logger().info("感知节点已启动（占位模式）")

    def _image_callback(self, msg: Image):
        """相机图像回调 — 当前不处理，等组员实现 YOLO 时填充"""
        pass

    def _timer_callback(self):
        """定时发送空消息"""
        # 空检测结果
        detection = Detection()
        self._detection_pub.publish(detection)

        # 空障碍物地图
        obs_map = ObstacleMap()
        obs_map.timestamp = self.get_clock().now().to_msg()
        obs_map.width = 0
        obs_map.height = 0
        obs_map.resolution = 0.0
        obs_map.data = []
        self._obstacle_pub.publish(obs_map)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
