#!/usr/bin/env python3
"""
planning_node.py - 路径规划节点

功能：
  - 提供 /planning/plan_path 服务：直线插值生成航点
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from aeromind_interfaces.msg import Path
from aeromind_interfaces.srv import PlanPath


class PlanningNode(Node):
    """路径规划节点"""

    def __init__(self):
        super().__init__("planning_node")

        # 提供路径规划服务
        self._plan_srv = self.create_service(
            PlanPath, "/planning/plan_path", self._plan_path_callback
        )

        self.get_logger().info("规划节点已启动")

    def _plan_path_callback(self, request, response):
        """路径规划服务回调：直线插值生成航点"""
        start = request.start
        goal = request.goal

        self.get_logger().info(
            f"收到规划请求: start=({start.position.x:.2f}, {start.position.y:.2f}, "
            f"{start.position.z:.2f}), "
            f"goal=({goal.position.x:.2f}, {goal.position.y:.2f}, {goal.position.z:.2f})"
        )

        # 直线插值：在 start 和 goal 之间均匀生成 10 个航点
        path = Path()
        path.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id="map",
        )

        num_waypoints = 10
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = Point(
                x=start.position.x + t * (goal.position.x - start.position.x),
                y=start.position.y + t * (goal.position.y - start.position.y),
                z=start.position.z + t * (goal.position.z - start.position.z),
            )
            # 简单四元数（朝向 goal）
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            path.waypoints.append(pose)

        response.path = path
        response.success = True
        response.message = f"已生成 {num_waypoints + 1} 个直线插值航点"
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
