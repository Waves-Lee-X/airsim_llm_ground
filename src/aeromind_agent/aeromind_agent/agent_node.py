#!/usr/bin/env python3
"""
agent_node.py - LLM Agent 任务调度节点

功能：
  - 提供 /agent/execute_task 服务：接收自然语言任务
  - 链式调用 planning + control 服务验证链路
  - 预留 LLM 接口参数（后续接入 LLM + 工具调用链）

调用链路：
  execute_task → plan_path → takeoff → (后续 path + land)
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion
from aeromind_interfaces.srv import ExecuteTask, PlanPath, Takeoff


class AgentNode(Node):
    """LLM Agent 任务调度节点"""

    def __init__(self):
        super().__init__("agent_node")

        # 声明 LLM 相关参数（后续接入 LLM 时使用）
        self.declare_parameter("llm_api_url", "http://localhost:11434/v1")
        self.declare_parameter("llm_model", "llama3")
        self._llm_api_url = self.get_parameter("llm_api_url").value
        self._llm_model = self.get_parameter("llm_model").value

        # 提供任务执行服务
        self._task_srv = self.create_service(
            ExecuteTask, "/agent/execute_task", self._execute_task_callback
        )

        # 创建服务客户端（用于调用其他模块的服务）
        self._plan_client = self.create_client(PlanPath, "/planning/plan_path")
        self._takeoff_client = self.create_client(Takeoff, "/control/takeoff")

        # 等待下游服务就绪
        self.get_logger().info("等待下游服务就绪...")
        if self._plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("  /planning/plan_path 就绪")
        else:
            self.get_logger().warn("  /planning/plan_path 未就绪（超时）")

        if self._takeoff_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("  /control/takeoff 就绪")
        else:
            self.get_logger().warn("  /control/takeoff 未就绪（超时）")

        self.get_logger().info("Agent 节点已启动")
        self.get_logger().info(f"LLM 配置: url={self._llm_api_url}, model={self._llm_model}")

    def _execute_task_callback(self, request, response):
        """任务执行服务回调：验证调用链路"""
        task = request.task_description
        self.get_logger().info(f"收到任务: {task}")

        # 当前为占位实现，硬编码一组起点和终点以验证链路
        # 后续接入 LLM 后，由 LLM 解析任务并规划具体参数

        # Step 1: 调用路径规划
        self.get_logger().info("Step 1: 调用路径规划...")
        plan_req = PlanPath.Request()
        plan_req.start = Pose(
            position=Point(x=0.0, y=0.0, z=-10.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        plan_req.goal = Pose(
            position=Point(x=20.0, y=10.0, z=-10.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

        if not self._plan_client.service_is_ready():
            response.success = False
            response.result = "路径规划服务未就绪"
            response.message = "请确保 planning_node 已启动"
            return response

        future_plan = self._plan_client.call_async(plan_req)
        rclpy.spin_until_future_complete(self, future_plan)
        plan_result = future_plan.result()

        if plan_result is None or not plan_result.success:
            response.success = False
            response.result = "路径规划失败"
            response.message = plan_result.message if plan_result else "无响应"
            return response

        self.get_logger().info(f"路径规划成功: {plan_result.message}")

        # Step 2: 调用起飞
        self.get_logger().info("Step 2: 调用起飞...")
        takeoff_req = Takeoff.Request()
        takeoff_req.altitude = 10.0

        if not self._takeoff_client.service_is_ready():
            response.success = False
            response.result = "起飞服务未就绪"
            response.message = "请确保 control_node 已启动"
            return response

        future_takeoff = self._takeoff_client.call_async(takeoff_req)
        rclpy.spin_until_future_complete(self, future_takeoff)
        takeoff_result = future_takeoff.result()

        if takeoff_result is None or not takeoff_result.success:
            response.success = False
            response.result = "起飞失败"
            response.message = takeoff_result.message if takeoff_result else "无响应"
            return response

        self.get_logger().info(f"起飞成功: {takeoff_result.message}")

        # 链路验证通过
        response.success = True
        response.result = "任务执行链路验证通过：规划 + 起飞均已成功调用"
        response.message = (
            f"任务 '{task}' 已执行（占位模式）。"
            f"规划: {plan_result.message}; 起飞: {takeoff_result.message}"
        )
        self.get_logger().info(response.result)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
