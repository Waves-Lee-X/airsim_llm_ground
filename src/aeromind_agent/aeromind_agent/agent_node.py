#!/usr/bin/env python3
"""
agent_node.py - LLM Agent 任务调度节点

功能：
  - 提供 /agent/execute_task 服务：接收自然语言任务
  - 初级自然语言规则解析：解锁、加锁、起飞、降落、查询状态
  - 预留 LLM 接口参数（后续可升级为 LLM + 工具调用链）

调用链路：
  execute_task → parse intent → call control/status
"""

import json
import re

import rclpy
from rclpy.node import Node

from aeromind_interfaces.msg import DroneState
from aeromind_interfaces.srv import ArmDrone, ExecuteTask, Land, Takeoff


class AgentNode(Node):
    """LLM Agent 任务调度节点"""

    def __init__(self):
        super().__init__("agent_node")

        # 声明 LLM 相关参数（后续接入 LLM 时使用）
        self.declare_parameter("llm_api_url", "http://localhost:11434/v1")
        self.declare_parameter("llm_model", "llama3")
        self._llm_api_url = self.get_parameter("llm_api_url").value
        self._llm_model = self.get_parameter("llm_model").value
        self._latest_state = None

        # 提供任务执行服务
        self._task_srv = self.create_service(
            ExecuteTask, "/agent/execute_task", self._execute_task_callback
        )

        # 创建服务客户端（用于调用其他模块的服务）
        self._arm_client = self.create_client(ArmDrone, "/control/arm")
        self._takeoff_client = self.create_client(Takeoff, "/control/takeoff")
        self._land_client = self.create_client(Land, "/control/land")

        self.create_subscription(
            DroneState, "/control/drone_state", self._drone_state_callback, 10
        )

        # 等待下游服务就绪
        self.get_logger().info("等待下游服务就绪...")
        self._wait_for_optional_service(self._arm_client, "/control/arm")
        self._wait_for_optional_service(self._takeoff_client, "/control/takeoff")
        self._wait_for_optional_service(self._land_client, "/control/land")

        self.get_logger().info("Agent 节点已启动（规则解析模式）")
        self.get_logger().info(f"LLM 配置: url={self._llm_api_url}, model={self._llm_model}")

    def _execute_task_callback(self, request, response):
        """任务执行服务回调：解析自然语言并调用对应能力。"""
        task = request.task_description.strip()
        self.get_logger().info(f"收到任务: {task}")

        if not task:
            response.success = False
            response.result = self._json_result("unknown", {}, "任务为空")
            response.message = "请输入任务，例如：起飞到10米、降落、解锁、查询状态"
            return response

        parsed = self._parse_task(task)
        self.get_logger().info(f"解析结果: {json.dumps(parsed, ensure_ascii=False)}")

        intent = parsed["intent"]
        args = parsed["args"]

        if intent == "arm":
            return self._execute_arm(True, parsed, response)
        if intent == "disarm":
            return self._execute_arm(False, parsed, response)
        if intent == "takeoff":
            return self._execute_takeoff(args["altitude"], parsed, response)
        if intent == "land":
            return self._execute_land(parsed, response)
        if intent == "status":
            return self._execute_status(parsed, response)

        response.success = False
        response.result = json.dumps(parsed, ensure_ascii=False)
        response.message = parsed["reason"]
        return response

    def _drone_state_callback(self, msg: DroneState):
        self._latest_state = {
            "armed": bool(msg.armed),
            "mode": msg.mode,
            "battery": float(msg.battery),
            "gps_fix": int(msg.gps_fix),
            "ekf_healthy": bool(msg.ekf_healthy),
        }

    def _wait_for_optional_service(self, client, name: str):
        if client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f"  {name} 就绪")
        else:
            self.get_logger().warn(f"  {name} 未就绪（超时）")

    def _parse_task(self, task: str):
        text = task.strip().lower()
        compact = re.sub(r"\s+", "", text)

        if any(word in compact for word in ("状态", "情况", "电量", "模式", "查询", "检查")):
            return self._parsed("status", {}, "查询当前无人机状态")

        if any(word in compact for word in ("加锁", "上锁", "锁定", "disarm")):
            return self._parsed("disarm", {}, "执行加锁")

        if any(word in compact for word in ("解锁", "启动电机", "arm")):
            return self._parsed("arm", {}, "执行解锁")

        if any(word in compact for word in ("降落", "着陆", "落地", "land")):
            return self._parsed("land", {}, "执行降落")

        if any(word in compact for word in ("起飞", "升空", "takeoff")):
            altitude = self._extract_altitude(compact)
            if altitude is None:
                altitude = 10.0
            if altitude < 1.0 or altitude > 50.0:
                return self._parsed(
                    "unknown",
                    {"altitude": altitude},
                    "起飞高度必须在 1 到 50 米之间",
                    risk_level="high",
                    need_confirm=True,
                )
            return self._parsed(
                "takeoff",
                {"altitude": altitude},
                f"起飞到 {altitude:.1f} 米",
            )

        if any(word in compact for word in ("前进", "后退", "左移", "右移", "飞到", "飞向", "航点")):
            return self._parsed(
                "unknown",
                {},
                "已识别为移动/航点任务，但当前初级版还没有接入路径执行",
                risk_level="medium",
                need_confirm=True,
            )

        return self._parsed(
            "unknown",
            {},
            "暂时无法理解该任务。当前支持：解锁、加锁、起飞到N米、降落、查询状态",
        )

    def _extract_altitude(self, text: str):
        patterns = (
            r"(?:到|至|高度|飞到)?(-?\d+(?:\.\d+)?)(?:米|m)",
            r"(-?\d+(?:\.\d+)?)",
        )
        for pattern in patterns:
            match = re.search(pattern, text)
            if match:
                return float(match.group(1))

        match = re.search(r"([零一二两三四五六七八九十]+)(?:米|m)?", text)
        if match:
            return self._parse_chinese_number(match.group(1))
        return None

    def _parse_chinese_number(self, text: str):
        digits = {
            "零": 0,
            "一": 1,
            "二": 2,
            "两": 2,
            "三": 3,
            "四": 4,
            "五": 5,
            "六": 6,
            "七": 7,
            "八": 8,
            "九": 9,
        }
        if not text:
            return None
        if text == "十":
            return 10.0
        if "十" in text:
            left, _, right = text.partition("十")
            tens = digits.get(left, 1) if left else 1
            ones = digits.get(right, 0) if right else 0
            return float(tens * 10 + ones)
        total = 0
        for char in text:
            if char not in digits:
                return None
            total = total * 10 + digits[char]
        return float(total)

    def _parsed(
        self,
        intent: str,
        args: dict,
        reason: str,
        risk_level: str = "low",
        need_confirm: bool = False,
    ):
        return {
            "intent": intent,
            "args": args,
            "risk_level": risk_level,
            "need_confirm": need_confirm,
            "reason": reason,
        }

    def _execute_arm(self, arm: bool, parsed: dict, response):
        req = ArmDrone.Request()
        req.arm = arm
        result = self._call_service(self._arm_client, req, "/control/arm")
        return self._fill_response(response, parsed, result, "解锁" if arm else "加锁")

    def _execute_takeoff(self, altitude: float, parsed: dict, response):
        req = Takeoff.Request()
        req.altitude = float(altitude)
        result = self._call_service(self._takeoff_client, req, "/control/takeoff")
        return self._fill_response(response, parsed, result, f"起飞到 {altitude:.1f} 米")

    def _execute_land(self, parsed: dict, response):
        req = Land.Request()
        result = self._call_service(self._land_client, req, "/control/land")
        return self._fill_response(response, parsed, result, "降落")

    def _execute_status(self, parsed: dict, response):
        response.success = True
        if self._latest_state is None:
            parsed["state"] = None
            response.result = json.dumps(parsed, ensure_ascii=False)
            response.message = "尚未收到 /control/drone_state，请确认 bridge/control 已启动"
            return response

        parsed["state"] = self._latest_state
        response.result = json.dumps(parsed, ensure_ascii=False)
        response.message = (
            f"当前状态：armed={self._latest_state['armed']}, "
            f"mode={self._latest_state['mode']}, "
            f"battery={self._latest_state['battery']:.1f}V, "
            f"gps_fix={self._latest_state['gps_fix']}, "
            f"ekf_healthy={self._latest_state['ekf_healthy']}"
        )
        return response

    def _call_service(self, client, request, service_name: str):
        if not client.service_is_ready():
            return {
                "success": False,
                "message": f"{service_name} 服务未就绪，请确认对应节点已启动",
            }

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result is None:
            return {"success": False, "message": f"{service_name} 无响应或调用超时"}

        return {
            "success": bool(getattr(result, "success", True)),
            "message": getattr(result, "message", "服务调用完成"),
        }

    def _fill_response(self, response, parsed: dict, service_result: dict, action_name: str):
        parsed["service_result"] = service_result
        response.success = service_result["success"]
        response.result = json.dumps(parsed, ensure_ascii=False)
        response.message = (
            f"{action_name}成功：{service_result['message']}"
            if service_result["success"]
            else f"{action_name}失败：{service_result['message']}"
        )
        return response

    def _json_result(self, intent: str, args: dict, reason: str):
        return json.dumps(self._parsed(intent, args, reason), ensure_ascii=False)


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
