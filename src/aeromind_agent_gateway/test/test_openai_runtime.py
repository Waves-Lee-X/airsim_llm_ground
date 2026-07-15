import json
import os
import tempfile
import unittest

import httpx

from aeromind_agent_gateway.config import GatewayConfig, OpenAIProviderConfig
from aeromind_agent_gateway.drone_tools import (
    _safe_takeoff_workflow,
    execute_openai_drone_tool,
)
from aeromind_agent_gateway.openai_runtime import OpenAICompatibleRuntime


class FakeRosState:
    def drone_state(self):
        return {"available": True, "state": {"armed": False}}

    def perception_summary(self):
        return {"available": False}

    def snapshot(self):
        return {"available": True}


class OpenAICompatibleRuntimeTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.provider = OpenAIProviderConfig(
            name="deepseek",
            base_url="https://example.invalid/v1",
            api_key="secret",
            models=("deepseek-chat",),
        )
        self.config = GatewayConfig(
            host="127.0.0.1",
            port=8090,
            database_path=os.path.join(self.temp_dir.name, "agent.db"),
            workspace=self.temp_dir.name,
            access_token="",
            default_model="deepseek-chat",
            max_turns=4,
            max_budget_usd=0.1,
            default_provider="deepseek",
            openai_providers=(self.provider,),
        )

    async def asyncTearDown(self):
        self.temp_dir.cleanup()

    async def test_streaming_tool_call_uses_control_confirmation(self):
        requests = []
        controls = []

        def handler(request):
            payload = json.loads(request.content)
            requests.append(payload)
            if len(requests) == 1:
                body = (
                    'data: {"choices":[{"delta":{"tool_calls":[{"index":0,'
                    '"id":"call-1","function":{"name":"request_takeoff",'
                    '"arguments":"{\\"altitude\\":5}"}}]}}]}\n\n'
                    "data: [DONE]\n\n"
                )
            else:
                body = (
                    'data: {"choices":[{"delta":{"content":"已创建起飞确认请求"}}]}\n\n'
                    "data: [DONE]\n\n"
                )
            return httpx.Response(200, text=body)

        async def request_control(action, args):
            controls.append((action, args))
            return {"success": True, "status": "pending_confirmation"}

        runtime = OpenAICompatibleRuntime(
            self.config,
            self.provider,
            FakeRosState(),
            "deepseek-chat",
            request_control=request_control,
        )
        runtime._client = httpx.AsyncClient(
            base_url=self.provider.base_url,
            transport=httpx.MockTransport(handler),
        )
        events = []

        async def emit(event_type, payload):
            events.append((event_type, payload))

        result = await runtime.run_turn("起飞到5米", emit)
        await runtime.close()

        self.assertFalse(result["is_error"])
        self.assertEqual(result["provider"], "deepseek")
        self.assertEqual(result["text"], "已创建起飞确认请求")
        self.assertEqual(controls, [("takeoff", {"altitude": 5})])
        self.assertEqual(len(requests), 2)
        self.assertEqual(
            [item[0] for item in events],
            ["tool.started", "tool.completed", "assistant.delta"],
        )

    async def test_model_registry_rejects_unknown_model(self):
        runtime = OpenAICompatibleRuntime(
            self.config,
            self.provider,
            FakeRosState(),
            "unknown-model",
        )
        with self.assertRaises(ValueError):
            await runtime.start()

    async def test_square_mission_tool_creates_one_workflow_confirmation(self):
        controls = []

        async def request_control(action, args):
            controls.append((action, args))
            return {"success": True, "status": "pending_confirmation"}

        result = await execute_openai_drone_tool(
            "request_square_mission",
            {"side_length": 10, "altitude": 6, "land_after": True},
            FakeRosState(),
            request_control,
        )

        self.assertTrue(result["success"])
        self.assertEqual(len(controls), 1)
        self.assertEqual(controls[0][0], "workflow")
        workflow = controls[0][1]
        self.assertEqual(
            [step["action"] for step in workflow["steps"]],
            ["takeoff", "follow_waypoints", "land"],
        )

    async def test_safe_takeoff_creates_check_then_takeoff_workflow(self):
        controls = []

        async def request_control(action, args):
            controls.append((action, args))
            return {"success": True, "status": "pending_confirmation"}

        result = await execute_openai_drone_tool(
            "request_safe_takeoff",
            {"altitude": 10.0, "minimum_obstacle_distance": 2.0},
            FakeRosState(),
            request_control,
        )

        self.assertTrue(result["success"])
        self.assertEqual(len(controls), 1)
        self.assertEqual(controls[0][0], "workflow")
        steps = controls[0][1]["steps"]
        self.assertEqual(
            [step["action"] for step in steps], ["safety_check", "takeoff"]
        )
        self.assertEqual(steps[1]["depends_on"], ["safety-check"])

    async def test_registered_v_shape_skill_creates_workflow_confirmation(self):
        controls = []

        async def request_control(action, args):
            controls.append((action, args))
            return {"success": True, "status": "pending_confirmation"}

        result = await execute_openai_drone_tool(
            "request_skill",
            {
                "name": "flight.v_shape",
                "args": {"width": 10, "depth": 8, "capture_at_vertex": True},
            },
            FakeRosState(),
            request_control,
        )

        self.assertTrue(result["success"])
        self.assertEqual(controls[0][0], "workflow")
        self.assertEqual(
            [step["action"] for step in controls[0][1]["steps"]],
            ["takeoff", "move", "capture_image", "move"],
        )

    def test_safe_takeoff_defaults_are_validated(self):
        workflow = _safe_takeoff_workflow({"altitude": 8.0})
        check = workflow["steps"][0]
        self.assertEqual(check["args"]["minimum_obstacle_distance"], 2.0)
        self.assertTrue(check["args"]["require_gps"])


if __name__ == "__main__":
    unittest.main()
