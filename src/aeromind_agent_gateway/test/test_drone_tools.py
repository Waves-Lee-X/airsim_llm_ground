import unittest

from aeromind_agent_gateway.drone_tools import (
    OPENAI_DRONE_TOOLS,
    execute_openai_drone_tool,
)


class FakeRosState:
    async def analyze_current_image(self, prompt):
        return {
            "success": True,
            "message": f"已分析：{prompt}",
            "source": "vlm",
        }


class DroneToolsTest(unittest.IsolatedAsyncioTestCase):
    async def test_openai_image_tool_calls_ros_vlm_service(self):
        result = await execute_openai_drone_tool(
            "analyze_current_image",
            {"prompt": "分析当前画面"},
            FakeRosState(),
            None,
        )
        self.assertTrue(result["success"])
        self.assertEqual(result["source"], "vlm")

    def test_openai_image_tool_is_exposed(self):
        names = {
            item["function"]["name"]
            for item in OPENAI_DRONE_TOOLS
        }
        self.assertIn("analyze_current_image", names)


if __name__ == "__main__":
    unittest.main()
