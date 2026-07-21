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

    async def query_world_model(self, filters):
        return {
            "success": True,
            "query_type": filters["query_type"],
            "objects": [{"id": "person_0001", "class_name": "person"}],
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

    async def test_openai_world_query_calls_read_only_ros_service(self):
        result = await execute_openai_drone_tool(
            "query_world_model",
            {"query_type": "nearest", "class_name": "person"},
            FakeRosState(),
            None,
        )
        self.assertTrue(result["success"])
        self.assertEqual(result["objects"][0]["id"], "person_0001")

    def test_openai_world_query_is_exposed(self):
        names = {item["function"]["name"] for item in OPENAI_DRONE_TOOLS}
        self.assertIn("query_world_model", names)


if __name__ == "__main__":
    unittest.main()
