import asyncio
import threading
import time
import unittest

from aeromind_agent_gateway.ros_state import RosStateBridge
from aeromind_agent_gateway.skill_registry import build_skill, skill_catalog
from aeromind_agent_gateway.workflow import build_square_workflow, validate_workflow


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class WorkflowTest(unittest.IsolatedAsyncioTestCase):
    def test_square_skill_builds_valid_closed_sequence(self):
        workflow = build_skill(
            "flight.square",
            {"side_length": 8, "altitude": 6, "land_after": True},
        )
        moves = [
            step["args"]["direction"]
            for step in workflow["steps"]
            if step["action"] == "move"
        ]
        self.assertEqual(moves, ["forward", "right", "backward", "left"])
        self.assertEqual(workflow["steps"][0]["condition"], "if_not_airborne")
        self.assertEqual(workflow["steps"][-1]["action"], "land")
        self.assertEqual(skill_catalog()[0]["name"], "flight.square")

    def test_workflow_rejects_unknown_actions_and_forward_dependencies(self):
        with self.assertRaises(ValueError):
            validate_workflow(
                {
                    "name": "危险任务",
                    "steps": [{"id": "shell", "action": "bash", "args": {}}],
                }
            )
        with self.assertRaises(ValueError):
            validate_workflow(
                {
                    "name": "错误依赖",
                    "steps": [
                        {
                            "id": "first",
                            "action": "hover",
                            "depends_on": ["later"],
                        },
                        {"id": "later", "action": "hover"},
                    ],
                }
            )

    async def test_executor_retries_and_completes_steps(self):
        bridge = self._bridge()
        attempts = []

        async def execute(action, args, progress):
            attempts.append((action, args))
            await progress("verifying", {"message": "checking"})
            if action == "move" and len(attempts) == 1:
                return {"success": False, "message": "temporary"}
            return {
                "success": True,
                "message": "done",
                "physical_complete": True,
            }

        bridge.execute_confirmed_action = execute
        events = []

        async def progress(phase, details):
            events.append((phase, details))

        workflow = validate_workflow(
            {
                "name": "重试任务",
                "steps": [
                    {
                        "id": "move",
                        "action": "move",
                        "args": {"direction": "forward", "distance": 2},
                        "retries": 1,
                    },
                    {
                        "id": "hover",
                        "action": "hover",
                        "depends_on": ["move"],
                    },
                ],
            }
        )
        result = await bridge._execute_workflow(workflow, progress)

        self.assertTrue(result["success"])
        self.assertEqual([item[0] for item in attempts], ["move", "move", "hover"])
        self.assertEqual(result["workflow"]["steps"][0]["attempt"], 2)
        self.assertTrue(events)

    async def test_safety_and_perception_steps_use_live_snapshot(self):
        bridge = self._bridge()
        bridge.snapshot = lambda: {
            "available": True,
            "state": {
                "stamp": time.time(),
                "armed": False,
                "ekf_healthy": True,
                "gps_fix": 3,
            },
            "odometry": {"position_m": {"z": 0.0}},
            "autonomy": {
                "stamp": time.time(),
                "nearest_obstacle_m": 6.0,
            },
            "detections": [
                {"class_name": "person", "confidence": 0.82}
            ],
        }
        events = []

        async def progress(phase, details):
            events.append((phase, details))

        workflow = validate_workflow(
            {
                "name": "检查后观察人员",
                "steps": [
                    {"id": "safe", "action": "safety_check"},
                    {
                        "id": "person",
                        "action": "perception_check",
                        "args": {"target": "person"},
                        "depends_on": ["safe"],
                    },
                ],
            }
        )
        result = await bridge._execute_workflow(workflow, progress)

        self.assertTrue(result["success"])
        self.assertTrue(result["step_results"]["safe"]["success"])
        self.assertEqual(
            result["step_results"]["person"]["detections"][0]["class_name"],
            "person",
        )

    async def test_failed_safety_check_stops_workflow(self):
        bridge = self._bridge()
        bridge.snapshot = lambda: {
            "available": True,
            "state": {
                "stamp": time.time(),
                "armed": False,
                "ekf_healthy": False,
                "gps_fix": 1,
            },
            "odometry": {"position_m": {"z": 0.0}},
            "autonomy": {"nearest_obstacle_m": 0.8},
            "detections": [],
        }
        called = []

        async def execute(action, args, progress):
            called.append(action)
            return {"success": True}

        bridge.execute_confirmed_action = execute

        async def progress(_phase, _details):
            return None

        result = await bridge._execute_workflow(
            {
                "name": "安全起飞",
                "steps": [
                    {"id": "safe", "action": "safety_check"},
                    {
                        "id": "takeoff",
                        "action": "takeoff",
                        "args": {"altitude": 5},
                        "depends_on": ["safe"],
                    },
                ],
            },
            progress,
        )
        self.assertFalse(result["success"])
        self.assertEqual(called, [])

    def test_workflow_control_state_machine(self):
        bridge = self._bridge()
        bridge._workflow_controls["workflow-1"] = "running"
        paused = bridge.control_workflow("workflow-1", "pause")
        self.assertEqual(paused["status"], "paused")
        resumed = bridge.control_workflow("workflow-1", "resume")
        self.assertEqual(resumed["status"], "running")
        cancelled = bridge.control_workflow("workflow-1", "cancel")
        self.assertEqual(cancelled["status"], "cancelled")
        self.assertEqual(len(bridge._autonomy_cancel_pub.messages), 2)

    @staticmethod
    def _bridge():
        bridge = object.__new__(RosStateBridge)
        bridge._lock = threading.RLock()
        bridge._workflow_controls = {}
        bridge._autonomy_cancel_pub = FakePublisher()
        bridge.snapshot = lambda: {
            "state": {"armed": True},
            "odometry": {"position_m": {"z": 5.0}},
        }
        return bridge


if __name__ == "__main__":
    unittest.main()
