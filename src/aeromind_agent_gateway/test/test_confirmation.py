import asyncio
import os
import tempfile
import unittest

import numpy as np

from aeromind_agent_gateway.confirmation import ConfirmationCenter, validate_action
from aeromind_agent_gateway.events import EventBus
from aeromind_agent_gateway.store import SessionStore


class ConfirmationCenterTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.store = SessionStore(os.path.join(self.temp_dir.name, "agent.db"))
        self.store.ensure_session("s1", "u1", "web", "sonnet")
        self.events = EventBus(self.store)
        self.executions = []

        async def execute(action, args, progress):
            self.executions.append((action, args))
            await progress("verifying", {"message": "checking"})
            return {
                "success": True,
                "message": "executed",
                "physical_complete": True,
            }

        self.center = ConfirmationCenter(
            self.store, self.events, execute, ttl_seconds=1.0
        )

    async def asyncTearDown(self):
        await self.center.close()
        self.store.close()
        self.temp_dir.cleanup()

    async def test_approval_executes_exactly_once(self):
        created = await self.center.create("s1", "u1", "takeoff", {"altitude": 5})
        confirmation_id = created["confirmation_id"]
        await self.center.respond(confirmation_id, "u1", "approve")

        for _ in range(20):
            if self.store.get_confirmation(confirmation_id)["status"] == "completed":
                break
            await asyncio.sleep(0.01)

        self.assertEqual(self.executions, [("takeoff", {"altitude": 5.0})])
        self.assertEqual(
            self.store.get_confirmation(confirmation_id)["status"], "completed"
        )
        mission = self.store.gateway_mission_for_confirmation(confirmation_id)
        self.assertEqual(mission["status"], "completed")
        self.assertEqual(mission["phase"], "completed")
        self.assertTrue(mission["physical_complete"])
        self.assertTrue(mission["result"]["success"])
        repeated = await self.center.respond(confirmation_id, "u1", "approve")
        self.assertTrue(repeated["idempotent"])
        self.assertIn("未重复执行", repeated["message"])
        self.assertEqual(len(self.executions), 1)

    async def test_progress_with_numpy_float32_evidence_completes(self):
        async def execute_with_ros_evidence(action, args, progress):
            await progress(
                "workflow_step",
                {
                    "message": "感知证据已更新",
                    "evidence": {
                        "position_covariance": [
                            np.float32(0.1),
                            np.float32(0.0),
                        ]
                    },
                },
            )
            return {
                "success": True,
                "message": "workflow completed",
                "physical_complete": True,
            }

        self.center._execute = execute_with_ros_evidence
        created = await self.center.create(
            "s1",
            "u1",
            "workflow",
            {
                "name": "带感知证据的任务",
                "steps": [{"id": "report", "action": "mission_report"}],
            },
        )
        confirmation_id = created["confirmation_id"]
        await self.center.respond(confirmation_id, "u1", "approve")

        for _ in range(30):
            item = self.store.get_confirmation(confirmation_id)
            if item["status"] == "completed":
                break
            await asyncio.sleep(0.01)

        self.assertEqual(item["status"], "completed")
        progress_event = next(
            event
            for event in self.store.events_after("s1")
            if event["type"] == "control.progress"
        )
        covariance = progress_event["details"]["evidence"][
            "position_covariance"
        ]
        self.assertAlmostEqual(covariance[0], 0.1)

    async def test_wrong_user_cannot_approve(self):
        created = await self.center.create("s1", "u1", "land", {})
        with self.assertRaises(PermissionError):
            await self.center.respond(created["confirmation_id"], "u2", "approve")
        self.assertEqual(self.executions, [])

    async def test_late_success_cannot_overwrite_cancelled_terminal_state(self):
        release = asyncio.Event()

        async def delayed_success(action, args, progress):
            await progress("verifying", {"message": "waiting"})
            await release.wait()
            return {"success": True, "message": "late success", "physical_complete": True}

        self.center._execute = delayed_success
        created = await self.center.create("s1", "u1", "takeoff", {"altitude": 5})
        confirmation_id = created["confirmation_id"]
        await self.center.respond(confirmation_id, "u1", "approve")
        await asyncio.sleep(0.02)
        cancelled_result = {"success": False, "status": "cancelled", "message": "operator cancelled"}
        self.store.complete_confirmation(confirmation_id, "cancelled", cancelled_result)
        self.store.update_gateway_mission(
            confirmation_id, "cancelled", cancelled_result, phase="cancelled"
        )
        release.set()

        for _ in range(30):
            events = self.store.events_after("s1")
            if any(event["type"] == "control.completed" for event in events):
                break
            await asyncio.sleep(0.01)

        mission = self.store.gateway_mission_for_confirmation(confirmation_id)
        self.assertEqual(mission["status"], "cancelled")
        self.assertFalse(mission["physical_complete"])
        completed_event = next(
            event for event in events if event["type"] == "control.completed"
        )
        self.assertFalse(completed_event["success"])
        self.assertEqual(completed_event["result"]["message"], "operator cancelled")

    async def test_cancelled_workflow_is_not_reported_as_failed(self):
        async def cancelled(_action, _args, _progress):
            return {
                "success": False,
                "status": "cancelled",
                "message": "workflow cancelled",
            }

        self.center._execute = cancelled
        created = await self.center.create(
            "s1",
            "u1",
            "workflow",
            {
                "name": "测试任务",
                "steps": [{"id": "hover", "action": "hover"}],
            },
        )
        await self.center.respond(created["confirmation_id"], "u1", "approve")
        for _ in range(20):
            item = self.store.get_confirmation(created["confirmation_id"])
            if item["status"] == "cancelled":
                break
            await asyncio.sleep(0.01)
        self.assertEqual(item["status"], "cancelled")
        mission = self.store.gateway_mission_for_confirmation(item["id"])
        self.assertEqual(mission["status"], "cancelled")

    async def test_expired_confirmation_cannot_execute(self):
        created = await self.center.create("s1", "u1", "hover", {})
        await asyncio.sleep(1.05)
        with self.assertRaises(ValueError):
            await self.center.respond(created["confirmation_id"], "u1", "approve")
        self.assertEqual(self.executions, [])
        self.assertEqual(
            self.store.get_confirmation(created["confirmation_id"])["status"],
            "expired",
        )
        mission = self.store.gateway_mission_for_confirmation(
            created["confirmation_id"]
        )
        self.assertEqual(mission["status"], "expired")

    async def test_expiration_is_pushed_as_events(self):
        queue = await self.events.subscribe("s1")
        await self.center.create("s1", "u1", "hover", {})
        self.assertEqual((await queue.get())["type"], "mission.updated")
        self.assertEqual((await queue.get())["type"], "confirmation.required")
        self.assertEqual((await queue.get())["type"], "confirmation.expired")
        self.assertEqual((await queue.get())["type"], "mission.updated")
        await self.events.unsubscribe("s1", queue)

    def test_action_bounds(self):
        self.assertEqual(
            validate_action("move", {"direction": "left", "distance": 20}),
            {"direction": "左侧", "distance": 20.0},
        )
        with self.assertRaises(ValueError):
            validate_action("takeoff", {"altitude": 100})
        with self.assertRaises(ValueError):
            validate_action("move", {"direction": "around", "distance": 5})


if __name__ == "__main__":
    unittest.main()
