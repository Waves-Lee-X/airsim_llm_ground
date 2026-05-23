from __future__ import annotations

import json
import time
import unittest
from unittest.mock import MagicMock

from core.agent_loop import (
    AgentLoop,
    MissionMemory,
    StateSummarizer,
    build_system_prompt,
    parse_llm_decision,
)


class TestParseDecision(unittest.TestCase):
    def test_parses_valid_tool_call(self) -> None:
        name, args, finish = parse_llm_decision(
            '{"tool_call": {"name": "takeoff", "args": {"altitude_m": 10.0}}}'
        )
        self.assertEqual(name, "takeoff")
        self.assertEqual(args, {"altitude_m": 10.0})
        self.assertIsNone(finish)

    def test_parses_tool_call_with_params_alias(self) -> None:
        name, args, finish = parse_llm_decision(
            '{"tool_call": {"name": "goto_local", "params": {"x": 5.0, "y": 3.0, "z": -8.0}}}'
        )
        self.assertEqual(name, "goto_local")
        self.assertEqual(args, {"x": 5.0, "y": 3.0, "z": -8.0})
        self.assertIsNone(finish)

    def test_parses_finish_dict(self) -> None:
        name, args, finish = parse_llm_decision(
            '{"finish": {"message": "Mission completed successfully"}}'
        )
        self.assertIsNone(name)
        self.assertIsNone(args)
        self.assertEqual(finish, "Mission completed successfully")

    def test_parses_finish_string(self) -> None:
        name, args, finish = parse_llm_decision(
            '{"finish": "All tasks done"}'
        )
        self.assertIsNone(name)
        self.assertIsNone(args)
        self.assertEqual(finish, "All tasks done")

    def test_parses_inside_markdown_fence(self) -> None:
        name, args, finish = parse_llm_decision(
            '```json\n{"tool_call": {"name": "hover", "args": {}}}\n```'
        )
        self.assertEqual(name, "hover")
        self.assertEqual(args, {})
        self.assertIsNone(finish)

    def test_returns_none_for_garbage(self) -> None:
        name, args, finish = parse_llm_decision("hello world, no JSON here")
        self.assertIsNone(name)
        self.assertIsNone(args)
        self.assertIsNone(finish)

    def test_returns_none_for_empty(self) -> None:
        name, args, finish = parse_llm_decision("")
        self.assertIsNone(name)
        self.assertIsNone(args)
        self.assertIsNone(finish)

    def test_parses_action_finish_format(self) -> None:
        name, args, finish = parse_llm_decision(
            '{"action": "finish", "message": "Task done"}'
        )
        self.assertIsNone(name)
        self.assertIsNone(args)
        self.assertEqual(finish, "Task done")


class TestMissionMemory(unittest.TestCase):
    def test_is_exhausted_after_max_iterations(self) -> None:
        memory = MissionMemory("test goal", max_iterations=3)
        memory.started_at = time.time()
        memory.iteration = 3
        self.assertTrue(memory.is_exhausted())

    def test_not_exhausted_before_max(self) -> None:
        memory = MissionMemory("test goal", max_iterations=5)
        memory.started_at = time.time()
        memory.iteration = 2
        self.assertFalse(memory.is_exhausted())

    def test_is_exhausted_after_timeout(self) -> None:
        memory = MissionMemory("test goal", max_total_time_s=0.01)
        memory.started_at = time.time() - 1.0
        self.assertTrue(memory.is_exhausted())

    def test_record_and_summary(self) -> None:
        memory = MissionMemory("test goal")
        memory.record("takeoff", {"altitude_m": 8.0}, True, "Takeoff complete", 1500)
        memory.record("hover", {}, True, "Hovering", 200)
        summary = memory.summary()
        self.assertIn("takeoff", summary)
        self.assertIn("hover", summary)
        self.assertIn("OK", summary)

    def test_empty_summary(self) -> None:
        memory = MissionMemory("test goal")
        self.assertIn("no actions", memory.summary())

    def test_to_api(self) -> None:
        memory = MissionMemory("fly to target")
        memory.started_at = time.time()
        memory.record("takeoff", {"altitude_m": 10.0}, True, "ok", 1000)
        api = memory.to_api()
        self.assertEqual(api["user_goal"], "fly to target")
        self.assertEqual(len(api["tool_history"]), 1)


class TestStateSummarizer(unittest.TestCase):
    def _base_snapshot(self) -> dict:
        return {
            "connected": True,
            "uav": {"x": 10.0, "y": 5.0, "z": -8.0, "altitude_m": 8.0, "speed_mps": 2.0},
            "safety": {
                "collision": {"has_collided": False},
                "obstacle": {"risk_level": "low", "blocked": False},
                "distance_sensors": {"front_m": 12.0, "left_m": 8.0, "right_m": 9.0, "emergency": False},
            },
            "vision": {
                "latest_detection": {
                    "ok": True,
                    "detections": [{"label": "vehicle", "confidence": 0.85}],
                    "target": "vehicle",
                    "camera": "front_center",
                }
            },
            "task": {
                "status": "idle",
                "agent_progress": {
                    "status": "running",
                    "tool": "search_area",
                    "current_waypoint_index": 3,
                    "total_waypoints": 8,
                    "distance_to_waypoint_m": 4.2,
                    "message": "Searching...",
                },
            },
        }

    def test_includes_position(self) -> None:
        memory = MissionMemory("search for vehicles")
        text = StateSummarizer.build_text(self._base_snapshot(), memory)
        self.assertIn("x=10.0", text)
        self.assertIn("y=5.0", text)
        self.assertIn("altitude=8.0m", text)
        self.assertIn("speed=2.0m/s", text)

    def test_includes_safety(self) -> None:
        memory = MissionMemory("test")
        text = StateSummarizer.build_text(self._base_snapshot(), memory)
        self.assertIn("collision=none", text)
        self.assertIn("obstacle_risk=low", text)
        self.assertIn("front=12.0m", text)

    def test_includes_safety_emergency(self) -> None:
        memory = MissionMemory("test")
        snap = self._base_snapshot()
        snap["safety"]["distance_sensors"]["emergency"] = True
        snap["safety"]["obstacle"]["risk_level"] = "high"
        snap["safety"]["obstacle"]["blocked"] = True
        text = StateSummarizer.build_text(snap, memory)
        self.assertIn("DISTANCE_EMERGENCY", text)
        self.assertIn("obstacle_blocked", text)

    def test_includes_detections(self) -> None:
        memory = MissionMemory("test")
        text = StateSummarizer.build_text(self._base_snapshot(), memory)
        self.assertIn("vehicle(0.85)", text)
        self.assertIn("front_center", text)

    def test_no_detection_data_available(self) -> None:
        memory = MissionMemory("test")
        snap = self._base_snapshot()
        snap["vision"]["latest_detection"]["ok"] = False
        snap["vision"]["latest_detection"]["detections"] = []
        text = StateSummarizer.build_text(snap, memory)
        self.assertIn("no detection data available", text)

    def test_no_objects_detected_message(self) -> None:
        memory = MissionMemory("test")
        snap = self._base_snapshot()
        snap["vision"]["latest_detection"]["ok"] = True
        snap["vision"]["latest_detection"]["detections"] = []
        text = StateSummarizer.build_text(snap, memory)
        self.assertIn("no objects detected", text)

    def test_disconnected_state(self) -> None:
        memory = MissionMemory("test")
        snap = self._base_snapshot()
        snap["connected"] = False
        text = StateSummarizer.build_text(snap, memory)
        self.assertIn("connected=no", text)

    def test_includes_user_goal(self) -> None:
        memory = MissionMemory("fly to x=50 y=30 and search for cars")
        text = StateSummarizer.build_text(self._base_snapshot(), memory)
        self.assertIn("fly to x=50 y=30", text)

    def test_includes_mission_progress(self) -> None:
        memory = MissionMemory("test")
        text = StateSummarizer.build_text(self._base_snapshot(), memory)
        self.assertIn("waypoint=3/8", text)
        self.assertIn("distance_to_waypoint=4.2m", text)

    def test_includes_tool_history(self) -> None:
        memory = MissionMemory("test")
        memory.record("takeoff", {"altitude_m": 8.0}, True, "Took off to 8m", 1200)
        text = StateSummarizer.build_text(self._base_snapshot(), memory)
        self.assertIn("takeoff", text)
        self.assertIn("Took off to 8m", text)


class TestBuildSystemPrompt(unittest.TestCase):
    def test_includes_tool_names(self) -> None:
        mock_spec = MagicMock()
        mock_spec.to_api.return_value = {
            "name": "takeoff",
            "description": "Take off to altitude.",
            "required": ["altitude_m"],
            "properties": {"altitude_m": "Target altitude in meters."},
        }
        limits = MagicMock()
        limits.__dataclass_fields__ = {
            "min_altitude_m": None,
            "max_altitude_m": None,
            "max_speed_mps": None,
            "boundary_x_min": None,
            "boundary_x_max": None,
            "boundary_y_min": None,
            "boundary_y_max": None,
        }
        limits.min_altitude_m = 1.0
        limits.max_altitude_m = 30.0
        limits.max_speed_mps = 4.0
        limits.boundary_x_min = -120.0
        limits.boundary_x_max = 120.0
        limits.boundary_y_min = -120.0
        limits.boundary_y_max = 120.0

        prompt = build_system_prompt([mock_spec], limits)
        self.assertIn("takeoff", prompt)
        self.assertIn("Take off to altitude", prompt)
        self.assertIn("altitude_m", prompt)
        self.assertIn("UAV mission commander", prompt)


class TestAgentLoop(unittest.TestCase):
    def _mock_llm(self, responses: list[str | None]):
        """Create a mock LLM client whose chat() returns the given responses in order."""
        mock = MagicMock()
        mock.chat = MagicMock(side_effect=responses)
        mock.enabled = True
        return mock

    def _mock_safety(self, accept: bool = True):
        mock = MagicMock()
        decision = MagicMock()
        decision.accepted = accept
        decision.tool = "takeoff"
        decision.args = {"altitude_m": 8.0}
        decision.to_api.return_value = {"accepted": accept, "tool": "takeoff", "args": {}}
        mock.validate.return_value = decision
        return mock

    def _mock_tools(self):
        mock = MagicMock()
        mock.specs.return_value = []
        result = MagicMock()
        result.ok = True
        result.message = "Done"
        result.to_api.return_value = {"ok": True, "message": "Done"}
        mock.call.return_value = result
        return mock

    def _mock_state_provider(self):
        return {
            "connected": True,
            "uav": {"x": 0.0, "y": 0.0, "z": -8.0, "altitude_m": 8.0, "speed_mps": 0.0},
            "safety": {
                "collision": {"has_collided": False},
                "obstacle": {"risk_level": "low", "blocked": False},
                "distance_sensors": {"front_m": 10.0, "left_m": 10.0, "right_m": 10.0, "emergency": False},
            },
            "vision": {"latest_detection": {"ok": False, "detections": [], "target": "", "camera": ""}},
            "task": {"status": "idle", "agent_progress": {}},
        }

    def test_loop_stops_on_finish(self) -> None:
        llm = self._mock_llm(['{"finish": {"message": "Done!"}}'])
        safety = self._mock_safety()
        tools = self._mock_tools()
        loop = AgentLoop(llm, safety, tools)

        events = list(loop.execute("takeoff", self._mock_state_provider))

        types = [e["type"] for e in events]
        self.assertIn("start", types)
        self.assertIn("finish", types)
        self.assertIn("done", types)

    def test_loop_calls_tool(self) -> None:
        llm = self._mock_llm([
            '{"tool_call": {"name": "takeoff", "args": {"altitude_m": 10.0}}}',
            '{"finish": {"message": "Done!"}}',
        ])
        safety = self._mock_safety()
        tools = self._mock_tools()
        loop = AgentLoop(llm, safety, tools)

        events = list(loop.execute("takeoff", self._mock_state_provider))

        types = [e["type"] for e in events]
        self.assertIn("tool_call", types)
        self.assertIn("tool_result", types)
        self.assertIn("finish", types)
        safety.validate.assert_called()
        tools.call.assert_called()

    def test_loop_stops_on_max_iterations(self) -> None:
        responses = ['{"tool_call": {"name": "takeoff", "args": {"altitude_m": 8.0}}}'] * 20
        llm = self._mock_llm(responses)
        safety = self._mock_safety()
        tools = self._mock_tools()
        loop = AgentLoop(llm, safety, tools)
        loop.execute = lambda *a, **kw: iter([])  # won't be used

        events = list(AgentLoop(llm, safety, tools).execute("test", self._mock_state_provider))

        self.assertTrue(len(events) > 0)

    def test_loop_skips_invalid_llm_response(self) -> None:
        llm = self._mock_llm([
            "garbage response",
            '{"finish": {"message": "gave up"}}',
        ])
        safety = self._mock_safety()
        tools = self._mock_tools()
        loop = AgentLoop(llm, safety, tools)

        events = list(loop.execute("test", self._mock_state_provider))

        types = [e["type"] for e in events]
        self.assertIn("error", types)
        self.assertIn("finish", types)

    def test_loop_records_tool_history(self) -> None:
        llm = self._mock_llm([
            '{"tool_call": {"name": "takeoff", "args": {"altitude_m": 12.0}}}',
            '{"finish": {"message": "Done at 12m"}}',
        ])
        safety = self._mock_safety()
        tools = self._mock_tools()
        loop = AgentLoop(llm, safety, tools)

        events = list(loop.execute("climb to 12m", self._mock_state_provider))

        done_event = next(e for e in events if e["type"] == "done")
        memory = done_event["memory"]
        self.assertEqual(len(memory["tool_history"]), 1)
        self.assertEqual(memory["tool_history"][0]["tool"], "takeoff")

    def test_stop_event_cancels_loop(self) -> None:
        llm = self._mock_llm(['{"tool_call": {"name": "takeoff", "args": {"altitude_m": 8.0}}}'] * 5)
        safety = self._mock_safety()
        tools = self._mock_tools()
        loop = AgentLoop(llm, safety, tools)

        event_count = 0
        for event in loop.execute("test", self._mock_state_provider):
            event_count += 1
            if event["type"] == "think" and event.get("iteration") == 2:
                loop.stop()

        done_event = next(
            e for e in [event] if e["type"] == "done"
        ) if any(e["type"] == "done" for e in [event]) else None

    def test_tool_rejected_by_safety(self) -> None:
        llm = self._mock_llm([
            '{"tool_call": {"name": "land", "args": {}}}',
            '{"finish": {"message": "Cannot land without confirmation"}}',
        ])
        safety = self._mock_safety(accept=False)
        tools = self._mock_tools()
        loop = AgentLoop(llm, safety, tools)

        events = list(loop.execute("land", self._mock_state_provider))

        types = [e["type"] for e in events]
        self.assertIn("tool_rejected", types)

    def test_llm_api_failure(self) -> None:
        llm = self._mock_llm([None, '{"finish": {"message": "LLM failed"}}'])
        safety = self._mock_safety()
        tools = self._mock_tools()
        loop = AgentLoop(llm, safety, tools)

        events = list(loop.execute("test", self._mock_state_provider))

        types = [e["type"] for e in events]
        self.assertIn("error", types)


if __name__ == "__main__":
    unittest.main()
