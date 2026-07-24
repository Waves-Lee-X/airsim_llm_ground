import asyncio
import threading
import time
import unittest

from aeromind_agent_gateway.ros_state import RosStateBridge
from aeromind_agent_gateway.skill_registry import build_skill, skill_catalog
from aeromind_agent_gateway.ros_state import _action_task
from aeromind_agent_gateway.capability_registry import capability_catalog
from aeromind_agent_gateway.workflow import validate_workflow, workflow_json_schema


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
        path = next(step for step in workflow["steps"] if step["action"] == "follow_waypoints")
        self.assertEqual(len(path["args"]["points"]), 4)
        self.assertEqual(path["args"]["points"][2]["forward_m"], -8.0)
        self.assertEqual(workflow["steps"][0]["condition"], "if_not_airborne")
        self.assertEqual(workflow["steps"][-1]["action"], "land")
        self.assertEqual(skill_catalog()[0]["name"], "flight.square")

    def test_v_shape_skill_uses_vector_moves_and_capture_action(self):
        workflow = build_skill(
            "flight.v_shape",
            {"width": 10, "depth": 8, "capture_at_vertex": True},
        )
        self.assertEqual(
            [step["action"] for step in workflow["steps"]],
            ["takeoff", "move", "capture_image", "move"],
        )
        self.assertEqual(
            workflow["steps"][1]["args"],
            {"forward_m": 8.0, "right_m": 5.0, "up_m": 0.0},
        )
        task, intent = _action_task("move", workflow["steps"][1]["args"])
        self.assertTrue(task.startswith("__aeromind_move_vector__:"))
        self.assertEqual(intent, "move_to")
        self.assertEqual(workflow["steps"][2]["retries"], 0)

    def test_person_inspection_skill_contains_true_and_false_branches(self):
        workflow = build_skill("inspection.person_branch", {"distance": 20})
        conditions = {
            step["id"]: step["condition"] for step in workflow["steps"]
        }
        self.assertEqual(
            conditions["capture-on-person"]["type"], "target_detected"
        )
        self.assertEqual(
            conditions["continue-if-clear"]["type"], "target_not_detected"
        )
        self.assertEqual(
            conditions["hover-if-sensor-unavailable"]["type"], "step_failed"
        )

    def test_skill_catalog_is_loaded_from_manifests_with_parameters(self):
        skills = {item["name"]: item for item in skill_catalog()}
        self.assertEqual(skills["flight.square"]["version"], "3.0.0")
        self.assertIn("side_length", skills["flight.square"]["parameters"])
        with self.assertRaises(ValueError):
            build_skill("flight.square", {"side_length": 10, "unknown": True})

    def test_workflow_rejects_unknown_actions_and_forward_dependencies(self):
        with self.assertRaises(ValueError):
            validate_workflow(
                {
                    "name": "危险任务",
                    "steps": [{"id": "shell", "action": "bash", "args": {}}],
                }
            )

    def test_repeat_block_expands_into_bounded_sequential_steps(self):
        workflow = validate_workflow(
            {
                "name": "重复巡检",
                "steps": [
                    {
                        "id": "scan",
                        "repeat": {
                            "count": 2,
                            "steps": [
                                {
                                    "id": "move",
                                    "action": "move",
                                    "args": {"direction": "forward", "distance": 5},
                                },
                                {"id": "capture", "action": "capture_image"},
                            ],
                        },
                    },
                    {
                        "id": "report",
                        "action": "mission_report",
                        "depends_on": ["scan"],
                    },
                ],
            }
        )

        self.assertEqual(
            [step["id"] for step in workflow["steps"]],
            [
                "scan-1-move",
                "scan-1-capture",
                "scan-2-move",
                "scan-2-capture",
                "report",
            ],
        )
        self.assertEqual(
            workflow["steps"][2]["depends_on"], ["scan-1-capture"]
        )
        self.assertEqual(
            workflow["steps"][-1]["depends_on"], ["scan-2-capture"]
        )

    def test_waypoint_primitive_is_validated_and_cannot_auto_retry(self):
        workflow = validate_workflow(
            {
                "name": "三角形",
                "steps": [
                    {
                        "id": "path",
                        "action": "follow_waypoints",
                        "args": {
                            "points": [
                                {"forward_m": 8, "right_m": 4},
                                {"forward_m": 0, "right_m": 8},
                                {"forward_m": -8, "right_m": -4},
                            ]
                        },
                    }
                ],
            }
        )
        self.assertEqual(len(workflow["steps"][0]["args"]["points"]), 3)
        with self.assertRaisesRegex(ValueError, "不允许自动重试"):
            validate_workflow(
                {
                    "name": "错误重试",
                    "steps": [
                        {
                            "action": "follow_waypoints",
                            "args": {"points": [{"forward_m": 2}]},
                            "retries": 1,
                        }
                    ],
                }
            )

    def test_model_schema_and_capability_catalog_share_action_definitions(self):
        schema_text = str(workflow_json_schema())
        self.assertIn("follow_waypoints", schema_text)
        self.assertIn("analyze_image", schema_text)
        self.assertIn("return_to_start", schema_text)
        capabilities = {item["name"]: item for item in capability_catalog()}
        self.assertIn("parameters", capabilities["flight.follow_waypoints"])
        self.assertIn("parameters", capabilities["flight.return_to_start"])
        self.assertIn("preconditions", capabilities["perception.semantic_image"])
        with self.assertRaises(ValueError):
            validate_workflow(
                {
                    "name": "错误条件引用",
                    "steps": [
                        {
                            "id": "capture",
                            "action": "capture_image",
                            "condition": {
                                "type": "step_succeeded",
                                "step_id": "future",
                            },
                        }
                    ],
                }
            )
        with self.assertRaisesRegex(ValueError, "相对移动，不允许自动重试"):
            validate_workflow(
                {
                    "name": "危险移动重试",
                    "steps": [
                        {
                            "id": "move",
                            "action": "move",
                            "args": {"direction": "forward", "distance": 2},
                            "retries": 1,
                        }
                    ],
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
            if action == "hover" and len(attempts) == 1:
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
                        "id": "hover",
                        "action": "hover",
                        "retries": 1,
                    },
                    {
                        "id": "land",
                        "action": "land",
                        "depends_on": ["hover"],
                    },
                ],
            }
        )
        result = await bridge._execute_workflow(workflow, progress)

        self.assertTrue(result["success"])
        self.assertEqual([item[0] for item in attempts], ["hover", "hover", "land"])
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

    async def test_person_inspection_selects_capture_branch(self):
        bridge = self._bridge()
        bridge.snapshot = lambda: {
            "available": True,
            "state": {"stamp": time.time(), "armed": True},
            "odometry": {"position_m": {"z": 5.0}},
            "autonomy": {},
            "detections_fresh": True,
            "detections": [{"class_name": "person", "confidence": 0.9}],
        }
        called = []

        async def execute(action, args, progress):
            called.append(action)
            return {"success": True, "physical_complete": True, "message": "done"}

        bridge.execute_confirmed_action = execute

        async def progress(_phase, _details):
            return None

        result = await bridge._execute_workflow(
            build_skill("inspection.person_branch", {"distance": 20}), progress
        )
        self.assertTrue(result["success"])
        self.assertEqual(called, ["hover", "capture_image"])

    async def test_person_inspection_moves_only_after_fresh_negative_observation(self):
        bridge = self._bridge()
        bridge.snapshot = lambda: {
            "available": True,
            "state": {"stamp": time.time(), "armed": True},
            "odometry": {"position_m": {"z": 5.0}},
            "autonomy": {},
            "detections_fresh": True,
            "detections": [],
        }
        called = []

        async def execute(action, args, progress):
            called.append(action)
            return {"success": True, "physical_complete": True, "message": "done"}

        bridge.execute_confirmed_action = execute

        async def progress(_phase, _details):
            return None

        result = await bridge._execute_workflow(
            build_skill("inspection.person_branch", {"distance": 20}), progress
        )
        self.assertTrue(result["success"])
        self.assertEqual(called, ["move"])

    async def test_capture_action_completes_without_telemetry_wait(self):
        bridge = self._bridge()

        async def execute_action(action, args, request_id, timeout_sec):
            self.assertEqual(action, "capture_image")
            self.assertEqual(args, {})
            self.assertTrue(request_id.startswith("gateway-"))
            self.assertEqual(timeout_sec, 45.0)
            return {
                "success": True,
                "message": "图像已保存",
                "result": '{"parsed_task":{"intent":"capture_image"}}',
            }

        bridge._execute_action_service = execute_action
        events = []

        async def progress(phase, details):
            events.append((phase, details))

        result = await bridge.execute_confirmed_action(
            "capture_image", {}, progress
        )
        self.assertTrue(result["success"])
        self.assertTrue(result["physical_complete"])
        self.assertEqual(events[-1][0], "completed")

    async def test_generic_waypoint_vlm_wait_and_report_primitives_execute(self):
        bridge = self._bridge()
        waypoint_requests = []

        async def execute_waypoints(args, progress):
            waypoint_requests.append(args)
            return {
                "success": True,
                "physical_complete": True,
                "message": "Action done",
                "completed_waypoints": len(args["points"]),
            }

        async def analyze(prompt):
            return {
                "success": True,
                "message": "VLM 完成",
                "source": "vlm",
                "prompt": prompt,
            }

        bridge._execute_waypoint_action = execute_waypoints
        bridge.analyze_current_image = analyze

        async def progress(_phase, _details):
            return None

        result = await bridge._execute_workflow(
            {
                "name": "通用组合任务",
                "steps": [
                    {
                        "id": "path",
                        "action": "follow_waypoints",
                        "args": {
                            "points": [
                                {"forward_m": 2},
                                {"right_m": 2},
                            ],
                            "capture_on_semantic_hold": True,
                        },
                    },
                    {
                        "id": "wait",
                        "action": "wait",
                        "args": {"duration_sec": 0.1},
                        "depends_on": ["path"],
                    },
                    {
                        "id": "vision",
                        "action": "analyze_image",
                        "args": {"prompt": "检查画面中是否有人"},
                        "depends_on": ["wait"],
                    },
                    {
                        "id": "report",
                        "action": "mission_report",
                        "depends_on": ["vision"],
                    },
                ],
            },
            progress,
        )
        self.assertTrue(
            waypoint_requests[0]["capture_on_semantic_hold"]
        )

        self.assertTrue(result["success"])
        self.assertEqual(len(waypoint_requests), 1)
        self.assertEqual(len(waypoint_requests[0]["points"]), 2)
        self.assertEqual(result["step_results"]["vision"]["source"], "vlm")
        self.assertIn("snapshot", result["step_results"]["report"])

    async def test_demo_skill_returns_to_recorded_start_before_landing(self):
        bridge = self._bridge()
        pose = {"x": 2.0, "y": -3.0, "z": 0.0}
        yaw = 0.0

        def snapshot():
            return {
                "available": True,
                "state": {
                    "stamp": time.time(),
                    "armed": pose["z"] > 0.5,
                    "ekf_healthy": True,
                    "gps_fix": 3,
                },
                "odometry": {
                    "stamp": time.time(),
                    "frame_id": "odom",
                    "position_m": dict(pose),
                    "orientation": {"yaw": yaw},
                },
                "autonomy": {
                    "stamp": time.time(),
                    "nearest_obstacle_m": 8.0,
                    "takeoff_clearance_valid": True,
                    "takeoff_clearance_m": 8.0,
                },
                "detections": [],
            }

        bridge.snapshot = snapshot
        calls = []

        async def execute(action, args, progress):
            calls.append((action, dict(args)))
            if action == "takeoff":
                pose["z"] = float(args["altitude"])
            elif action == "move":
                direction = args.get("direction")
                distance = float(args.get("distance", 0.0))
                pose["x"] += float(args.get("right_m", 0.0))
                pose["y"] += float(args.get("forward_m", 0.0))
                if direction == "forward":
                    pose["y"] += distance
                elif direction == "backward":
                    pose["y"] -= distance
            elif action == "land":
                pose["z"] = 0.0
            return {"success": True, "physical_complete": True, "message": "done"}

        bridge.execute_confirmed_action = execute
        bridge.analyze_current_image = lambda _prompt: asyncio.sleep(
            0,
            result={"success": True, "message": "done", "source": "vlm"},
        )

        async def progress(_phase, _details):
            return None

        result = await bridge._execute_workflow(
            build_skill(
                "mission.demo_main",
                {
                    "altitude": 10,
                    "distance": 10,
                    "minimum_obstacle_distance": 2,
                    "require_gps": True,
                },
            ),
            progress,
        )

        self.assertTrue(result["success"])
        actions = [item[0] for item in calls]
        self.assertEqual(actions[:3], ["takeoff", "move", "hover"])
        self.assertEqual(actions[-2:], ["move", "land"])
        self.assertAlmostEqual(calls[-2][1]["forward_m"], -10.0)
        self.assertAlmostEqual(pose["x"], 2.0)
        self.assertAlmostEqual(pose["y"], -3.0)
        self.assertEqual(
            result["step_results"]["return-to-start"]["horizontal_error_m"],
            0.0,
        )
        self.assertEqual(
            result["workflow_context"]["start_pose"]["frame_id"],
            "odom",
        )

    async def test_return_to_start_rejects_final_position_outside_tolerance(self):
        bridge = self._bridge()
        snapshots = [
            {
                "odometry": {
                    "stamp": time.time(),
                    "frame_id": "odom",
                    "position_m": {"x": 5.0, "y": 0.0, "z": 8.0},
                    "orientation": {"yaw": 0.0},
                }
            },
            {
                "odometry": {
                    "stamp": time.time(),
                    "frame_id": "odom",
                    "position_m": {"x": 2.0, "y": 0.0, "z": 8.0},
                    "orientation": {"yaw": 0.0},
                }
            },
        ]
        bridge.snapshot = lambda: snapshots.pop(0) if len(snapshots) > 1 else snapshots[0]

        async def execute(_action, _args, _progress):
            return {"success": True, "physical_complete": True, "message": "goal reached"}

        bridge.execute_confirmed_action = execute

        async def progress(_phase, _details):
            return None

        result = await bridge._execute_return_to_start(
            {"horizontal_tolerance_m": 1.0},
            progress,
            {
                "start_pose": {
                    "frame_id": "odom",
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "valid": True,
                }
            },
        )

        self.assertFalse(result["success"])
        self.assertEqual(result["horizontal_error_m"], 2.0)
        self.assertIn("超过容差", result["message"])

    async def test_vlm_semantic_and_risk_conditions_select_safe_branch(self):
        bridge = self._bridge()
        called = []

        async def analyze(_prompt):
            return {
                "success": True,
                "message": "分析完成",
                "source": "vlm",
                "risk_level": "high",
                "objects": [{"name": "person", "confidence": 0.8}],
            }

        async def execute(action, args, progress):
            called.append(action)
            return {"success": True, "physical_complete": True, "message": "done"}

        bridge.analyze_current_image = analyze
        bridge.execute_confirmed_action = execute

        async def progress(_phase, _details):
            return None

        result = await bridge._execute_workflow(
            {
                "name": "VLM 风险分支",
                "steps": [
                    {
                        "id": "vision",
                        "action": "analyze_image",
                        "args": {"prompt": "分析人员与飞行风险"},
                    },
                    {
                        "id": "hover-person",
                        "action": "hover",
                        "condition": {
                            "type": "semantic_target_detected",
                            "step_id": "vision",
                            "target": "人",
                        },
                    },
                    {
                        "id": "capture-high-risk",
                        "action": "capture_image",
                        "condition": {
                            "type": "risk_level_is",
                            "step_id": "vision",
                            "value": "high",
                        },
                    },
                    {
                        "id": "move-if-clear",
                        "action": "move",
                        "args": {"direction": "forward", "distance": 5},
                        "condition": {
                            "type": "semantic_target_not_detected",
                            "step_id": "vision",
                            "target": "person",
                        },
                    },
                ],
            },
            progress,
        )

        self.assertTrue(result["success"])
        self.assertEqual(called, ["hover", "capture_image"])
        self.assertTrue(result["step_results"]["move-if-clear"]["skipped"])

    async def test_failed_move_verification_cancels_active_autonomy_goal(self):
        bridge = self._bridge()

        async def execute_action(_action, _args, request_id, timeout_sec):
            self.assertTrue(request_id.startswith("gateway-"))
            self.assertEqual(timeout_sec, 45.0)
            return {"success": True, "message": "目标已发布", "result": "{}"}

        async def wait_for_completion(*_args, **_kwargs):
            return {
                "terminal": True,
                "success": False,
                "phase": "timeout",
                "message": "移动验证超时",
            }

        bridge._execute_action_service = execute_action
        bridge._wait_for_action_completion = wait_for_completion
        events = []

        async def progress(phase, details):
            events.append((phase, details))

        result = await bridge.execute_confirmed_action(
            "move", {"direction": "forward", "distance": 10.0}, progress
        )

        self.assertFalse(result["success"])
        self.assertEqual(len(bridge._autonomy_cancel_pub.messages), 1)
        self.assertEqual(events[-1][0], "stopped")

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
            "autonomy": {
                "stamp": time.time(),
                "nearest_obstacle_m": 0.8,
                "takeoff_clearance_valid": True,
                "takeoff_clearance_m": 0.8,
                "takeoff_clearance_source": "test_esdf",
            },
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
                    {
                        "id": "safe",
                        "action": "safety_check",
                        "args": {"check_takeoff_zone": True},
                    },
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
        self.assertIn("最近障碍物 0.80 米", result["message"])
        self.assertIn("安全距离 2.00 米", result["message"])
        self.assertEqual(result["workflow"]["steps"][0]["status"], "failed")
        self.assertEqual(result["workflow"]["steps"][1]["status"], "skipped")

    async def test_cancelled_workflow_settles_running_and_pending_steps(self):
        bridge = self._bridge()
        workflow = {
            "workflow_id": "workflow-cancelled",
            "name": "取消测试",
            "steps": [
                {"id": "move", "status": "running"},
                {"id": "capture", "status": "pending"},
            ],
        }
        result = bridge._cancelled_workflow_result(workflow, {})
        self.assertEqual(result["status"], "cancelled")
        self.assertEqual(workflow["steps"][0]["status"], "cancelled")
        self.assertEqual(workflow["steps"][1]["status"], "skipped")

    def test_takeoff_safety_rejects_missing_clearance_evidence(self):
        bridge = self._bridge()
        now = time.time()
        bridge.snapshot = lambda: {
            "available": True,
            "state": {
                "stamp": now,
                "fresh": True,
                "armed": False,
                "ekf_healthy": True,
                "gps_fix": 2,
            },
            "autonomy": {
                "stamp": now,
                "fresh": True,
                "nearest_obstacle_m": 5.0,
                "takeoff_clearance_valid": False,
                "takeoff_clearance_m": 12.0,
                "takeoff_clearance_source": "unavailable",
            },
            "evidence": [],
        }

        result = bridge._safety_check({"check_takeoff_zone": True})

        self.assertFalse(result["success"])
        self.assertIn("起飞区域净空证据不可用", result["message"])
        self.assertIsNone(result["checks"]["nearest_obstacle_m"])

    def test_takeoff_safety_rejects_px4_preflight_failure(self):
        bridge = self._bridge()
        now = time.time()
        bridge.snapshot = lambda: {
            "available": True,
            "state": {
                "stamp": now,
                "armed": False,
                "ekf_healthy": True,
                "preflight_valid": True,
                "preflight_ok": False,
                "gps_fix": 2,
            },
            "autonomy": {
                "stamp": now,
                "takeoff_clearance_valid": True,
                "takeoff_clearance_m": 6.0,
                "takeoff_clearance_source": "test",
            },
            "evidence": [],
        }

        result = bridge._safety_check({"check_takeoff_zone": True})

        self.assertFalse(result["success"])
        self.assertIn("PX4 解锁预检未通过", result["message"])
        self.assertFalse(result["checks"]["preflight_ok"])

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
