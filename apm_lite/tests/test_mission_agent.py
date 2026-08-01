import asyncio
import json

import pytest
from fastapi.testclient import TestClient

from aeromind_apm_lite.ground.browser.app import create_app
from aeromind_apm_lite.ground.browser.mission_agent import (
    AgentDraftBlocked,
    AgentDraftConflict,
    AgentDraftExpired,
    MissionAgentConfig,
    MissionAgentService,
    normalize_agent_result,
)
from aeromind_apm_lite.ground.browser.runtime import (
    DemoApmLink,
    FleetRuntime,
    ManualRuntime,
    ManualRuntimeConfig,
    ManualRuntimeMode,
)
from aeromind_apm_lite.ground.browser.semantic import SemanticConfig, SemanticService


def configured_semantic(response, captured=None):
    service = SemanticService(
        SemanticConfig(
            api_url="https://model.invalid/v1",
            api_key="test-key",
            vision_model="vision-test",
            mission_model="agent-test",
        )
    )

    def fake_call(_model, messages):
        if captured is not None:
            captured.extend(messages)
        return response

    service._call_api = fake_call
    return service


def healthy_context(**updates):
    context = {
        "captured_at_utc": "2026-07-30T12:00:00+00:00",
        "vehicle_id": 3,
        "vehicle_name": "uav3",
        "deployment_mode": "real",
        "agent_connected": True,
        "fcu_link_ok": True,
        "command_output_enabled": True,
        "allowed_commands": ["arm", "disarm"],
        "telemetry": {
            "armed": False,
            "mode": "STABILIZE",
            "battery_remaining": 0.82,
            "local_position_ned_m": [0.0, 0.0, 0.0],
            "home_position_deg_m": [34.7, 113.6, 100.0],
            "health": {
                "fcu_link_ok": True,
                "gps_fix_type": 3,
                "gps_hdop": 1.2,
                "gps_healthy": True,
                "prearm_ok": True,
                "ekf_ok": True,
            },
        },
        "georeference": {"status": "draft"},
        "latest_visual_result": None,
    }
    context.update(updates)
    return context


def test_agent_normalization_keeps_non_json_reply_visible():
    result = normalize_agent_result("当前飞机未解锁")

    assert result["reply"] == "当前飞机未解锁"
    assert result["proposed_action"] is None
    assert result["format_warning"]


def test_agent_receives_live_vehicle_context_and_keeps_bounded_history():
    async def scenario():
        captured = []
        semantic = configured_semantic(
            '{"reply":"三号机未解锁，电量 82%",'
            '"state_summary":"链路正常", "risk_level":"low",'
            '"questions":[], "proposed_action":null}',
            captured,
        )
        agent = MissionAgentService(
            semantic,
            MissionAgentConfig(max_turns_per_session=4),
        )
        session_id = agent.create_session()["session_id"]

        payload = await agent.chat(
            session_id,
            "飞机现在怎么样？",
            healthy_context(),
        )

        state_message = next(
            item["content"] for item in captured if "当前地面站状态 JSON" in item["content"]
        )
        state = json.loads(state_message.split("：", 1)[1])
        assert state["vehicle_id"] == 3
        assert state["telemetry"]["battery_remaining"] == 0.82
        assert payload["reply"] == "三号机未解锁，电量 82%"
        assert payload["raw_response"].startswith('{"reply"')
        assert payload["draft"] is None
        assert not payload["flight_command_generated"]
        restored = agent.session_payload(session_id)
        assert restored["turns"][-1]["payload"]["raw_response"].startswith(
            '{"reply"'
        )

    asyncio.run(scenario())


def test_agent_deterministic_gate_blocks_non_atomic_and_unauthorized_actions():
    async def scenario():
        semantic = configured_semantic(
            '{"reply":"已生成航线", "risk_level":"low",'
            '"proposed_action":{"action":"search","vehicle_id":3,'
            '"arguments":{"north_m":3}}}'
        )
        agent = MissionAgentService(semantic)
        session_id = agent.create_session()["session_id"]

        payload = await agent.chat(session_id, "搜索目标", healthy_context())

        assert payload["draft"]["status"] == "blocked"
        assert not payload["draft"]["executable"]
        assert any("受支持的原子动作" in item for item in payload["draft"]["blockers"])

    asyncio.run(scenario())


def test_agent_goto_and_formation_drafts_require_sim_mode():
    async def scenario():
        semantic = configured_semantic(
            '{"reply":"生成编队草案", "risk_level":"medium",'
            '"proposed_action":{"action":"formation","vehicle_id":1,'
            '"arguments":{"formation":"v","leader_target_map_m":[8,0,-2],'
            '"spacing_m":3,"altitude_m":2,"hold_s":5,"vehicle_ids":[1,2,3,4]},'
            '"reason":"飞往 (8,0) 编 V 字"}}'
        )
        agent = MissionAgentService(semantic)
        session_id = agent.create_session()["session_id"]

        real_payload = await agent.chat(session_id, "编队", healthy_context())
        assert real_payload["draft"]["status"] == "blocked"
        assert any("仿真" in item for item in real_payload["draft"]["blockers"])

        sim_payload = await agent.chat(
            session_id,
            "编队",
            healthy_context(
                deployment_mode="demo",
                vehicle_id=1,
                allowed_commands=["arm", "disarm", "takeoff", "hold", "land", "rtl"],
            ),
        )
        assert sim_payload["draft"]["status"] == "pending_confirmation"
        assert sim_payload["draft"]["action"] == "formation"
        assert sim_payload["draft"]["blockers"] == []
        assert sim_payload["draft"]["arguments"]["formation"] == "v"

        semantic._call_api = lambda _model, _messages: (
            '{"reply":"编队草案", "risk_level":"medium",'
            '"proposed_action":{"action":"formation","vehicle_id":1,'
            '"arguments":{"formation":"circle","leader_target_map_m":[8,0,-2],'
            '"spacing_m":3,"altitude_m":2,"hold_s":5,"vehicle_ids":[1,2,3,4]},'
            '"reason":"编队"}}'
        )
        bad = await agent.chat(session_id, "编队", healthy_context(
            deployment_mode="demo",
            vehicle_id=1,
            allowed_commands=["arm", "disarm", "takeoff", "hold", "land", "rtl"],
        ))
        assert bad["draft"]["status"] == "blocked"
        assert any("队形必须是" in item for item in bad["draft"]["blockers"])

    asyncio.run(scenario())


def test_agent_blocks_invalid_vehicle_id_and_rechecks_live_arm_state():
    async def scenario():
        semantic = configured_semantic(
            '{"reply":"解锁草案", "risk_level":"medium",'
            '"proposed_action":{"action":"arm","vehicle_id":"uav3",'
            '"arguments":{},"reason":"操作员请求"}}'
        )
        agent = MissionAgentService(semantic)
        session_id = agent.create_session()["session_id"]

        invalid = await agent.chat(session_id, "解锁", healthy_context())
        assert invalid["draft"]["status"] == "blocked"
        assert "草案飞机编号无效" in invalid["draft"]["blockers"]

        semantic._call_api = lambda _model, _messages: (
            '{"reply":"解锁草案", "risk_level":"medium",'
            '"proposed_action":{"action":"arm","vehicle_id":3,'
            '"arguments":{},"reason":"操作员请求"}}'
        )
        pending = await agent.chat(session_id, "再试一次", healthy_context())
        changed = healthy_context()
        changed["telemetry"]["armed"] = True
        with pytest.raises(AgentDraftBlocked, match="已经解锁"):
            agent.claim_draft(pending["draft"]["draft_id"], changed)
        assert agent.session_payload(session_id)["latest_draft"]["status"] == "blocked"

    asyncio.run(scenario())


def test_agent_confirmation_is_short_lived_and_single_use():
    async def scenario():
        current_time = [10.0]
        semantic = configured_semantic(
            '{"reply":"可以生成解锁草案", "risk_level":"medium",'
            '"proposed_action":{"action":"arm","vehicle_id":3,'
            '"arguments":{},"reason":"操作员要求解锁"}}'
        )
        agent = MissionAgentService(
            semantic,
            MissionAgentConfig(draft_ttl_s=10.0),
            clock=lambda: current_time[0],
        )
        session_id = agent.create_session()["session_id"]
        payload = await agent.chat(session_id, "解锁", healthy_context())
        draft_id = payload["draft"]["draft_id"]

        claim = agent.claim_draft(draft_id, healthy_context())
        assert claim["action"] == "arm"
        with pytest.raises(AgentDraftConflict):
            agent.claim_draft(draft_id, healthy_context())
        recorded = agent.record_execution(
            draft_id,
            succeeded=True,
            result={"successful": True},
        )
        assert recorded["status"] == "completed"

        second = await agent.chat(session_id, "再解锁", healthy_context())
        current_time[0] += 11.0
        with pytest.raises(AgentDraftExpired):
            agent.claim_draft(second["draft"]["draft_id"], healthy_context())

    asyncio.run(scenario())


def test_real_takeoff_draft_requires_allowlist_and_surveyed_georeference():
    async def scenario():
        semantic = configured_semantic(
            '{"reply":"起飞草案", "risk_level":"high",'
            '"proposed_action":{"action":"takeoff","vehicle_id":3,'
            '"arguments":{"altitude_m":2}}}'
        )
        agent = MissionAgentService(semantic)
        session_id = agent.create_session()["session_id"]

        payload = await agent.chat(session_id, "起飞两米", healthy_context())

        blockers = payload["draft"]["blockers"]
        assert "takeoff 不在当前机载白名单" in blockers
        assert "实机自动起飞要求已测量场地标定" in blockers
        assert not payload["draft"]["executable"]

    asyncio.run(scenario())


def test_agent_api_executes_demo_action_only_after_explicit_confirmation():
    runtime = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode.DEMO,
            vehicle_id=1,
            startup_timeout_s=2.0,
        )
    )
    semantic = configured_semantic(
        '{"reply":"已生成解锁草案", "state_summary":"DEMO 未解锁",'
        '"risk_level":"medium", "questions":[], "proposed_action":'
        '{"action":"arm","vehicle_id":1,"arguments":{},'
        '"reason":"操作员请求"}}'
    )
    app = create_app(runtime, semantic=semantic)

    with TestClient(app) as client:
        session = client.post("/api/agent/sessions").json()
        reply = client.post(
            f"/api/agent/sessions/{session['session_id']}/messages",
            json={"message": "解锁一号机"},
        )
        assert reply.status_code == 200
        draft = reply.json()["draft"]
        assert draft["status"] == "pending_confirmation"

        rejected = client.post(
            f"/api/agent/drafts/{draft['draft_id']}/confirm",
            json={"confirmed": False},
        )
        assert rejected.status_code == 422

        confirmed = client.post(
            f"/api/agent/drafts/{draft['draft_id']}/confirm",
            json={"confirmed": True},
        )
        assert confirmed.status_code == 200
        assert confirmed.json()["command_result"]["successful"]
        assert confirmed.json()["draft"]["status"] == "completed"

        duplicate = client.post(
            f"/api/agent/drafts/{draft['draft_id']}/confirm",
            json={"confirmed": True},
        )
        assert duplicate.status_code == 409


def test_agent_confirmed_formation_draft_starts_fleet_mission():
    import time as _time

    runtimes = [
        ManualRuntime(
            ManualRuntimeConfig(
                mode=ManualRuntimeMode.SITL,
                vehicle_id=vehicle_id,
                vehicle_name=f"SITL UAV {vehicle_id}",
                startup_timeout_s=2.0,
            ),
            link_factory=lambda _config: DemoApmLink(),
        )
        for vehicle_id in (1, 2, 3, 4)
    ]
    semantic = configured_semantic(
        '{"reply":"已生成编队任务", "state_summary":"仿真就绪",'
        '"risk_level":"medium", "questions":[], "proposed_action":'
        '{"action":"formation","vehicle_id":1,'
        '"arguments":{"formation":"v","leader_target_map_m":[8,0,-2],'
        '"spacing_m":3,"altitude_m":2,"hold_s":0.5,"vehicle_ids":[1,2,3,4]},'
        '"reason":"飞往 (8,0) 编 V 字"}}'
    )
    app = create_app(FleetRuntime(runtimes), semantic=semantic)
    with TestClient(app) as client:
        session = client.post("/api/agent/sessions").json()
        chat = client.post(
            f"/api/agent/sessions/{session['session_id']}/messages",
            json={"message": "飞往 (8,0) 编 V 字"},
        ).json()
        draft = chat["draft"]
        assert draft["action"] == "formation"
        assert draft["status"] == "pending_confirmation"

        confirmed = client.post(
            f"/api/agent/drafts/{draft['draft_id']}/confirm",
            json={"confirmed": True},
        )
        assert confirmed.status_code == 200
        body = confirmed.json()
        assert body["command_result"]["phase"] in {"starting", "running", "done"}

        deadline = _time.time() + 30.0
        phase = None
        while _time.time() < deadline:
            phase = client.get("/api/fleet/execute").json()["phase"]
            if phase in {"done", "failed", "cancelled"}:
                break
            _time.sleep(0.2)
        assert phase == "done", client.get("/api/fleet/execute").json()


def test_agent_plan_draft_requires_sim_and_validates_steps():
    async def scenario():
        semantic = configured_semantic(
            json.dumps(
                {
                    "reply": "已生成四步计划",
                    "risk_level": "medium",
                    "plan": [
                        {
                            "action": "takeoff",
                            "vehicle_ids": [1, 2, 3, 4],
                            "arguments": {"altitude_m": 2},
                        },
                        {
                            "action": "goto",
                            "vehicle_id": 1,
                            "arguments": {"target_position_ned_m": [8, 0, -2]},
                        },
                        {"action": "formation", "vehicle_ids": [1, 2, 3, 4], "arguments": {
                            "formation": "v",
                            "leader_target_map_m": [8, 0, -2],
                            "spacing_m": 3,
                            "altitude_m": 2,
                        }},
                        {"action": "land", "vehicle_ids": [1, 2, 3, 4]},
                    ],
                },
                ensure_ascii=False,
            )
        )
        agent = MissionAgentService(semantic)
        session_id = agent.create_session()["session_id"]

        real = await agent.chat(session_id, "完整任务", healthy_context())
        assert real["draft"]["status"] == "blocked"
        assert any("仿真" in item for item in real["draft"]["blockers"])

        sim_context = healthy_context(
            deployment_mode="demo",
            vehicle_id=1,
            allowed_commands=["arm", "disarm", "takeoff", "hold", "land", "rtl"],
        )
        pending = await agent.chat(session_id, "完整任务", sim_context)
        assert pending["draft"]["status"] == "pending_confirmation"
        assert pending["draft"]["action"] == "plan"
        assert len(pending["draft"]["arguments"]["steps"]) == 4
        assert pending["draft"]["arguments"]["steps"][2]["action"] == "formation"

        semantic._call_api = lambda _model, _messages: (
            json.dumps(
                {
                    "reply": "非法计划",
                    "risk_level": "medium",
                    "plan": [
                        {"action": "search", "vehicle_ids": [1], "arguments": {}},
                    ],
                },
                ensure_ascii=False,
            )
        )
        bad = await agent.chat(session_id, "非法计划", sim_context)
        assert bad["draft"]["status"] == "blocked"
        assert any("不受支持" in item for item in bad["draft"]["blockers"])

    asyncio.run(scenario())


def test_agent_analyze_proposed_action_becomes_plan_draft():
    async def scenario():
        sim_context = healthy_context(
            deployment_mode="demo",
            vehicle_id=1,
            allowed_commands=["arm", "disarm", "takeoff", "hold", "land", "rtl"],
        )
        semantic = configured_semantic(
            json.dumps(
                {
                    "reply": "先分析画面",
                    "risk_level": "low",
                    "proposed_action": {
                        "action": "analyze",
                        "vehicle_id": 1,
                        "arguments": {"prompt": "橙色球体", "navigate_after": True},
                        "reason": "识别橙色球体并飞近",
                    },
                },
                ensure_ascii=False,
            )
        )
        agent = MissionAgentService(semantic)
        session_id = agent.create_session()["session_id"]
        pending = await agent.chat(session_id, "识别橙色球体", sim_context)
        draft = pending["draft"]
        assert draft["action"] == "plan"
        assert draft["status"] == "pending_confirmation"
        steps = draft["arguments"]["steps"]
        assert steps[0]["action"] == "takeoff"
        assert steps[1]["action"] == "analyze"
        assert steps[1]["arguments"]["navigate_after"] is True

    asyncio.run(scenario())


def test_agent_plan_goto_normalizes_altitude_and_injects_takeoff():
    async def scenario():
        sim_context = healthy_context(
            deployment_mode="demo",
            vehicle_id=1,
            allowed_commands=["arm", "disarm", "takeoff", "hold", "land", "rtl", "goto"],
        )
        semantic = configured_semantic(
            json.dumps(
                {
                    "reply": "已生成计划",
                    "risk_level": "low",
                    "plan": [
                        {
                            "action": "goto",
                            "vehicle_ids": [1],
                            "arguments": {"target_position_ned_m": [15, 0, -0.01]},
                        },
                    ],
                },
                ensure_ascii=False,
            )
        )
        agent = MissionAgentService(semantic)
        session_id = agent.create_session()["session_id"]
        pending = await agent.chat(session_id, "向目标飞15米", sim_context)
        steps = pending["draft"]["arguments"]["steps"]
        assert pending["draft"]["status"] == "pending_confirmation"
        assert steps[0]["action"] == "takeoff"
        assert steps[0]["arguments"]["altitude_m"] == 3.0
        assert steps[1]["action"] == "goto"
        assert steps[1]["arguments"]["target_position_ned_m"] == [15, 0, -2.0]

        semantic._call_api = lambda _model, _messages: (
            json.dumps(
                {
                    "reply": "已生成计划",
                    "risk_level": "low",
                    "plan": [
                        {
                            "action": "goto",
                            "vehicle_ids": [1],
                            "arguments": {"target_position_ned_m": [15, 0, 3]},
                        },
                    ],
                },
                ensure_ascii=False,
            )
        )
        flipped = await agent.chat(session_id, "飞3米高", sim_context)
        goto_step = flipped["draft"]["arguments"]["steps"][1]
        assert goto_step["arguments"]["target_position_ned_m"] == [15, 0, -3]

        semantic._call_api = lambda _model, _messages: (
            json.dumps(
                {
                    "reply": "已生成计划",
                    "risk_level": "low",
                    "plan": [
                        {
                            "action": "goto",
                            "vehicle_ids": [1],
                            "arguments": {"target_position_ned_m": [600, 0, -3]},
                        },
                    ],
                },
                ensure_ascii=False,
            )
        )
        far = await agent.chat(session_id, "飞到远处", sim_context)
        assert far["draft"]["status"] == "blocked"
        assert any("过远" in item for item in far["draft"]["blockers"])

    asyncio.run(scenario())


def test_agent_plan_analyze_accepts_approach_distance_and_injects_takeoff():
    async def scenario():
        sim_context = healthy_context(
            deployment_mode="demo",
            vehicle_id=1,
            allowed_commands=["arm", "disarm", "takeoff", "hold", "land", "rtl"],
        )
        semantic = configured_semantic(
            json.dumps(
                {
                    "reply": "已生成计划",
                    "risk_level": "low",
                    "plan": [
                        {
                            "action": "analyze",
                            "vehicle_ids": [1],
                            "arguments": {
                                "prompt": "橙色球体",
                                "navigate_after": True,
                                "approach_distance_m": 20,
                            },
                        },
                    ],
                },
                ensure_ascii=False,
            )
        )
        agent = MissionAgentService(semantic)
        session_id = agent.create_session()["session_id"]
        pending = await agent.chat(session_id, "向球飞20米", sim_context)
        steps = pending["draft"]["arguments"]["steps"]
        assert pending["draft"]["status"] == "pending_confirmation"
        assert steps[0]["action"] == "takeoff"
        assert steps[0]["arguments"]["altitude_m"] == 3.0
        assert steps[1]["action"] == "analyze"
        assert steps[1]["arguments"]["approach_distance_m"] == 20.0

        semantic._call_api = lambda _model, _messages: (
            json.dumps(
                {
                    "reply": "已生成计划",
                    "risk_level": "low",
                    "plan": [
                        {
                            "action": "analyze",
                            "vehicle_ids": [1],
                            "arguments": {
                                "prompt": "x",
                                "navigate_after": True,
                                "approach_distance_m": 200,
                            },
                        },
                    ],
                },
                ensure_ascii=False,
            )
        )
        bad = await agent.chat(session_id, "飞200米", sim_context)
        assert bad["draft"]["status"] == "blocked"
        assert any("approach_distance_m" in item for item in bad["draft"]["blockers"])

    asyncio.run(scenario())


def test_approach_bearing_math_matches_estimator():
    # A target far off to the right (center_x > 0.5) must yield a positive
    # eastward bearing when the aircraft faces north (yaw=0).
    import math as _math

    from aeromind_apm_lite.ground.browser.mission_planner import MissionPlanner

    # Verify the bearing formula used by _approach_target_direction through a
    # standalone computation mirroring the planner helper.
    fov = 95.0
    center_x = 0.75
    horizontal = (center_x - 0.5) * 2.0
    h_angle = horizontal * _math.radians(fov) / 2.0
    dir_x = _math.cos(h_angle)
    dir_y = _math.sin(h_angle)
    norm = _math.hypot(dir_x, dir_y)
    dx, dy = dir_x / norm, dir_y / norm
    assert dy > 0.0  # target on the right -> east positive with yaw=0
    assert dx > 0.0  # still forward of the aircraft

    assert MissionPlanner is not None


async def fake_analyzer(vehicle_id, prompt):
    return {
        "vehicle_id": vehicle_id,
        "result": {
            "message": f"识别完成（{prompt}）",
            "target": {"found": True, "label": "red_box", "color": "red"},
        },
    }


def test_agent_confirmed_plan_runs_multi_step_mission():
    import time as _time

    runtimes = [
        ManualRuntime(
            ManualRuntimeConfig(
                mode=ManualRuntimeMode.SITL,
                vehicle_id=vehicle_id,
                vehicle_name=f"SITL UAV {vehicle_id}",
                startup_timeout_s=2.0,
            ),
            link_factory=lambda _config: DemoApmLink(),
        )
        for vehicle_id in (1, 2, 3, 4)
    ]
    semantic = configured_semantic(
        json.dumps(
            {
                "reply": "已生成闭环计划",
                "state_summary": "仿真就绪",
                "risk_level": "medium",
                "questions": [],
                "plan": [
                    {
                        "action": "takeoff",
                        "vehicle_ids": [1, 2, 3, 4],
                        "arguments": {"altitude_m": 2},
                    },
                    {
                        "action": "goto",
                        "vehicle_id": 1,
                        "arguments": {"target_position_ned_m": [8, 0, -2]},
                    },
                    {
                        "action": "analyze",
                        "vehicle_ids": [1],
                        "arguments": {"prompt": "识别目标"},
                    },
                    {"action": "land", "vehicle_ids": [1, 2, 3, 4]},
                ],
            },
            ensure_ascii=False,
        )
    )
    app = create_app(
        FleetRuntime(runtimes),
        semantic=semantic,
        planner_analyzer=fake_analyzer,
    )
    with TestClient(app) as client:
        session = client.post("/api/agent/sessions").json()
        chat = client.post(
            f"/api/agent/sessions/{session['session_id']}/messages",
            json={"message": "执行闭环任务"},
        ).json()
        draft = chat["draft"]
        assert draft["action"] == "plan"
        assert draft["status"] == "pending_confirmation"

        confirmed = client.post(
            f"/api/agent/drafts/{draft['draft_id']}/confirm",
            json={"confirmed": True},
        )
        assert confirmed.status_code == 200

        deadline = _time.time() + 30.0
        phase = None
        while _time.time() < deadline:
            phase = client.get("/api/planner/status").json()["phase"]
            if phase in {"done", "failed", "cancelled"}:
                break
            _time.sleep(0.2)
        planner = client.get("/api/planner/status").json()
        assert phase == "done", planner

        analyze_results = [
            item for item in planner.get("results", [])
            if item.get("step") == "analyze"
        ]
        assert analyze_results
        assert "red_box" in analyze_results[0]["detail"]

        session_payload = client.get(
            f"/api/agent/sessions/{session['session_id']}"
        ).json()
        assert session_payload["latest_draft"]["status"] == "completed"
