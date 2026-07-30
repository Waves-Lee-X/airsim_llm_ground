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
            '{"reply":"已生成航点", "risk_level":"low",'
            '"proposed_action":{"action":"goto","vehicle_id":3,'
            '"arguments":{"north_m":3}}}'
        )
        agent = MissionAgentService(semantic)
        session_id = agent.create_session()["session_id"]

        payload = await agent.chat(session_id, "向北飞三米", healthy_context())

        assert payload["draft"]["status"] == "blocked"
        assert not payload["draft"]["executable"]
        assert any("六种原子动作" in item for item in payload["draft"]["blockers"])

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
