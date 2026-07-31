import asyncio
import json
from datetime import datetime, timezone

from fastapi.testclient import TestClient

from aeromind_apm_lite.ground.browser.app import create_app
from aeromind_apm_lite.ground.browser.camera import CameraFrame
from aeromind_apm_lite.ground.browser.runtime import (
    ManualRuntime,
    ManualRuntimeConfig,
    ManualRuntimeMode,
)
from aeromind_apm_lite.ground.browser.semantic import (
    SemanticConfig,
    SemanticService,
    extract_json_object,
    normalize_mission_result,
    normalize_visual_result,
)


class FakeCamera:
    def __init__(self):
        self.started = False
        self.frame = CameraFrame(
            data=b"fake-jpeg-data",
            media_type="image/jpeg",
            sequence=17,
            captured_at_utc=datetime(2026, 7, 30, tzinfo=timezone.utc),
            captured_monotonic_s=1.0,
        )

    async def start(self):
        self.started = True

    async def stop(self):
        self.started = False

    def status_payload(self):
        return {
            "state": "online",
            "stream_available": True,
            "stream_url": "/api/camera/frame",
        }

    async def get_frame(self):
        return self.frame


def test_normalize_mission_result_extracts_fleet_plan():
    plan = normalize_mission_result(
        json.dumps(
            {
                "summary": "飞到 (8,0) 编 V 字",
                "intent": "formation",
                "fleet_plan": {
                    "formation": "v",
                    "leader_target_map_m": [8.0, 0.0, -2.0],
                    "spacing_m": 3.0,
                    "altitude_m": 2.0,
                    "hold_s": 5.0,
                    "vehicle_ids": [1, 2, 3, 4],
                },
            },
            ensure_ascii=False,
        ),
        "飞到 (8,0) 编 V 字",
    )
    assert plan["fleet_plan"] == {
        "formation": "v",
        "leader_target_map_m": [8.0, 0.0, -2.0],
        "spacing_m": 3.0,
        "altitude_m": 2.0,
        "hold_s": 5.0,
        "vehicle_ids": [1, 2, 3, 4],
    }


def test_normalize_mission_result_rejects_incomplete_fleet_plan():
    plan = normalize_mission_result(
        json.dumps(
            {
                "summary": "编队",
                "fleet_plan": {"formation": "v"},
            },
            ensure_ascii=False,
        ),
        "编队",
    )
    assert plan["fleet_plan"] is None


def configured_service():
    return SemanticService(
        SemanticConfig(
            api_url="https://model.invalid/v1",
            api_key="test-key",
            vision_model="vision-test",
            mission_model="mission-test",
        )
    )


def test_json_extraction_and_visual_normalization_are_tolerant():
    content = """```json
    {"scene":"箱体", "target":{"found":true,"color":"red",
    "confidence":92,}, "risk_level":"low"}
    ```"""
    assert extract_json_object(content)["target"]["color"] == "red"

    result = normalize_visual_result(content)
    assert result["target"]["found"]
    assert result["target"]["color"] == "red"
    assert result["target"]["confidence"] == 0.92
    assert result["risk_level"] == "low"


def test_non_json_visual_text_remains_visible():
    result = normalize_visual_result("画面中可见一个蓝色箱体")
    assert "蓝色箱体" in result["scene"]
    assert result["format_warning"]


def test_mission_normalization_is_always_preview_only():
    result = normalize_mission_result(
        '{"summary":"寻找红色箱体","steps":[{"action":"search"}]}',
        "寻找红色箱体",
    )
    assert result["steps"][0]["order"] == 1
    assert result["execution_policy"] == "preview_only"
    assert not result["executable"]


def test_service_analyzes_latest_frame_without_generating_flight_command():
    async def scenario():
        service = configured_service()
        service._call_api = lambda _model, _messages: (
            '{"message":"完成","scene":"3号箱体",'
            '"target":{"found":true,"label":"box","box_id":"3",'
            '"color":"red","face":"front","confidence":0.91},'
            '"objects":[],"risk_level":"low","suggestion":"保持观察"}'
        )
        camera = FakeCamera()
        payload = await service.analyze_frame(camera.frame, "找红色箱体")
        assert payload["frame"]["sequence"] == 17
        assert payload["result"]["target"]["box_id"] == "3"
        assert payload["result"]["target"]["color"] == "red"
        assert not payload["flight_command_generated"]
        assert "api_key" not in str(service.latest_payload()).lower()

    asyncio.run(scenario())


def test_browser_api_returns_visual_result_and_mission_preview():
    runtime = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode.DEMO,
            startup_timeout_s=2.0,
        )
    )
    service = configured_service()

    def fake_call(model, _messages):
        if model == "vision-test":
            return (
                '{"message":"识别完成","scene":"红色箱体",'
                '"target":{"found":true,"label":"box","color":"red",'
                '"confidence":0.88},"risk_level":"low"}'
            )
        return (
            '{"summary":"3号机寻找红色箱体","intent":"search",'
            '"vehicle_ids":[3],"requires_visual":true,'
            '"steps":[{"action":"search","vehicle_id":3}],'
            '"ambiguities":["缺少搜索区域"]}'
        )

    service._call_api = fake_call
    app = create_app(runtime, camera=FakeCamera(), semantic=service)
    with TestClient(app) as client:
        status = client.get("/api/semantic/status").json()
        assert status["vision_available"]
        assert status["execution_policy"] == "preview_only"

        visual = client.post(
            "/api/semantic/vision/analyze",
            json={"prompt": "找红色箱体"},
        )
        assert visual.status_code == 200
        assert visual.json()["result"]["target"]["color"] == "red"

        mission = client.post(
            "/api/semantic/mission/parse",
            json={"instruction": "让3号机寻找红色箱体"},
        )
        assert mission.status_code == 200
        outcome = mission.json()
        assert outcome["visual_context_used"]
        assert outcome["plan"]["vehicle_ids"] == [3]
        assert outcome["plan"]["execution_policy"] == "preview_only"
        assert not outcome["flight_command_generated"]

        latest = client.get("/api/semantic/latest").json()
        assert latest["visual"] is not None
        assert latest["mission"] is not None


def test_unconfigured_semantic_api_fails_with_actionable_status():
    runtime = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode.DEMO,
            startup_timeout_s=2.0,
        )
    )
    app = create_app(
        runtime,
        camera=FakeCamera(),
        semantic=SemanticService(SemanticConfig()),
    )
    with TestClient(app) as client:
        response = client.post("/api/semantic/vision/analyze", json={})
        assert response.status_code == 503
        assert "AEROMIND_VLM_API_URL" in response.json()["detail"]
