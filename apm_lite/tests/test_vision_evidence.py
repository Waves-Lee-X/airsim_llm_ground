"""Deterministic color cross-validation, consensus and vision evidence store."""

import hashlib
from datetime import datetime, timezone

import pytest

cv2 = pytest.importorskip("cv2")
import numpy as np  # noqa: E402

from fastapi.testclient import TestClient  # noqa: E402

from aeromind_apm_lite.ground.browser.app import create_app  # noqa: E402
from aeromind_apm_lite.ground.browser.camera import CameraFrame  # noqa: E402
from aeromind_apm_lite.ground.browser.runtime import (  # noqa: E402
    ManualRuntime,
    ManualRuntimeConfig,
    ManualRuntimeMode,
)
from aeromind_apm_lite.ground.browser.semantic import (  # noqa: E402
    SemanticConfig,
    SemanticService,
)
from aeromind_apm_lite.ground.browser.vision_evidence import (  # noqa: E402
    ColorDetector,
    VisionAnalysisEvidence,
    VisionConsensusTracker,
    VisionEvidenceError,
    VisionEvidenceStore,
    VisionFrameInfo,
    cross_validate_color,
    normalize_color,
)


def solid_frame(color_bgr, sequence=1, width=640, height=480):
    # Cyan background is intentionally outside the detector palette so the
    # coloured target box is always the dominant palette colour.
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :] = (255, 255, 0)
    image[120:300, 180:460] = color_bgr
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return CameraFrame(
        data=encoded.tobytes(),
        media_type="image/jpeg",
        sequence=sequence,
        captured_at_utc=datetime.now(timezone.utc),
        captured_monotonic_s=0.0,
    )


def configured_service():
    return SemanticService(
        SemanticConfig(
            api_url="https://model.invalid/v1",
            api_key="test-key",
            vision_model="vision-test",
            mission_model="mission-test",
        )
    )


def test_normalize_color_accepts_english_and_chinese_labels():
    assert normalize_color("red") == "red"
    assert normalize_color(" 红色 ") == "red"
    assert normalize_color("蓝") == "blue"
    assert normalize_color("GREEN") == "green"
    assert normalize_color(None) is None
    assert normalize_color("深红色") == "red"  # deep red normalizes to red


def test_color_detector_finds_dominant_solid_color():
    detector = ColorDetector()
    red = detector.detect(
        solid_frame((0, 0, 255)).data,
        sequence=1,
        captured_at_utc=datetime.now(timezone.utc),
    )
    assert red is not None
    assert red.color == "red"
    assert red.coverage_ratio > 0.1
    assert red.bbox is not None

    green = detector.detect(
        solid_frame((0, 255, 0)).data,
        sequence=2,
        captured_at_utc=datetime.now(timezone.utc),
    )
    assert green is not None
    assert green.color == "green"

    blue = detector.detect(
        solid_frame((255, 0, 0)).data,
        sequence=3,
        captured_at_utc=datetime.now(timezone.utc),
    )
    assert blue is not None
    assert blue.color == "blue"


def test_color_detector_returns_none_for_uninformative_frame():
    detector = ColorDetector()
    result = detector.detect(
        b"",
        sequence=1,
        captured_at_utc=datetime.now(timezone.utc),
    )
    assert result is None


def test_cross_validate_color_agreement_matrix():
    detection = ColorDetector().detect(
        solid_frame((0, 0, 255)).data,
        sequence=1,
        captured_at_utc=datetime.now(timezone.utc),
    )
    agreed = cross_validate_color("red", detection)
    assert agreed["agreement"] is True
    assert agreed["status"] == "agreed"

    disagreed = cross_validate_color("蓝色", detection)
    assert disagreed["agreement"] is False
    assert disagreed["status"] == "disagreed"

    missing = cross_validate_color(None, detection)
    assert missing["agreement"] is None
    assert missing["status"] == "vlm_color_missing"

    not_analyzed = cross_validate_color(None, detection, vlm_analyzed=False)
    assert not_analyzed["agreement"] is None
    assert not_analyzed["status"] == "vlm_not_analyzed"

    unavailable = cross_validate_color("red", None)
    assert unavailable["agreement"] is None
    assert unavailable["status"] == "unavailable"


def test_consensus_tracker_confirms_after_required_consistent_votes():
    tracker = VisionConsensusTracker(window=3, required=2)
    payload = {
        "analyzed_at_utc": "2026-07-31T00:00:00Z",
        "frame": {"sequence": 1},
        "result": {
            "target": {
                "found": True,
                "label": "box",
                "box_id": "3",
                "color": "红色",
                "face": "front",
            }
        },
    }
    assert not tracker.append(payload)["confirmed"]
    second = tracker.append(payload)
    assert second["confirmed"]
    assert second["confirmed_color"] == "red"
    assert second["votes"]["color"]["red"] == 2

    conflict = dict(payload)
    conflict["frame"] = {"sequence": 3}
    conflict["result"] = {
        "target": {
            "found": True,
            "label": "box",
            "box_id": "3",
            "color": "blue",
            "face": "front",
        }
    }
    third = tracker.append(conflict)
    assert third["confirmed"]
    assert third["votes"]["color"] == {"red": 2, "blue": 1}


def test_consensus_tracker_rejects_invalid_window():
    with pytest.raises(ValueError):
        VisionConsensusTracker(window=3, required=4)
    with pytest.raises(ValueError):
        VisionConsensusTracker(window=0, required=1)


def test_vision_evidence_store_is_immutable_and_atomic(tmp_path):
    store = VisionEvidenceStore(tmp_path)
    frame = solid_frame((0, 0, 255)).data
    evidence = VisionAnalysisEvidence(
        vehicle_id=3,
        frame=VisionFrameInfo(
            sequence=1,
            captured_at_utc=datetime.now(timezone.utc),
            media_type="image/jpeg",
            size_bytes=len(frame),
            sha256=hashlib.sha256(frame).hexdigest(),
        ),
        vision_model="vision-test",
        result={"target": {"color": "red"}},
        cross_validation={
            "agreement": True,
            "status": "agreed",
            "vlm_color": "red",
            "opencv_color": "red",
        },
        consensus={"confirmed": True},
        producer_version="test",
    )
    saved = store.save(evidence, frame)
    assert saved.evidence_hash == evidence.evidence_hash
    loaded = store.load(saved.evidence_id)
    assert loaded.evidence_hash == saved.evidence_hash
    assert store.load_frame(saved.evidence_id) == frame

    summaries = store.list_summaries()
    assert len(summaries) == 1
    assert summaries[0]["vlm_color"] == "red"
    assert summaries[0]["agreement"] is True

    duplicate = store.save(evidence, frame)
    assert duplicate.evidence_id == saved.evidence_id

    tampered = evidence.model_copy(update={"result": {"target": {"color": "blue"}}})
    with pytest.raises(VisionEvidenceError):
        store.save(tampered, frame)

    with pytest.raises(VisionEvidenceError):
        store.load("00000000-0000-0000-0000-000000000000")


def test_vision_evidence_store_rejects_wrong_frame_bytes(tmp_path):
    store = VisionEvidenceStore(tmp_path)
    evidence = VisionAnalysisEvidence(
        vehicle_id=3,
        frame=VisionFrameInfo(
            sequence=1,
            captured_at_utc=datetime.now(timezone.utc),
            media_type="image/jpeg",
            size_bytes=10,
            sha256=hashlib.sha256(b"other").hexdigest(),
        ),
        vision_model="vision-test",
        result={},
        cross_validation={"status": "unavailable"},
        consensus={},
        producer_version="test",
    )
    with pytest.raises(VisionEvidenceError):
        store.save(evidence, b"not-the-frame")


def test_browser_crosscheck_runs_without_model(tmp_path):
    runtime = ManualRuntime(
        ManualRuntimeConfig(mode=ManualRuntimeMode.DEMO, startup_timeout_s=2.0)
    )
    app = create_app(
        runtime,
        camera=StaticCamera(solid_frame((0, 0, 255)).data),
        semantic=SemanticService(SemanticConfig()),
        vision_evidence=VisionEvidenceStore(tmp_path),
    )
    with TestClient(app) as client:
        response = client.post("/api/vision/crosscheck", json={})
        assert response.status_code == 200
        body = response.json()
        assert body["vehicle_id"] == 1
        assert body["cross_validation"]["opencv_color"] == "red"
        assert body["cross_validation"]["vlm_color"] is None
        assert body["cross_validation"]["status"] == "vlm_not_analyzed"
        assert body["consensus"]["window_size"] == 0


def test_browser_visual_analysis_records_cross_validation_and_evidence(tmp_path):
    runtime = ManualRuntime(
        ManualRuntimeConfig(mode=ManualRuntimeMode.DEMO, startup_timeout_s=2.0)
    )
    service = configured_service()

    def fake_call(model, _messages):
        return (
            '{"message":"识别完成","scene":"红色箱体",'
            '"target":{"found":true,"label":"box","box_id":"3",'
            '"color":"red","face":"front","confidence":0.91},'
            '"risk_level":"low"}'
        )

    service._call_api = fake_call
    app = create_app(
        runtime,
        camera=StaticCamera(solid_frame((0, 0, 255)).data),
        semantic=service,
        vision_evidence=VisionEvidenceStore(tmp_path),
    )
    with TestClient(app) as client:
        analysis = client.post(
            "/api/semantic/vision/analyze",
            json={"prompt": "找红色箱体"},
        )
        assert analysis.status_code == 200
        payload = analysis.json()
        assert payload["cross_validation"]["agreement"] is True
        assert payload["cross_validation"]["status"] == "agreed"
        assert payload["consensus"]["confirmed"] is False
        evidence_id = payload["evidence"]["evidence_id"]

        listed = client.get("/api/vision/evidence").json()
        assert len(listed["evidence"]) == 1
        assert listed["evidence"][0]["evidence_id"] == evidence_id

        record = client.get(f"/api/vision/evidence/{evidence_id}").json()
        assert record["evidence_hash"] == payload["evidence"]["evidence_hash"]
        assert record["evidence"]["cross_validation"]["agreement"] is True

        frame_response = client.get(f"/api/vision/evidence/{evidence_id}/frame")
        assert frame_response.status_code == 200
        assert frame_response.headers["content-type"].startswith("image/jpeg")
        assert len(frame_response.content) > 0


class StaticCamera:
    def __init__(self, data):
        self._frame = CameraFrame(
            data=data,
            media_type="image/jpeg",
            sequence=7,
            captured_at_utc=datetime.now(timezone.utc),
            captured_monotonic_s=0.0,
        )

    async def start(self):
        return None

    async def stop(self):
        return None

    def status_payload(self):
        return {
            "name": "test",
            "kind": "test_rgb",
            "state": "online",
            "stream_available": True,
            "stream_url": "/api/camera/frame",
        }

    async def get_frame(self):
        return self._frame


def test_normalize_color_handles_compound_descriptions():
    from aeromind_apm_lite.ground.browser.vision_evidence import (
        normalize_color,
    )

    assert normalize_color(None) is None
    assert normalize_color("orange") == "orange"
    assert normalize_color("橙色") == "orange"
    assert normalize_color("blue and white") == "blue"
    assert normalize_color("blue-gray with white grid pattern") == "blue"
    assert normalize_color("blueandwhite") == "blue"
    assert normalize_color("bright orange sphere") == "orange"
    assert normalize_color("dark gray") == "gray"
    assert normalize_color("浅蓝色") == "blue"
    assert normalize_color("grey") == "gray"
    assert normalize_color("红橙色") == "red"
    assert normalize_color("无法确定颜色") is None
