import asyncio
import time
from collections import deque
from datetime import datetime, timezone

import pytest
from fastapi.testclient import TestClient

from aeromind_apm_lite.ground.browser.app import create_app
from aeromind_apm_lite.ground.browser.camera import (
    AirSimCameraBridge,
    AirSimCameraConfig,
    CameraFrame,
    CameraUnavailable,
    RtspCameraBridge,
    RtspCameraConfig,
)
from aeromind_apm_lite.ground.browser.runtime import (
    ManualRuntime,
    ManualRuntimeConfig,
    ManualRuntimeMode,
)


PNG_FRAME = b"\x89PNG\r\n\x1a\nframe-data"
JPEG_FRAME = b"\xff\xd8\xffframe-data"


def run(coroutine):
    return asyncio.run(coroutine)


async def wait_until(predicate, timeout_s=2.0):
    deadline = asyncio.get_running_loop().time() + timeout_s
    while not predicate():
        if asyncio.get_running_loop().time() >= deadline:
            raise TimeoutError("condition did not become true")
        await asyncio.sleep(0.01)


class FakeAirSimClient:
    def __init__(self, responses, *, ping=True):
        self.responses = deque(responses)
        self.ping_result = ping
        self.calls = []

    def ping(self):
        return self.ping_result

    def simGetImage(
        self,
        camera_name,
        image_type,
        vehicle_name="",
        external=False,
    ):
        self.calls.append((camera_name, image_type, vehicle_name, external))
        response = self.responses[0] if len(self.responses) == 1 else self.responses.popleft()
        if isinstance(response, Exception):
            raise response
        return response


class FakeVideoFrame:
    shape = (240, 424, 3)


class FakeRtspCapture:
    def __init__(self, responses, *, opened=True, read_delay_s=0.001):
        self.responses = deque(responses)
        self.opened = opened
        self.released = False
        self.read_delay_s = read_delay_s

    def isOpened(self):
        return self.opened

    def read(self):
        time.sleep(self.read_delay_s)
        response = self.responses[0] if len(self.responses) == 1 else self.responses.popleft()
        if isinstance(response, Exception):
            raise response
        return response

    def release(self):
        self.released = True


def test_camera_bridge_captures_and_reports_a_fresh_scene_frame():
    async def scenario():
        client = FakeAirSimClient([PNG_FRAME])
        bridge = AirSimCameraBridge(
            AirSimCameraConfig(
                rpc_host="192.0.2.10",
                capture_fps=20.0,
                reconnect_delay_s=0.01,
            ),
            client_factory=lambda _config: client,
        )
        await bridge.start()
        try:
            await wait_until(lambda: bridge.status_payload()["stream_available"])
            frame = await bridge.get_frame()
            status = bridge.status_payload()

            assert frame.data == PNG_FRAME
            assert frame.media_type == "image/png"
            assert status["state"] == "online"
            assert status["stream_url"] == "/api/camera/frame"
            assert status["frame_sequence"] >= 1
            assert client.calls[0] == ("front_center", 0, "Drone1", False)
        finally:
            await bridge.stop()

    run(scenario())


def test_camera_bridge_reconnects_after_an_airsim_capture_failure():
    async def scenario():
        clients = deque(
            [
                FakeAirSimClient([TimeoutError("RPC timed out")]),
                FakeAirSimClient([PNG_FRAME]),
            ]
        )
        created = []

        def factory(_config):
            client = clients.popleft()
            created.append(client)
            return client

        bridge = AirSimCameraBridge(
            AirSimCameraConfig(
                rpc_host="192.0.2.10",
                capture_fps=20.0,
                reconnect_delay_s=0.01,
            ),
            client_factory=factory,
        )
        await bridge.start()
        try:
            await wait_until(
                lambda: len(created) == 2
                and bridge.status_payload()["stream_available"]
            )
            assert bridge.status_payload()["state"] == "online"
            assert len(created) == 2
        finally:
            await bridge.stop()

    run(scenario())


def test_camera_bridge_marks_an_old_cached_frame_unavailable():
    async def scenario():
        now = [10.0]
        client = FakeAirSimClient([PNG_FRAME])
        bridge = AirSimCameraBridge(
            AirSimCameraConfig(
                rpc_host="192.0.2.10",
                stale_after_s=1.0,
            ),
            client_factory=lambda _config: client,
            clock=lambda: now[0],
        )
        try:
            assert await bridge.capture_once()
            assert bridge.status_payload()["stream_available"]
            now[0] = 12.0
            assert not bridge.status_payload()["stream_available"]
            assert bridge.status_payload()["state"] == "connecting"
        finally:
            await bridge.stop()

    run(scenario())


def test_camera_bridge_rejects_non_image_payloads():
    async def scenario():
        bridge = AirSimCameraBridge(
            AirSimCameraConfig(rpc_host="192.0.2.10"),
            client_factory=lambda _config: FakeAirSimClient([b"not-an-image"]),
        )
        try:
            with pytest.raises(CameraUnavailable, match="not a PNG or JPEG"):
                await bridge.get_frame()
            status = bridge.status_payload()
            assert status["state"] == "offline"
            assert not status["stream_available"]
        finally:
            await bridge.stop()

    run(scenario())


def test_rtsp_camera_bridge_captures_jpeg_and_reports_onboard_rgb():
    async def scenario():
        capture = FakeRtspCapture([(True, FakeVideoFrame())])
        bridge = RtspCameraBridge(
            RtspCameraConfig(
                stream_url="rtsp://192.0.2.20:15544/cam",
                capture_fps=20.0,
            ),
            capture_factory=lambda _config: capture,
            frame_encoder=lambda _frame, _quality: JPEG_FRAME,
        )
        await bridge.start()
        try:
            await wait_until(lambda: bridge.status_payload()["stream_available"])
            frame = await bridge.get_frame()
            status = bridge.status_payload()

            assert frame.data == JPEG_FRAME
            assert frame.media_type == "image/jpeg"
            assert status["state"] == "online"
            assert status["kind"] == "onboard_rgb"
            assert status["width"] == 424
            assert status["height"] == 240
            assert status["stream_url"] == "/api/camera/frame"
        finally:
            await bridge.stop()
        assert capture.released

    run(scenario())


def test_rtsp_camera_bridge_reopens_after_a_read_failure():
    async def scenario():
        captures = deque(
            [
                FakeRtspCapture([(False, None)]),
                FakeRtspCapture([(True, FakeVideoFrame())]),
            ]
        )
        created = []

        def factory(_config):
            capture = captures.popleft()
            created.append(capture)
            return capture

        bridge = RtspCameraBridge(
            RtspCameraConfig(
                stream_url="rtsp://192.0.2.20:15544/cam",
                capture_fps=20.0,
                reconnect_delay_s=0.01,
            ),
            capture_factory=factory,
            frame_encoder=lambda _frame, _quality: JPEG_FRAME,
        )
        await bridge.start()
        try:
            await wait_until(
                lambda: len(created) == 2
                and bridge.status_payload()["stream_available"]
            )
            assert created[0].released
            assert bridge.status_payload()["state"] == "online"
        finally:
            await bridge.stop()

    run(scenario())


def test_rtsp_camera_bridge_drains_source_without_capture_rate_sleep():
    async def scenario():
        capture = FakeRtspCapture(
            [(True, FakeVideoFrame())],
            read_delay_s=0.01,
        )
        bridge = RtspCameraBridge(
            RtspCameraConfig(
                stream_url="rtsp://192.0.2.20:15544/cam",
                capture_fps=0.2,
            ),
            capture_factory=lambda _config: capture,
            frame_encoder=lambda _frame, _quality: JPEG_FRAME,
        )
        await bridge.start()
        try:
            await wait_until(
                lambda: bridge.status_payload()["frames_received"] >= 5,
                timeout_s=0.5,
            )
            status = bridge.status_payload()
            assert status["source_fps"] > 10.0
            assert status["frames_received"] >= 5
        finally:
            await bridge.stop()

    run(scenario())


class StubCameraBridge:
    def __init__(self):
        self.started = False
        self.stopped = False
        self.frame = CameraFrame(
            data=PNG_FRAME,
            media_type="image/png",
            sequence=7,
            captured_at_utc=datetime(2026, 7, 28, tzinfo=timezone.utc),
            captured_monotonic_s=10.0,
        )

    async def start(self):
        self.started = True

    async def stop(self):
        self.stopped = True

    def status_payload(self):
        return {
            "name": "front_center",
            "kind": "scene_rgb",
            "state": "online",
            "stream_available": True,
            "stream_url": "/api/camera/frame",
            "detail": "test frame online",
        }

    async def get_frame(self):
        return self.frame


def test_browser_camera_api_serves_the_bridge_frame_and_dynamic_status():
    runtime = ManualRuntime(
        ManualRuntimeConfig(mode=ManualRuntimeMode.DEMO, startup_timeout_s=2.0)
    )
    camera = StubCameraBridge()
    app = create_app(runtime, camera=camera)

    with TestClient(app) as client:
        assert camera.started
        config = client.get("/api/config").json()
        status = client.get("/api/camera/status").json()
        frame = client.get("/api/camera/frame")

        assert config["camera"]["stream_available"]
        assert status["state"] == "online"
        assert frame.status_code == 200
        assert frame.headers["content-type"] == "image/png"
        assert frame.headers["x-camera-sequence"] == "7"
        assert frame.headers["cache-control"].startswith("no-store")
        assert frame.content == PNG_FRAME

    assert camera.stopped


def test_browser_camera_frame_fails_closed_when_the_bridge_is_disabled():
    runtime = ManualRuntime(
        ManualRuntimeConfig(mode=ManualRuntimeMode.DEMO, startup_timeout_s=2.0)
    )
    app = create_app(runtime)

    with TestClient(app) as client:
        response = client.get("/api/camera/frame")
        assert response.status_code == 503
        assert response.json()["detail"] == "camera bridge is disabled"
