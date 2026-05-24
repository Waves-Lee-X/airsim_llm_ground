from __future__ import annotations

import logging
import threading
import time

logger = logging.getLogger(__name__)


class CameraFrame:
    """Lightweight camera frame value object (avoids airsim_adapter import)."""

    __slots__ = ("data", "content_type", "camera_name", "image_type")

    def __init__(self, data: bytes, content_type: str, camera_name: str, image_type: int) -> None:
        self.data = data
        self.content_type = content_type
        self.camera_name = camera_name
        self.image_type = image_type


class VideoStream:
    """AirSim MJPEG streaming with its own AirSim client.

    Runs an independent capture thread with a dedicated AirSim RPC client
    so that camera frames are never blocked by the engine thread's
    command queue (e.g. during long-running goto_local calls).
    """

    boundary = "aeromind-frame"

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 41451,
        vehicle_name: str = "Drone1",
        camera_candidates: tuple[str, ...] = ("front_center", "bottom_center", "0"),
    ) -> None:
        self._host = host
        self._port = int(port)
        self._vehicle_name = vehicle_name
        self._camera_candidates = camera_candidates
        self._active_camera = camera_candidates[0] if camera_candidates else "front_center"

        self.last_frame: CameraFrame | None = None
        self.last_frame_at = 0.0
        self.target_fps = 8.0
        self._lock = threading.Lock()
        self._worker_started = False
        self._stop_event = threading.Event()

    # -- public API ---------------------------------------------------------

    def is_ready(self) -> bool:
        self._start_once()
        with self._lock:
            return self.last_frame is not None

    def latest_frame(self) -> CameraFrame | None:
        self._start_once()
        with self._lock:
            return self.last_frame

    def reset(self) -> None:
        with self._lock:
            self.last_frame = None
            self.last_frame_at = 0.0

    def start(self) -> None:
        self._start_once()

    def stop(self) -> None:
        self._stop_event.set()

    def set_camera(self, camera_name: str) -> None:
        """Switch the active camera (called when user changes camera in UI)."""
        name = str(camera_name).strip()
        if name in self._camera_candidates:
            self._active_camera = name

    def set_vehicle(self, vehicle_name: str) -> None:
        self._vehicle_name = str(vehicle_name).strip()

    def frames(self, interval_s: float = 0.08):
        self._start_once()
        last_sent_at = 0.0
        while True:
            frame = self.latest_frame()
            if frame is not None and self.last_frame_at != last_sent_at:
                last_sent_at = self.last_frame_at
                yield frame
            time.sleep(max(0.02, float(interval_s)))

    # -- internals ----------------------------------------------------------

    def _start_once(self) -> None:
        with self._lock:
            if self._worker_started:
                return
            self._worker_started = True
        thread = threading.Thread(
            target=self._capture_loop, name="aeromind-video-capture", daemon=True,
        )
        thread.start()

    def _capture_loop(self) -> None:
        """Own AirSim client — never touches the engine command queue."""
        import airsim

        interval = 1.0 / max(1.0, float(self.target_fps))
        client = None
        failed_count = 0
        connect_logged = False

        while not self._stop_event.is_set():
            started = time.time()

            # -- connect / reconnect -----------------------------------------
            if client is None:
                try:
                    client = airsim.MultirotorClient(
                        ip=self._host, port=self._port, timeout_value=2.0,
                    )
                    client.confirmConnection()
                    if not connect_logged:
                        logger.info(
                            "video capture connected to AirSim at %s:%s",
                            self._host, self._port,
                        )
                    connect_logged = True
                    failed_count = 0
                except Exception:
                    if not connect_logged:
                        logger.warning(
                            "video capture cannot connect to AirSim at %s:%s",
                            self._host, self._port,
                        )
                        connect_logged = True
                    client = None
                    self._stop_event.wait(2.0)
                    continue

            # -- grab frame from active camera -------------------------------
            camera = self._active_camera
            try:
                responses = client.simGetImages(
                    [airsim.ImageRequest(camera, airsim.ImageType.Scene, False, True)],
                    vehicle_name=self._vehicle_name,
                )
                if responses and responses[0].image_data_uint8:
                    raw = bytes(responses[0].image_data_uint8)
                    content_type = self._guess_content_type(raw)
                    with self._lock:
                        self.last_frame = CameraFrame(
                            data=raw,
                            content_type=content_type,
                            camera_name=camera,
                            image_type=0,
                        )
                        self.last_frame_at = time.time()
                    failed_count = 0
                else:
                    failed_count += 1
            except Exception:
                failed_count += 1
                client = None  # force reconnect next iteration
                self._stop_event.wait(0.5)
                continue

            # -- try camera fallback on repeated failures --------------------
            if failed_count >= 5:
                self._try_switch_camera(client)
                failed_count = 0

            elapsed = time.time() - started
            self._stop_event.wait(max(0.01, interval - elapsed))

        if client is not None:
            try:
                client.close()
            except Exception:
                pass

    def _try_switch_camera(self, client: object) -> None:
        for camera_name in self._camera_candidates:
            if camera_name == self._active_camera:
                continue
            try:
                import airsim
                c = client
                c.simGetImages(
                    [airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, True)],
                    vehicle_name=self._vehicle_name,
                )
                self._active_camera = camera_name
                logger.info("video capture switched to camera %s", camera_name)
                return
            except Exception:
                continue

    @staticmethod
    def _guess_content_type(data: bytes) -> str:
        if data.startswith(b"\xff\xd8"):
            return "image/jpeg"
        if data.startswith(b"\x89PNG\r\n\x1a\n"):
            return "image/png"
        return "application/octet-stream"
