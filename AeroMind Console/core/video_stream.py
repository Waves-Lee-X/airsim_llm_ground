from __future__ import annotations

import threading
import time

from core.airsim_adapter import AirSimAdapter, CameraFrame


class VideoStream:
    """AirSim front camera MJPEG streaming helper."""

    boundary = "aeromind-frame"

    def __init__(self, adapter: AirSimAdapter) -> None:
        self.adapter = adapter
        self.last_frame: CameraFrame | None = None
        self.last_frame_at = 0.0
        self.target_fps = 8.0
        self._lock = threading.Lock()
        self._worker_started = False
        self._stop_event = threading.Event()

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

    def _start_once(self) -> None:
        with self._lock:
            if self._worker_started:
                return
            self._worker_started = True
        thread = threading.Thread(target=self._capture_loop, name="aeromind-video-capture", daemon=True)
        thread.start()

    def start(self) -> None:
        self._start_once()

    def stop(self) -> None:
        self._stop_event.set()

    def frames(self, interval_s: float = 0.08):
        self._start_once()
        last_sent_at = 0.0
        while True:
            frame = self.latest_frame()
            if frame is not None and self.last_frame_at != last_sent_at:
                last_sent_at = self.last_frame_at
                yield frame
            time.sleep(max(0.02, float(interval_s)))

    def _capture_loop(self) -> None:
        interval = 1.0 / max(1.0, float(self.target_fps))
        failed_count = 0
        while not self._stop_event.is_set():
            started = time.time()
            frame = self.adapter.camera_frame()
            if frame is not None:
                with self._lock:
                    self.last_frame = frame
                    self.last_frame_at = time.time()
                failed_count = 0
            else:
                failed_count += 1
                if failed_count >= 5:
                    self._try_switch_camera()
                    failed_count = 0
            elapsed = time.time() - started
            time.sleep(max(0.01, interval - elapsed))

    def _try_switch_camera(self) -> None:
        current = self.adapter.last_camera_name or self.adapter.active_camera_name
        for camera_name in self.adapter.camera_candidates:
            if camera_name == current:
                continue
            try:
                self.adapter.select_camera(camera_name)
                break
            except Exception:
                continue
