"""Fault-isolated AirSim and onboard RTSP camera bridges."""

from __future__ import annotations

import asyncio
import math
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable, Optional, Protocol, Tuple


class CameraUnavailable(RuntimeError):
    """Raised when no fresh camera frame is available."""


def read_airsim_depth_at(
    *,
    rpc_host: str,
    rpc_port: int,
    vehicle_name: str,
    camera_name: str,
    center_normalized: tuple[float, float],
    width_px: int,
    height_px: int,
    fov_degrees: float,
) -> dict[str, Any] | None:
    """Read the AirSim float depth at a normalized pixel center.

    Returns ``{"distance_m": ...}`` (Euclidean distance along the camera ray
    through the pixel) or ``None`` when depth is unavailable or the pixel is
    out of range.  Callers combine the distance with the pixel bearing and
    the vehicle pose to localize the target in the FCU local frame.
    """
    try:
        import airsim
        import numpy as np
    except ImportError:  # pragma: no cover - ground extra dependency
        return None
    try:
        client = airsim.MultirotorClient(ip=rpc_host, port=rpc_port)
        client.confirmConnection()
        responses = client.simGetImages(
            [
                # AirSim 1.8.x clients do not accept width/height kwargs here;
                # the default depth resolution (256x144) is enough because the
                # pixel is sampled with normalized coordinates.
                airsim.ImageRequest(
                    camera_name,
                    airsim.ImageType.DepthPerspective,
                    pixels_as_float=True,
                    compress=False,
                )
            ],
            vehicle_name=vehicle_name,
        )
        if not responses or getattr(responses[0], "width", 0) <= 0:
            return None
        response = responses[0]
        depth = np.array(
            response.image_data_float,
            dtype=np.float32,
        ).reshape(response.height, response.width)
        cx, cy = center_normalized
        u = int(round(cx * response.width))
        v = int(round(cy * response.height))
        if not (0 <= u < response.width and 0 <= v < response.height):
            return None
        z_depth = float(depth[v, u])
        if not math.isfinite(z_depth) or z_depth <= 0.05:
            return None
        # DepthPerspective stores the camera-Z distance; convert it to the
        # Euclidean distance along the ray through the pixel.
        half_h = math.radians(float(fov_degrees)) / 2.0
        half_v = math.atan(
            math.tan(half_h) / (float(response.width) / max(1, response.height))
        )
        h_angle = (float(cx) - 0.5) * 2.0 * half_h
        v_angle = (0.5 - float(cy)) * 2.0 * half_v
        along = math.cos(v_angle) * math.cos(h_angle)
        if along <= 0.01:
            return None
        return {"distance_m": float(z_depth / along)}
    except Exception:  # pragma: no cover - fault isolation
        return None


class AirSimImageClient(Protocol):
    def ping(self) -> bool: ...

    def simGetImage(
        self,
        camera_name: str,
        image_type: int,
        vehicle_name: str = "",
        external: bool = False,
    ) -> Any: ...


@dataclass(frozen=True)
class AirSimCameraConfig:
    rpc_host: str
    rpc_port: int = 41451
    vehicle_name: str = "Drone1"
    camera_name: str = "front_center"
    width_px: int = 640
    height_px: int = 360
    fov_degrees: float = 95.0
    capture_fps: float = 5.0
    request_timeout_s: float = 2.0
    reconnect_delay_s: float = 1.0
    stale_after_s: float = 3.0

    def __post_init__(self) -> None:
        if not self.rpc_host.strip():
            raise ValueError("rpc_host must not be empty")
        if not 1 <= self.rpc_port <= 65535:
            raise ValueError("rpc_port must be in [1, 65535]")
        if not self.vehicle_name.strip() or not self.camera_name.strip():
            raise ValueError("vehicle_name and camera_name must not be empty")
        if not 160 <= self.width_px <= 7680:
            raise ValueError("width_px must be in [160, 7680]")
        if not 120 <= self.height_px <= 4320:
            raise ValueError("height_px must be in [120, 4320]")
        if not 10.0 <= self.fov_degrees <= 170.0:
            raise ValueError("fov_degrees must be in [10, 170]")
        if not 0.2 <= self.capture_fps <= 30.0:
            raise ValueError("capture_fps must be in [0.2, 30]")
        for label, value in (
            ("request_timeout_s", self.request_timeout_s),
            ("reconnect_delay_s", self.reconnect_delay_s),
            ("stale_after_s", self.stale_after_s),
        ):
            if value <= 0.0:
                raise ValueError(f"{label} must be positive")


@dataclass(frozen=True)
class CameraFrame:
    data: bytes
    media_type: str
    sequence: int
    captured_at_utc: datetime
    captured_monotonic_s: float


class CameraBridge(Protocol):
    async def start(self) -> None: ...

    async def stop(self) -> None: ...

    def status_payload(self) -> dict[str, Any]: ...

    async def get_frame(self) -> CameraFrame: ...


ClientFactory = Callable[[AirSimCameraConfig], AirSimImageClient]


def _default_client_factory(config: AirSimCameraConfig) -> AirSimImageClient:
    try:
        import airsim
    except ImportError as exc:  # pragma: no cover - depends on deployment extras
        raise CameraUnavailable(
            "AirSim Python client is not installed; install the ground extra"
        ) from exc
    return airsim.MultirotorClient(
        ip=config.rpc_host,
        port=config.rpc_port,
        timeout_value=config.request_timeout_s,
    )


class AirSimCameraBridge:
    """Capture compressed AirSim frames without blocking the asyncio loop."""

    _SCENE_IMAGE_TYPE = 0
    _MAX_FRAME_BYTES = 32 * 1024 * 1024

    def __init__(
        self,
        config: AirSimCameraConfig,
        *,
        client_factory: ClientFactory = _default_client_factory,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self._client_factory = client_factory
        self._clock = clock
        self._executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="airsim-camera",
        )
        self._client: AirSimImageClient | None = None
        self._runner: asyncio.Task[None] | None = None
        self._capture_lock = asyncio.Lock()
        self._frame: CameraFrame | None = None
        self._last_error: str | None = None
        self._sequence = 0
        self._stopped = False

    @property
    def running(self) -> bool:
        return self._runner is not None and not self._runner.done()

    async def start(self) -> None:
        if self._runner is not None:
            raise RuntimeError("camera bridge can only be started once")
        self._runner = asyncio.create_task(
            self._capture_loop(),
            name="airsim-camera-capture",
        )

    async def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        runner = self._runner
        self._runner = None
        if runner is not None:
            runner.cancel()
            await asyncio.gather(runner, return_exceptions=True)
        self._executor.shutdown(wait=True, cancel_futures=True)

    def status_payload(self) -> dict[str, Any]:
        now = self._clock()
        frame = self._frame
        frame_age_s = (
            None if frame is None else max(0.0, now - frame.captured_monotonic_s)
        )
        available = frame_age_s is not None and frame_age_s <= self.config.stale_after_s
        if available and self._last_error is None:
            state = "online"
            detail = (
                f"AirSim {self.config.vehicle_name}/{self.config.camera_name} live"
            )
        elif available:
            state = "degraded"
            detail = f"showing cached frame; latest capture failed: {self._last_error}"
        elif self._last_error is not None:
            state = "offline"
            detail = self._last_error
        else:
            state = "connecting"
            detail = "waiting for the first AirSim frame"
        return {
            "name": self.config.camera_name,
            "kind": "scene_rgb",
            "vehicle_name": self.config.vehicle_name,
            "width": self.config.width_px,
            "height": self.config.height_px,
            "fov_degrees": self.config.fov_degrees,
            "rpc_endpoint": f"{self.config.rpc_host}:{self.config.rpc_port}",
            "state": state,
            "stream_available": available,
            "stream_url": "/api/camera/frame",
            "frame_sequence": frame.sequence if frame is not None else None,
            "frame_age_s": frame_age_s,
            "last_frame_at_utc": (
                frame.captured_at_utc.isoformat() if frame is not None else None
            ),
            "detail": detail,
        }

    async def get_frame(self) -> CameraFrame:
        frame = self._fresh_frame()
        if frame is not None:
            return frame
        await self.capture_once()
        frame = self._fresh_frame()
        if frame is None:
            raise CameraUnavailable(
                self._last_error or "no fresh AirSim camera frame is available"
            )
        return frame

    async def capture_once(self) -> bool:
        if self._stopped:
            return False
        async with self._capture_lock:
            loop = asyncio.get_running_loop()
            try:
                data, media_type = await loop.run_in_executor(
                    self._executor,
                    self._capture_sync,
                )
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self._last_error = f"{type(exc).__name__}: {exc}"
                return False
            self._sequence += 1
            self._frame = CameraFrame(
                data=data,
                media_type=media_type,
                sequence=self._sequence,
                captured_at_utc=datetime.now(timezone.utc),
                captured_monotonic_s=self._clock(),
            )
            self._last_error = None
            return True

    async def _capture_loop(self) -> None:
        frame_period_s = 1.0 / self.config.capture_fps
        while True:
            started_at = self._clock()
            succeeded = await self.capture_once()
            elapsed = max(0.0, self._clock() - started_at)
            delay = (
                max(0.0, frame_period_s - elapsed)
                if succeeded
                else self.config.reconnect_delay_s
            )
            await asyncio.sleep(delay)

    def _capture_sync(self) -> tuple[bytes, str]:
        try:
            if self._client is None:
                self._client = self._client_factory(self.config)
                if not self._client.ping():
                    raise CameraUnavailable("AirSim RPC ping returned false")
            raw = self._client.simGetImage(
                self.config.camera_name,
                self._SCENE_IMAGE_TYPE,
                vehicle_name=self.config.vehicle_name,
            )
            if raw is None or raw == "":
                raise CameraUnavailable(
                    "AirSim returned no Scene image; check UE Play mode, vehicle and camera"
                )
            data = bytes(raw)
            if not data or len(data) > self._MAX_FRAME_BYTES:
                raise CameraUnavailable("AirSim returned an invalid frame size")
            return data, self._media_type(data)
        except Exception:
            self._client = None
            raise

    def _fresh_frame(self) -> CameraFrame | None:
        frame = self._frame
        if frame is None:
            return None
        if self._clock() - frame.captured_monotonic_s > self.config.stale_after_s:
            return None
        return frame

    @staticmethod
    def _media_type(data: bytes) -> str:
        if data.startswith(b"\x89PNG\r\n\x1a\n"):
            return "image/png"
        if data.startswith(b"\xff\xd8\xff"):
            return "image/jpeg"
        raise CameraUnavailable("AirSim frame is not a PNG or JPEG image")


class RtspCapture(Protocol):
    def isOpened(self) -> bool: ...

    def read(self) -> Tuple[bool, Any]: ...

    def release(self) -> None: ...


@dataclass(frozen=True)
class RtspCameraConfig:
    stream_url: str
    name: str = "D435i RGB"
    width_px: int = 424
    height_px: int = 240
    capture_fps: float = 15.0
    open_timeout_ms: int = 3_000
    read_timeout_ms: int = 2_000
    reconnect_delay_s: float = 1.0
    stale_after_s: float = 10.0
    jpeg_quality: int = 80

    def __post_init__(self) -> None:
        if not self.stream_url.startswith(("rtsp://", "rtsps://")):
            raise ValueError("stream_url must use rtsp:// or rtsps://")
        if not self.name.strip():
            raise ValueError("name must not be empty")
        if not 160 <= self.width_px <= 7680:
            raise ValueError("width_px must be in [160, 7680]")
        if not 120 <= self.height_px <= 4320:
            raise ValueError("height_px must be in [120, 4320]")
        if not 0.2 <= self.capture_fps <= 30.0:
            raise ValueError("capture_fps must be in [0.2, 30]")
        if self.open_timeout_ms <= 0 or self.read_timeout_ms <= 0:
            raise ValueError("RTSP timeouts must be positive")
        if self.reconnect_delay_s <= 0.0 or self.stale_after_s <= 0.0:
            raise ValueError("RTSP reconnect and stale intervals must be positive")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in [1, 100]")


RtspCaptureFactory = Callable[[RtspCameraConfig], RtspCapture]
RtspFrameEncoder = Callable[[Any, int], bytes]


def _default_rtsp_capture_factory(config: RtspCameraConfig) -> RtspCapture:
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - deployment dependency
        raise CameraUnavailable(
            "OpenCV is not installed; install the ground camera dependencies"
        ) from exc

    params = []
    if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
        params.extend([cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, config.open_timeout_ms])
    if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
        params.extend([cv2.CAP_PROP_READ_TIMEOUT_MSEC, config.read_timeout_ms])
    try:
        capture = cv2.VideoCapture(config.stream_url, cv2.CAP_FFMPEG, params)
    except (TypeError, cv2.error):
        capture = cv2.VideoCapture(config.stream_url, cv2.CAP_FFMPEG)
    if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def _default_rtsp_frame_encoder(frame: Any, quality: int) -> bytes:
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - deployment dependency
        raise CameraUnavailable(
            "OpenCV is not installed; install the ground camera dependencies"
        ) from exc
    succeeded, encoded = cv2.imencode(
        ".jpg",
        frame,
        [cv2.IMWRITE_JPEG_QUALITY, quality],
    )
    if not succeeded:
        raise CameraUnavailable("OpenCV could not encode the RTSP frame")
    return bytes(encoded)


class RtspCameraBridge:
    """Read an onboard RTSP stream and expose fresh JPEG frames to FastAPI."""

    _MAX_FRAME_BYTES = 16 * 1024 * 1024

    def __init__(
        self,
        config: RtspCameraConfig,
        *,
        capture_factory: RtspCaptureFactory = _default_rtsp_capture_factory,
        frame_encoder: RtspFrameEncoder = _default_rtsp_frame_encoder,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self._capture_factory = capture_factory
        self._frame_encoder = frame_encoder
        self._clock = clock
        self._executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="rtsp-camera",
        )
        self._capture: Optional[RtspCapture] = None
        self._runner: Optional[asyncio.Task[None]] = None
        self._capture_lock = asyncio.Lock()
        self._frame: Optional[CameraFrame] = None
        self._last_error: Optional[str] = None
        self._sequence = 0
        self._capture_times: deque[float] = deque(maxlen=60)
        self._last_delivered_sequence = 0
        self._consumer_skipped_frames = 0
        self._width_px = config.width_px
        self._height_px = config.height_px
        self._stopped = False

    @property
    def running(self) -> bool:
        return self._runner is not None and not self._runner.done()

    async def start(self) -> None:
        if self._runner is not None:
            raise RuntimeError("camera bridge can only be started once")
        self._runner = asyncio.create_task(
            self._capture_loop(),
            name="rtsp-camera-capture",
        )

    async def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        runner = self._runner
        self._runner = None
        if runner is not None:
            runner.cancel()
            await asyncio.gather(runner, return_exceptions=True)
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(self._executor, self._release_capture)
        self._executor.shutdown(wait=True, cancel_futures=True)

    def status_payload(self) -> dict[str, Any]:
        frame = self._frame
        frame_age_s = (
            None
            if frame is None
            else max(0.0, self._clock() - frame.captured_monotonic_s)
        )
        available = frame_age_s is not None and frame_age_s <= self.config.stale_after_s
        if available and self._last_error is None:
            state = "online"
            detail = "onboard D435i RGB stream live"
        elif available:
            state = "degraded"
            detail = f"showing cached frame; latest RTSP read failed: {self._last_error}"
        elif self._last_error is not None:
            state = "offline"
            detail = self._last_error
        else:
            state = "connecting"
            detail = "waiting for the first onboard RTSP frame"
        return {
            "name": self.config.name,
            "kind": "onboard_rgb",
            "width": self._width_px,
            "height": self._height_px,
            "source_endpoint": self.config.stream_url,
            "state": state,
            "stream_available": available,
            "stream_url": "/api/camera/frame",
            "frame_sequence": frame.sequence if frame is not None else None,
            "frame_age_s": frame_age_s,
            "source_fps": self._measured_source_fps(),
            "target_source_fps": self.config.capture_fps,
            "frames_received": self._sequence,
            "consumer_skipped_frames": self._consumer_skipped_frames,
            "last_frame_at_utc": (
                frame.captured_at_utc.isoformat() if frame is not None else None
            ),
            "detail": detail,
        }

    async def get_frame(self) -> CameraFrame:
        frame = self._fresh_frame()
        if frame is None and not self.running:
            await self.capture_once()
            frame = self._fresh_frame()
        if frame is None:
            raise CameraUnavailable(
                self._last_error or "no fresh onboard RTSP frame is available"
            )
        if self._last_delivered_sequence:
            self._consumer_skipped_frames += max(
                0,
                frame.sequence - self._last_delivered_sequence - 1,
            )
        self._last_delivered_sequence = frame.sequence
        return frame

    async def capture_once(self) -> bool:
        if self._stopped:
            return False
        async with self._capture_lock:
            loop = asyncio.get_running_loop()
            try:
                data, width_px, height_px = await loop.run_in_executor(
                    self._executor,
                    self._capture_sync,
                )
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self._last_error = f"{type(exc).__name__}: {exc}"
                return False
            self._sequence += 1
            self._width_px = width_px
            self._height_px = height_px
            captured_monotonic_s = self._clock()
            self._capture_times.append(captured_monotonic_s)
            self._frame = CameraFrame(
                data=data,
                media_type="image/jpeg",
                sequence=self._sequence,
                captured_at_utc=datetime.now(timezone.utc),
                captured_monotonic_s=captured_monotonic_s,
            )
            self._last_error = None
            return True

    async def _capture_loop(self) -> None:
        while True:
            succeeded = await self.capture_once()
            if succeeded:
                # VideoCapture.read() is paced by the RTSP source. Do not sleep here:
                # draining every decoded frame prevents stale frames building up.
                await asyncio.sleep(0)
            else:
                await asyncio.sleep(self.config.reconnect_delay_s)

    def _capture_sync(self) -> Tuple[bytes, int, int]:
        try:
            if self._capture is None:
                self._capture = self._capture_factory(self.config)
                if not self._capture.isOpened():
                    raise CameraUnavailable("could not open the onboard RTSP stream")
            succeeded, raw_frame = self._capture.read()
            if not succeeded or raw_frame is None:
                raise CameraUnavailable("onboard RTSP stream returned no frame")
            data = self._frame_encoder(raw_frame, self.config.jpeg_quality)
            if not data or len(data) > self._MAX_FRAME_BYTES:
                raise CameraUnavailable("RTSP bridge produced an invalid JPEG size")
            shape = getattr(raw_frame, "shape", ())
            height_px = int(shape[0]) if len(shape) >= 2 else self._height_px
            width_px = int(shape[1]) if len(shape) >= 2 else self._width_px
            return data, width_px, height_px
        except Exception:
            self._release_capture()
            raise

    def _release_capture(self) -> None:
        capture = self._capture
        self._capture = None
        if capture is not None:
            capture.release()

    def _fresh_frame(self) -> Optional[CameraFrame]:
        frame = self._frame
        if frame is None:
            return None
        if self._clock() - frame.captured_monotonic_s > self.config.stale_after_s:
            return None
        return frame

    def _measured_source_fps(self) -> float:
        if len(self._capture_times) < 2:
            return 0.0
        elapsed = self._capture_times[-1] - self._capture_times[0]
        if elapsed <= 0.0:
            return 0.0
        return round((len(self._capture_times) - 1) / elapsed, 1)
