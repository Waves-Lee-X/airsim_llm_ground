"""Small frame containers shared by AirSim, RealSense and replay adapters."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from enum import Enum


class ImageEncoding(str, Enum):
    RGB8 = "rgb8"
    BGR8 = "bgr8"
    Z16_MM = "z16_mm"
    FLOAT32_M = "float32_m"


def _require_aware(timestamp: datetime) -> None:
    if timestamp.tzinfo is None or timestamp.utcoffset() is None:
        raise ValueError("sensor timestamp must include a timezone")


@dataclass(frozen=True)
class RgbFrame:
    observed_at_utc: datetime
    width: int
    height: int
    encoding: ImageEncoding
    data: bytes
    source: str

    def __post_init__(self) -> None:
        _require_aware(self.observed_at_utc)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("frame dimensions must be positive")
        if self.encoding not in {ImageEncoding.RGB8, ImageEncoding.BGR8}:
            raise ValueError("RGB frame must use rgb8 or bgr8")
        if not self.data:
            raise ValueError("RGB frame data must not be empty")
        if len(self.data) != self.width * self.height * 3:
            raise ValueError("RGB frame byte length does not match its dimensions")
        if not self.source.strip():
            raise ValueError("RGB frame source must not be empty")


@dataclass(frozen=True)
class DepthFrame:
    observed_at_utc: datetime
    width: int
    height: int
    encoding: ImageEncoding
    data: bytes
    depth_scale_m: float
    source: str

    def __post_init__(self) -> None:
        _require_aware(self.observed_at_utc)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("frame dimensions must be positive")
        if self.encoding not in {ImageEncoding.Z16_MM, ImageEncoding.FLOAT32_M}:
            raise ValueError("depth frame must use z16_mm or float32_m")
        if not self.data:
            raise ValueError("depth frame data must not be empty")
        if self.depth_scale_m <= 0.0:
            raise ValueError("depth_scale_m must be positive")
        bytes_per_pixel = 2 if self.encoding == ImageEncoding.Z16_MM else 4
        if len(self.data) != self.width * self.height * bytes_per_pixel:
            raise ValueError("depth frame byte length does not match its dimensions")
        if not self.source.strip():
            raise ValueError("depth frame source must not be empty")


@dataclass(frozen=True)
class SensorHealth:
    rgb_ok: bool
    depth_ok: bool
    detail: str = ""
