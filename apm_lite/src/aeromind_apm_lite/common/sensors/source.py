"""Interface implemented by AirSim, RealSense and offline replay sources."""

from __future__ import annotations

from typing import Protocol

from .models import DepthFrame, RgbFrame, SensorHealth


class SensorSource(Protocol):
    def read_rgb(self) -> RgbFrame | None:
        ...

    def read_depth(self) -> DepthFrame | None:
        ...

    def health(self) -> SensorHealth:
        ...
