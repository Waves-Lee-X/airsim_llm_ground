"""Internal transport types shared by the two stack adapters."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MissionWireItem:
    sequence: int
    frame: int
    command: int
    current: int
    autocontinue: int
    params: tuple[float, float, float, float]
    x: int
    y: int
    z: float


__all__ = ["MissionWireItem"]
