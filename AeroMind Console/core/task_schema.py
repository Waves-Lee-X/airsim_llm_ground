from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class MissionArea:
    x_min: float = 0.0
    x_max: float = 60.0
    y_min: float = -20.0
    y_max: float = 20.0

    def to_api(self) -> dict[str, float]:
        return {
            "x_min": self.x_min,
            "x_max": self.x_max,
            "y_min": self.y_min,
            "y_max": self.y_max,
        }


@dataclass(frozen=True)
class PlanStep:
    name: str
    detail: str
    action: str = "note"

    def to_api(self) -> dict[str, str]:
        return {
            "name": self.name,
            "detail": self.detail,
            "action": self.action,
        }


@dataclass(frozen=True)
class MissionPlan:
    task: str
    intent: str
    target: str = "unknown"
    altitude_m: float = 8.0
    strategy: str = "guided"
    area: MissionArea | None = None
    plan: list[PlanStep] = field(default_factory=list)
    raw: dict[str, Any] = field(default_factory=dict)

    def to_api(self) -> dict[str, Any]:
        return {
            "task": self.task,
            "intent": self.intent,
            "target": self.target,
            "altitude_m": self.altitude_m,
            "strategy": self.strategy,
            "area": self.area.to_api() if self.area else None,
            "plan": [step.to_api() for step in self.plan],
            "raw": self.raw,
        }

