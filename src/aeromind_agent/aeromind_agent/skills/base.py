"""Skill metadata primitives for the AeroMind Agent."""

from dataclasses import asdict, dataclass, field


@dataclass(frozen=True)
class SkillSpec:
    name: str
    label: str
    intent: str
    type: str
    risk_level: str
    description: str
    example_task: str
    tools: list[str] = field(default_factory=list)
    enabled: bool = False

    def to_dict(self):
        return asdict(self)
