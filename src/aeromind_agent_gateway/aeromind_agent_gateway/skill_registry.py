"""Gateway skill registry shared by all model providers."""

from __future__ import annotations

import importlib
import os
from typing import Any, Callable

from .workflow import build_square_workflow


SkillBuilder = Callable[..., dict[str, Any]]


SKILLS: dict[str, dict[str, Any]] = {}
_LOADED_MODULES: set[str] = set()


def register_skill(
    name: str,
    *,
    label: str,
    version: str,
    risk_level: str,
    description: str,
    example_task: str,
    builder: SkillBuilder,
) -> None:
    clean_name = name.strip()
    if not clean_name or clean_name in SKILLS:
        raise ValueError(f"技能名称无效或重复: {clean_name}")
    if risk_level not in {"low", "medium", "high"}:
        raise ValueError(f"技能风险等级无效: {risk_level}")
    SKILLS[clean_name] = {
        "name": clean_name,
        "label": label,
        "version": version,
        "type": "workflow",
        "enabled": True,
        "risk_level": risk_level,
        "description": description,
        "example_task": example_task,
        "builder": builder,
    }


def load_configured_skill_modules() -> None:
    modules = os.getenv("AEROMIND_GATEWAY_SKILL_MODULES", "")
    for name in (item.strip() for item in modules.split(",")):
        if not name or name in _LOADED_MODULES:
            continue
        importlib.import_module(name)
        _LOADED_MODULES.add(name)


register_skill(
    "flight.square",
    label="正方形轨迹",
    version="1.0.0",
    risk_level="high",
    description="必要时起飞，依次飞行四条边，可选择结束后降落。",
    example_task="飞一个边长 10 米的正方形轨迹",
    builder=build_square_workflow,
)


def skill_catalog() -> list[dict[str, Any]]:
    load_configured_skill_modules()
    return [
        {key: value for key, value in item.items() if key != "builder"}
        for item in SKILLS.values()
    ]


def build_skill(name: str, args: dict[str, Any]) -> dict[str, Any]:
    load_configured_skill_modules()
    skill = SKILLS.get(name)
    if skill is None:
        raise ValueError(f"未知技能: {name}")
    return skill["builder"](**args)
