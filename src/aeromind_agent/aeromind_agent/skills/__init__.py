"""Skill registry entrypoint."""

from .core import SKILLS as CORE_SKILLS
from .future import SKILLS as FUTURE_SKILLS


def all_skill_specs():
    return [*CORE_SKILLS, *FUTURE_SKILLS]
