"""AeroMind skill catalog backed by modular skill specs."""

from .skills import all_skill_specs


def get_skill_catalog():
    return [skill.to_dict() for skill in all_skill_specs()]


def skill_by_name(name: str):
    for skill in get_skill_catalog():
        if skill["name"] == name:
            return skill
    return None
