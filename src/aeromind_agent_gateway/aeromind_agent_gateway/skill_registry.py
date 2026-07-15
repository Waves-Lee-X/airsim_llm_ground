"""File-backed Gateway skill registry shared by all model providers."""

from __future__ import annotations

import ast
import importlib
import math
import os
from pathlib import Path
import re
from typing import Any, Callable

import yaml

from .workflow import validate_workflow


SkillBuilder = Callable[..., dict[str, Any]]
SKILLS: dict[str, dict[str, Any]] = {}
_LOADED_MODULES: set[str] = set()
_BUILTINS_LOADED = False
_EXPRESSION = re.compile(r"\$\{([^}]+)\}")


def register_skill(
    name: str,
    *,
    label: str,
    version: str,
    risk_level: str,
    description: str,
    example_task: str,
    builder: SkillBuilder,
    document: str | None = None,
    parameters: dict[str, Any] | None = None,
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
        "parameters": parameters or {},
        "builder": builder,
        "document": document,
    }


def load_configured_skill_modules() -> None:
    _load_builtin_manifests()
    modules = os.getenv("AEROMIND_GATEWAY_SKILL_MODULES", "")
    for name in (item.strip() for item in modules.split(",")):
        if not name or name in _LOADED_MODULES:
            continue
        importlib.import_module(name)
        _LOADED_MODULES.add(name)


def skill_catalog() -> list[dict[str, Any]]:
    load_configured_skill_modules()
    catalog = []
    for item in SKILLS.values():
        value = {
            key: field
            for key, field in item.items()
            if key not in {"builder", "document"}
        }
        if item.get("document"):
            value["instructions"] = _read_skill_document(item["document"])
        catalog.append(value)
    return catalog


def build_skill(name: str, args: dict[str, Any]) -> dict[str, Any]:
    load_configured_skill_modules()
    skill = SKILLS.get(name)
    if skill is None:
        raise ValueError(f"未知技能: {name}")
    return skill["builder"](**args)


def _load_builtin_manifests() -> None:
    global _BUILTINS_LOADED
    if _BUILTINS_LOADED:
        return
    root = Path(__file__).with_name("skill_docs")
    for path in sorted(root.glob("*/skill.yaml")):
        manifest = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(manifest, dict):
            raise ValueError(f"技能清单必须是对象: {path}")
        parameters = manifest.get("parameters") or {}
        register_skill(
            str(manifest["name"]),
            label=str(manifest["label"]),
            version=str(manifest.get("version", "1.0.0")),
            risk_level=str(manifest.get("risk_level", "high")),
            description=str(manifest.get("description", "")),
            example_task=str(manifest.get("example_task", "")),
            builder=_manifest_builder(manifest),
            document=f"{path.parent.name}/SKILL.md",
            parameters=parameters,
        )
    _BUILTINS_LOADED = True


def _manifest_builder(manifest: dict[str, Any]) -> SkillBuilder:
    def build(**provided):
        values = _validate_parameters(manifest.get("parameters") or {}, provided)
        steps = []
        for raw_step in manifest.get("workflow") or []:
            include_if = raw_step.get("include_if")
            if include_if and not bool(values.get(str(include_if))):
                continue
            step = {key: value for key, value in raw_step.items() if key != "include_if"}
            steps.append(_render_value(step, values))
        return validate_workflow({
            "name": _render_value(manifest.get("workflow_name", manifest["label"]), values),
            "steps": steps,
        })

    return build


def _validate_parameters(specs: dict[str, Any], provided: dict[str, Any]) -> dict[str, Any]:
    unknown = set(provided) - set(specs)
    if unknown:
        raise ValueError(f"未知技能参数: {', '.join(sorted(unknown))}")
    values = {}
    for name, spec in specs.items():
        if name in provided:
            value = provided[name]
        elif "default" in spec:
            value = spec["default"]
        elif spec.get("required"):
            raise ValueError(f"缺少技能参数: {name}")
        else:
            continue
        kind = spec.get("type", "string")
        if kind == "number":
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"参数 {name} 必须是有限数值")
            if "minimum" in spec and value < float(spec["minimum"]):
                raise ValueError(f"参数 {name} 小于最小值")
            if "maximum" in spec and value > float(spec["maximum"]):
                raise ValueError(f"参数 {name} 超过最大值")
        elif kind == "boolean":
            if not isinstance(value, bool):
                raise ValueError(f"参数 {name} 必须是布尔值")
        elif kind == "string":
            value = str(value)
        else:
            raise ValueError(f"参数 {name} 类型不受支持: {kind}")
        values[name] = value
    return values


def _render_value(value: Any, variables: dict[str, Any]) -> Any:
    if isinstance(value, dict):
        return {key: _render_value(field, variables) for key, field in value.items()}
    if isinstance(value, list):
        return [_render_value(item, variables) for item in value]
    if not isinstance(value, str):
        return value
    matches = list(_EXPRESSION.finditer(value))
    if len(matches) == 1 and matches[0].span() == (0, len(value)):
        return _safe_expression(matches[0].group(1), variables)
    return _EXPRESSION.sub(
        lambda match: str(_safe_expression(match.group(1), variables)), value
    )


def _safe_expression(expression: str, variables: dict[str, Any]) -> Any:
    tree = ast.parse(expression.strip(), mode="eval")

    def evaluate(node):
        if isinstance(node, ast.Expression):
            return evaluate(node.body)
        if isinstance(node, ast.Name) and node.id in variables:
            return variables[node.id]
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float, str, bool)):
            return node.value
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
            value = evaluate(node.operand)
            return -value if isinstance(node.op, ast.USub) else value
        if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)):
            left, right = evaluate(node.left), evaluate(node.right)
            return {
                ast.Add: lambda: left + right,
                ast.Sub: lambda: left - right,
                ast.Mult: lambda: left * right,
                ast.Div: lambda: left / right,
            }[type(node.op)]()
        raise ValueError(f"不允许的技能表达式: {expression}")

    return evaluate(tree)


def _read_skill_document(relative_path: str) -> str:
    path = Path(__file__).with_name("skill_docs") / relative_path
    try:
        content = path.read_text(encoding="utf-8")
    except OSError:
        return ""
    if content.startswith("---"):
        _, _, content = content.partition("---")
        _, _, content = content.partition("---")
    return content.strip()
