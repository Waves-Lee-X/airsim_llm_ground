"""Strict JSON conversion for values crossing Gateway boundaries."""

from __future__ import annotations

import json
import math


def json_compatible(value):
    """Recursively convert ROS and NumPy values to strict JSON primitives."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [json_compatible(item) for item in value]

    item = getattr(value, "item", None)
    if callable(item):
        try:
            converted = item()
            if converted is not value:
                return json_compatible(converted)
        except (TypeError, ValueError):
            pass

    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        try:
            converted = tolist()
            if converted is not value:
                return json_compatible(converted)
        except (TypeError, ValueError):
            pass

    return str(value)


def json_dumps(value, **kwargs) -> str:
    """Serialize a value after normalization and reject non-standard numbers."""
    kwargs.setdefault("ensure_ascii", False)
    kwargs["allow_nan"] = False
    return json.dumps(json_compatible(value), **kwargs)
