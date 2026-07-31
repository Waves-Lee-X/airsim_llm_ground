"""Deterministic formation geometry for the four-vehicle demo."""

from __future__ import annotations

import math
from enum import Enum

Offset = tuple[float, float]

_V_ANGLE_DEG = 45.0
_COS45 = math.cos(math.radians(_V_ANGLE_DEG))
_SIN45 = math.sin(math.radians(_V_ANGLE_DEG))


class FormationType(str, Enum):
    """Formations that the fleet must hold and smoothly switch between."""

    LINE = "line"
    V = "v"
    DIAMOND = "diamond"


FORMATION_LABELS = {
    FormationType.LINE: "一字横队",
    FormationType.V: "V字队形",
    FormationType.DIAMOND: "菱形队形",
}


def formation_offsets(
    formation: FormationType,
    vehicle_count: int,
    spacing_m: float,
) -> tuple[Offset, ...]:
    """Return map-frame slot offsets (north, east) for ``vehicle_count`` slots.

    Offsets are relative to the formation reference point. All formations are
    designed so every pairwise slot distance is at least ``spacing_m``.
    """
    if vehicle_count < 1:
        raise ValueError("vehicle_count must be at least 1")
    if spacing_m <= 0.0:
        raise ValueError("spacing_m must be positive")
    count = int(vehicle_count)
    if formation == FormationType.LINE:
        return tuple(
            (
                0.0,
                round((index - (count - 1) / 2.0) * spacing_m, 6),
            )
            for index in range(count)
        )
    if formation == FormationType.V:
        slots: list[Offset] = [(0.0, 0.0)]
        rank = 1
        while len(slots) < count:
            back = -rank * spacing_m
            slots.append(
                (
                    round(back * _COS45, 6),
                    round(-back * _SIN45, 6),
                )
            )
            if len(slots) == count:
                break
            slots.append(
                (
                    round(back * _COS45, 6),
                    round(+back * _SIN45, 6),
                )
            )
            rank += 1
        return tuple(slots)
    if formation == FormationType.DIAMOND:
        base: tuple[Offset, ...] = (
            (0.0, 0.0),
            (spacing_m, 0.0),
            (0.0, -spacing_m),
            (0.0, spacing_m),
            (-spacing_m, 0.0),
        )
        return base[:count]
    raise ValueError(f"unsupported formation {formation!r}")


def minimum_pairwise_distance(offsets: tuple[Offset, ...]) -> float:
    """Return the smallest distance between any two slot offsets."""
    if len(offsets) < 2:
        return math.inf
    best = math.inf
    for index in range(len(offsets)):
        x0, y0 = offsets[index]
        for other in range(index + 1, len(offsets)):
            x1, y1 = offsets[other]
            best = min(best, math.hypot(x1 - x0, y1 - y0))
    return best


def validate_spacing(
    offsets: tuple[Offset, ...],
    min_spacing_m: float,
    tolerance_m: float = 1e-6,
) -> None:
    """Raise ValueError when any slot pair is closer than ``min_spacing_m``."""
    if min_spacing_m <= 0.0:
        raise ValueError("min_spacing_m must be positive")
    closest = minimum_pairwise_distance(offsets)
    if closest < min_spacing_m - tolerance_m:
        raise ValueError(
            f"formation slots violate minimum spacing: closest {closest:.3f} m "
            f"< required {min_spacing_m} m"
        )


def transition_offsets(
    from_formation: FormationType,
    to_formation: FormationType,
    vehicle_count: int,
    spacing_m: float,
    progress: float,
) -> tuple[Offset, ...]:
    """Interpolate slot offsets from one formation to another.

    Slot indices are kept stable, so each vehicle stays in its own slot while
    the whole formation morphs. ``progress`` must be in ``[0, 1]``.
    """
    if not 0.0 <= progress <= 1.0:
        raise ValueError("progress must be in [0, 1]")
    if progress == 0.0:
        return formation_offsets(from_formation, vehicle_count, spacing_m)
    if progress == 1.0:
        return formation_offsets(to_formation, vehicle_count, spacing_m)
    start = formation_offsets(from_formation, vehicle_count, spacing_m)
    end = formation_offsets(to_formation, vehicle_count, spacing_m)
    return tuple(
        (
            round(sx + (ex - sx) * progress, 6),
            round(sy + (ey - sy) * progress, 6),
        )
        for (sx, sy), (ex, ey) in zip(start, end)
    )
