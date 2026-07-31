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
    FormationType.DIAMOND: "正方形队形",
}


def _square_corners(half: float) -> tuple[Offset, ...]:
    return (
        (round(half, 6), round(-half, 6)),   # front-right
        (round(half, 6), round(half, 6)),    # front-left
        (round(-half, 6), round(-half, 6)),  # back-right
        (round(-half, 6), round(half, 6)),   # back-left
    )


def formation_offsets(
    formation: FormationType,
    vehicle_count: int,
    spacing_m: float,
) -> tuple[Offset, ...]:
    """Return map-frame slot offsets (north, east) for ``vehicle_count`` slots.

    Offsets are relative to the formation reference point. All formations are
    designed so every pairwise slot distance is at least ``spacing_m``.
    The diamond formation is a 2x2 square; the V is symmetric for both odd
    (leader on the vertex) and even (equal wing pairs) vehicle counts.
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
        if count % 2 == 1:
            # Odd count: leader on the vertex, symmetric wing pairs behind.
            slots: list[Offset] = [(0.0, 0.0)]
            for rank in range(1, (count - 1) // 2 + 1):
                back = -rank * spacing_m
                slots.append(
                    (round(back * _COS45, 6), round(-back * _SIN45, 6))
                )
                slots.append(
                    (round(back * _COS45, 6), round(+back * _SIN45, 6))
                )
            return tuple(slots)
        # Even count: no drone on the vertex; the wings carry equal numbers so
        # the V stays perfectly symmetric (front-left slot first).
        slots = []
        for rank in range(1, count // 2 + 1):
            back = -rank * spacing_m
            slots.append(
                (round(back * _COS45, 6), round(-back * _SIN45, 6))
            )
            slots.append(
                (round(back * _COS45, 6), round(+back * _SIN45, 6))
            )
        return tuple(slots)
    if formation == FormationType.DIAMOND:
        if count > 5:
            raise ValueError("square formation supports at most 5 vehicles")
        if count <= 4:
            # Square side length equals the minimum spacing.
            return _square_corners(spacing_m / 2.0)[:count]
        # Five vehicles: centre plus four corners, with the centre exactly
        # one spacing away from every corner (side becomes spacing * sqrt(2)).
        return ((0.0, 0.0),) + _square_corners(spacing_m / math.sqrt(2.0))
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
