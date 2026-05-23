from __future__ import annotations

import logging
import math
from typing import Any

from core.path_planner import Waypoint
from core.task_schema import MissionArea

logger = logging.getLogger(__name__)


def blocked_zones_from_points(area: MissionArea, points: set[tuple[float, float]]) -> list[dict[str, Any]]:
    if not points:
        return []
    cell_size = 2.0
    cells: dict[tuple[int, int], int] = {}
    for x, y in points:
        if not (area.x_min <= x <= area.x_max and area.y_min <= y <= area.y_max):
            continue
        col = int((x - area.x_min) / cell_size)
        row = int((y - area.y_min) / cell_size)
        cells[(col, row)] = cells.get((col, row), 0) + 1
    zones: list[dict[str, Any]] = []
    for (col, row), count in cells.items():
        if count < 4:
            continue
        x_min = area.x_min + col * cell_size
        y_min = area.y_min + row * cell_size
        zones.append(
            {
                "x_min": round(x_min, 2),
                "x_max": round(min(area.x_max, x_min + cell_size), 2),
                "y_min": round(y_min, 2),
                "y_max": round(min(area.y_max, y_min + cell_size), 2),
                "points": count,
                "type": "obstacle",
            }
        )
    zones.sort(key=lambda zone: int(zone["points"]), reverse=True)
    return zones


def count_route_hits(route: list[Any], zones: list[dict[str, Any]]) -> int:
    hits = 0
    for start, end in zip(route, route[1:]):
        sx = float(getattr(start, "x", 0.0))
        sy = float(getattr(start, "y", 0.0))
        ex = float(getattr(end, "x", 0.0))
        ey = float(getattr(end, "y", 0.0))
        for zone in zones:
            if segment_intersects_zone(sx, sy, ex, ey, zone, padding_m=1.5):
                hits += 1
                break
    return hits


def segment_intersects_zone(
    sx: float,
    sy: float,
    ex: float,
    ey: float,
    zone: dict[str, Any],
    padding_m: float,
) -> bool:
    try:
        x_min = float(zone["x_min"]) - padding_m
        x_max = float(zone["x_max"]) + padding_m
        y_min = float(zone["y_min"]) - padding_m
        y_max = float(zone["y_max"]) + padding_m
    except Exception as zone_parse_err:
        logger.warning("Failed to parse zone for intersection check: %s", zone_parse_err)
        return False
    if max(sx, ex) < x_min or min(sx, ex) > x_max or max(sy, ey) < y_min or min(sy, ey) > y_max:
        return False
    if abs(sy - ey) <= 0.2:
        return y_min <= sy <= y_max
    if abs(sx - ex) <= 0.2:
        return x_min <= sx <= x_max
    return True


def replan_route_around_zones(route: list[Any], zones: Any, padding_m: float = 2.0) -> list[Waypoint]:
    if not isinstance(zones, list) or not zones:
        return [Waypoint(float(point.x), float(point.y), float(point.z)) for point in route]
    safe_route: list[Waypoint] = []
    min_segment_m = 3.0
    for start, end in zip(route, route[1:]):
        sx = float(getattr(start, "x", 0.0))
        sy = float(getattr(start, "y", 0.0))
        sz = float(getattr(start, "z", -10.0))
        ex = float(getattr(end, "x", 0.0))
        ey = float(getattr(end, "y", 0.0))
        ez = float(getattr(end, "z", sz))
        if abs(sy - ey) > 0.2:
            continue
        lane_y = sy
        x_low = min(sx, ex)
        x_high = max(sx, ex)
        blocked_intervals: list[tuple[float, float]] = []
        for zone in zones:
            try:
                zone_y_min = float(zone["y_min"]) - padding_m
                zone_y_max = float(zone["y_max"]) + padding_m
                if zone_y_min <= lane_y <= zone_y_max:
                    blocked_intervals.append(
                        (
                            max(x_low, float(zone["x_min"]) - padding_m),
                            min(x_high, float(zone["x_max"]) + padding_m),
                        )
                    )
            except Exception as zone_iter_err:
                logger.warning("Failed to parse zone for route replanning: %s", zone_iter_err)
                continue
        intervals = subtract_intervals(x_low, x_high, blocked_intervals)
        if sx > ex:
            intervals = list(reversed(intervals))
        for seg_start, seg_end in intervals:
            if abs(seg_end - seg_start) < min_segment_m:
                continue
            a, b = (seg_start, seg_end) if sx <= ex else (seg_end, seg_start)
            if not safe_route or math.hypot(safe_route[-1].x - a, safe_route[-1].y - lane_y) > 0.5:
                safe_route.append(Waypoint(a, lane_y, sz))
            safe_route.append(Waypoint(b, lane_y, ez))
    return safe_route


def subtract_intervals(x_low: float, x_high: float, blocked: list[tuple[float, float]]) -> list[tuple[float, float]]:
    intervals = [(float(x_low), float(x_high))]
    for block_start, block_end in sorted(blocked):
        next_intervals: list[tuple[float, float]] = []
        for start, end in intervals:
            if block_end <= start or block_start >= end:
                next_intervals.append((start, end))
                continue
            if block_start > start:
                next_intervals.append((start, min(block_start, end)))
            if block_end < end:
                next_intervals.append((max(block_end, start), end))
        intervals = next_intervals
    return intervals


def area_coverage(area: MissionArea, zones: list[dict[str, Any]]) -> float:
    area_size = max(1.0, (float(area.x_max) - float(area.x_min)) * (float(area.y_max) - float(area.y_min)))
    blocked = 0.0
    for zone in zones:
        blocked += max(0.0, float(zone["x_max"]) - float(zone["x_min"])) * max(0.0, float(zone["y_max"]) - float(zone["y_min"]))
    return min(1.0, blocked / area_size)


def assessment_risk(coverage: float, route_hits: int) -> str:
    if route_hits >= 3 or coverage >= 0.28:
        return "critical"
    if route_hits >= 1 or coverage >= 0.12:
        return "high"
    if coverage >= 0.04:
        return "medium"
    return "low"


def assessment_recommendation(risk: str, route_hits: int, coverage: float) -> str:
    if risk == "critical":
        return "Area contains dense obstacles or the planned scan route intersects blocked cells; split or shrink the mission area."
    if risk == "high" and route_hits > 0:
        return "Planned scan route crosses suspected obstacles; increase scan_margin_m or split the area."
    if risk == "high":
        return "Area has significant obstacle coverage; proceed only with planned avoidance enabled."
    if risk == "medium":
        return "Area has scattered obstacles; planned avoidance is recommended."
    return "Area appears flyable from pre-scan samples."


def inset_area(area: MissionArea, margin_m: float) -> MissionArea:
    margin = max(0.0, float(margin_m))
    width = float(area.x_max) - float(area.x_min)
    height = float(area.y_max) - float(area.y_min)
    max_margin = max(0.0, min(width, height) / 2.0 - 1.0)
    margin = min(margin, max_margin)
    if margin <= 0.0:
        return area
    return MissionArea(
        x_min=float(area.x_min) + margin,
        x_max=float(area.x_max) - margin,
        y_min=float(area.y_min) + margin,
        y_max=float(area.y_max) - margin,
    )
