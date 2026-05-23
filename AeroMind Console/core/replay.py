from __future__ import annotations

import json
import logging
import math
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


def load_replay_frames(log_folder: Path) -> list[dict[str, Any]]:
    files = sorted(
        log_folder.glob("*.json"),
        key=lambda item: int(item.stem) if item.stem.isdigit() else item.stem,
    )
    frames: list[dict[str, Any]] = []
    for file in files:
        try:
            payload = json.loads(file.read_text(encoding="utf-8"))
            if isinstance(payload, dict):
                frames.append(payload)
        except Exception as load_err:
            logger.warning("Failed to load replay frame %s: %s", file, load_err)
            continue
    return frames


def deduplicate_replay_frames(frames: list[dict[str, Any]], min_dist_m: float = 0.5) -> list[dict[str, Any]]:
    if not frames:
        return []
    kept = [frames[0]]
    min_dist = max(0.05, float(min_dist_m))
    for item in frames[1:]:
        try:
            p0 = kept[-1]["sensors"]["state"]["position"]
            p1 = item["sensors"]["state"]["position"]
            dx = float(p1[0]) - float(p0[0])
            dy = float(p1[1]) - float(p0[1])
            dz = float(p1[2]) - float(p0[2])
            if math.sqrt(dx * dx + dy * dy + dz * dz) >= min_dist:
                kept.append(item)
        except Exception as dedup_err:
            logger.warning("Failed to deduplicate replay frame: %s", dedup_err)
            continue
    return [
        {
            "position": [float(v) for v in frame["sensors"]["state"]["position"]],
            "orientation": [float(v) for v in frame["sensors"]["state"]["orientation"]],
        }
        for frame in kept
        if isinstance(frame, dict) and "sensors" in frame and "state" in frame["sensors"]
    ]


def turn_angle_deg(waypoints: list[dict[str, Any]], i: int) -> float:
    p0 = waypoints[i - 1]["position"]
    p1 = waypoints[i]["position"]
    p2 = waypoints[i + 1]["position"]
    a1 = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    a2 = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    diff = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
    return abs(math.degrees(diff))


def split_replay_by_turns(waypoints: list[dict[str, Any]], threshold_deg: float) -> list[int]:
    if len(waypoints) < 3:
        return [0, len(waypoints) - 1]
    indexes = [0]
    for idx in range(1, len(waypoints) - 1):
        if turn_angle_deg(waypoints, idx) > float(threshold_deg):
            indexes.append(idx)
    indexes.append(len(waypoints) - 1)
    return indexes


def closest_spawn_area(coord: list[float], areas: Any) -> list[Any] | None:
    if not isinstance(areas, list):
        return None
    cx, cy, cz = float(coord[0]), float(coord[1]), float(coord[2])
    best = None
    best_dist = float("inf")
    for item in areas:
        if not isinstance(item, list) or len(item) < 18:
            continue
        tx = float(item[0]) + 1.0
        ty = float(item[1]) + 1.0
        tz = float(item[2]) + 0.5
        dist_sq = (cx - tx) ** 2 + (cy - ty) ** 2 + (cz - tz) ** 2
        if dist_sq < best_dist:
            best_dist = dist_sq
            best = item
    return best
