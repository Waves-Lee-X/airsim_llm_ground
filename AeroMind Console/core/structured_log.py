"""Structured JSON logging for AeroMind.

Usage:
    from core.structured_log import log_event

    log_event("mission_start", tool="search_area", altitude_m=8.0, waypoints=12)
    log_event("waypoint_reached", index=3, distance_m=0.5)
"""

from __future__ import annotations

import json
import logging
import time
from typing import Any

logger = logging.getLogger("aeromind.events")


def log_event(event: str, **fields: Any) -> None:
    """Emit a structured event log line as JSON."""
    record = {
        "ts": time.time(),
        "event": event,
        **fields,
    }
    try:
        logger.info(json.dumps(record, default=str))
    except Exception:
        logger.info(json.dumps({"ts": time.time(), "event": event, "error": "serialization_failed"}))


def log_mission_event(event: str, progress: dict[str, Any] | None = None, **fields: Any) -> None:
    """Emit a mission-phase event enriched with current progress state."""
    payload = {**fields}
    if progress:
        payload["status"] = progress.get("status")
        payload["tool"] = progress.get("tool")
        payload["current_waypoint"] = progress.get("current_waypoint_index")
        payload["total_waypoints"] = progress.get("total_waypoints")
        payload["distance_to_waypoint_m"] = progress.get("distance_to_waypoint_m")
    log_event("mission." + event, **payload)
