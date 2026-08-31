"""Read-only HTTP observer for persisted parallel-domain live state."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit


LINE_IDS = ("px4", "apm")
STATIC_DIRECTORY = Path(__file__).with_name("observer_static")


class ParallelDomainSnapshotReader:
    def __init__(self, state_directory: str | Path, *, max_points: int = 1_000) -> None:
        if not 2 <= max_points <= 20_000:
            raise ValueError("max_points must be in [2, 20000]")
        self.state_directory = Path(state_directory).resolve()
        self.max_points = max_points

    def snapshot(self) -> dict[str, Any]:
        lines: dict[str, dict[str, Any]] = {}
        calibration_hashes: set[str] = set()
        for line_id in LINE_IDS:
            status = _read_json(
                self.state_directory / f"twin/{line_id}/real-status.json"
            )
            records = _tail_jsonl(
                self.state_directory / f"replay/{line_id}/observed.jsonl",
                self.max_points,
            )
            records = [
                item
                for item in records
                if item.get("line") == line_id
                and item.get("source") == "observed"
                and item.get("frame") == "map"
                and isinstance(item.get("position"), dict)
            ]
            points = [
                {
                    "sequence": item.get("sequence"),
                    "x": item["position"].get("x"),
                    "y": item["position"].get("y"),
                    "z": item["position"].get("z"),
                    "wall_time": item.get("wall_time"),
                    "sim_time": item.get("sim_time"),
                }
                for item in records
                if all(
                    isinstance(item["position"].get(axis), (int, float))
                    for axis in ("x", "y", "z")
                )
            ]
            latest = records[-1] if records else None
            calibration_hash = (
                str(latest.get("frame_calibration_sha256")) if latest else None
            )
            if calibration_hash:
                calibration_hashes.add(calibration_hash)
            lines[line_id] = {
                "line": line_id,
                "vehicle_id": (
                    latest.get("vehicle_id")
                    if latest is not None
                    else status.get("vehicle_id")
                ),
                "state": status.get("state", "waiting"),
                "reason": status.get("reason", "waiting_for_telemetry"),
                "mirror_active": status.get("mirror_active", False),
                "observed_sequence": status.get("observed_sequence", 0),
                "last_fresh_wall_time": status.get("last_fresh_wall_time"),
                "calibration_id": (
                    latest.get("frame_calibration_id") if latest else None
                ),
                "calibration_sha256": calibration_hash,
                "calibration_status": (
                    latest.get("calibration_status") if latest else None
                ),
                "preview_only": latest.get("preview_only") if latest else None,
                "latest": points[-1] if points else None,
                "points": points,
            }
        return {
            "schema_version": "1.0",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "read_only": True,
            "frame": "map",
            "axis_convention": {"x": "east", "y": "north", "z": "up"},
            "calibration_consistent": len(calibration_hashes) <= 1,
            "lines": lines,
        }


class ObserverHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        reader: ParallelDomainSnapshotReader,
    ) -> None:
        super().__init__(server_address, ObserverRequestHandler)
        self.reader = reader


class ObserverRequestHandler(BaseHTTPRequestHandler):
    server: ObserverHTTPServer

    def do_GET(self) -> None:  # noqa: N802 - stdlib HTTP API
        path = urlsplit(self.path).path
        if path == "/api/observed":
            self._send_json(self.server.reader.snapshot())
            return
        asset = {
            "/": ("index.html", "text/html; charset=utf-8"),
            "/app.js": ("app.js", "text/javascript; charset=utf-8"),
            "/styles.css": ("styles.css", "text/css; charset=utf-8"),
        }.get(path)
        if asset is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            payload = (STATIC_DIRECTORY / asset[0]).read_bytes()
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", asset[1])
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_POST(self) -> None:  # noqa: N802 - stdlib HTTP API
        self.send_error(HTTPStatus.METHOD_NOT_ALLOWED, "observer is read-only")

    def _send_json(self, value: dict[str, Any]) -> None:
        payload = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, format: str, *args: object) -> None:
        return


def create_observer_server(
    state_directory: str | Path,
    *,
    host: str = "127.0.0.1",
    port: int = 8091,
    max_points: int = 1_000,
) -> ObserverHTTPServer:
    if not 0 <= port <= 65_535:
        raise ValueError("port must be in [0, 65535]")
    return ObserverHTTPServer(
        (host, port),
        ParallelDomainSnapshotReader(state_directory, max_points=max_points),
    )


def serve_observer(
    state_directory: str | Path,
    *,
    host: str = "127.0.0.1",
    port: int = 8091,
    max_points: int = 1_000,
) -> int:
    server = create_observer_server(
        state_directory,
        host=host,
        port=port,
        max_points=max_points,
    )
    print(f"parallel-domain observer: http://{host}:{server.server_port}")
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        return 130
    finally:
        server.server_close()
    return 0


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _tail_jsonl(path: Path, limit: int) -> list[dict[str, Any]]:
    scan_limit = limit + 64
    try:
        with path.open("rb") as stream:
            stream.seek(0, 2)
            position = stream.tell()
            payload = b""
            while position > 0 and payload.count(b"\n") <= scan_limit:
                size = min(65_536, position)
                position -= size
                stream.seek(position)
                payload = stream.read(size) + payload
    except OSError:
        return []
    result: list[dict[str, Any]] = []
    for raw in payload.splitlines()[-scan_limit:]:
        try:
            item = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if isinstance(item, dict):
            result.append(item)
    return result[-limit:]


__all__ = [
    "ParallelDomainSnapshotReader",
    "create_observer_server",
    "serve_observer",
]
