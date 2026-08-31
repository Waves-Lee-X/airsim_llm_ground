from __future__ import annotations

import json
import threading
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from aeromind_apm_lite.ground.parallel_domain.observer import (
    ParallelDomainSnapshotReader,
    create_observer_server,
)


def _write_line(state_directory: Path, line_id: str, vehicle_id: int) -> None:
    status_path = state_directory / f"twin/{line_id}/real-status.json"
    replay_path = state_directory / f"replay/{line_id}/observed.jsonl"
    status_path.parent.mkdir(parents=True, exist_ok=True)
    replay_path.parent.mkdir(parents=True, exist_ok=True)
    status_path.write_text(
        json.dumps(
            {
                "line": line_id,
                "vehicle_id": vehicle_id,
                "state": "fresh",
                "reason": "telemetry_fresh",
                "mirror_active": True,
                "observed_sequence": 5,
            }
        ),
        encoding="utf-8",
    )
    records = []
    for sequence in range(1, 6):
        records.append(
            json.dumps(
                {
                    "line": line_id,
                    "vehicle_id": vehicle_id,
                    "source": "observed",
                    "frame": "map",
                    "sequence": sequence,
                    "sim_time": float(sequence),
                    "wall_time": f"2026-08-16T00:00:0{sequence}Z",
                    "position": {"x": sequence, "y": -sequence, "z": 2.0},
                    "frame_calibration_id": "observer-test-v1",
                    "frame_calibration_sha256": "a" * 64,
                    "calibration_status": "surveyed",
                    "preview_only": False,
                }
            )
        )
    replay_path.write_text("\n".join(records) + "\n{partial", encoding="utf-8")


def test_observer_snapshot_is_bounded_map_observed_and_read_only(tmp_path: Path):
    _write_line(tmp_path, "px4", 1)
    _write_line(tmp_path, "apm", 2)
    before = sorted((path, path.stat().st_mtime_ns) for path in tmp_path.rglob("*"))

    snapshot = ParallelDomainSnapshotReader(tmp_path, max_points=3).snapshot()

    assert snapshot["read_only"] is True
    assert snapshot["frame"] == "map"
    assert snapshot["axis_convention"] == {
        "x": "east",
        "y": "north",
        "z": "up",
    }
    assert snapshot["calibration_consistent"] is True
    for line_id, vehicle_id in (("px4", 1), ("apm", 2)):
        line = snapshot["lines"][line_id]
        assert line["state"] == "fresh"
        assert line["vehicle_id"] == vehicle_id
        assert [point["sequence"] for point in line["points"]] == [3, 4, 5]
        assert line["latest"]["x"] == 5
    after = sorted((path, path.stat().st_mtime_ns) for path in tmp_path.rglob("*"))
    assert after == before


def test_observer_http_surface_has_no_write_method(tmp_path: Path):
    _write_line(tmp_path, "px4", 1)
    _write_line(tmp_path, "apm", 2)
    server = create_observer_server(tmp_path, port=0, max_points=2)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urlopen(f"{base}/api/observed", timeout=2.0) as response:
            payload = json.loads(response.read())
            assert response.headers["Cache-Control"] == "no-store"
            assert payload["lines"]["px4"]["latest"]["sequence"] == 5
        with urlopen(f"{base}/", timeout=2.0) as response:
            assert "Observed Monitor" in response.read().decode("utf-8")
        try:
            urlopen(Request(f"{base}/api/observed", method="POST"), timeout=2.0)
        except HTTPError as exc:
            assert exc.code == 405
        else:
            raise AssertionError("read-only observer unexpectedly accepted POST")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
