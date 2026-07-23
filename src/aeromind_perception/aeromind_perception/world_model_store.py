"""SQLite persistence for timestamped semantic object observations."""

from __future__ import annotations

import json
import os
import sqlite3
import threading
import time

from .json_support import json_compatible


class WorldModelStore:
    def __init__(self, path: str, retention_hours: float = 24.0, max_rows: int = 200000):
        self.path = os.path.abspath(os.path.expanduser(path))
        self.retention_hours = max(0.1, float(retention_hours))
        self.max_rows = max(1000, int(max_rows))
        self._lock = threading.RLock()
        self._writes_since_cleanup = 0
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self._connection = sqlite3.connect(
            self.path, timeout=3.0, check_same_thread=False
        )
        self._connection.row_factory = sqlite3.Row
        self._connection.execute("PRAGMA journal_mode=WAL")
        self._connection.execute("PRAGMA synchronous=NORMAL")
        self._create_schema()

    def _create_schema(self):
        with self._lock, self._connection:
            self._connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS object_observations (
                    sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                    track_id TEXT NOT NULL,
                    class_name TEXT NOT NULL,
                    frame_id TEXT NOT NULL,
                    observed_at REAL NOT NULL,
                    confidence REAL NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    z REAL NOT NULL,
                    vx REAL NOT NULL,
                    vy REAL NOT NULL,
                    vz REAL NOT NULL,
                    dynamic INTEGER NOT NULL,
                    lifecycle TEXT NOT NULL,
                    hit_count INTEGER NOT NULL,
                    miss_count INTEGER NOT NULL,
                    covariance_json TEXT NOT NULL,
                    sources_json TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS idx_world_track_time
                    ON object_observations(track_id, observed_at DESC);
                CREATE INDEX IF NOT EXISTS idx_world_class_time
                    ON object_observations(class_name, observed_at DESC);
                CREATE TABLE IF NOT EXISTS world_model_metadata (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL
                );
                """
            )

    def allocate_track_id(self, class_name: str):
        with self._lock, self._connection:
            row = self._connection.execute(
                "SELECT value FROM world_model_metadata WHERE key = 'track_sequence'"
            ).fetchone()
            sequence = int(row["value"]) + 1 if row is not None else 1
            self._connection.execute(
                """
                INSERT INTO world_model_metadata(key, value) VALUES('track_sequence', ?)
                ON CONFLICT(key) DO UPDATE SET value = excluded.value
                """,
                (str(sequence),),
            )
        safe_class = "".join(
            character.lower() if character.isalnum() else "_"
            for character in str(class_name)
        ).strip("_") or "object"
        return f"{safe_class}_{sequence:06d}"

    def record(self, frame_id: str, observed_at: float, records):
        rows = []
        for record in records:
            if not record.get("position_valid", True):
                continue
            rows.append(
                (
                    str(record["id"]),
                    str(record["class_name"]),
                    str(frame_id),
                    float(observed_at),
                    float(record.get("confidence", 0.0)),
                    float(record["position"][0]),
                    float(record["position"][1]),
                    float(record["position"][2]),
                    float(record["velocity"][0]),
                    float(record["velocity"][1]),
                    float(record["velocity"][2]),
                    int(bool(record.get("dynamic"))),
                    str(record.get("state", "unknown")),
                    int(record.get("hit_count", 0)),
                    int(record.get("miss_count", 0)),
                    json.dumps(
                        json_compatible(
                            record.get("position_covariance")
                            if record.get("position_covariance") is not None
                            else []
                        ),
                        allow_nan=False,
                    ),
                    json.dumps(
                        json_compatible(
                            record.get("sources")
                            if record.get("sources") is not None
                            else []
                        ),
                        ensure_ascii=False,
                        allow_nan=False,
                    ),
                )
            )
        if not rows:
            return 0
        with self._lock, self._connection:
            self._connection.executemany(
                """
                INSERT INTO object_observations (
                    track_id, class_name, frame_id, observed_at, confidence,
                    x, y, z, vx, vy, vz, dynamic, lifecycle,
                    hit_count, miss_count, covariance_json, sources_json
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                rows,
            )
            self._writes_since_cleanup += len(rows)
            if self._writes_since_cleanup >= 1000:
                self._cleanup_locked(float(observed_at))
                self._writes_since_cleanup = 0
        return len(rows)

    def query_history(
        self,
        object_id: str = "",
        class_name: str = "",
        fresh_within_sec: float = 0.0,
        limit: int = 100,
        now: float | None = None,
    ):
        clauses = []
        parameters = []
        if object_id:
            clauses.append("track_id = ?")
            parameters.append(str(object_id))
        if class_name:
            clauses.append("LOWER(class_name) = LOWER(?)")
            parameters.append(str(class_name))
        if fresh_within_sec > 0.0:
            clauses.append("observed_at >= ?")
            parameters.append(float(now if now is not None else time.time()) - float(fresh_within_sec))
        where = " WHERE " + " AND ".join(clauses) if clauses else ""
        parameters.append(max(1, min(int(limit), 1000)))
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT sequence, track_id, class_name, frame_id, observed_at,
                       confidence, x, y, z, vx, vy, vz, dynamic, lifecycle,
                       hit_count, miss_count, covariance_json, sources_json
                  FROM object_observations
                """ + where + " ORDER BY observed_at DESC, sequence DESC LIMIT ?",
                parameters,
            ).fetchall()
        return [_decode_row(row) for row in rows]

    def count(self):
        with self._lock:
            row = self._connection.execute(
                "SELECT COUNT(*) AS count FROM object_observations"
            ).fetchone()
        return int(row["count"])

    def cleanup(self, now: float | None = None):
        with self._lock, self._connection:
            return self._cleanup_locked(time.time() if now is None else float(now))

    def _cleanup_locked(self, now: float):
        cutoff = float(now) - self.retention_hours * 3600.0
        removed = self._connection.execute(
            "DELETE FROM object_observations WHERE observed_at < ?", (cutoff,)
        ).rowcount
        overflow = self._connection.execute(
            "SELECT MAX(0, COUNT(*) - ?) AS overflow FROM object_observations",
            (self.max_rows,),
        ).fetchone()["overflow"]
        if overflow > 0:
            removed += self._connection.execute(
                """
                DELETE FROM object_observations
                 WHERE sequence IN (
                    SELECT sequence FROM object_observations
                    ORDER BY observed_at ASC, sequence ASC LIMIT ?
                 )
                """,
                (int(overflow),),
            ).rowcount
        return int(removed)

    def close(self):
        with self._lock:
            self._connection.close()


def _decode_row(row):
    return {
        "sequence": int(row["sequence"]),
        "id": row["track_id"],
        "class_name": row["class_name"],
        "frame_id": row["frame_id"],
        "observed_at": float(row["observed_at"]),
        "confidence": float(row["confidence"]),
        "position_m": {
            "x": float(row["x"]),
            "y": float(row["y"]),
            "z": float(row["z"]),
        },
        "velocity_mps": {
            "x": float(row["vx"]),
            "y": float(row["vy"]),
            "z": float(row["vz"]),
        },
        "dynamic": bool(row["dynamic"]),
        "state": row["lifecycle"],
        "hit_count": int(row["hit_count"]),
        "miss_count": int(row["miss_count"]),
        "position_covariance": json.loads(row["covariance_json"]),
        "sources": json.loads(row["sources_json"]),
    }
