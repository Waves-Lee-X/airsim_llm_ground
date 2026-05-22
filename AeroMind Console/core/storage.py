from __future__ import annotations

import sqlite3
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

from core.task_schema import MissionPlan


@dataclass(frozen=True)
class TaskRecord:
    id: int
    task_type: str
    task_text: str
    status: str
    created_at: datetime
    started_at: Optional[datetime]
    completed_at: Optional[datetime]
    plan_json: str
    result_json: str


@dataclass(frozen=True)
class FlightLogRecord:
    id: int
    task_id: int
    timestamp: datetime
    x: float
    y: float
    z: float
    altitude_m: float
    speed_mps: float
    status: str


class TaskStorage:
    def __init__(self, db_path: Path | str) -> None:
        self._db_path = Path(db_path)
        self._db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _init_db(self) -> None:
        with sqlite3.connect(self._db_path) as conn:
            cursor = conn.cursor()
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS tasks (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task_type TEXT NOT NULL,
                    task_text TEXT NOT NULL,
                    status TEXT NOT NULL DEFAULT 'pending',
                    created_at TEXT NOT NULL,
                    started_at TEXT,
                    completed_at TEXT,
                    plan_json TEXT NOT NULL,
                    result_json TEXT DEFAULT '{}'
                )
            """)
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS flight_logs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task_id INTEGER NOT NULL,
                    timestamp TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    z REAL NOT NULL,
                    altitude_m REAL NOT NULL,
                    speed_mps REAL NOT NULL,
                    status TEXT NOT NULL,
                    FOREIGN KEY (task_id) REFERENCES tasks(id)
                )
            """)
            cursor.execute("""
                CREATE INDEX IF NOT EXISTS idx_flight_logs_task_id ON flight_logs(task_id)
            """)
            cursor.execute("""
                CREATE INDEX IF NOT EXISTS idx_tasks_status ON tasks(status)
            """)
            conn.commit()

    def create_task(self, task_type: str, task_text: str, plan_json: str) -> int:
        now = datetime.now().isoformat()
        with sqlite3.connect(self._db_path) as conn:
            cursor = conn.cursor()
            cursor.execute(
                """INSERT INTO tasks (task_type, task_text, status, created_at, plan_json)
                   VALUES (?, ?, ?, ?, ?)""",
                (task_type, task_text, "pending", now, plan_json)
            )
            conn.commit()
            return cursor.lastrowid

    def update_task_status(self, task_id: int, status: str) -> None:
        now = datetime.now().isoformat()
        with sqlite3.connect(self._db_path) as conn:
            cursor = conn.cursor()
            if status == "running":
                cursor.execute(
                    "UPDATE tasks SET status = ?, started_at = ? WHERE id = ?",
                    (status, now, task_id)
                )
            elif status in ("completed", "failed", "cancelled"):
                cursor.execute(
                    "UPDATE tasks SET status = ?, completed_at = ? WHERE id = ?",
                    (status, now, task_id)
                )
            else:
                cursor.execute(
                    "UPDATE tasks SET status = ? WHERE id = ?",
                    (status, task_id)
                )
            conn.commit()

    def update_task_result(self, task_id: int, result_json: str) -> None:
        with sqlite3.connect(self._db_path) as conn:
            cursor = conn.cursor()
            cursor.execute(
                "UPDATE tasks SET result_json = ? WHERE id = ?",
                (result_json, task_id)
            )
            conn.commit()

    def get_task(self, task_id: int) -> Optional[TaskRecord]:
        with sqlite3.connect(self._db_path) as conn:
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM tasks WHERE id = ?", (task_id,))
            row = cursor.fetchone()
            if row:
                return TaskRecord(
                    id=row["id"],
                    task_type=row["task_type"],
                    task_text=row["task_text"],
                    status=row["status"],
                    created_at=datetime.fromisoformat(row["created_at"]),
                    started_at=datetime.fromisoformat(row["started_at"]) if row["started_at"] else None,
                    completed_at=datetime.fromisoformat(row["completed_at"]) if row["completed_at"] else None,
                    plan_json=row["plan_json"],
                    result_json=row["result_json"],
                )
        return None

    def list_tasks(self, status: str | None = None, limit: int = 50) -> list[TaskRecord]:
        with sqlite3.connect(self._db_path) as conn:
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            if status:
                cursor.execute(
                    "SELECT * FROM tasks WHERE status = ? ORDER BY created_at DESC LIMIT ?",
                    (status, limit)
                )
            else:
                cursor.execute("SELECT * FROM tasks ORDER BY created_at DESC LIMIT ?", (limit,))
            rows = cursor.fetchall()
            return [
                TaskRecord(
                    id=row["id"],
                    task_type=row["task_type"],
                    task_text=row["task_text"],
                    status=row["status"],
                    created_at=datetime.fromisoformat(row["created_at"]),
                    started_at=datetime.fromisoformat(row["started_at"]) if row["started_at"] else None,
                    completed_at=datetime.fromisoformat(row["completed_at"]) if row["completed_at"] else None,
                    plan_json=row["plan_json"],
                    result_json=row["result_json"],
                )
                for row in rows
            ]

    def add_flight_log(self, task_id: int, x: float, y: float, z: float, altitude_m: float, speed_mps: float, status: str) -> None:
        now = datetime.now().isoformat()
        with sqlite3.connect(self._db_path) as conn:
            cursor = conn.cursor()
            cursor.execute(
                """INSERT INTO flight_logs (task_id, timestamp, x, y, z, altitude_m, speed_mps, status)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
                (task_id, now, x, y, z, altitude_m, speed_mps, status)
            )
            conn.commit()

    def get_flight_logs(self, task_id: int) -> list[FlightLogRecord]:
        with sqlite3.connect(self._db_path) as conn:
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            cursor.execute(
                "SELECT * FROM flight_logs WHERE task_id = ? ORDER BY timestamp ASC",
                (task_id,)
            )
            rows = cursor.fetchall()
            return [
                FlightLogRecord(
                    id=row["id"],
                    task_id=row["task_id"],
                    timestamp=datetime.fromisoformat(row["timestamp"]),
                    x=row["x"],
                    y=row["y"],
                    z=row["z"],
                    altitude_m=row["altitude_m"],
                    speed_mps=row["speed_mps"],
                    status=row["status"],
                )
                for row in rows
            ]

    def delete_task(self, task_id: int) -> bool:
        with sqlite3.connect(self._db_path) as conn:
            cursor = conn.cursor()
            cursor.execute("DELETE FROM flight_logs WHERE task_id = ?", (task_id,))
            cursor.execute("DELETE FROM tasks WHERE id = ?", (task_id,))
            conn.commit()
            return cursor.rowcount > 0

    def get_stats(self) -> dict[str, int]:
        with sqlite3.connect(self._db_path) as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT COUNT(*) FROM tasks")
            total = cursor.fetchone()[0]
            cursor.execute("SELECT COUNT(*) FROM tasks WHERE status = 'completed'")
            completed = cursor.fetchone()[0]
            cursor.execute("SELECT COUNT(*) FROM tasks WHERE status = 'failed'")
            failed = cursor.fetchone()[0]
            cursor.execute("SELECT COUNT(*) FROM tasks WHERE status = 'running'")
            running = cursor.fetchone()[0]
            return {
                "total": total,
                "completed": completed,
                "failed": failed,
                "running": running,
            }