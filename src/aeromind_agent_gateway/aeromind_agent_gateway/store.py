"""SQLite-backed sessions, messages, and replayable events."""

from __future__ import annotations

import json
import hashlib
import os
import re
import sqlite3
import threading
import time
import uuid
from typing import Any


MISSION_TERMINAL_STATUSES = {"completed", "failed", "cancelled", "expired"}
MISSION_ALLOWED_TRANSITIONS = {
    "pending_confirmation": {"pending_confirmation", "executing", "cancelled", "expired", "failed"},
    "executing": {"executing", "completed", "failed", "cancelled"},
}


def _search_terms(value: str) -> set[str]:
    text = str(value).casefold()
    words = set(re.findall(r"[a-z0-9_]+", text))
    chinese = "".join(re.findall(r"[\u4e00-\u9fff]", text))
    words.update(chinese[index : index + 2] for index in range(len(chinese) - 1))
    if len(chinese) == 1:
        words.add(chinese)
    return words


class SessionStore:
    def __init__(self, path: str):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._lock = threading.RLock()
        self._db = sqlite3.connect(path, check_same_thread=False)
        self._db.row_factory = sqlite3.Row
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA foreign_keys=ON")
        self._create_schema()

    def _create_schema(self):
        with self._lock, self._db:
            self._db.executescript(
                """
                CREATE TABLE IF NOT EXISTS sessions (
                    id TEXT PRIMARY KEY,
                    user_id TEXT NOT NULL,
                    channel TEXT NOT NULL,
                    title TEXT NOT NULL DEFAULT '',
                    provider TEXT NOT NULL DEFAULT 'claude',
                    model TEXT NOT NULL,
                    runtime_session_id TEXT,
                    status TEXT NOT NULL DEFAULT 'active',
                    created_at REAL NOT NULL,
                    updated_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS messages (
                    id TEXT PRIMARY KEY,
                    session_id TEXT NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
                    role TEXT NOT NULL,
                    content_json TEXT NOT NULL,
                    provider TEXT,
                    model TEXT,
                    created_at REAL NOT NULL
                );
                CREATE INDEX IF NOT EXISTS idx_messages_session_time
                    ON messages(session_id, created_at);
                CREATE TABLE IF NOT EXISTS events (
                    id TEXT PRIMARY KEY,
                    session_id TEXT NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
                    sequence INTEGER NOT NULL,
                    type TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    created_at REAL NOT NULL,
                    UNIQUE(session_id, sequence)
                );
                CREATE INDEX IF NOT EXISTS idx_events_session_sequence
                    ON events(session_id, sequence);
                CREATE TABLE IF NOT EXISTS confirmations (
                    id TEXT PRIMARY KEY,
                    session_id TEXT NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
                    user_id TEXT NOT NULL,
                    action TEXT NOT NULL,
                    args_json TEXT NOT NULL,
                    summary TEXT NOT NULL,
                    risk_level TEXT NOT NULL,
                    status TEXT NOT NULL,
                    expires_at REAL NOT NULL,
                    result_json TEXT,
                    created_at REAL NOT NULL,
                    resolved_at REAL
                );
                CREATE INDEX IF NOT EXISTS idx_confirmations_session_status
                    ON confirmations(session_id, status, created_at);
                CREATE TABLE IF NOT EXISTS inbound_messages (
                    message_id TEXT PRIMARY KEY,
                    channel TEXT NOT NULL,
                    sender_id TEXT NOT NULL,
                    received_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS gateway_missions (
                    id TEXT PRIMARY KEY,
                    confirmation_id TEXT NOT NULL UNIQUE
                        REFERENCES confirmations(id) ON DELETE CASCADE,
                    session_id TEXT NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
                    user_id TEXT NOT NULL,
                    source_channel TEXT NOT NULL,
                    title TEXT NOT NULL,
                    action TEXT NOT NULL,
                    args_json TEXT NOT NULL,
                    status TEXT NOT NULL,
                    phase TEXT NOT NULL DEFAULT 'pending_confirmation',
                    ros_mission_id TEXT,
                    physical_complete INTEGER NOT NULL DEFAULT 0,
                    result_json TEXT,
                    revision INTEGER NOT NULL DEFAULT 1,
                    created_at REAL NOT NULL,
                    updated_at REAL NOT NULL
                );
                CREATE INDEX IF NOT EXISTS idx_gateway_missions_user_time
                    ON gateway_missions(user_id, updated_at);
                CREATE TABLE IF NOT EXISTS operator_memories (
                    user_id TEXT PRIMARY KEY,
                    summary TEXT NOT NULL,
                    source_session_id TEXT,
                    message_count INTEGER NOT NULL DEFAULT 0,
                    semantic_summary TEXT NOT NULL DEFAULT '',
                    semantic_model TEXT,
                    semantic_message_count INTEGER NOT NULL DEFAULT 0,
                    updated_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS semantic_memories (
                    id TEXT PRIMARY KEY,
                    user_id TEXT NOT NULL,
                    kind TEXT NOT NULL,
                    content TEXT NOT NULL,
                    importance REAL NOT NULL DEFAULT 0.5,
                    source_session_id TEXT,
                    fingerprint TEXT NOT NULL,
                    metadata_json TEXT NOT NULL DEFAULT '{}',
                    status TEXT NOT NULL DEFAULT 'active',
                    created_at REAL NOT NULL,
                    updated_at REAL NOT NULL,
                    UNIQUE(user_id, fingerprint)
                );
                CREATE INDEX IF NOT EXISTS idx_semantic_memories_user_time
                    ON semantic_memories(user_id, status, updated_at);
                """
            )
            self._ensure_column(
                "gateway_missions", "phase", "TEXT NOT NULL DEFAULT 'pending_confirmation'"
            )
            self._ensure_column("gateway_missions", "ros_mission_id", "TEXT")
            self._ensure_column(
                "gateway_missions", "physical_complete", "INTEGER NOT NULL DEFAULT 0"
            )
            self._ensure_column(
                "gateway_missions", "revision", "INTEGER NOT NULL DEFAULT 1"
            )
            self._ensure_column(
                "operator_memories", "semantic_summary", "TEXT NOT NULL DEFAULT ''"
            )
            self._ensure_column("operator_memories", "semantic_model", "TEXT")
            self._ensure_column(
                "operator_memories",
                "semantic_message_count",
                "INTEGER NOT NULL DEFAULT 0",
            )
            self._ensure_column("messages", "provider", "TEXT")

    def _ensure_column(self, table: str, column: str, declaration: str):
        columns = {
            row["name"]
            for row in self._db.execute(f"PRAGMA table_info({table})").fetchall()
        }
        if column not in columns:
            self._db.execute(
                f"ALTER TABLE {table} ADD COLUMN {column} {declaration}"
            )

    def close(self):
        with self._lock:
            self._db.close()

    def ensure_session(
        self,
        session_id: str,
        user_id: str,
        channel: str,
        model: str,
        provider: str = "claude",
    ) -> dict[str, Any]:
        now = time.time()
        with self._lock, self._db:
            self._db.execute(
                """
                INSERT INTO sessions(
                    id, user_id, channel, provider, model, created_at, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(id) DO UPDATE SET
                    updated_at=excluded.updated_at
                """,
                (session_id, user_id, channel, provider, model, now, now),
            )
        return self.get_session(session_id)

    def get_session(self, session_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._db.execute(
                "SELECT * FROM sessions WHERE id=?", (session_id,)
            ).fetchone()
        return dict(row) if row else None

    def update_runtime_session(self, session_id: str, runtime_session_id: str):
        with self._lock, self._db:
            self._db.execute(
                """
                UPDATE sessions
                SET runtime_session_id=?, updated_at=?
                WHERE id=?
                """,
                (runtime_session_id, time.time(), session_id),
            )

    def clear_session_history(self, session_id: str) -> dict[str, int]:
        """Clear conversational state without deleting missions or confirmations."""
        now = time.time()
        with self._lock, self._db:
            cursor = self._db.execute(
                "DELETE FROM messages WHERE session_id=?", (session_id,)
            )
            self._db.execute(
                """
                UPDATE sessions
                SET runtime_session_id=NULL, updated_at=?
                WHERE id=?
                """,
                (now, session_id),
            )
        return {"deleted_messages": max(0, int(cursor.rowcount))}

    def update_model(self, session_id: str, model: str):
        with self._lock, self._db:
            self._db.execute(
                "UPDATE sessions SET model=?, updated_at=? WHERE id=?",
                (model, time.time(), session_id),
            )

    def update_provider_model(self, session_id: str, provider: str, model: str):
        with self._lock, self._db:
            self._db.execute(
                """
                UPDATE sessions
                SET provider=?, model=?, runtime_session_id=NULL, updated_at=?
                WHERE id=?
                """,
                (provider, model, time.time(), session_id),
            )

    def rebind_user(self, alias: str, principal: str):
        if not alias or alias == principal:
            return
        with self._lock, self._db:
            self._db.execute(
                "UPDATE sessions SET user_id=? WHERE user_id=?", (principal, alias)
            )
            self._db.execute(
                "UPDATE confirmations SET user_id=? WHERE user_id=?",
                (principal, alias),
            )
            self._db.execute(
                "UPDATE gateway_missions SET user_id=? WHERE user_id=?",
                (principal, alias),
            )
            alias_memory = self._db.execute(
                "SELECT * FROM operator_memories WHERE user_id=?", (alias,)
            ).fetchone()
            if alias_memory is not None:
                principal_memory = self._db.execute(
                    "SELECT 1 FROM operator_memories WHERE user_id=?", (principal,)
                ).fetchone()
                if principal_memory is None:
                    self._db.execute(
                        "UPDATE operator_memories SET user_id=? WHERE user_id=?",
                        (principal, alias),
                    )
                else:
                    self._db.execute(
                        "DELETE FROM operator_memories WHERE user_id=?", (alias,)
                    )
            self._db.execute(
                """
                INSERT INTO semantic_memories(
                    id, user_id, kind, content, importance, source_session_id,
                    fingerprint, metadata_json, status, created_at, updated_at
                )
                SELECT id || '-rebound', ?, kind, content, importance,
                    source_session_id, fingerprint, metadata_json, status,
                    created_at, updated_at
                FROM semantic_memories WHERE user_id=?
                ON CONFLICT(user_id, fingerprint) DO UPDATE SET
                    importance=MAX(importance, excluded.importance),
                    updated_at=MAX(updated_at, excluded.updated_at)
                """,
                (principal, alias),
            )
            self._db.execute(
                "DELETE FROM semantic_memories WHERE user_id=?", (alias,)
            )

    def claim_inbound_message(
        self, message_id: str, channel: str, sender_id: str
    ) -> bool:
        """Atomically claim an external message so retries execute only once."""
        if not message_id:
            return False
        with self._lock, self._db:
            cursor = self._db.execute(
                """
                INSERT OR IGNORE INTO inbound_messages(
                    message_id, channel, sender_id, received_at
                ) VALUES (?, ?, ?, ?)
                """,
                (message_id, channel, sender_id, time.time()),
            )
        return cursor.rowcount == 1

    def add_message(
        self,
        session_id: str,
        role: str,
        content: Any,
        model: str | None = None,
        provider: str | None = None,
    ) -> dict[str, Any]:
        item = {
            "id": f"msg-{uuid.uuid4().hex}",
            "session_id": session_id,
            "role": role,
            "content": content,
            "provider": provider,
            "model": model,
            "created_at": time.time(),
        }
        with self._lock, self._db:
            self._db.execute(
                """
                INSERT INTO messages(
                    id, session_id, role, content_json, provider, model, created_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    item["id"],
                    session_id,
                    role,
                    json.dumps(content, ensure_ascii=False),
                    provider,
                    model,
                    item["created_at"],
                ),
            )
            self._db.execute(
                "UPDATE sessions SET updated_at=? WHERE id=?",
                (item["created_at"], session_id),
            )
        return item

    def messages(self, session_id: str, limit: int = 100) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._db.execute(
                """
                SELECT * FROM (
                    SELECT * FROM messages
                    WHERE session_id=?
                    ORDER BY created_at DESC LIMIT ?
                ) ORDER BY created_at ASC
                """,
                (session_id, max(1, min(limit, 500))),
            ).fetchall()
        return [
            {
                **dict(row),
                "content": json.loads(row["content_json"]),
            }
            for row in rows
        ]

    def operator_messages(
        self, user_id: str, limit: int = 20
    ) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._db.execute(
                """
                SELECT * FROM (
                    SELECT messages.*, sessions.channel
                    FROM messages
                    JOIN sessions ON sessions.id=messages.session_id
                    WHERE sessions.user_id=?
                    ORDER BY messages.created_at DESC LIMIT ?
                ) ORDER BY created_at ASC
                """,
                (user_id, max(1, min(limit, 500))),
            ).fetchall()
        return [
            {
                **dict(row),
                "content": json.loads(row["content_json"]),
            }
            for row in rows
        ]

    def operator_timeline(
        self, user_id: str, limit: int = 100
    ) -> list[dict[str, Any]]:
        return [
            {
                "id": item["id"],
                "session_id": item["session_id"],
                "channel": item["channel"],
                "role": item["role"],
                "content": item["content"],
                "provider": item["provider"],
                "model": item["model"],
                "created_at": item["created_at"],
            }
            for item in self.operator_messages(user_id, limit=limit)
        ]

    def operator_message_count(self, user_id: str) -> int:
        with self._lock:
            row = self._db.execute(
                """
                SELECT COUNT(*) AS count
                FROM messages JOIN sessions ON sessions.id=messages.session_id
                WHERE sessions.user_id=?
                """,
                (user_id,),
            ).fetchone()
        return int(row["count"])

    def upsert_operator_memory(
        self,
        user_id: str,
        summary: str,
        source_session_id: str,
        message_count: int,
    ) -> dict[str, Any]:
        now = time.time()
        with self._lock, self._db:
            self._db.execute(
                """
                INSERT INTO operator_memories(
                    user_id, summary, source_session_id, message_count, updated_at
                ) VALUES (?, ?, ?, ?, ?)
                ON CONFLICT(user_id) DO UPDATE SET
                    summary=excluded.summary,
                    source_session_id=excluded.source_session_id,
                    message_count=excluded.message_count,
                    updated_at=excluded.updated_at
                """,
                (
                    user_id,
                    summary,
                    source_session_id,
                    max(0, int(message_count)),
                    now,
                ),
            )
        memory = self.operator_memory(user_id)
        assert memory is not None
        return memory

    def operator_memory(self, user_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._db.execute(
                "SELECT * FROM operator_memories WHERE user_id=?", (user_id,)
            ).fetchone()
        return dict(row) if row else None

    def update_semantic_memory(
        self,
        user_id: str,
        source_session_id: str,
        model: str,
        message_count: int,
        summary: str,
        items: list[dict[str, Any]],
    ) -> dict[str, Any]:
        now = time.time()
        with self._lock, self._db:
            self._db.execute(
                """
                UPDATE operator_memories
                SET semantic_summary=?, semantic_model=?,
                    semantic_message_count=?, updated_at=?
                WHERE user_id=?
                """,
                (summary, model, max(0, int(message_count)), now, user_id),
            )
            for item in items:
                content = " ".join(str(item.get("content", "")).split()).strip()
                if not content:
                    continue
                fingerprint = hashlib.sha256(
                    content.casefold().encode("utf-8")
                ).hexdigest()
                kind = str(item.get("kind", "fact"))[:32] or "fact"
                importance = max(0.0, min(1.0, float(item.get("importance", 0.5))))
                metadata = item.get("metadata")
                if not isinstance(metadata, dict):
                    metadata = {}
                self._db.execute(
                    """
                    INSERT INTO semantic_memories(
                        id, user_id, kind, content, importance, source_session_id,
                        fingerprint, metadata_json, status, created_at, updated_at
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, 'active', ?, ?)
                    ON CONFLICT(user_id, fingerprint) DO UPDATE SET
                        kind=excluded.kind,
                        importance=MAX(importance, excluded.importance),
                        source_session_id=excluded.source_session_id,
                        metadata_json=excluded.metadata_json,
                        status='active',
                        updated_at=excluded.updated_at
                    """,
                    (
                        f"memory-{uuid.uuid4().hex}",
                        user_id,
                        kind,
                        content,
                        importance,
                        source_session_id,
                        fingerprint,
                        json.dumps(metadata, ensure_ascii=False),
                        now,
                        now,
                    ),
                )
        memory = self.operator_memory(user_id)
        assert memory is not None
        return memory

    def semantic_memories(
        self, user_id: str, query: str = "", limit: int = 20
    ) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._db.execute(
                """
                SELECT * FROM semantic_memories
                WHERE user_id=? AND status='active'
                ORDER BY importance DESC, updated_at DESC LIMIT 200
                """,
                (user_id,),
            ).fetchall()
        terms = _search_terms(query)
        items = [self._semantic_memory_row(row) for row in rows]
        if terms:
            for item in items:
                content_terms = _search_terms(item["content"])
                overlap = len(terms & content_terms) / max(1, len(terms))
                item["relevance"] = overlap + item["importance"] * 0.2
            items.sort(key=lambda item: (item["relevance"], item["updated_at"]), reverse=True)
        return items[: max(1, min(limit, 100))]

    def delete_semantic_memory(self, user_id: str, memory_id: str) -> bool:
        with self._lock, self._db:
            cursor = self._db.execute(
                """
                UPDATE semantic_memories SET status='deleted', updated_at=?
                WHERE id=? AND user_id=? AND status='active'
                """,
                (time.time(), memory_id, user_id),
            )
        return cursor.rowcount == 1

    @staticmethod
    def _semantic_memory_row(row: sqlite3.Row) -> dict[str, Any]:
        return {
            "id": row["id"],
            "user_id": row["user_id"],
            "kind": row["kind"],
            "content": row["content"],
            "importance": row["importance"],
            "source_session_id": row["source_session_id"],
            "metadata": json.loads(row["metadata_json"]),
            "status": row["status"],
            "created_at": row["created_at"],
            "updated_at": row["updated_at"],
        }

    def append_event(
        self, session_id: str, event_type: str, payload: dict[str, Any]
    ) -> dict[str, Any]:
        now = time.time()
        with self._lock, self._db:
            row = self._db.execute(
                "SELECT COALESCE(MAX(sequence), 0) + 1 AS next FROM events WHERE session_id=?",
                (session_id,),
            ).fetchone()
            sequence = int(row["next"])
            event = {
                "id": f"evt-{uuid.uuid4().hex}",
                "type": event_type,
                "session_id": session_id,
                "sequence": sequence,
                "created_at": now,
                **payload,
            }
            self._db.execute(
                """
                INSERT INTO events(
                    id, session_id, sequence, type, payload_json, created_at
                ) VALUES (?, ?, ?, ?, ?, ?)
                """,
                (
                    event["id"],
                    session_id,
                    sequence,
                    event_type,
                    json.dumps(payload, ensure_ascii=False),
                    now,
                ),
            )
        return event

    def events_after(
        self, session_id: str, sequence: int = 0, limit: int = 500
    ) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._db.execute(
                """
                SELECT * FROM events
                WHERE session_id=? AND sequence>?
                ORDER BY sequence ASC LIMIT ?
                """,
                (session_id, max(0, sequence), max(1, min(limit, 2000))),
            ).fetchall()
        result = []
        for row in rows:
            result.append(
                {
                    "id": row["id"],
                    "type": row["type"],
                    "session_id": row["session_id"],
                    "sequence": row["sequence"],
                    "created_at": row["created_at"],
                    **json.loads(row["payload_json"]),
                }
            )
        return result

    def latest_event(
        self, session_id: str, event_type: str
    ) -> dict[str, Any] | None:
        with self._lock:
            row = self._db.execute(
                """
                SELECT * FROM events
                WHERE session_id=? AND type=?
                ORDER BY sequence DESC LIMIT 1
                """,
                (session_id, event_type),
            ).fetchone()
        if row is None:
            return None
        return {
            "id": row["id"],
            "type": row["type"],
            "session_id": row["session_id"],
            "sequence": row["sequence"],
            "created_at": row["created_at"],
            **json.loads(row["payload_json"]),
        }

    def create_confirmation(
        self,
        session_id: str,
        user_id: str,
        action: str,
        args: dict[str, Any],
        summary: str,
        risk_level: str,
        ttl_seconds: float,
    ) -> dict[str, Any]:
        now = time.time()
        item = {
            "id": f"confirm-{uuid.uuid4().hex}",
            "session_id": session_id,
            "user_id": user_id,
            "action": action,
            "args": args,
            "summary": summary,
            "risk_level": risk_level,
            "status": "pending",
            "expires_at": now + max(1.0, ttl_seconds),
            "result": None,
            "created_at": now,
            "resolved_at": None,
        }
        with self._lock, self._db:
            self._db.execute(
                """
                INSERT INTO confirmations(
                    id, session_id, user_id, action, args_json, summary,
                    risk_level, status, expires_at, created_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    item["id"],
                    session_id,
                    user_id,
                    action,
                    json.dumps(args, ensure_ascii=False),
                    summary,
                    risk_level,
                    item["status"],
                    item["expires_at"],
                    now,
                ),
            )
        return item

    def get_confirmation(self, confirmation_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._db.execute(
                "SELECT * FROM confirmations WHERE id=?", (confirmation_id,)
            ).fetchone()
        return self._confirmation_row(row) if row else None

    def pending_confirmations(self, session_id: str) -> list[dict[str, Any]]:
        now = time.time()
        with self._lock, self._db:
            self._db.execute(
                """
                UPDATE confirmations SET status='expired', resolved_at=?
                WHERE session_id=? AND status='pending' AND expires_at<=?
                """,
                (now, session_id, now),
            )
            self._db.execute(
                """
                UPDATE gateway_missions
                SET status='expired', phase='expired', revision=revision+1, updated_at=?
                WHERE confirmation_id IN (
                    SELECT id FROM confirmations
                    WHERE session_id=? AND status='expired' AND resolved_at=?
                )
                """,
                (now, session_id, now),
            )
            rows = self._db.execute(
                """
                SELECT * FROM confirmations
                WHERE session_id=? AND status='pending'
                ORDER BY created_at ASC
                """,
                (session_id,),
            ).fetchall()
        return [self._confirmation_row(row) for row in rows]

    def pending_confirmations_for_user(self, user_id: str) -> list[dict[str, Any]]:
        now = time.time()
        with self._lock, self._db:
            self._db.execute(
                """
                UPDATE confirmations SET status='expired', resolved_at=?
                WHERE user_id=? AND status='pending' AND expires_at<=?
                """,
                (now, user_id, now),
            )
            self._db.execute(
                """
                UPDATE gateway_missions
                SET status='expired', phase='expired', revision=revision+1, updated_at=?
                WHERE confirmation_id IN (
                    SELECT id FROM confirmations
                    WHERE user_id=? AND status='expired' AND resolved_at=?
                )
                """,
                (now, user_id, now),
            )
            rows = self._db.execute(
                """
                SELECT * FROM confirmations
                WHERE user_id=? AND status='pending'
                ORDER BY created_at ASC
                """,
                (user_id,),
            ).fetchall()
        return [self._confirmation_row(row) for row in rows]

    def pending_confirmations_all(self) -> list[dict[str, Any]]:
        now = time.time()
        with self._lock, self._db:
            self._db.execute(
                """
                UPDATE confirmations SET status='expired', resolved_at=?
                WHERE status='pending' AND expires_at<=?
                """,
                (now, now),
            )
            self._db.execute(
                """
                UPDATE gateway_missions
                SET status='expired', phase='expired', revision=revision+1, updated_at=?
                WHERE confirmation_id IN (
                    SELECT id FROM confirmations
                    WHERE status='expired' AND resolved_at=?
                )
                """,
                (now, now),
            )
            rows = self._db.execute(
                """
                SELECT * FROM confirmations WHERE status='pending'
                ORDER BY expires_at ASC
                """
            ).fetchall()
        return [self._confirmation_row(row) for row in rows]

    def create_gateway_mission(
        self, confirmation: dict[str, Any]
    ) -> dict[str, Any]:
        session = self.get_session(confirmation["session_id"])
        if session is None:
            raise ValueError("会话不存在")
        now = time.time()
        mission_id = f"gateway-mission-{uuid.uuid4().hex}"
        with self._lock, self._db:
            self._db.execute(
                """
                INSERT INTO gateway_missions(
                    id, confirmation_id, session_id, user_id, source_channel,
                    title, action, args_json, status, phase, created_at, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    mission_id,
                    confirmation["id"],
                    confirmation["session_id"],
                    confirmation["user_id"],
                    session["channel"],
                    confirmation["summary"],
                    confirmation["action"],
                    json.dumps(confirmation["args"], ensure_ascii=False),
                    "pending_confirmation",
                    "pending_confirmation",
                    now,
                    now,
                ),
            )
        mission = self.gateway_mission_for_confirmation(confirmation["id"])
        assert mission is not None
        return mission

    def update_gateway_mission(
        self,
        confirmation_id: str,
        status: str,
        result: dict[str, Any] | None = None,
        phase: str | None = None,
        ros_mission_id: str | None = None,
        physical_complete: bool | None = None,
    ) -> dict[str, Any] | None:
        with self._lock, self._db:
            current = self._db.execute(
                "SELECT status FROM gateway_missions WHERE confirmation_id=?",
                (confirmation_id,),
            ).fetchone()
            if current is None:
                return None
            current_status = str(current["status"])
            if current_status in MISSION_TERMINAL_STATUSES:
                return self.gateway_mission_for_confirmation(confirmation_id)
            allowed = MISSION_ALLOWED_TRANSITIONS.get(current_status, {current_status})
            if status not in allowed:
                return self.gateway_mission_for_confirmation(confirmation_id)
            terminal = status in MISSION_TERMINAL_STATUSES
            final_phase = status if terminal else phase
            final_physical_complete = (
                bool(physical_complete) and status == "completed"
                if physical_complete is not None else None
            )
            self._db.execute(
                """
                UPDATE gateway_missions
                SET status=?, result_json=?,
                    phase=COALESCE(?, phase),
                    ros_mission_id=COALESCE(?, ros_mission_id),
                    physical_complete=COALESCE(?, physical_complete),
                    revision=revision+1,
                    updated_at=?
                WHERE confirmation_id=?
                """,
                (
                    status,
                    json.dumps(result, ensure_ascii=False) if result is not None else None,
                    final_phase,
                    ros_mission_id,
                    int(final_physical_complete) if final_physical_complete is not None else None,
                    time.time(),
                    confirmation_id,
                ),
            )
        return self.gateway_mission_for_confirmation(confirmation_id)

    def gateway_mission_for_confirmation(
        self, confirmation_id: str
    ) -> dict[str, Any] | None:
        with self._lock:
            row = self._db.execute(
                "SELECT * FROM gateway_missions WHERE confirmation_id=?",
                (confirmation_id,),
            ).fetchone()
        return self._gateway_mission_row(row) if row else None

    def gateway_missions_for_user(
        self, user_id: str, limit: int = 50
    ) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._db.execute(
                """
                SELECT * FROM gateway_missions WHERE user_id=?
                ORDER BY updated_at DESC LIMIT ?
                """,
                (user_id, max(1, min(limit, 200))),
            ).fetchall()
        return [self._gateway_mission_row(row) for row in rows]

    def active_gateway_missions_for_user(
        self, user_id: str
    ) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._db.execute(
                """
                SELECT * FROM gateway_missions
                WHERE user_id=? AND status IN ('pending_confirmation', 'executing')
                ORDER BY created_at
                """,
                (user_id,),
            ).fetchall()
        return [self._gateway_mission_row(row) for row in rows]

    def gateway_mission_for_workflow(
        self, user_id: str, workflow_id: str
    ) -> dict[str, Any] | None:
        for mission in self.gateway_missions_for_user(user_id, limit=200):
            if (
                mission["action"] == "workflow"
                and mission["args"].get("workflow_id") == workflow_id
            ):
                return mission
        return None

    def interrupt_executing_workflows(self) -> list[dict[str, Any]]:
        now = time.time()
        result = {
            "success": False,
            "status": "interrupted",
            "physical_complete": False,
            "message": "Gateway 重启，组合任务已安全中止，请检查无人机状态",
        }
        with self._lock, self._db:
            rows = self._db.execute(
                """
                SELECT confirmation_id FROM gateway_missions
                WHERE action='workflow' AND status='executing'
                """
            ).fetchall()
            for row in rows:
                confirmation_id = row["confirmation_id"]
                self._db.execute(
                    """
                    UPDATE gateway_missions
                    SET status='failed', phase='interrupted', result_json=?,
                        revision=revision+1, updated_at=?
                    WHERE confirmation_id=?
                    """,
                    (json.dumps(result, ensure_ascii=False), now, confirmation_id),
                )
                self._db.execute(
                    """
                    UPDATE confirmations
                    SET status='failed', result_json=?, resolved_at=?
                    WHERE id=? AND status='executing'
                    """,
                    (json.dumps(result, ensure_ascii=False), now, confirmation_id),
                )
        return [
            self.gateway_mission_for_confirmation(row["confirmation_id"])
            for row in rows
        ]

    def resolve_confirmation(
        self,
        confirmation_id: str,
        user_id: str,
        decision: str,
    ) -> dict[str, Any]:
        now = time.time()
        expired = False
        with self._lock, self._db:
            row = self._db.execute(
                "SELECT * FROM confirmations WHERE id=?", (confirmation_id,)
            ).fetchone()
            if row is None:
                raise ValueError("确认请求不存在")
            if row["user_id"] != user_id:
                raise PermissionError("当前用户无权处理该确认请求")
            if row["status"] != "pending":
                status = str(row["status"])
                same_decision = (
                    decision == "approve" and status in {"executing", "completed", "failed"}
                ) or (
                    decision == "cancel" and status in {"cancelled", "expired"}
                )
                if not same_decision:
                    raise ValueError(f"确认请求已经处理: {status}")
                item = self._confirmation_row(row)
                item["idempotent"] = True
                return item
            if float(row["expires_at"]) <= now:
                self._db.execute(
                    "UPDATE confirmations SET status='expired', resolved_at=? WHERE id=?",
                    (now, confirmation_id),
                )
                self._db.execute(
                    """
                    UPDATE gateway_missions
                    SET status='expired', phase='expired', revision=revision+1, updated_at=?
                    WHERE confirmation_id=?
                    """,
                    (now, confirmation_id),
                )
                expired = True
            else:
                status = "executing" if decision == "approve" else "cancelled"
                self._db.execute(
                    "UPDATE confirmations SET status=?, resolved_at=? WHERE id=?",
                    (status, now, confirmation_id),
                )
        if expired:
            raise ValueError("确认请求已过期")
        item = self.get_confirmation(confirmation_id)
        assert item is not None
        return item

    def complete_confirmation(
        self,
        confirmation_id: str,
        status: str,
        result: dict[str, Any],
    ) -> dict[str, Any]:
        if status not in {"completed", "failed", "cancelled"}:
            raise ValueError("invalid confirmation completion status")
        with self._lock, self._db:
            row = self._db.execute(
                "SELECT status FROM confirmations WHERE id=?", (confirmation_id,)
            ).fetchone()
            if row is None:
                raise ValueError("确认请求不存在")
            if row["status"] != "executing":
                item = self.get_confirmation(confirmation_id)
                assert item is not None
                return item
            self._db.execute(
                """
                UPDATE confirmations
                SET status=?, result_json=?, resolved_at=?
                WHERE id=? AND status='executing'
                """,
                (
                    status,
                    json.dumps(result, ensure_ascii=False),
                    time.time(),
                    confirmation_id,
                ),
            )
        item = self.get_confirmation(confirmation_id)
        if item is None:
            raise ValueError("确认请求不存在")
        return item

    @staticmethod
    def _confirmation_row(row: sqlite3.Row) -> dict[str, Any]:
        return {
            "id": row["id"],
            "session_id": row["session_id"],
            "user_id": row["user_id"],
            "action": row["action"],
            "args": json.loads(row["args_json"]),
            "summary": row["summary"],
            "risk_level": row["risk_level"],
            "status": row["status"],
            "expires_at": row["expires_at"],
            "result": json.loads(row["result_json"]) if row["result_json"] else None,
            "created_at": row["created_at"],
            "resolved_at": row["resolved_at"],
        }

    @staticmethod
    def _gateway_mission_row(row: sqlite3.Row) -> dict[str, Any]:
        return {
            "id": row["id"],
            "confirmation_id": row["confirmation_id"],
            "session_id": row["session_id"],
            "user_id": row["user_id"],
            "source_channel": row["source_channel"],
            "title": row["title"],
            "action": row["action"],
            "args": json.loads(row["args_json"]),
            "status": row["status"],
            "phase": row["phase"],
            "ros_mission_id": row["ros_mission_id"],
            "physical_complete": bool(row["physical_complete"]),
            "revision": int(row["revision"]),
            "result": json.loads(row["result_json"]) if row["result_json"] else None,
            "created_at": row["created_at"],
            "updated_at": row["updated_at"],
        }
