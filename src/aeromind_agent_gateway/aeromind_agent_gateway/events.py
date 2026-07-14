"""In-process fan-out for persisted Agent events."""

from __future__ import annotations

import asyncio
from collections import defaultdict
from typing import Any

from .store import SessionStore


class EventBus:
    def __init__(self, store: SessionStore):
        self._store = store
        self._subscribers: dict[str, set[asyncio.Queue]] = defaultdict(set)
        self._user_subscribers: dict[str, set[asyncio.Queue]] = defaultdict(set)
        self._lock = asyncio.Lock()

    async def subscribe(self, session_id: str) -> asyncio.Queue:
        queue: asyncio.Queue = asyncio.Queue(maxsize=256)
        async with self._lock:
            self._subscribers[session_id].add(queue)
        return queue

    async def unsubscribe(self, session_id: str, queue: asyncio.Queue):
        async with self._lock:
            queues = self._subscribers.get(session_id)
            if not queues:
                return
            queues.discard(queue)
            if not queues:
                self._subscribers.pop(session_id, None)

    async def subscribe_user(self, user_id: str) -> asyncio.Queue:
        queue: asyncio.Queue = asyncio.Queue(maxsize=256)
        async with self._lock:
            self._user_subscribers[user_id].add(queue)
        return queue

    async def unsubscribe_user(self, user_id: str, queue: asyncio.Queue):
        async with self._lock:
            queues = self._user_subscribers.get(user_id)
            if not queues:
                return
            queues.discard(queue)
            if not queues:
                self._user_subscribers.pop(user_id, None)

    async def emit(
        self, session_id: str, event_type: str, **payload: Any
    ) -> dict[str, Any]:
        event = self._store.append_event(session_id, event_type, payload)
        session = self._store.get_session(session_id)
        user_id = session["user_id"] if session else ""
        async with self._lock:
            queues = set(self._subscribers.get(session_id, ()))
            queues.update(self._user_subscribers.get(user_id, ()))
        for queue in queues:
            if queue.full():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            queue.put_nowait(event)
        return event
