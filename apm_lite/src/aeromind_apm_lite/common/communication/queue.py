"""Async bounded queue that coalesces replaceable state such as telemetry."""

from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import dataclass
from typing import Generic, Hashable, Optional, TypeVar

ItemT = TypeVar("ItemT")


class QueueClosed(RuntimeError):
    pass


@dataclass
class _Entry(Generic[ItemT]):
    value: ItemT
    replaceable: bool
    replacement_key: Optional[Hashable] = None


class BoundedLatestQueue(Generic[ItemT]):
    """Keep control events, while allowing stale state updates to be replaced.

    A non-replaceable producer waits when the queue contains only control
    events. A replaceable producer never waits: it evicts the oldest
    replaceable entry, or drops the new entry when no such entry exists.
    """

    def __init__(self, maxsize: int) -> None:
        if maxsize < 1:
            raise ValueError("maxsize must be positive")
        self._maxsize = maxsize
        self._entries: deque[_Entry[ItemT]] = deque()
        self._condition = asyncio.Condition()
        self._closed = False
        self._dropped_replaceable = 0

    @property
    def maxsize(self) -> int:
        return self._maxsize

    @property
    def qsize(self) -> int:
        return len(self._entries)

    @property
    def dropped_replaceable(self) -> int:
        return self._dropped_replaceable

    async def put(
        self,
        value: ItemT,
        *,
        replaceable: bool = False,
        replacement_key: Optional[Hashable] = None,
    ) -> bool:
        if replacement_key is not None and not replaceable:
            raise ValueError("replacement_key requires replaceable=True")
        async with self._condition:
            if self._closed:
                raise QueueClosed("queue is closed")
            if replacement_key is not None:
                for index, entry in enumerate(self._entries):
                    if entry.replacement_key == replacement_key:
                        del self._entries[index]
                        self._dropped_replaceable += 1
                        break
            while len(self._entries) >= self._maxsize:
                if self._evict_oldest_replaceable():
                    self._dropped_replaceable += 1
                    break
                if replaceable:
                    self._dropped_replaceable += 1
                    return False
                await self._condition.wait()
                if self._closed:
                    raise QueueClosed("queue is closed")
            self._entries.append(
                _Entry(
                    value=value,
                    replaceable=replaceable,
                    replacement_key=replacement_key,
                )
            )
            self._condition.notify_all()
            return True

    async def get(self) -> ItemT:
        async with self._condition:
            while not self._entries:
                if self._closed:
                    raise QueueClosed("queue is closed")
                await self._condition.wait()
            entry = self._entries.popleft()
            self._condition.notify_all()
            return entry.value

    async def close(self) -> None:
        async with self._condition:
            self._closed = True
            self._condition.notify_all()

    def _evict_oldest_replaceable(self) -> bool:
        for index, entry in enumerate(self._entries):
            if entry.replaceable:
                del self._entries[index]
                return True
        return False
