"""Common interface implemented by all conversational Agent runtimes."""

from __future__ import annotations

from typing import Any, Awaitable, Callable, Protocol


EmitCallback = Callable[[str, dict[str, Any]], Awaitable[None]]


class AgentRuntime(Protocol):
    provider: str
    model: str
    runtime_session_id: str | None

    async def start(self): ...

    async def close(self): ...

    async def interrupt(self): ...

    async def set_model(self, model: str): ...

    async def run_turn(
        self, prompt: str, emit: EmitCallback
    ) -> dict[str, Any]: ...
