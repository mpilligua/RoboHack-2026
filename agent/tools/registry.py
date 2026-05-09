from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Callable

from .base import ToolContext, ToolResult

CALLER_PLANNER = "planner"
CALLER_OPERATOR = "operator"


@dataclass
class _HandlerEntry:
    handler: Callable
    allowed_callers: list[str]


class ToolRegistry:
    def __init__(self) -> None:
        self._handlers: dict[str, _HandlerEntry] = {}

    def register(
        self,
        name: str,
        handler: Callable,
        allowed_callers: list[str],
        **_kwargs,  # absorb legacy requires_*_safety kwargs silently
    ) -> None:
        self._handlers[name] = _HandlerEntry(
            handler=handler,
            allowed_callers=allowed_callers,
        )

    def call(self, name: str, args: dict, ctx: ToolContext, caller: str) -> ToolResult:
        from memory.schemas import Event  # avoid circular at module level

        entry = self._handlers.get(name)
        if entry is None:
            return ToolResult(ok=False, tool=name, error=f"unknown tool: {name!r}")

        if caller not in entry.allowed_callers:
            ctx.memory.add_event(Event(time.time(), "tool_blocked", name, f"caller '{caller}' not allowed"))
            return ToolResult(ok=False, tool=name, error=f"caller '{caller}' is not allowed to call '{name}'")

        try:
            ctx.memory.add_event(Event(time.time(), "tool_call", name, json.dumps(args)[:300]))
            result = entry.handler(ctx, args)
            ctx.memory.add_event(Event(time.time(), "tool_result", name, result.to_str()[:300]))
            return result
        except Exception as exc:
            err = f"{type(exc).__name__}: {exc}"
            ctx.memory.add_event(Event(time.time(), "tool_error", name, err))
            return ToolResult(ok=False, tool=name, error=err)

    def names(self) -> list[str]:
        return list(self._handlers.keys())
