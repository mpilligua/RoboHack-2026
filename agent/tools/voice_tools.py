from __future__ import annotations

from .base import ToolContext, ToolResult


def handle_speak_to_user(ctx: ToolContext, args: dict) -> ToolResult:
    text = str(args.get("text", "")).strip()
    if not text:
        return ToolResult(ok=False, tool="speak_to_user", error="empty text")
    if ctx.speak_queue is not None:
        ctx.speak_queue.put(text)
    return ToolResult(ok=True, tool="speak_to_user", result={"spoken": text})
