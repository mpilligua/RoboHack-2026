from __future__ import annotations

from .base import ToolContext, ToolResult


def _require_basic_goal(ctx: ToolContext, tool: str):
    if ctx.basic_goal is None:
        raise RuntimeError(f"basic goal adapter not connected (tool: {tool})")


def handle_cancel_basic_goal(ctx: ToolContext, args: dict) -> ToolResult:
    _require_basic_goal(ctx, "cancel_basic_goal")
    reason = str(args.get("reason", "stop"))
    ctx.basic_goal.cancel_goal(reason)
    return ToolResult(ok=True, tool="cancel_basic_goal", result={"reason": reason})


def handle_get_basic_goal_status(ctx: ToolContext, args: dict) -> ToolResult:
    _require_basic_goal(ctx, "get_basic_goal_status")
    wait_for = args.get("wait_for")
    timeout_s = args.get("timeout_s")
    if wait_for is not None and timeout_s is None:
        timeout_s = 5.0
    status = ctx.basic_goal.get_status(wait_for=wait_for, timeout_s=timeout_s)
    if status.get("status") is None:
        status["note"] = "no status yet"
    return ToolResult(ok=True, tool="get_basic_goal_status", result=status)


def handle_send_simple_cmd(ctx: ToolContext, args: dict) -> ToolResult:
    _require_basic_goal(ctx, "send_simple_cmd")
    cmd_code = int(args["cmd_code"])
    size = int(args["size"])
    msg_type = int(args["type"])
    ctx.basic_goal.send_simple_cmd(cmd_code, size, msg_type)
    return ToolResult(
        ok=True,
        tool="send_simple_cmd",
        result={"cmd_code": cmd_code, "size": size, "type": msg_type},
    )


def handle_send_complex_cmd(ctx: ToolContext, args: dict) -> ToolResult:
    _require_basic_goal(ctx, "send_complex_cmd")
    cmd_code = int(args["cmd_code"])
    size = int(args["size"])
    msg_type = int(args["type"])
    data = float(args["data"])
    ctx.basic_goal.send_complex_cmd(cmd_code, size, msg_type, data)
    return ToolResult(
        ok=True,
        tool="send_complex_cmd",
        result={"cmd_code": cmd_code, "size": size, "type": msg_type, "data": data},
    )
