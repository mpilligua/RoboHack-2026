from __future__ import annotations

from .base import ToolContext, ToolResult


def _require_motion(ctx: ToolContext, tool: str):
    if ctx.motion is None:
        raise RuntimeError(f"motion adapter not connected (tool: {tool})")


def handle_walk_forward(ctx: ToolContext, args: dict) -> ToolResult:
    _require_motion(ctx, "walk_forward")
    duration_s = float(args["duration_s"])
    speed = float(args.get("speed", 0.15))
    ctx.motion.forward(speed=speed, duration_s=duration_s)
    return ToolResult(ok=True, tool="walk_forward", result={"duration_s": duration_s, "speed": speed})


def handle_walk_backward(ctx: ToolContext, args: dict) -> ToolResult:
    _require_motion(ctx, "walk_backward")
    duration_s = float(args["duration_s"])
    speed = float(args.get("speed", 0.15))
    ctx.motion.backward(speed=speed, duration_s=duration_s)
    return ToolResult(ok=True, tool="walk_backward", result={"duration_s": duration_s, "speed": speed})


def handle_turn_left(ctx: ToolContext, args: dict) -> ToolResult:
    _require_motion(ctx, "turn_left")
    duration_s = float(args["duration_s"])
    omega = float(args.get("omega", 0.4))
    ctx.motion.turn_left(omega=omega, duration_s=duration_s)
    return ToolResult(ok=True, tool="turn_left", result={"duration_s": duration_s, "omega": omega})


def handle_turn_right(ctx: ToolContext, args: dict) -> ToolResult:
    _require_motion(ctx, "turn_right")
    duration_s = float(args["duration_s"])
    omega = float(args.get("omega", 0.4))
    ctx.motion.turn_right(omega=omega, duration_s=duration_s)
    return ToolResult(ok=True, tool="turn_right", result={"duration_s": duration_s, "omega": omega})


def handle_stop_motion(ctx: ToolContext, args: dict) -> ToolResult:
    errors = []
    if ctx.motion is not None:
        try:
            ctx.motion.stop()
        except Exception as e:
            errors.append(str(e))
    if errors:
        return ToolResult(ok=False, tool="stop_motion", error="; ".join(errors))
    return ToolResult(ok=True, tool="stop_motion", result={"action": "stop_motion"})
