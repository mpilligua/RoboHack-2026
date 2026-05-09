from __future__ import annotations

import math

from .base import ToolContext, ToolResult

_DEFAULT_SPEED = 0.15   # m/s
_DEFAULT_OMEGA = 0.4    # rad/s
_MAX_CHUNK_S   = 2.0    # Lite3Motion hard cap per _drive() call


def _require_motion(ctx: ToolContext, tool: str):
    if ctx.motion is None:
        raise RuntimeError(f"motion adapter not connected (tool: {tool})")


def _drive_for(move_fn, total_s: float) -> None:
    """Call move_fn() in chunks to cover total_s seconds."""
    remaining = total_s
    while remaining > 0:
        chunk = min(remaining, _MAX_CHUNK_S)
        move_fn(chunk)
        remaining -= chunk


def handle_walk_forward(ctx: ToolContext, args: dict) -> ToolResult:
    _require_motion(ctx, "walk_forward")
    speed = float(args.get("speed", _DEFAULT_SPEED))
    if "distance_m" in args:
        duration_s = float(args["distance_m"]) / speed
        result = {"distance_m": float(args["distance_m"]), "speed": speed}
    elif "duration_s" in args:
        duration_s = float(args["duration_s"])
        result = {"duration_s": duration_s, "speed": speed}
    else:
        return ToolResult(ok=False, tool="walk_forward", error="provide distance_m or duration_s")
    _drive_for(lambda d: ctx.motion.forward(speed=speed, duration_s=d), duration_s)
    return ToolResult(ok=True, tool="walk_forward", result=result)


def handle_walk_backward(ctx: ToolContext, args: dict) -> ToolResult:
    _require_motion(ctx, "walk_backward")
    speed = float(args.get("speed", _DEFAULT_SPEED))
    if "distance_m" in args:
        duration_s = float(args["distance_m"]) / speed
        result = {"distance_m": float(args["distance_m"]), "speed": speed}
    elif "duration_s" in args:
        duration_s = float(args["duration_s"])
        result = {"duration_s": duration_s, "speed": speed}
    else:
        return ToolResult(ok=False, tool="walk_backward", error="provide distance_m or duration_s")
    _drive_for(lambda d: ctx.motion.backward(speed=speed, duration_s=d), duration_s)
    return ToolResult(ok=True, tool="walk_backward", result=result)


def handle_turn_left(ctx: ToolContext, args: dict) -> ToolResult:
    _require_motion(ctx, "turn_left")
    omega = float(args.get("omega", _DEFAULT_OMEGA))
    if "angle_deg" in args:
        duration_s = math.radians(float(args["angle_deg"])) / omega
        result = {"angle_deg": float(args["angle_deg"]), "omega": omega}
    elif "duration_s" in args:
        duration_s = float(args["duration_s"])
        result = {"duration_s": duration_s, "omega": omega}
    else:
        return ToolResult(ok=False, tool="turn_left", error="provide angle_deg or duration_s")
    _drive_for(lambda d: ctx.motion.turn_left(omega=omega, duration_s=d), duration_s)
    return ToolResult(ok=True, tool="turn_left", result=result)


def handle_turn_right(ctx: ToolContext, args: dict) -> ToolResult:
    _require_motion(ctx, "turn_right")
    omega = float(args.get("omega", _DEFAULT_OMEGA))
    if "angle_deg" in args:
        duration_s = math.radians(float(args["angle_deg"])) / omega
        result = {"angle_deg": float(args["angle_deg"]), "omega": omega}
    elif "duration_s" in args:
        duration_s = float(args["duration_s"])
        result = {"duration_s": duration_s, "omega": omega}
    else:
        return ToolResult(ok=False, tool="turn_right", error="provide angle_deg or duration_s")
    _drive_for(lambda d: ctx.motion.turn_right(omega=omega, duration_s=d), duration_s)
    return ToolResult(ok=True, tool="turn_right", result=result)


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
