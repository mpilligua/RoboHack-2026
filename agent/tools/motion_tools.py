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
    distance_m = float(args["distance_m"])
    speed = float(args.get("speed", _DEFAULT_SPEED))
    duration_s = distance_m / speed
    _drive_for(lambda d: ctx.motion.forward(speed=speed, duration_s=d), duration_s)
    return ToolResult(ok=True, tool="walk_forward", result={"distance_m": distance_m, "speed": speed})


def handle_walk_backward(ctx: ToolContext, args: dict) -> ToolResult:
    _require_motion(ctx, "walk_backward")
    distance_m = float(args["distance_m"])
    speed = float(args.get("speed", _DEFAULT_SPEED))
    duration_s = distance_m / speed
    _drive_for(lambda d: ctx.motion.backward(speed=speed, duration_s=d), duration_s)
    return ToolResult(ok=True, tool="walk_backward", result={"distance_m": distance_m, "speed": speed})


def handle_turn_left(ctx: ToolContext, args: dict) -> ToolResult:
    _require_motion(ctx, "turn_left")
    angle_deg = float(args["angle_deg"])
    omega = float(args.get("omega", _DEFAULT_OMEGA))
    duration_s = math.radians(angle_deg) / omega
    _drive_for(lambda d: ctx.motion.turn_left(omega=omega, duration_s=d), duration_s)
    return ToolResult(ok=True, tool="turn_left", result={"angle_deg": angle_deg, "omega": omega})


def handle_turn_right(ctx: ToolContext, args: dict) -> ToolResult:
    _require_motion(ctx, "turn_right")
    angle_deg = float(args["angle_deg"])
    omega = float(args.get("omega", _DEFAULT_OMEGA))
    duration_s = math.radians(angle_deg) / omega
    _drive_for(lambda d: ctx.motion.turn_right(omega=omega, duration_s=d), duration_s)
    return ToolResult(ok=True, tool="turn_right", result={"angle_deg": angle_deg, "omega": omega})


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
