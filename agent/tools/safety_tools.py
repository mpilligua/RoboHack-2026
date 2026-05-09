from __future__ import annotations

import time

from .base import ToolContext, ToolResult


def handle_stop(ctx: ToolContext, args: dict) -> ToolResult:
    errors = []
    if ctx.motion is not None:
        try:
            ctx.motion.stop()
        except Exception as e:
            errors.append(f"motion.stop: {e}")
    if ctx.follow is not None:
        try:
            ctx.follow.stop()
        except Exception as e:
            errors.append(f"follow.stop: {e}")
    if errors:
        return ToolResult(ok=False, tool="stop", error="; ".join(errors))
    return ToolResult(ok=True, tool="stop", result={"action": "stop"})


def handle_emergency_stop(ctx: ToolContext, args: dict) -> ToolResult:
    ctx.safety.trigger_emergency_stop()
    handle_stop(ctx, {})
    return ToolResult(ok=True, tool="emergency_stop", result={"action": "emergency_stop", "latched": True})


def handle_reset_emergency_stop(ctx: ToolContext, args: dict) -> ToolResult:
    ctx.safety.reset_emergency_stop()
    return ToolResult(ok=True, tool="reset_emergency_stop", result={"action": "reset_emergency_stop", "latched": False})


def handle_check_safety(ctx: ToolContext, args: dict) -> ToolResult:
    sr = ctx.safety.check_forward_motion()
    return ToolResult(
        ok=True,
        tool="check_safety",
        result={"safe": sr.safe, "reason": sr.reason},
    )


def handle_get_robot_status(ctx: ToolContext, args: dict) -> ToolResult:
    try:
        summary = ctx.robot.depth_summary(timeout_s=2.0)
        ctx.memory.update_robot_state(
            depth_stamp=time.time(),
            depth_min_mm=summary.get("min_mm"),
            depth_center_mm=summary.get("center_mm"),
            nearest_obstacle_mm=summary.get("min_mm"),
            rosbridge_connected=True,
        )
    except Exception as e:
        ctx.memory.update_robot_state(rosbridge_connected=False)
        return ToolResult(ok=False, tool="get_robot_status", error=f"depth_summary failed: {e}")

    if ctx.motion is not None:
        ctx.memory.update_robot_state(motion_connected=True)
    if ctx.follow is not None:
        ctx.memory.update_robot_state(follow_connected=True)

    snap = ctx.memory.snapshot()
    return ToolResult(ok=True, tool="get_robot_status", result={"robot": snap["robot"]})
