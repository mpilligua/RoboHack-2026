from __future__ import annotations

import threading
import time

import numpy as _np

from .base import ToolContext, ToolResult


def _require_follow(ctx: ToolContext, tool: str):
    if ctx.follow is None:
        raise RuntimeError(f"follow adapter not connected (tool: {tool})")


def handle_follow_person(ctx: ToolContext, args: dict) -> ToolResult:
    _require_follow(ctx, "follow_person")
    yolo_id = int(args["yolo_id"])
    ctx.follow.follow(yolo_id)
    return ToolResult(ok=True, tool="follow_person", result={"action": "follow_person", "yolo_id": yolo_id})


def handle_go_to_object(ctx: ToolContext, args: dict) -> ToolResult:
    _require_follow(ctx, "go_to_object")
    yolo_id = int(args["yolo_id"])
    stop_distance_m = float(args.get("stop_distance_m", 0.8))
    timeout_s = float(args.get("timeout_s", 30.0))

    ctx.follow.follow(yolo_id)

    def watcher():
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            time.sleep(0.5)
            dets = ctx.follow.get_detections()
            target = next((d for d in dets if d.id == yolo_id), None)
            if target is None:
                continue
            try:
                rgb = ctx.robot.get_rgb(timeout_s=2.0)
                depth = ctx.robot.get_depth(timeout_s=2.0)
            except TimeoutError:
                continue
            sx = depth.width / float(rgb.width)
            sy = depth.height / float(rgb.height)
            x1, y1, x2, y2 = target.bbox
            dx1 = max(0, int(x1 * sx))
            dy1 = max(0, int(y1 * sy))
            dx2 = min(depth.width, int(x2 * sx))
            dy2 = min(depth.height, int(y2 * sy))
            patch = depth.depth_mm[dy1:dy2, dx1:dx2]
            valid = patch[(patch > 100) & (patch < 8000)]
            if valid.size < 50:
                continue
            median_m = float(_np.median(valid)) / 1000.0
            if median_m <= stop_distance_m:
                ctx.follow.stop()
                return
        ctx.follow.stop()

    threading.Thread(target=watcher, daemon=True).start()

    return ToolResult(
        ok=True,
        tool="go_to_object",
        result={
            "action": "go_to_object",
            "yolo_id": yolo_id,
            "stop_distance_m": stop_distance_m,
            "timeout_s": timeout_s,
            "note": "Walking toward target; will auto-stop when median depth in bbox <= stop_distance_m.",
        },
    )


def handle_stop_tracking(ctx: ToolContext, args: dict) -> ToolResult:
    if ctx.follow is None:
        return ToolResult(ok=True, tool="stop_tracking", result={"action": "stop_tracking", "note": "no follow adapter"})
    try:
        ctx.follow.stop()
    except Exception as e:
        return ToolResult(ok=False, tool="stop_tracking", error=str(e))
    return ToolResult(ok=True, tool="stop_tracking", result={"action": "stop_tracking"})
