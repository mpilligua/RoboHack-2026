from __future__ import annotations

import time

from .base import ToolContext, ToolResult


def _serialize_object(o, *, now: float) -> dict:
    """Common JSON shape for find_object / list_world_objects / get_visible_objects."""
    age_s = round(max(0.0, now - o.last_seen_ts), 2) if o.last_seen_ts else None
    has_world_pos = o.x_odom is not None and o.y_odom is not None
    return {
        "id": o.yolo_id,
        "label": o.label,
        "description": o.description,
        "position": o.position_text,
        "depth_m": o.depth_m,
        "seen_count": o.seen_count,
        "age_s": age_s,
        "last_seen_s_ago": age_s,  # alias kept for callers that read this name
        # World position in /leg_odom frame (drifts over minutes). None if the
        # background world-tick hasn't yet projected this detection.
        "x_odom": round(o.x_odom, 3) if o.x_odom is not None else None,
        "y_odom": round(o.y_odom, 3) if o.y_odom is not None else None,
        "z_odom": round(o.z_odom, 3) if o.z_odom is not None else None,
        "has_world_position": has_world_pos,
    }


def handle_get_visible_objects(ctx: ToolContext, args: dict) -> ToolResult:
    now = time.time()
    objects = [_serialize_object(o, now=now) for o in ctx.memory.get_objects()]
    return ToolResult(ok=True, tool="get_visible_objects", result={"objects": objects})


def handle_find_object(ctx: ToolContext, args: dict) -> ToolResult:
    label = args.get("label", "")
    if not label:
        return ToolResult(ok=False, tool="find_object", error="label is required")
    now = time.time()
    found = ctx.memory.find_objects_by_label(label)
    objects = [_serialize_object(o, now=now) for o in found]
    return ToolResult(
        ok=True, tool="find_object",
        result={"objects": objects, "count": len(objects)},
    )


def handle_resolve_reference(ctx: ToolContext, args: dict) -> ToolResult:
    ref = args.get("ref", "")
    if not ref:
        return ToolResult(ok=False, tool="resolve_reference", error="ref is required")
    obj = ctx.memory.resolve_reference(ref)
    if obj is None:
        return ToolResult(ok=False, tool="resolve_reference", error=f"could not resolve reference: {ref!r}")
    ctx.memory.set_selected_object_id(obj.yolo_id)
    return ToolResult(
        ok=True,
        tool="resolve_reference",
        result=_serialize_object(obj, now=time.time()),
    )


def handle_find_objects_matching_constraints(ctx: ToolContext, args: dict) -> ToolResult:
    now = time.time()
    found = ctx.memory.find_objects_matching_constraints(args)
    objects = [_serialize_object(o, now=now) for o in found]
    return ToolResult(
        ok=True, tool="find_objects_matching_constraints",
        result={"objects": objects, "count": len(objects)},
    )


def handle_go_to_world_object(ctx: ToolContext, args: dict) -> ToolResult:
    """Navigate to a remembered object using its stored world position.

    Calls perception/external_pose.py's `get_pose()` to find the robot's
    current pose, computes a goal point ``stop_distance_m`` short of the
    target along the line from robot to target, and calls `goto(x, y, z)`.

    BLOCKING: returns when goto returns (arrival, timeout, or error).
    """
    import math

    from perception.external_pose import get_pose, goto

    label = args.get("label", "")
    if not label:
        return ToolResult(ok=False, tool="go_to_world_object", error="label is required")
    stop_distance_m = float(args.get("stop_distance_m", 0.8))
    timeout_s = float(args.get("timeout_s", 30.0))

    all_matches = ctx.memory.find_objects_by_label(label)
    matches = [o for o in all_matches if o.x_odom is not None and o.y_odom is not None]
    if not matches:
        # Diagnostic: surface what we DID find so we can see why filter rejected.
        seen = [
            {"id": o.yolo_id, "label": o.label, "x_odom": o.x_odom, "y_odom": o.y_odom,
             "age_s": round(time.time() - o.last_seen_ts, 1)}
            for o in all_matches
        ]
        return ToolResult(
            ok=False, tool="go_to_world_object",
            error=f"no remembered object matching {label!r} with a world position.",
            result={"label_matches_without_world_pos": seen, "count": len(seen)},
        )
    target = max(matches, key=lambda o: o.last_seen_ts)
    tx, ty = float(target.x_odom), float(target.y_odom)
    tz = float(target.z_odom) if target.z_odom is not None else 0.0

    try:
        pose = get_pose()
    except Exception as e:
        return ToolResult(
            ok=False, tool="go_to_world_object",
            error=f"external get_pose() failed: {e}",
        )
    if pose is None:
        return ToolResult(
            ok=False, tool="go_to_world_object",
            error="get_pose() returned None; can't compute approach offset.",
        )

    rx, ry = float(pose.x), float(pose.y)
    dx, dy = tx - rx, ty - ry
    dist = math.hypot(dx, dy)
    if dist <= stop_distance_m:
        return ToolResult(
            ok=True, tool="go_to_world_object",
            result={
                "note": "already within stop_distance_m",
                "target_label": target.label,
                "target_xyz": [tx, ty, tz],
                "robot_xy": [rx, ry],
                "distance_m": round(dist, 3),
            },
        )

    ux, uy = dx / dist, dy / dist
    goal_x = tx - ux * stop_distance_m
    goal_y = ty - uy * stop_distance_m

    try:
        result = goto(goal_x, goal_y, tz, timeout_s=timeout_s)
    except Exception as e:
        return ToolResult(
            ok=False, tool="go_to_world_object",
            error=f"external goto() raised: {e}",
        )

    return ToolResult(
        ok=(result.status == "reached"),
        tool="go_to_world_object",
        result={
            "target_label": target.label,
            "target_id": target.yolo_id,
            "target_xyz": [round(tx, 3), round(ty, 3), round(tz, 3)],
            "goal_xyz": [round(goal_x, 3), round(goal_y, 3), round(tz, 3)],
            "goto_status": result.status,
            "goto_detail": result.detail,
            "robot_start_xy": [round(rx, 3), round(ry, 3)],
            "distance_m": round(dist, 3),
            "stop_distance_m": stop_distance_m,
        },
    )


def handle_list_world_objects(ctx: ToolContext, args: dict) -> ToolResult:
    """Like get_visible_objects but only returns ones with a world position.

    Optional args:
        max_age_s: drop objects whose last_seen is older than this.
    """
    now = time.time()
    max_age = args.get("max_age_s")
    out = []
    for o in ctx.memory.get_objects():
        if o.x_odom is None or o.y_odom is None:
            continue
        if max_age is not None and (now - o.last_seen_ts) > float(max_age):
            continue
        out.append(_serialize_object(o, now=now))
    return ToolResult(
        ok=True, tool="list_world_objects",
        result={"objects": out, "count": len(out), "frame": "leg_odom"},
    )
