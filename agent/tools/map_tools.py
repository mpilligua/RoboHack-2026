from __future__ import annotations

import math

from .base import ToolContext, ToolResult


def _require_map_runtime(ctx: ToolContext, tool: str):
    if ctx.map_runtime is None:
        raise RuntimeError(f"map runtime not connected (tool: {tool})")


def _require_waypoints(ctx: ToolContext, tool: str):
    if ctx.waypoints is None:
        raise RuntimeError(f"waypoint store not connected (tool: {tool})")


def _success(tool: str, payload: dict) -> ToolResult:
    warnings = list(payload.get("warnings") or [])
    data = payload.get("data") or {}
    return ToolResult(ok=True, tool=tool, result={"ok": True, "warnings": warnings, "data": data})


def _failure(tool: str, message: str, warnings: list[str] | None = None) -> ToolResult:
    result = {"ok": False, "error": str(message), "warnings": list(warnings or [])}
    return ToolResult(ok=False, tool=tool, error=str(message), result=result)


def _route_summary(path_points: list[dict], start_pose: dict, caution_text: str | None) -> str:
    if len(path_points) < 2:
        return "The route is very short from the current position."
    dx = path_points[1]["x_m"] - start_pose["x_m"]
    dy = path_points[1]["y_m"] - start_pose["y_m"]
    initial_heading = math.atan2(dy, dx)
    rel = _normalize(initial_heading - start_pose["yaw_rad"])
    if abs(rel) < math.radians(20):
        turn = "mostly straight ahead"
    elif rel > 0:
        turn = "starting with a left turn"
    else:
        turn = "starting with a right turn"
    bendiness = 0.0
    last_heading = None
    for prev, cur in zip(path_points, path_points[1:]):
        heading = math.atan2(cur["y_m"] - prev["y_m"], cur["x_m"] - prev["x_m"])
        if last_heading is not None:
            bendiness += abs(_normalize(heading - last_heading))
        last_heading = heading
    shape = "fairly direct"
    if bendiness > math.pi / 2:
        shape = "winding"
    elif bendiness > math.pi / 6:
        shape = "curving"
    text = f"The route is {shape}, {turn}."
    if caution_text:
        text += f" Caution: {caution_text}."
    return text


def _normalize(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def handle_get_robot_pose_in_map(ctx: ToolContext, _args: dict) -> ToolResult:
    tool = "get_robot_pose_in_map"
    try:
        _require_map_runtime(ctx, tool)
        payload = ctx.map_runtime.get_robot_pose_in_map()
        return _success(tool, {"warnings": [], "data": payload})
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_get_map_summary(ctx: ToolContext, _args: dict) -> ToolResult:
    tool = "get_map_summary"
    try:
        _require_map_runtime(ctx, tool)
        payload = ctx.map_runtime.get_map_summary()
        return _success(tool, payload)
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_get_local_map_context(ctx: ToolContext, args: dict) -> ToolResult:
    tool = "get_local_map_context"
    try:
        _require_map_runtime(ctx, tool)
        radius_m = float(args.get("radius_m", 3.0))
        payload = ctx.map_runtime.get_local_map_context(radius_m=radius_m)
        return _success(tool, payload)
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_get_local_occupancy_grid(ctx: ToolContext, args: dict) -> ToolResult:
    tool = "get_local_occupancy_grid"
    try:
        _require_map_runtime(ctx, tool)
        size_m = float(args.get("size_m", 5.0))
        cell_size_m = float(args.get("cell_size_m", 0.25))
        payload = ctx.map_runtime.get_local_occupancy_grid(size_m=size_m, cell_size_m=cell_size_m)
        return _success(tool, payload)
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_save_waypoint(ctx: ToolContext, args: dict) -> ToolResult:
    tool = "save_waypoint"
    try:
        _require_map_runtime(ctx, tool)
        _require_waypoints(ctx, tool)
        name = str(args["name"]).strip()
        if not name:
            raise ValueError("name is required")
        waypoint_type = str(args.get("type", "generic")).strip() or "generic"
        pose_blob = ctx.map_runtime.get_robot_pose_in_map()
        record = ctx.waypoints.save(
            name=name,
            waypoint_type=waypoint_type,
            pose=pose_blob["pose"],
            frame_id=pose_blob["frame_id"],
            base_frame=pose_blob["base_frame"],
            map_signature=ctx.map_runtime.get_map_signature(),
        )
        return _success(tool, {"warnings": [], "data": {"waypoint": _record_to_dict(record)}})
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_list_waypoints(ctx: ToolContext, _args: dict) -> ToolResult:
    tool = "list_waypoints"
    try:
        _require_waypoints(ctx, tool)
        records = [_record_to_dict(r) for r in ctx.waypoints.list()]
        return _success(tool, {"warnings": [], "data": {"count": len(records), "waypoints": records}})
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_get_waypoint(ctx: ToolContext, args: dict) -> ToolResult:
    tool = "get_waypoint"
    try:
        _require_waypoints(ctx, tool)
        name = str(args["name"])
        record = ctx.waypoints.get(name)
        if record is None:
            raise RuntimeError(f"waypoint {name!r} not found")
        return _success(tool, {"warnings": [], "data": {"waypoint": _record_to_dict(record)}})
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_check_waypoint_reachable(ctx: ToolContext, args: dict) -> ToolResult:
    tool = "check_waypoint_reachable"
    try:
        _require_map_runtime(ctx, tool)
        _require_waypoints(ctx, tool)
        record = _require_matching_waypoint(ctx, str(args["name"]))
        plan = ctx.map_runtime.compute_path_to_waypoint(record.pose)
        distance_to_goal_m = math.hypot(
            record.pose["x_m"] - ctx.map_runtime.get_robot_pose_in_map()["pose"]["x_m"],
            record.pose["y_m"] - ctx.map_runtime.get_robot_pose_in_map()["pose"]["y_m"],
        )
        reachable = plan.get("reachable")
        if reachable is None:
            reachable = bool(plan.get("path_points"))
        data = {
            "name": record.name,
            "reachable": bool(reachable),
            "method": plan.get("method"),
            "distance_to_goal_m": distance_to_goal_m,
            "path_length_m": float(plan.get("path_length_m", distance_to_goal_m)),
            "blocking_reason": plan.get("blocking_reason"),
            "confidence": plan.get("confidence", "medium" if reachable else "low"),
            "live_clearance_ok": plan.get("blocking_reason") is None,
            "freshness": ctx.map_runtime.freshness(),
        }
        return _success(tool, {"warnings": plan.get("warnings", []), "data": data})
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_get_route_summary_to_waypoint(ctx: ToolContext, args: dict) -> ToolResult:
    tool = "get_route_summary_to_waypoint"
    try:
        _require_map_runtime(ctx, tool)
        _require_waypoints(ctx, tool)
        record = _require_matching_waypoint(ctx, str(args["name"]))
        plan = ctx.map_runtime.compute_path_to_waypoint(record.pose)
        start_pose = ctx.map_runtime.get_robot_pose_in_map()["pose"]
        path_points = plan.get("path_points") or [start_pose, record.pose]
        caution_text = None
        blocked_dirs = []
        if plan.get("method") != "nav2_plan":
            local_context = ctx.map_runtime.get_local_map_context(radius_m=3.0)
            blocked_dirs = local_context["data"].get("blocked_directions") or []
        if blocked_dirs:
            caution_text = f"nearby obstacles are present toward {', '.join(blocked_dirs[:2])}"
        data = {
            "name": record.name,
            "summary_text": _route_summary(path_points, start_pose, caution_text),
            "distance_m": math.hypot(record.pose["x_m"] - start_pose["x_m"], record.pose["y_m"] - start_pose["y_m"]),
            "initial_heading_rad": math.atan2(
                path_points[min(1, len(path_points) - 1)]["y_m"] - start_pose["y_m"],
                path_points[min(1, len(path_points) - 1)]["x_m"] - start_pose["x_m"],
            ) if path_points else 0.0,
            "path_length_m": float(plan.get("path_length_m", 0.0)),
            "coarse_path": [{"x_m": p["x_m"], "y_m": p["y_m"]} for p in path_points[:12]],
            "cautions": [caution_text] if caution_text else [],
            "method": plan.get("method"),
            "confidence": plan.get("confidence", "medium"),
            "freshness": ctx.map_runtime.freshness(),
        }
        return _success(tool, {"warnings": plan.get("warnings", []), "data": data})
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_compare_map_vs_live_scan(ctx: ToolContext, _args: dict) -> ToolResult:
    tool = "compare_map_vs_live_scan"
    try:
        _require_map_runtime(ctx, tool)
        payload = ctx.map_runtime.compare_map_vs_live_scan()
        return _success(tool, payload)
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_go_to_map_pose(ctx: ToolContext, args: dict) -> ToolResult:
    tool = "go_to_map_pose"
    try:
        _require_map_runtime(ctx, tool)
        x_m = float(args["x"])
        y_m = float(args["y"])
        yaw_rad = float(args.get("yaw_rad", args.get("theta", 0.0)))
        nav = ctx.map_runtime.navigate_to_pose(
            {"x_m": x_m, "y_m": y_m, "yaw_rad": yaw_rad},
            timeout_s=float(args.get("timeout_s", 3.0)),
        )
        data = {
            "goal_pose": {"x_m": x_m, "y_m": y_m, "yaw_rad": yaw_rad},
            "frame_id": ctx.map_runtime.map_frame,
            "status": nav.get("status"),
            "navigation": nav,
            "freshness": ctx.map_runtime.freshness(),
        }
        return _success(tool, {"warnings": [], "data": data})
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_go_to_waypoint(ctx: ToolContext, args: dict) -> ToolResult:
    tool = "go_to_waypoint"
    try:
        _require_map_runtime(ctx, tool)
        _require_waypoints(ctx, tool)
        record = _require_matching_waypoint(ctx, str(args["name"]))
        nav = ctx.map_runtime.navigate_to_pose(
            record.pose,
            timeout_s=float(args.get("timeout_s", 3.0)),
        )
        data = {
            "name": record.name,
            "goal_pose": dict(record.pose),
            "frame_id": record.frame_id,
            "status": nav.get("status"),
            "navigation": nav,
            "freshness": ctx.map_runtime.freshness(),
        }
        return _success(tool, {"warnings": [], "data": data})
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_get_navigation_status(ctx: ToolContext, _args: dict) -> ToolResult:
    tool = "get_navigation_status"
    try:
        _require_map_runtime(ctx, tool)
        status = ctx.map_runtime.get_navigation_status()
        return _success(tool, {"warnings": [], "data": status})
    except Exception as exc:
        return _failure(tool, str(exc))


def handle_cancel_navigation(ctx: ToolContext, args: dict) -> ToolResult:
    tool = "cancel_navigation"
    try:
        _require_map_runtime(ctx, tool)
        status = ctx.map_runtime.cancel_navigation(reason=str(args.get("reason", "stop")))
        return _success(tool, {"warnings": [], "data": status})
    except Exception as exc:
        return _failure(tool, str(exc))


def _record_to_dict(record) -> dict:
    return {
        "name": record.name,
        "type": record.type,
        "pose": dict(record.pose),
        "frame_id": record.frame_id,
        "base_frame": record.base_frame,
        "created_at": record.created_at,
        "map_signature": record.map_signature,
    }


def _require_matching_waypoint(ctx: ToolContext, name: str):
    record = ctx.waypoints.get(name)
    if record is None:
        raise RuntimeError(f"waypoint {name!r} not found")
    current_signature = ctx.map_runtime.get_map_signature()
    if record.map_signature != current_signature:
        raise RuntimeError(f"waypoint {name!r} belongs to a different map")
    return record
