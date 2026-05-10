"""Bedrock-native tool schemas and dispatcher for the planner agent."""

from __future__ import annotations

from tools.base import ToolContext, ToolResult
from tools.registry import CALLER_PLANNER, ToolRegistry

# Neutral tool spec: (name, description, properties, required)
_SPECS = [
    ("stop", "Stop all robot motion and tracking immediately.", {}, []),
    (
        "get_robot_status",
        "Get robot connection status and depth summary.",
        {}, [],
    ),
    (
        "get_robot_pose_in_map",
        "Get the robot's current pose in the map frame from live TF. Use this before saving or reasoning about waypoints.",
        {},
        [],
    ),
    (
        "get_map_summary",
        "Summarize the static map, map freshness, and whether robot pose is available. Use this for broad map context, not for clearance.",
        {},
        [],
    ),
    (
        "get_local_map_context",
        "Describe nearby free, blocked, and unknown directions around the robot using live scan first, then local costmap. Prefer this over trusting the static map when discussing safe movement nearby.",
        {
            "radius_m": {"type": "number", "description": "Radius around the robot to inspect in meters (default 3.0)."},
        },
        [],
    ),
    (
        "get_local_occupancy_grid",
        "Return a small robot-centered symbolic occupancy grid fused from live scan, local costmap, and map fill-in. Use when you need compact spatial structure for reasoning.",
        {
            "size_m": {"type": "number", "description": "Width and height of the square local grid in meters (default 5.0)."},
            "cell_size_m": {"type": "number", "description": "Cell size in meters (default 0.25)."},
        },
        [],
    ),
    (
        "save_waypoint",
        "Save the robot's current pose as a named waypoint in the current session. Only use when the user explicitly wants to remember a place.",
        {
            "name": {"type": "string", "description": "Unique waypoint name."},
            "type": {"type": "string", "description": "Optional waypoint category (default 'generic')."},
        },
        ["name"],
    ),
    (
        "list_waypoints",
        "List all saved waypoints for this agent session.",
        {},
        [],
    ),
    (
        "get_waypoint",
        "Get one saved waypoint by name.",
        {
            "name": {"type": "string", "description": "Waypoint name."},
        },
        ["name"],
    ),
    (
        "check_waypoint_reachable",
        "Check whether a saved waypoint is likely reachable now. Prefer this for route analysis or debugging. Uses Nav2 planning when available and falls back to heuristics. Advisory only; does not move the robot.",
        {
            "name": {"type": "string", "description": "Waypoint name."},
        },
        ["name"],
    ),
    (
        "get_route_summary_to_waypoint",
        "Summarize the likely route to a saved waypoint for a blind user. Advisory only; does not move the robot. Use this when the user wants a route description, not as a required pre-check before Nav2 navigation.",
        {
            "name": {"type": "string", "description": "Waypoint name."},
        },
        ["name"],
    ),
    (
        "compare_map_vs_live_scan",
        "Compare the static map against the current live scan and costmaps. Use when you suspect dynamic obstacles or localization drift.",
        {},
        [],
    ),
    (
        "go_to_map_pose",
        "Navigate to an absolute map-frame pose using Nav2. Use this for requests like 'go to 0,0' or 'go to x y heading'. Trust Nav2 and local avoidance to handle obstacles during execution; do not add manual local obstacle pre-checks unless the user asked for analysis or a prior attempt failed.",
        {
            "x": {"type": "number", "description": "Goal x position in the map frame, meters."},
            "y": {"type": "number", "description": "Goal y position in the map frame, meters."},
            "yaw_rad": {"type": "number", "description": "Goal heading in radians. Defaults to 0.0."},
            "timeout_s": {"type": "number", "description": "Optional short dispatch timeout while sending the Nav2 goal."},
        },
        ["x", "y"],
    ),
    (
        "go_to_waypoint",
        "Navigate to a previously saved waypoint using Nav2. Prefer this over manually looking up the waypoint and then calling go_to_map_pose. Trust Nav2 and local avoidance during execution instead of requiring manual obstacle pre-checks.",
        {
            "name": {"type": "string", "description": "Waypoint name."},
            "timeout_s": {"type": "number", "description": "Optional short dispatch timeout while sending the Nav2 goal."},
        },
        ["name"],
    ),
    (
        "get_navigation_status",
        "Get the latest Nav2 navigation status for the active map-pose or waypoint goal.",
        {},
        [],
    ),
    (
        "cancel_navigation",
        "Cancel the active Nav2 navigation goal.",
        {"reason": {"type": "string", "description": "Optional cancellation reason."}},
        [],
    ),
    (
        "describe_scene",
        "Open-vocabulary scene narration via VLM. Use when the user asks about the situation, "
        "atmosphere, layout, or hazards that COCO/YOLO can't name (cables, puddles, signs, "
        "open boxes). Do NOT also call list_visible_objects in the same turn.",
        {"focus": {"type": "string", "description": "Optional hint, e.g. 'obstacles ahead', 'people'."}},
        [],
    ),
    (
        "read_label",
        "Capture the current frame and read all visible text on a specific item. Self-contained: "
        "do NOT call describe_scene or list_visible_objects first.",
        {"item_description": {"type": "string", "description": "Which item to read, e.g. 'the red can'."}},
        ["item_description"],
    ),
    ("get_rgbd_summary", "Return depth statistics (min, center, median, valid_fraction). "
        "walk_forward and go_to_object run their own depth check internally — do NOT call this "
        "as a manual safety pre-flight before motion.", {}, []),
    (
        "get_depth_at_pixel",
        "Get approximate depth at a specific RGB pixel (u, v).",
        {
            "u": {"type": "integer", "description": "Horizontal pixel coordinate."},
            "v": {"type": "integer", "description": "Vertical pixel coordinate."},
            "window_radius": {"type": "integer", "description": "Search window radius (default 3)."},
        },
        ["u", "v"],
    ),
    (
        "get_visible_objects",
        "Return recently seen objects from memory with ids, labels, descriptions, positions, "
        "depth, and recency. Cheap read-only memory lookup: prefer this before calling "
        "list_visible_objects when a recent object may already be known.",
        {},
        [],
    ),
    (
        "list_visible_objects",
        "YOLO-detected objects (COCO classes only) with per-detection VLM descriptions and "
        "yolo_ids you can pass to follow_person / go_to_object. Use when the user wants to "
        "act on a specific object and you need a fresh current id, or asks 'is there a <COCO "
        "thing> nearby'. Prefer get_visible_objects first when recent memory may already be "
        "enough. Do NOT also call describe_scene in the same turn.",
        {"label_filter": {"type": "string", "description": "Optional case-insensitive substring "
            "to filter by label, e.g. 'chair'. Omit to list everything."}},
        [],
    ),
    (
        "resolve_reference",
        "Resolve 'it', 'that', 'the chair on the left' to a specific remembered object. If it "
        "returns an id, you can pass that id straight to go_to_object or follow_person.",
        {"ref": {"type": "string", "description": "The reference string to resolve."}},
        ["ref"],
    ),
    (
        "find_objects_matching_constraints",
        "Filter remembered objects by constraints like label, position, or depth. Prefer this "
        "over fresh perception when the needed object may already be in memory.",
        {
            "label": {"type": "string", "description": "Optional case-insensitive label match."},
            "position": {"type": "string", "description": "Optional position hint: left, center, or right."},
            "max_depth_m": {"type": "number", "description": "Optional maximum depth in meters."},
            "min_depth_m": {"type": "number", "description": "Optional minimum depth in meters."},
        },
        [],
    ),
    (
        "walk_forward",
        "Walk forward. Specify distance_m (preferred) or duration_s. Runs its own depth-based "
        "safety check internally — do NOT call get_rgbd_summary or describe_scene first. "
        "Returns ok=false with a depth_check field if the path is blocked.",
        {
            "distance_m": {"type": "number", "description": "Distance to walk in meters."},
            "duration_s": {"type": "number", "description": "Alternatively, walk for this many seconds."},
            "speed": {"type": "number", "description": "Speed in m/s (default 0.15, max 0.3)."},
        },
        [],
    ),
    (
        "walk_backward",
        "Walk backward. Specify distance_m (preferred) or duration_s.",
        {
            "distance_m": {"type": "number", "description": "Distance to walk in meters."},
            "duration_s": {"type": "number", "description": "Alternatively, walk for this many seconds."},
            "speed": {"type": "number", "description": "Speed in m/s (default 0.15)."},
        },
        [],
    ),
    (
        "turn_left",
        "Rotate left (counter-clockwise). Specify angle_deg (preferred) or duration_s.",
        {
            "angle_deg": {"type": "number", "description": "Angle to rotate in degrees."},
            "duration_s": {"type": "number", "description": "Alternatively, rotate for this many seconds."},
            "omega": {"type": "number", "description": "Angular speed in rad/s (default 0.4)."},
        },
        [],
    ),
    (
        "turn_right",
        "Rotate right (clockwise). Specify angle_deg (preferred) or duration_s.",
        {
            "angle_deg": {"type": "number", "description": "Angle to rotate in degrees."},
            "duration_s": {"type": "number", "description": "Alternatively, rotate for this many seconds."},
            "omega": {"type": "number", "description": "Angular speed in rad/s (default 0.4)."},
        },
        [],
    ),
    ("stop_motion", "Stop all robot motion without stopping the tracker.", {}, []),
    (
        "send_simple_cmd",
        "Send a low-level MotionSimpleCMD message.",
        {
            "cmd_code": {"type": "integer"},
            "size": {"type": "integer"},
            "type": {"type": "integer"},
        },
        ["cmd_code", "size", "type"],
    ),
    (
        "send_complex_cmd",
        "Send a low-level MotionComplexCMD message.",
        {
            "cmd_code": {"type": "integer"},
            "size": {"type": "integer"},
            "type": {"type": "integer"},
            "data": {"type": "number"},
        },
        ["cmd_code", "size", "type", "data"],
    ),
    (
        "follow_person",
        "Continuously follow a YOLO-tracked target. If you already have a recent yolo_id from "
        "memory, resolve_reference, or find_person_and_follow, call this directly. Otherwise "
        "get or refresh an id first.",
        {"yolo_id": {"type": "integer", "description": "The YOLO detection id to follow."}},
        ["yolo_id"],
    ),
    (
        "go_to_object",
        "Approach a YOLO-tracked object and auto-stop when close. If you already have a recent "
        "yolo_id from memory, resolve_reference, or find_object, call this directly. Only "
        "refresh with list_visible_objects when you truly need a fresh id. Returns IMMEDIATELY "
        "after dispatching the goal; the robot "
        "stops itself when median depth in the target bbox <= stop_distance_m. Do NOT poll "
        "get_basic_goal_status afterwards — assume success unless an error follows.",
        {
            "yolo_id": {"type": "integer"},
            "stop_distance_m": {"type": "number", "description": "Stop when depth <= this (default 0.8 m)."},
        },
        ["yolo_id"],
    ),
    (
        "find_and_go_to",
        "Locate an object by label and walk to it, even if it isn't currently visible. "
        "Rotates the robot in chunks (default 45° at a time, sweeping up to 360°), sampling "
        "YOLO between chunks; stops as soon as a matching object appears, then auto-walks to "
        "it (same auto-stop behavior as go_to_object). USE THIS instead of "
        "list_visible_objects + go_to_object when the user asks to find/go to an object that "
        "may not be in the current view (e.g. 'find the door and walk to it', 'go find a "
        "chair'). Returns ok=false if a full 360° sweep finds nothing.",
        {
            "label": {"type": "string", "description": "COCO label to search for, e.g. 'door', 'chair', 'person'."},
            "direction": {"type": "string", "enum": ["left", "right"], "description": "Sweep direction (default 'right')."},
            "stop_distance_m": {"type": "number", "description": "Stop when depth <= this (default 0.8 m)."},
            "max_sweep_deg": {"type": "number", "description": "Cap on total rotation (default 360)."},
        },
        ["label"],
    ),
    (
        "find_object",
        "Locate an object by label WITHOUT walking to it. Rotates the robot in chunks "
        "(default 45°, sweeping up to 360°) sampling YOLO between chunks; stops at the first "
        "match. Returns the yolo_id (the robot ends facing the target so a follow-up "
        "go_to_object / follow_person works without re-searching). USE THIS when the user "
        "asks 'where is X' or 'can you find X' WITHOUT a 'go to' / 'walk to' intent. "
        "Returns ok=false if nothing matches after the sweep.",
        {
            "label": {"type": "string", "description": "COCO label to search for."},
            "direction": {"type": "string", "enum": ["left", "right"], "description": "Sweep direction (default 'right')."},
            "max_sweep_deg": {"type": "number", "description": "Cap on total rotation (default 360)."},
        },
        ["label"],
    ),
    (
        "find_person_and_follow",
        "Locate a person and start continuously following them, even if they aren't in the "
        "current view. Same rotation-sampling search as find_and_go_to, but engages "
        "follow_person (continuous tracking) on the first match instead of one-shot "
        "approach. USE THIS when the user says 'find someone and follow them', 'look around "
        "for a person and follow', etc. Defaults label='person'.",
        {
            "label": {"type": "string", "description": "COCO label to search for (default 'person')."},
            "direction": {"type": "string", "enum": ["left", "right"], "description": "Sweep direction (default 'right')."},
            "max_sweep_deg": {"type": "number", "description": "Cap on total rotation (default 360)."},
        },
        [],
    ),
    ("stop_tracking", "Stop follow/go_to tracking.", {}, []),
]


def _make_bedrock_tool(name, description, properties, required):
    schema: dict = {"type": "object", "properties": properties}
    if required:
        schema["required"] = required
    return {"toolSpec": {"name": name, "description": description, "inputSchema": {"json": schema}}}



# Bedrock-native toolSpec format (default — works with boto3)
PLANNER_TOOLS_BEDROCK: list[dict] = [_make_bedrock_tool(*s) for s in _SPECS]


def dispatch_tool(
    name: str,
    args: dict,
    registry: ToolRegistry,
    ctx: ToolContext,
    caller: str = CALLER_PLANNER,
) -> str:
    result: ToolResult = registry.call(name, args, ctx, caller)
    return result.to_str()
