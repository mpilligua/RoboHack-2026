"""Tool schemas and dispatcher for the planner agent.

Provides both Bedrock-native (toolSpec) and OpenAI-compatible formats.
The boto3 path (PLANNER_TOOLS_BEDROCK) is the default.
The openai path (PLANNER_TOOLS) is used when AGENT_BASE_URL is set.
"""

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
        "list_visible_objects",
        "YOLO-detected objects (COCO classes only) with per-detection VLM descriptions and "
        "yolo_ids you can pass to follow_person / go_to_object. Use when the user wants to "
        "act on a specific object, or asks 'is there a <COCO thing> nearby'. Do NOT also call "
        "describe_scene in the same turn.",
        {"label_filter": {"type": "string", "description": "Optional case-insensitive substring "
            "to filter by label, e.g. 'chair'. Omit to list everything."}},
        [],
    ),
    (
        "resolve_reference",
        "Resolve 'it', 'that', 'the chair on the left' to a specific object in memory.",
        {"ref": {"type": "string", "description": "The reference string to resolve."}},
        ["ref"],
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
        "send_basic_goal",
        "Send an absolute 2D goal (x, y, theta) to the ROS2 basic_goal_controller.",
        {
            "x": {"type": "number", "description": "Goal x position (meters)."},
            "y": {"type": "number", "description": "Goal y position (meters)."},
            "theta": {"type": "number", "description": "Goal heading in radians."},
        },
        ["x", "y", "theta"],
    ),
    (
        "cancel_basic_goal",
        "Cancel the current basic goal.",
        {"reason": {"type": "string", "description": "Optional cancel message."}},
        [],
    ),
    (
        "get_basic_goal_status",
        "Get the latest basic goal status; can optionally wait for a target status.",
        {
            "wait_for": {"type": "string", "description": "Target status to wait for."},
            "timeout_s": {"type": "number", "description": "Max seconds to wait."},
        },
        [],
    ),
    ("get_ros2_odom", "Return latest ROS2 odometry pose (x, y, z, yaw).", {}, []),
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
        "Continuously follow a YOLO-tracked target. Call list_visible_objects first.",
        {"yolo_id": {"type": "integer", "description": "The YOLO detection id to follow."}},
        ["yolo_id"],
    ),
    (
        "go_to_object",
        "Approach a YOLO-tracked object and auto-stop when close. Call list_visible_objects "
        "first to get a yolo_id. Returns IMMEDIATELY after dispatching the goal; the robot "
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
    # World map (persistent, populated by background world-tick thread):
    (
        "list_world_objects",
        "List remembered objects with their world positions (x_odom, y_odom in /leg_odom "
        "frame). Use this when the user asks 'what objects do you remember', 'where are the "
        "things you've seen', or before navigating to a remembered object. Unlike "
        "list_visible_objects, returns objects EVEN IF NOT CURRENTLY IN VIEW, with their "
        "world coordinates so you can navigate to them.",
        {"max_age_s": {"type": "number", "description": "Drop entries older than this (seconds)."}},
        [],
    ),
    (
        "find_object_in_world",
        "Look up remembered objects matching a label substring. Returns world coordinates "
        "(x_odom, y_odom) for each match. Faster than find_object: no rotating, just "
        "memory lookup. Use when the user asks 'where is the chair' and you already have "
        "the object in memory (check list_world_objects first if unsure).",
        {"label": {"type": "string", "description": "Label substring, case-insensitive (e.g. 'chair')."}},
        ["label"],
    ),
    (
        "go_to_world_object",
        "Walk to a remembered object using its stored world position (no need to see it "
        "currently). Looks up the most-recently-seen object matching `label` in the world "
        "map and sends a basic_goal in /leg_odom frame. USE THIS for 'go to the chair you "
        "saw earlier', 'come back to the table'. Returns ok=false if no remembered match.",
        {
            "label": {"type": "string", "description": "Label substring (e.g. 'chair')."},
            "stop_distance_m": {
                "type": "number",
                "description": "Stop short of the object by this much (default 0.8 m).",
            },
        },
        ["label"],
    ),
]


def _make_openai_tool(name, description, properties, required):
    params = {"type": "object", "properties": properties}
    if required:
        params["required"] = required
    return {"type": "function", "function": {"name": name, "description": description, "parameters": params}}


def _make_bedrock_tool(name, description, properties, required):
    schema: dict = {"type": "object", "properties": properties}
    if required:
        schema["required"] = required
    return {"toolSpec": {"name": name, "description": description, "inputSchema": {"json": schema}}}


# OpenAI-compatible format (used when AGENT_BASE_URL is set)
PLANNER_TOOLS: list[dict] = [_make_openai_tool(*s) for s in _SPECS]

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
