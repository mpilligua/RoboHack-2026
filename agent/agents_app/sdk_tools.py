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
        "emergency_stop",
        "Trigger emergency stop and latch it. All motion is blocked until reset_emergency_stop is called.",
        {}, [],
    ),
    ("check_safety", "Check if it is safe to move forward. Returns safe=true/false and reason.", {}, []),
    (
        "get_robot_status",
        "Get robot connection status and depth summary. Refreshes safety supervisor depth data.",
        {}, [],
    ),
    (
        "describe_scene",
        "Capture the current camera frame and return a description of what the robot sees.",
        {"focus": {"type": "string", "description": "Optional hint, e.g. 'obstacles ahead', 'people'."}},
        [],
    ),
    (
        "read_label",
        "Capture the current frame and read all visible text on a specific item.",
        {"item_description": {"type": "string", "description": "Which item to read, e.g. 'the red can'."}},
        ["item_description"],
    ),
    (
        "get_rgbd_summary",
        "Return depth statistics (min, center, median, valid_fraction). "
        "Updates safety supervisor — call before walking forward.",
        {}, [],
    ),
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
        "List all YOLO-detected objects with VLM descriptions. Updates object memory. "
        "Always call before follow_person or go_to_object.",
        {}, [],
    ),
    ("get_visible_objects", "Return all objects currently in memory.", {}, []),
    (
        "find_object",
        "Find objects in memory by label (case-insensitive partial match).",
        {"label": {"type": "string", "description": "Label to search for, e.g. 'chair', 'person'."}},
        ["label"],
    ),
    (
        "resolve_reference",
        "Resolve 'it', 'that', 'the chair on the left' to a specific object in memory.",
        {"ref": {"type": "string", "description": "The reference string to resolve."}},
        ["ref"],
    ),
    (
        "find_objects_matching_constraints",
        "Filter objects in memory by label, position, and/or depth range.",
        {
            "label": {"type": "string"},
            "position": {"type": "string", "enum": ["left", "center", "right"]},
            "max_depth_m": {"type": "number"},
            "min_depth_m": {"type": "number"},
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
        "Approach a YOLO-tracked object and auto-stop when close. Call list_visible_objects first.",
        {
            "yolo_id": {"type": "integer"},
            "stop_distance_m": {"type": "number", "description": "Stop when depth <= this (default 0.8 m)."},
        },
        ["yolo_id"],
    ),
    ("stop_tracking", "Stop follow/go_to tracking.", {}, []),
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
