"""Tool definitions exposed to the LLM agent.

Schemas are in Bedrock Converse `toolSpec` shape. Each handler is a plain
Python callable that takes (robot, vlm_client, args) and returns a string.
"""

from __future__ import annotations

import json
from typing import Any

from robot import Lite3Robot
from vlm import vlm_describe


def _spec(name: str, description: str, properties: dict, required: list[str] | None = None) -> dict:
    schema: dict[str, Any] = {"type": "object", "properties": properties}
    if required:
        schema["required"] = required
    return {
        "toolSpec": {
            "name": name,
            "description": description,
            "inputSchema": {"json": schema},
        }
    }


TOOL_SCHEMAS: list[dict[str, Any]] = [
    _spec(
        "describe_scene",
        (
            "Capture the current RGB frame from the robot's front camera and "
            "return a free-form description of what is visible. Use when the "
            "user asks what the robot sees, or when you need scene context."
        ),
        {
            "focus": {
                "type": "string",
                "description": (
                    "Optional hint to bias the description, e.g. 'objects on "
                    "the table', 'obstacles ahead', 'people'."
                ),
            }
        },
    ),
    _spec(
        "read_label",
        (
            "Capture the current RGB frame and read all visible text on a "
            "specific item. Use when the user asks about ingredients, "
            "expiration dates, or any printed label."
        ),
        {
            "item_description": {
                "type": "string",
                "description": (
                    "Which item on screen to read, e.g. 'the red can on the "
                    "left' or 'the granola box'."
                ),
            }
        },
        required=["item_description"],
    ),
    _spec(
        "get_rgbd_summary",
        (
            "Return depth statistics from the front RGBD camera: closest "
            "valid distance, center-pixel distance, median, and fraction of "
            "valid depth. Use to check whether the path ahead is clear or "
            "how far an object is."
        ),
        {},
    ),
    _spec(
        "get_pose",
        (
            "Return the robot's current pose from leg odometry: x, y, z in "
            "meters and yaw in radians."
        ),
        {},
    ),
    _spec(
        "get_status",
        (
            "Return a small status blob: rosbridge connection, latest frame "
            "ages, latest pose."
        ),
        {},
    ),
]


def _describe_scene(robot: Lite3Robot, vlm, args: dict) -> str:
    focus = args.get("focus") or "the scene"
    prompt = (
        f"You are the perception system of an assistive guide-dog robot at "
        f"knee height. Briefly describe {focus} in the image. Be concrete: "
        f"name objects, count them, give rough left/center/right positions. "
        f"Two to four sentences."
    )
    jpeg = robot.rgb_jpeg_b64()
    return vlm_describe(vlm, jpeg, prompt)


def _read_label(robot: Lite3Robot, vlm, args: dict) -> str:
    item = args["item_description"]
    prompt = (
        f"Read every piece of text visible on '{item}' in this image. "
        f"Return the text verbatim, then a one-line summary of what the "
        f"product is. If you cannot find that item, say so explicitly."
    )
    jpeg = robot.rgb_jpeg_b64()
    return vlm_describe(vlm, jpeg, prompt)


def _get_rgbd_summary(robot: Lite3Robot, vlm, args: dict) -> str:
    return json.dumps(robot.depth_summary())


def _get_pose(robot: Lite3Robot, vlm, args: dict) -> str:
    pose = robot.get_pose()
    if pose is None:
        return json.dumps({"error": "no odometry yet"})
    return json.dumps(
        {"x": pose.x, "y": pose.y, "z": pose.z, "yaw_rad": pose.yaw}
    )


def _get_status(robot: Lite3Robot, vlm, args: dict) -> str:
    import time

    now = time.time()
    with robot._lock:  # noqa: SLF001 — small project, deliberate
        rgb_age = now - robot._rgb.stamp if robot._rgb else None
        depth_age = now - robot._depth.stamp if robot._depth else None
        pose = robot._pose
    return json.dumps(
        {
            "rosbridge_connected": robot._client.is_connected,
            "rgb_age_s": rgb_age,
            "depth_age_s": depth_age,
            "have_pose": pose is not None,
        }
    )


_HANDLERS = {
    "describe_scene": _describe_scene,
    "read_label": _read_label,
    "get_rgbd_summary": _get_rgbd_summary,
    "get_pose": _get_pose,
    "get_status": _get_status,
}


def dispatch(name: str, args: dict, robot: Lite3Robot, vlm) -> str:
    handler = _HANDLERS.get(name)
    if handler is None:
        return json.dumps({"error": f"unknown tool: {name}"})
    try:
        return handler(robot, vlm, args)
    except Exception as e:
        return json.dumps({"error": f"{type(e).__name__}: {e}"})
