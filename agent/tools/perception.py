"""Tool definitions exposed to the LLM agent.

Schemas are in Bedrock Converse `toolSpec` shape. Each handler is a plain
Python callable that takes (robot, vlm_client, args) and returns a string.
"""

from __future__ import annotations

import json
from typing import Any

from robot import Lite3Follow, Lite3Motion, Lite3Robot
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
    _spec(
        "walk_forward",
        (
            "Walk the robot forward for a short duration. The robot must "
            "already be standing in walk mode with auto mode enabled. "
            "Always describe what you see first if there's any chance of "
            "obstacles ahead. Speed is clamped to 0.3 m/s, duration to 2 s."
        ),
        {
            "duration_s": {
                "type": "number",
                "description": "How long to walk, in seconds. Try 0.5 to 2.",
            },
            "speed": {
                "type": "number",
                "description": "Forward speed in m/s, default 0.15.",
            },
        },
        required=["duration_s"],
    ),
    _spec(
        "walk_backward",
        "Walk the robot backward for a short duration.",
        {
            "duration_s": {"type": "number"},
            "speed": {"type": "number", "description": "m/s, default 0.15."},
        },
        required=["duration_s"],
    ),
    _spec(
        "turn_left",
        "Rotate the robot in place to the left (counter-clockwise).",
        {
            "duration_s": {"type": "number"},
            "omega": {"type": "number", "description": "rad/s, default 0.4."},
        },
        required=["duration_s"],
    ),
    _spec(
        "turn_right",
        "Rotate the robot in place to the right (clockwise).",
        {
            "duration_s": {"type": "number"},
            "omega": {"type": "number", "description": "rad/s, default 0.4."},
        },
        required=["duration_s"],
    ),
    _spec(
        "stop_motion",
        "Immediately stop all motion. Use as a safety command.",
        {},
    ),
    _spec(
        "list_people",
        (
            "Look at the live tracker view and return a description of each "
            "detected person, keyed by their YOLO id. Use this when the user "
            "wants to start following someone — call this first to figure "
            "out which YOLO id matches the user's description (e.g. 'the "
            "woman in red'). The tracker must be running."
        ),
        {},
    ),
    _spec(
        "follow_person",
        (
            "Tell the tracker to follow a specific YOLO id. Get the id from "
            "list_people first. The robot must be standing in walk mode."
        ),
        {
            "yolo_id": {
                "type": "integer",
                "description": "The YOLO id of the person to follow.",
            }
        },
        required=["yolo_id"],
    ),
    _spec(
        "stop_following",
        "Tell the tracker to stop following anyone.",
        {},
    ),
]


def _describe_scene(robot, motion, follow, vlm, args: dict) -> str:
    focus = args.get("focus") or "the scene"
    prompt = (
        f"You are the perception system of an assistive guide-dog robot at "
        f"knee height. Briefly describe {focus} in the image. Be concrete: "
        f"name objects, count them, give rough left/center/right positions. "
        f"Two to four sentences."
    )
    jpeg = robot.rgb_jpeg_b64()
    return vlm_describe(vlm, jpeg, prompt)


def _read_label(robot, motion, follow, vlm, args: dict) -> str:
    item = args["item_description"]
    prompt = (
        f"Read every piece of text visible on '{item}' in this image. "
        f"Return the text verbatim, then a one-line summary of what the "
        f"product is. If you cannot find that item, say so explicitly."
    )
    jpeg = robot.rgb_jpeg_b64()
    return vlm_describe(vlm, jpeg, prompt)


def _get_rgbd_summary(robot, motion, follow, vlm, args: dict) -> str:
    return json.dumps(robot.depth_summary())


def _get_pose(robot, motion, follow, vlm, args: dict) -> str:
    pose = robot.get_pose()
    if pose is None:
        return json.dumps({"error": "no odometry yet"})
    return json.dumps(
        {"x": pose.x, "y": pose.y, "z": pose.z, "yaw_rad": pose.yaw}
    )


def _get_status(robot, motion, follow, vlm, args: dict) -> str:
    import time

    now = time.time()
    with robot._lock:  # noqa: SLF001 — small project, deliberate
        rgb_age = now - robot._rgb.stamp if robot._rgb else None
        depth_age = now - robot._depth.stamp if robot._depth else None
        pose = robot._pose
    return json.dumps(
        {
            "rosbridge_connected": robot._client.is_connected,
            "motion_connected": motion is not None and motion._client.is_connected,
            "follow_connected": follow is not None and follow._client.is_connected,
            "rgb_age_s": rgb_age,
            "depth_age_s": depth_age,
            "have_pose": pose is not None,
        }
    )


def _require_motion(motion):
    if motion is None:
        raise RuntimeError("motion adapter not connected — start the foxy rosbridge on port 9091")


def _walk_forward(robot, motion, follow, vlm, args: dict) -> str:
    _require_motion(motion)
    motion.forward(speed=args.get("speed", 0.15), duration_s=args["duration_s"])
    return json.dumps({"ok": True, "action": "walk_forward", "duration_s": args["duration_s"]})


def _walk_backward(robot, motion, follow, vlm, args: dict) -> str:
    _require_motion(motion)
    motion.backward(speed=args.get("speed", 0.15), duration_s=args["duration_s"])
    return json.dumps({"ok": True, "action": "walk_backward", "duration_s": args["duration_s"]})


def _turn_left(robot, motion, follow, vlm, args: dict) -> str:
    _require_motion(motion)
    motion.turn_left(omega=args.get("omega", 0.4), duration_s=args["duration_s"])
    return json.dumps({"ok": True, "action": "turn_left", "duration_s": args["duration_s"]})


def _turn_right(robot, motion, follow, vlm, args: dict) -> str:
    _require_motion(motion)
    motion.turn_right(omega=args.get("omega", 0.4), duration_s=args["duration_s"])
    return json.dumps({"ok": True, "action": "turn_right", "duration_s": args["duration_s"]})


def _stop_motion(robot, motion, follow, vlm, args: dict) -> str:
    _require_motion(motion)
    motion.stop()
    return json.dumps({"ok": True, "action": "stop"})


def _require_follow(follow):
    if follow is None:
        raise RuntimeError(
            "follow adapter not connected — start run_tracker.py on the robot"
        )


def _list_people(robot, motion, follow, vlm, args: dict) -> str:
    _require_follow(follow)
    dets = follow.get_detections()
    if not dets:
        return json.dumps({"people": [], "note": "no people currently detected"})

    # Use the regular RGB camera frame (same physical camera YOLO sees) plus
    # the YOLO bbox list. The VLM gets the raw image and the bboxes as text;
    # it correlates them spatially.
    jpeg = robot.rgb_jpeg_b64()
    bbox_list = ", ".join(
        f"id {d.id} at bbox {[int(v) for v in d.bbox]}" for d in dets
    )
    prompt = (
        f"This image is from a guide-dog robot at knee height. A YOLO "
        f"tracker has detected {len(dets)} people in this frame. Their "
        f"bounding boxes (x1, y1, x2, y2 in image pixels) and tracker "
        f"ids are: {bbox_list}. The image is {robot.get_rgb().width} "
        f"pixels wide. For each id, give a one-sentence description of "
        f"that specific person — clothes, colors, position (left / "
        f"center / right). Return strict JSON: {{\"people\": [{{\"id\": "
        f"<int>, \"description\": \"...\"}}]}}. Use only the listed ids."
    )
    raw = vlm_describe(vlm, jpeg, prompt, max_tokens=600)
    return json.dumps(
        {
            "yolo_ids": [d.id for d in dets],
            "vlm_descriptions_raw": raw,
            "note": (
                "Match the user's description to one of the yolo_ids above, "
                "then call follow_person with that id."
            ),
        }
    )


def _follow_person(robot, motion, follow, vlm, args: dict) -> str:
    _require_follow(follow)
    follow.follow(int(args["yolo_id"]))
    return json.dumps({"ok": True, "action": "follow_person", "yolo_id": args["yolo_id"]})


def _stop_following(robot, motion, follow, vlm, args: dict) -> str:
    _require_follow(follow)
    follow.stop()
    return json.dumps({"ok": True, "action": "stop_following"})


_HANDLERS = {
    "describe_scene": _describe_scene,
    "read_label": _read_label,
    "get_rgbd_summary": _get_rgbd_summary,
    "get_pose": _get_pose,
    "get_status": _get_status,
    "walk_forward": _walk_forward,
    "walk_backward": _walk_backward,
    "turn_left": _turn_left,
    "list_people": _list_people,
    "follow_person": _follow_person,
    "stop_following": _stop_following,
    "turn_right": _turn_right,
    "stop_motion": _stop_motion,
}


def dispatch(name: str, args: dict, robot, motion, follow, vlm) -> str:
    handler = _HANDLERS.get(name)
    if handler is None:
        return json.dumps({"error": f"unknown tool: {name}"})
    try:
        return handler(robot, motion, follow, vlm, args)
    except Exception as e:
        return json.dumps({"error": f"{type(e).__name__}: {e}"})
