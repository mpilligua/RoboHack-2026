from __future__ import annotations

import time

from memory.schemas import ObjectRecord

from .base import ToolContext, ToolResult


def handle_describe_scene(ctx: ToolContext, args: dict) -> ToolResult:
    focus = args.get("focus", "")
    try:
        jpeg = ctx.robot.rgb_jpeg_b64()
    except Exception as e:
        return ToolResult(ok=False, tool="describe_scene", error=f"camera error: {e}")
    prompt = (
        "You are the vision system of a guide-dog robot at knee height. "
        "Describe what you see in one or two sentences, covering objects, people, "
        "obstacles, and spatial layout."
    )
    if focus:
        prompt += f" Focus especially on: {focus}."
    try:
        description = ctx.vlm.describe(jpeg, prompt, max_tokens=300)
    except Exception as e:
        return ToolResult(ok=False, tool="describe_scene", error=f"VLM error: {e}")
    return ToolResult(ok=True, tool="describe_scene", result={"description": description})


def handle_read_label(ctx: ToolContext, args: dict) -> ToolResult:
    item_description = args.get("item_description", "the item")
    try:
        jpeg = ctx.robot.rgb_jpeg_b64()
    except Exception as e:
        return ToolResult(ok=False, tool="read_label", error=f"camera error: {e}")
    prompt = (
        f"Read all visible text on or near {item_description}. "
        "Return the text verbatim, then a one-sentence summary of what it says."
    )
    try:
        text = ctx.vlm.describe(jpeg, prompt, max_tokens=400)
    except Exception as e:
        return ToolResult(ok=False, tool="read_label", error=f"VLM error: {e}")
    return ToolResult(ok=True, tool="read_label", result={"text": text})


def handle_get_rgbd_summary(ctx: ToolContext, args: dict) -> ToolResult:
    try:
        summary = ctx.robot.depth_summary(timeout_s=3.0)
    except Exception as e:
        return ToolResult(ok=False, tool="get_rgbd_summary", error=f"depth error: {e}")
    ctx.memory.update_robot_state(
        depth_stamp=time.time(),
        depth_min_mm=summary.get("min_mm"),
        depth_center_mm=summary.get("center_mm"),
        nearest_obstacle_mm=summary.get("min_mm"),
    )
    return ToolResult(ok=True, tool="get_rgbd_summary", result=summary)


def handle_get_depth_at_pixel(ctx: ToolContext, args: dict) -> ToolResult:
    u = int(args["u"])
    v = int(args["v"])
    window_radius = int(args.get("window_radius", 3))
    try:
        result = ctx.robot.depth_at_rgb_pixel_naive(u, v, window_radius=window_radius)
    except Exception as e:
        return ToolResult(ok=False, tool="get_depth_at_pixel", error=f"depth error: {e}")
    return ToolResult(ok=True, tool="get_depth_at_pixel", result=result)


def handle_list_visible_objects(ctx: ToolContext, args: dict) -> ToolResult:
    if ctx.follow is None:
        return ToolResult(ok=False, tool="list_visible_objects", error="follow adapter not connected")

    dets = ctx.follow.get_detections()
    if not dets:
        return ToolResult(ok=True, tool="list_visible_objects", result={"objects": [], "note": "no detections"})

    try:
        jpeg = ctx.robot.rgb_jpeg_b64()
        rgb_frame = ctx.robot.get_rgb()
        width = rgb_frame.width
    except Exception as e:
        return ToolResult(ok=False, tool="list_visible_objects", error=f"camera error: {e}")

    bbox_list = ", ".join(
        f"id {d.id} (label {d.label!r}) at bbox {[int(v) for v in d.bbox]}"
        for d in dets
    )
    prompt = (
        f"This image is from a guide-dog robot at knee height. A YOLO tracker has detected "
        f"{len(dets)} objects. The detections are: {bbox_list}. The image is {width} pixels wide. "
        f"For each id, give a one-sentence description that includes the label (already shown above) "
        f"and any distinguishing detail — color, material, position (left / center / right), what it's near. "
        f'Return strict JSON: {{"objects": [{{"id": <int>, "label": "<coco>", "description": "..."}}]}}. '
        f"Use only the listed ids."
    )
    try:
        import json as _json
        raw = ctx.vlm.describe(jpeg, prompt, max_tokens=700)
        parsed = _json.loads(raw)
        objects = parsed.get("objects", [])
    except Exception:
        objects = [{"id": d.id, "label": d.label, "description": ""} for d in dets]

    det_map = {d.id: d for d in dets}
    now = time.time()
    for obj in objects:
        det = det_map.get(obj.get("id"))
        if det is None:
            continue
        bbox = [float(v) for v in det.bbox]
        cx = (bbox[0] + bbox[2]) / 2.0
        if width > 0:
            frac = cx / width
            position_text = "left" if frac < 0.33 else ("right" if frac > 0.66 else "center")
        else:
            position_text = "center"
        record = ObjectRecord(
            yolo_id=det.id,
            label=obj.get("label", det.label),
            description=obj.get("description", ""),
            bbox=bbox,
            confidence=getattr(det, "conf", None),
            position_text=position_text,
            last_seen_ts=now,
        )
        ctx.memory.upsert_object(record)

    return ToolResult(ok=True, tool="list_visible_objects", result={"objects": objects})
