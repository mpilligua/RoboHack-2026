from __future__ import annotations

from .base import ToolContext, ToolResult


def handle_get_visible_objects(ctx: ToolContext, args: dict) -> ToolResult:
    objects = [
        {
            "id": o.yolo_id,
            "label": o.label,
            "description": o.description,
            "position": o.position_text,
            "depth_m": o.depth_m,
            "seen_count": o.seen_count,
        }
        for o in ctx.memory.get_objects()
    ]
    return ToolResult(ok=True, tool="get_visible_objects", result={"objects": objects})


def handle_find_object(ctx: ToolContext, args: dict) -> ToolResult:
    label = args.get("label", "")
    if not label:
        return ToolResult(ok=False, tool="find_object", error="label is required")
    found = ctx.memory.find_objects_by_label(label)
    objects = [
        {"id": o.yolo_id, "label": o.label, "description": o.description, "position": o.position_text}
        for o in found
    ]
    return ToolResult(ok=True, tool="find_object", result={"objects": objects, "count": len(objects)})


def handle_resolve_reference(ctx: ToolContext, args: dict) -> ToolResult:
    ref = args.get("ref", "")
    if not ref:
        return ToolResult(ok=False, tool="resolve_reference", error="ref is required")
    obj = ctx.memory.resolve_reference(ref)
    if obj is None:
        return ToolResult(ok=False, tool="resolve_reference", error=f"could not resolve reference: {ref!r}")
    return ToolResult(
        ok=True,
        tool="resolve_reference",
        result={
            "id": obj.yolo_id,
            "label": obj.label,
            "description": obj.description,
            "position": obj.position_text,
            "depth_m": obj.depth_m,
        },
    )


def handle_find_objects_matching_constraints(ctx: ToolContext, args: dict) -> ToolResult:
    found = ctx.memory.find_objects_matching_constraints(args)
    objects = [
        {
            "id": o.yolo_id,
            "label": o.label,
            "description": o.description,
            "position": o.position_text,
            "depth_m": o.depth_m,
        }
        for o in found
    ]
    return ToolResult(ok=True, tool="find_objects_matching_constraints", result={"objects": objects, "count": len(objects)})
