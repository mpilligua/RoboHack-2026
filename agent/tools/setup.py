from __future__ import annotations

from .registry import CALLER_OPERATOR, CALLER_PLANNER, ToolRegistry
from . import basic_goal_tools, follow_tools, memory_tools, motion_tools, perception_tools, safety_tools


def build_registry() -> ToolRegistry:
    reg = ToolRegistry()

    # Safety tools
    reg.register("stop", safety_tools.handle_stop, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("get_robot_status", safety_tools.handle_get_robot_status, [CALLER_PLANNER, CALLER_OPERATOR])

    # Perception tools
    reg.register("describe_scene", perception_tools.handle_describe_scene, [CALLER_PLANNER])
    reg.register("read_label", perception_tools.handle_read_label, [CALLER_PLANNER])
    reg.register("get_rgbd_summary", perception_tools.handle_get_rgbd_summary, [CALLER_PLANNER])
    reg.register("get_depth_at_pixel", perception_tools.handle_get_depth_at_pixel, [CALLER_PLANNER])
    reg.register("list_visible_objects", perception_tools.handle_list_visible_objects, [CALLER_PLANNER])

    # Memory tools
    reg.register("get_visible_objects", memory_tools.handle_get_visible_objects, [CALLER_PLANNER])
    reg.register("find_object", memory_tools.handle_find_object, [CALLER_PLANNER])
    reg.register("resolve_reference", memory_tools.handle_resolve_reference, [CALLER_PLANNER])
    reg.register("find_objects_matching_constraints", memory_tools.handle_find_objects_matching_constraints, [CALLER_PLANNER])

    # Motion tools
    reg.register("walk_forward", motion_tools.handle_walk_forward, [CALLER_PLANNER])
    reg.register("walk_backward", motion_tools.handle_walk_backward, [CALLER_PLANNER])
    reg.register("turn_left", motion_tools.handle_turn_left, [CALLER_PLANNER])
    reg.register("turn_right", motion_tools.handle_turn_right, [CALLER_PLANNER])
    reg.register("stop_motion", motion_tools.handle_stop_motion, [CALLER_PLANNER, CALLER_OPERATOR])

    # Basic goal + low-level command tools
    reg.register(
        "send_basic_goal",
        basic_goal_tools.handle_send_basic_goal,
        [CALLER_PLANNER],
    )
    reg.register(
        "cancel_basic_goal",
        basic_goal_tools.handle_cancel_basic_goal,
        [CALLER_PLANNER, CALLER_OPERATOR],
    )
    reg.register(
        "get_basic_goal_status",
        basic_goal_tools.handle_get_basic_goal_status,
        [CALLER_PLANNER, CALLER_OPERATOR],
    )
    reg.register(
        "get_ros2_odom",
        basic_goal_tools.handle_get_ros2_odom,
        [CALLER_PLANNER, CALLER_OPERATOR],
    )
    reg.register("send_simple_cmd", basic_goal_tools.handle_send_simple_cmd, [CALLER_PLANNER])
    reg.register("send_complex_cmd", basic_goal_tools.handle_send_complex_cmd, [CALLER_PLANNER])

    # Follow tools
    reg.register("follow_person", follow_tools.handle_follow_person, [CALLER_PLANNER])
    reg.register("go_to_object", follow_tools.handle_go_to_object, [CALLER_PLANNER])
    reg.register("stop_tracking", follow_tools.handle_stop_tracking, [CALLER_PLANNER, CALLER_OPERATOR])

    return reg
