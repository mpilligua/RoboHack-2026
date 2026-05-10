from __future__ import annotations

from .registry import CALLER_OPERATOR, CALLER_PLANNER, ToolRegistry
from . import basic_goal_tools, follow_tools, map_tools, memory_tools, motion_tools, perception_tools, safety_tools


def build_registry() -> ToolRegistry:
    reg = ToolRegistry()

    # Safety tools
    reg.register("stop", safety_tools.handle_stop, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("get_robot_status", safety_tools.handle_get_robot_status, [CALLER_PLANNER, CALLER_OPERATOR])

    # Map tools
    reg.register("get_robot_pose_in_map", map_tools.handle_get_robot_pose_in_map, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("get_map_summary", map_tools.handle_get_map_summary, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("get_local_map_context", map_tools.handle_get_local_map_context, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("get_local_occupancy_grid", map_tools.handle_get_local_occupancy_grid, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("save_waypoint", map_tools.handle_save_waypoint, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("list_waypoints", map_tools.handle_list_waypoints, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("get_waypoint", map_tools.handle_get_waypoint, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("check_waypoint_reachable", map_tools.handle_check_waypoint_reachable, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("get_route_summary_to_waypoint", map_tools.handle_get_route_summary_to_waypoint, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("compare_map_vs_live_scan", map_tools.handle_compare_map_vs_live_scan, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("go_to_map_pose", map_tools.handle_go_to_map_pose, [CALLER_PLANNER])
    reg.register("go_to_waypoint", map_tools.handle_go_to_waypoint, [CALLER_PLANNER])
    reg.register("get_navigation_status", map_tools.handle_get_navigation_status, [CALLER_PLANNER, CALLER_OPERATOR])
    reg.register("cancel_navigation", map_tools.handle_cancel_navigation, [CALLER_PLANNER, CALLER_OPERATOR])

    # Perception tools
    reg.register("describe_scene", perception_tools.handle_describe_scene, [CALLER_PLANNER])
    reg.register("read_label", perception_tools.handle_read_label, [CALLER_PLANNER])
    reg.register("get_rgbd_summary", perception_tools.handle_get_rgbd_summary, [CALLER_PLANNER])
    reg.register("get_depth_at_pixel", perception_tools.handle_get_depth_at_pixel, [CALLER_PLANNER])
    reg.register("list_visible_objects", perception_tools.handle_list_visible_objects, [CALLER_PLANNER])

    # Memory tools
    reg.register("get_visible_objects", memory_tools.handle_get_visible_objects, [CALLER_PLANNER])
    reg.register("resolve_reference", memory_tools.handle_resolve_reference, [CALLER_PLANNER])
    reg.register("find_objects_matching_constraints", memory_tools.handle_find_objects_matching_constraints, [CALLER_PLANNER])

    # Motion tools
    reg.register("walk_forward", motion_tools.handle_walk_forward, [CALLER_PLANNER])
    reg.register("walk_backward", motion_tools.handle_walk_backward, [CALLER_PLANNER])
    reg.register("turn_left", motion_tools.handle_turn_left, [CALLER_PLANNER])
    reg.register("turn_right", motion_tools.handle_turn_right, [CALLER_PLANNER])
    reg.register("stop_motion", motion_tools.handle_stop_motion, [CALLER_PLANNER, CALLER_OPERATOR])

    # Low-level command tools
    reg.register("send_simple_cmd", basic_goal_tools.handle_send_simple_cmd, [CALLER_PLANNER])
    reg.register("send_complex_cmd", basic_goal_tools.handle_send_complex_cmd, [CALLER_PLANNER])

    # Follow tools
    reg.register("follow_person", follow_tools.handle_follow_person, [CALLER_PLANNER])
    reg.register("go_to_object", follow_tools.handle_go_to_object, [CALLER_PLANNER])
    reg.register("find_and_go_to", follow_tools.handle_find_and_go_to, [CALLER_PLANNER])
    reg.register("find_object", follow_tools.handle_find_object, [CALLER_PLANNER])
    reg.register("find_person_and_follow", follow_tools.handle_find_person_and_follow, [CALLER_PLANNER])
    reg.register("stop_tracking", follow_tools.handle_stop_tracking, [CALLER_PLANNER, CALLER_OPERATOR])

    return reg
