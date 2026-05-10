import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from memory.store import MemoryStore
from robot.map_runtime import MapRuntime, Nav2PathClient
from safety.supervisor import SafetySupervisor
from tools.base import ToolContext
from tools.map_tools import (
    handle_check_waypoint_reachable,
    handle_compare_map_vs_live_scan,
    handle_get_local_map_context,
    handle_get_local_occupancy_grid,
    handle_get_map_summary,
    handle_get_robot_pose_in_map,
    handle_get_route_summary_to_waypoint,
    handle_get_waypoint,
    handle_list_waypoints,
    handle_save_waypoint,
)
from tools.waypoint_store import WaypointStore


def _quat(yaw: float) -> dict:
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}


def _tf(parent: str, child: str, x: float, y: float, yaw: float) -> dict:
    return {
        "transforms": [
            {
                "header": {"frame_id": parent, "stamp": {"sec": int(time.time()), "nanosec": 0}},
                "child_frame_id": child,
                "transform": {
                    "translation": {"x": x, "y": y, "z": 0.0},
                    "rotation": _quat(yaw),
                },
            }
        ]
    }


def _map_msg(width=20, height=20, resolution=0.5, origin=(-5.0, -5.0, 0.0), fill=0):
    ox, oy, oyaw = origin
    info = {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": {
            "position": {"x": ox, "y": oy, "z": 0.0},
            "orientation": _quat(oyaw),
        },
    }
    data = [fill for _ in range(width * height)]
    return {"info": info, "data": data}


def _metadata_msg(width=20, height=20, resolution=0.5, origin=(-5.0, -5.0, 0.0)):
    ox, oy, oyaw = origin
    return {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": {
            "position": {"x": ox, "y": oy, "z": 0.0},
            "orientation": _quat(oyaw),
        },
    }


def _set_cell(grid: dict, x: float, y: float, value: int) -> None:
    info = grid["info"]
    col = int((x - info["origin"]["position"]["x"]) / info["resolution"])
    row = int((y - info["origin"]["position"]["y"]) / info["resolution"])
    idx = row * info["width"] + col
    grid["data"][idx] = value


def _scan_msg(ranges, angle_min=-math.pi / 2, angle_increment=math.pi / 8, range_min=0.2, range_max=8.0):
    return {
        "angle_min": angle_min,
        "angle_increment": angle_increment,
        "range_min": range_min,
        "range_max": range_max,
        "ranges": ranges,
    }


def _ctx(runtime: MapRuntime) -> ToolContext:
    store = MemoryStore()
    return ToolContext(
        memory=store,
        robot=None,
        motion=None,
        follow=None,
        basic_goal=None,
        vlm=None,
        safety=SafetySupervisor(store),
        map_runtime=runtime,
        waypoints=WaypointStore(),
    )


def _make_runtime(with_nav2=False) -> MapRuntime:
    nav2 = None
    if with_nav2:
        nav2 = Nav2PathClient(
            None,
            injected_compute=lambda start, goal, timeout_s: {
                "path": {
                    "poses": [
                        {"pose": {"position": {"x": start["x_m"], "y": start["y_m"]}, "orientation": _quat(start["yaw_rad"])}},
                        {"pose": {"position": {"x": (start["x_m"] + goal["x_m"]) / 2.0, "y": start["y_m"]}, "orientation": _quat(0.0)}},
                        {"pose": {"position": {"x": goal["x_m"], "y": goal["y_m"]}, "orientation": _quat(goal["yaw_rad"])}},
                    ]
                }
            },
        )
    runtime = MapRuntime(base_frame="rslidar", nav2_client=nav2)
    static_map = _map_msg()
    local_costmap = _map_msg(width=20, height=20, resolution=0.5, origin=(-5.0, -5.0, 0.0), fill=0)
    global_costmap = _map_msg(width=20, height=20, resolution=0.5, origin=(-5.0, -5.0, 0.0), fill=0)
    _set_cell(local_costmap, 1.0, 0.0, 100)
    runtime.ingest_map(static_map)
    runtime.ingest_map_metadata(_metadata_msg())
    runtime.ingest_local_costmap(local_costmap)
    runtime.ingest_global_costmap(global_costmap)
    runtime.ingest_tf(_tf("map", "odom", 0.0, 0.0, 0.0))
    runtime.ingest_tf(_tf("odom", "rslidar", 1.0, 2.0, 0.25))
    runtime.ingest_scan(_scan_msg([1.0, 1.1, 1.2, 5.0, 5.0, 5.0, 5.0, 5.0], angle_min=-math.pi / 4, angle_increment=math.pi / 8))
    return runtime


def test_get_robot_pose_in_map_from_tf_chain():
    runtime = _make_runtime()
    pose = runtime.get_robot_pose_in_map()
    assert pose["frame_id"] == "map"
    assert abs(pose["pose"]["x_m"] - 1.0) < 1e-6
    assert abs(pose["pose"]["y_m"] - 2.0) < 1e-6
    assert abs(pose["pose"]["yaw_rad"] - 0.25) < 1e-6


def test_stale_scan_blocks_local_context():
    runtime = _make_runtime()
    runtime._topics["scan"].receipt_ts = time.time() - 2.0
    result = handle_get_local_map_context(_ctx(runtime), {})
    assert not result.ok
    assert "stale" in result.error.lower()


def test_map_summary_uses_metadata_and_signature():
    runtime = _make_runtime()
    result = handle_get_map_summary(_ctx(runtime), {})
    assert result.ok
    data = result.result["data"]
    assert data["map"]["width"] == 20
    assert data["map_signature"]


def test_local_map_context_reports_front_blocked():
    runtime = _make_runtime()
    result = handle_get_local_map_context(_ctx(runtime), {"radius_m": 3.0})
    assert result.ok
    assert "front_right" in result.result["data"]["blocked_directions"]


def test_local_occupancy_grid_is_bounded_and_contains_robot_cell():
    runtime = _make_runtime()
    result = handle_get_local_occupancy_grid(_ctx(runtime), {"size_m": 5.0, "cell_size_m": 0.25})
    assert result.ok
    data = result.result["data"]
    assert data["width"] == data["height"]
    center = data["robot_cell"]
    assert data["grid"][center["row"]][center["col"]] == 0
    assert data["legend"]["unknown"] == -1


def test_save_list_and_get_waypoint():
    runtime = _make_runtime()
    ctx = _ctx(runtime)
    saved = handle_save_waypoint(ctx, {"name": "desk", "type": "target"})
    assert saved.ok
    listed = handle_list_waypoints(ctx, {})
    assert listed.ok
    assert listed.result["data"]["count"] == 1
    fetched = handle_get_waypoint(ctx, {"name": "desk"})
    assert fetched.ok
    assert fetched.result["data"]["waypoint"]["type"] == "target"


def test_duplicate_waypoint_rejected():
    runtime = _make_runtime()
    ctx = _ctx(runtime)
    assert handle_save_waypoint(ctx, {"name": "desk"}).ok
    duplicate = handle_save_waypoint(ctx, {"name": "desk"})
    assert not duplicate.ok
    assert "already exists" in duplicate.error


def test_map_signature_mismatch_rejected_for_route_tools():
    runtime = _make_runtime(with_nav2=True)
    ctx = _ctx(runtime)
    assert handle_save_waypoint(ctx, {"name": "desk"}).ok
    other_map = _map_msg(origin=(-10.0, -10.0, 0.0))
    runtime.ingest_map(other_map)
    runtime.ingest_map_metadata(_metadata_msg(origin=(-10.0, -10.0, 0.0)))
    result = handle_check_waypoint_reachable(ctx, {"name": "desk"})
    assert not result.ok
    assert "different map" in result.error


def test_check_waypoint_reachable_uses_nav2_when_available():
    runtime = _make_runtime(with_nav2=True)
    ctx = _ctx(runtime)
    assert handle_save_waypoint(ctx, {"name": "desk"}).ok
    result = handle_check_waypoint_reachable(ctx, {"name": "desk"})
    assert result.ok
    assert result.result["data"]["method"] == "nav2_plan"
    assert result.result["data"]["reachable"] is True


def test_route_summary_returns_user_facing_text():
    runtime = _make_runtime(with_nav2=True)
    ctx = _ctx(runtime)
    assert handle_save_waypoint(ctx, {"name": "desk"}).ok
    result = handle_get_route_summary_to_waypoint(ctx, {"name": "desk"})
    assert result.ok
    assert "route" in result.result["data"]["summary_text"].lower()


def test_compare_map_vs_live_scan_flags_dynamic_obstacle():
    runtime = _make_runtime()
    runtime.ingest_scan(_scan_msg([1.0, 1.0, 1.1, 1.0, 5.0, 5.0, 5.0, 5.0], angle_min=-math.pi / 4, angle_increment=math.pi / 16))
    result = handle_compare_map_vs_live_scan(_ctx(runtime), {})
    assert result.ok
    assert result.result["data"]["possible_dynamic_obstacle"] is True
