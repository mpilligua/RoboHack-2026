"""External pose + nav functions, used by world_tick + go_to_world_object.

Wires MapRuntime (TF-based pose lookup over rosbridge) and its Nav2 client
through a module-level singleton. cli.py calls `set_map_runtime(map_runtime)`
once at startup; everything else just imports `get_pose` and `goto`.

If MapRuntime is unavailable (e.g. no rosbridge), get_pose returns None and
goto returns GotoResult(status='error').
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class Pose:
    x: float
    y: float
    z: float
    yaw: float  # radians; 0 = facing +x


@dataclass
class GotoResult:
    status: str       # "reached" | "timeout" | "blocked" | "error"
    final_pose: Optional[Pose] = None
    detail: Optional[str] = None


# Set by cli.py / voice_server.py once MapRuntime is constructed.
_MAP_RUNTIME = None


def set_map_runtime(map_runtime) -> None:
    """Inject the MapRuntime instance. Call once at startup."""
    global _MAP_RUNTIME
    _MAP_RUNTIME = map_runtime


def get_pose() -> Optional[Pose]:
    """Read the robot's pose in the map frame via MapRuntime's TF cache.

    MapRuntime rejects TF older than 0.5s. On a saturated Jetson TF often
    lags 0.5-2s, so we also fall back to the raw cached transform when the
    high-level lookup raises a "tf is stale" error.
    """
    mr = _MAP_RUNTIME
    if mr is None:
        return None
    try:
        d = mr.get_robot_pose_in_map()
        p = d.get("pose") or {}
        return Pose(
            x=float(p.get("x_m", 0.0)),
            y=float(p.get("y_m", 0.0)),
            z=0.0,
            yaw=float(p.get("yaw_rad", 0.0)),
        )
    except Exception as e:
        # Fallback: accept stale TF up to ~3s. Better drifty pose than no pose
        # for the world-map use case (we just need rough world coords).
        if "stale" not in str(e).lower():
            return None
        try:
            tf = mr._lookup_transform(mr.map_frame, mr.base_frame)
        except Exception:
            return None
        if tf.age_s() > 3.0:
            return None
        return Pose(x=float(tf.x), y=float(tf.y), z=0.0, yaw=float(tf.yaw))


def goto(x: float, y: float, z: float, *, timeout_s: float = 30.0) -> GotoResult:
    """Publish a Nav2 goal to /goal_pose and poll pose until arrival or timeout.

    Why /goal_pose instead of the NavigateToPose action: roslibpy 2.0 dropped
    the actionlib subpackage, so MapRuntime.navigate_to_pose() is unusable
    over rosbridge. /goal_pose is a plain topic that Nav2's bt_navigator
    subscribes to for the same purpose.

    Arrival detection: we poll get_pose() and call it 'reached' when within
    `arrival_radius_m` of the goal. Yaw is ignored (z too — 2D nav).
    """
    import math
    import time as _time

    mr = _MAP_RUNTIME
    if mr is None:
        return GotoResult(status="error", detail="MapRuntime not initialized")

    ros_client = getattr(mr, "_ros_client", None)
    if ros_client is None:
        return GotoResult(status="error", detail="MapRuntime has no ros_client")

    arrival_radius_m = 0.15
    poll_period_s = 0.3

    # Build PoseStamped in the map frame.
    yaw = 0.0  # we don't constrain final yaw; Nav2 will pick something reasonable
    half = yaw / 2.0
    msg = {
        "header": {"frame_id": getattr(mr, "map_frame", "map")},
        "pose": {
            "position": {"x": float(x), "y": float(y), "z": 0.0},
            "orientation": {
                "x": 0.0, "y": 0.0,
                "z": math.sin(half), "w": math.cos(half),
            },
        },
    }

    # Publish via roslibpy directly. Advertise once, publish, leave the topic
    # open (topic.advertise is idempotent enough — calling it twice is fine).
    try:
        import roslibpy
        topic = roslibpy.Topic(ros_client, "/goal_pose", "geometry_msgs/PoseStamped")
        topic.advertise()
        topic.publish(roslibpy.Message(msg))
    except Exception as e:
        return GotoResult(status="error", detail=f"/goal_pose publish failed: {e}")

    # If we're already within the arrival radius at dispatch time, don't claim
    # success — that almost always means the goal was a no-op or the arrival
    # threshold is too loose. Surface it explicitly so the caller can decide.
    p0 = get_pose()
    if p0 is not None:
        d0 = math.hypot(p0.x - x, p0.y - y)
        if d0 <= arrival_radius_m:
            return GotoResult(
                status="reached", final_pose=p0,
                detail=f"already within {arrival_radius_m}m of goal at dispatch (no motion)",
            )

    deadline = _time.monotonic() + timeout_s
    moved = False
    start_pose = p0
    while _time.monotonic() < deadline:
        p = get_pose()
        if p is not None:
            dx, dy = p.x - x, p.y - y
            if math.hypot(dx, dy) <= arrival_radius_m:
                return GotoResult(status="reached", final_pose=p,
                                  detail=f"within {arrival_radius_m}m of goal")
            if not moved and start_pose is not None:
                if math.hypot(p.x - start_pose.x, p.y - start_pose.y) > 0.05:
                    moved = True
        _time.sleep(poll_period_s)

    final = get_pose()
    detail = f"did not reach within {arrival_radius_m}m in {timeout_s}s"
    if not moved:
        detail += " (robot never moved >5cm — Nav2 may not have accepted the goal)"
    return GotoResult(status="timeout", final_pose=final, detail=detail)
