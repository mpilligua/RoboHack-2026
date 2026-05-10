"""External pose + nav functions, injected by you.

Plan B: instead of pulling pose from /leg_odom over rosbridge and dispatching
goals via /basic_goal, we just call two functions you provide.

CURRENT STATE: mock implementation for testing the world-map pipeline without
real navigation. Pose is read from a JSON file on disk so you can edit it
between agent turns to simulate robot motion. `goto` updates that same file
after a fake "walk" delay.

To swap in the real implementation: replace the function bodies of `get_pose`
and `goto` below with your real ones; the rest of the agent imports them
from this fixed path:

    from perception.external_pose import get_pose, goto

Contract:

    get_pose() -> Pose | None
        Return the robot's current pose in whatever world frame your nav uses.
        Called from world_tick (~1Hz). Should be cheap (<100 ms).
        Return None if temporarily unavailable.

    goto(x, y, z, *, timeout_s=30.0) -> GotoResult
        BLOCKING. Drive the robot to (x, y, z) in the same frame.
        Return when arrived, timed out, or blocked.
"""

from __future__ import annotations

import json
import math
import os
import time
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


# ---------------------------------------------------------------- mock state
# Mock pose stored at this path. Edit this file (or use the helper script
# scripts/set_pose.py) to simulate the robot moving while the agent runs.
_MOCK_POSE_PATH = os.environ.get(
    "MOCK_POSE_PATH",
    os.path.join(os.path.dirname(__file__), "..", ".mock_pose.json"),
)
_MOCK_POSE_PATH = os.path.abspath(_MOCK_POSE_PATH)

# Speed used when goto() simulates walking (meters per second).
_MOCK_WALK_SPEED = float(os.environ.get("MOCK_WALK_SPEED", "0.3"))


def _read_mock_pose() -> Pose:
    """Read the current mock pose; create the file with (0,0,0,0) if missing."""
    if not os.path.exists(_MOCK_POSE_PATH):
        _write_mock_pose(Pose(0.0, 0.0, 0.0, 0.0))
    with open(_MOCK_POSE_PATH) as f:
        d = json.load(f)
    return Pose(
        x=float(d.get("x", 0.0)),
        y=float(d.get("y", 0.0)),
        z=float(d.get("z", 0.0)),
        yaw=float(d.get("yaw", 0.0)),
    )


def _write_mock_pose(pose: Pose) -> None:
    tmp = _MOCK_POSE_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"x": pose.x, "y": pose.y, "z": pose.z, "yaw": pose.yaw}, f)
    os.replace(tmp, _MOCK_POSE_PATH)


# ============================================================================
# REAL CONTRACT — REPLACE WITH ACTUAL NAV WHEN READY
# ============================================================================


def get_pose() -> Optional[Pose]:
    """[MOCK] Read pose from .mock_pose.json. None if file is malformed."""
    try:
        return _read_mock_pose()
    except Exception:
        return None


def goto(x: float, y: float, z: float, *, timeout_s: float = 30.0) -> GotoResult:
    """[MOCK] Simulate walking from current pose to (x, y, z).

    Sleeps for `distance / MOCK_WALK_SPEED` seconds (capped by timeout_s),
    updates the mock pose to the goal, returns 'reached'. If the simulated
    walk would exceed timeout_s, returns 'timeout' at an interpolated point.
    """
    start = _read_mock_pose()
    dx, dy = x - start.x, y - start.y
    dist = math.hypot(dx, dy)
    if dist == 0:
        return GotoResult(status="reached", final_pose=start, detail="already there")

    walk_s = dist / _MOCK_WALK_SPEED
    yaw_to_target = math.atan2(dy, dx)

    if walk_s <= timeout_s:
        time.sleep(walk_s)
        final = Pose(x=x, y=y, z=z, yaw=yaw_to_target)
        _write_mock_pose(final)
        return GotoResult(
            status="reached", final_pose=final,
            detail=f"mock walked {dist:.2f}m in {walk_s:.1f}s",
        )

    # Truncated by timeout.
    time.sleep(timeout_s)
    frac = timeout_s / walk_s
    partial = Pose(
        x=start.x + dx * frac,
        y=start.y + dy * frac,
        z=z,
        yaw=yaw_to_target,
    )
    _write_mock_pose(partial)
    return GotoResult(
        status="timeout", final_pose=partial,
        detail=f"mock walked {dist*frac:.2f}m of {dist:.2f}m before timeout",
    )
