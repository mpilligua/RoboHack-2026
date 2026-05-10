"""Isolated test for perception/external_pose.goto().

No agent, no world_tick, no LLM. Just: connect to ROS 2 bridge, build
MapRuntime, inject it, read pose, dispatch a goal, watch what happens.

Usage:
    cd /Users/maria/Desktop/RoboHack/agent
    source .venv/bin/activate

    # read current pose only (no motion)
    python scripts/test_goto.py --pose-only

    # send a tiny relative goal: 0.5m forward, no rotation
    python scripts/test_goto.py --dx 0.5

    # absolute world goal
    python scripts/test_goto.py --x -2.33 --y 2.15

    # try a different goto backend (see external_pose.goto for options)
    python scripts/test_goto.py --dx 0.5 --backend basic_goal
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

from dotenv import load_dotenv

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot import MapRuntime, connect_ros2_rosbridge  # noqa: E402
from perception.external_pose import (  # noqa: E402
    GotoResult, Pose, get_pose, goto, set_map_runtime,
)


def main() -> int:
    load_dotenv()
    p = argparse.ArgumentParser()
    p.add_argument("--host", default=os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103"))
    p.add_argument("--port", type=int, default=int(os.environ.get("ROS2_BRIDGE_PORT", "9091")))
    p.add_argument("--x", type=float, help="absolute goal x (map frame)")
    p.add_argument("--y", type=float, help="absolute goal y (map frame)")
    p.add_argument("--dx", type=float, default=0.0, help="relative goal x (added to current pose)")
    p.add_argument("--dy", type=float, default=0.0, help="relative goal y (added to current pose)")
    p.add_argument("--timeout", type=float, default=20.0)
    p.add_argument("--pose-only", action="store_true", help="just read pose, don't move")
    p.add_argument("--watch-pose", type=float, metavar="SECONDS",
                   help="poll get_pose() this many seconds and print each sample")
    args = p.parse_args()

    print(f"connecting to ws://{args.host}:{args.port} ...", file=sys.stderr)
    t0 = time.time()
    client = connect_ros2_rosbridge(args.host, args.port)
    print(f"  connected in {time.time() - t0:.2f}s", file=sys.stderr)

    print("building MapRuntime ...", file=sys.stderr)
    map_runtime = MapRuntime(
        ros_client=client,
        base_frame=os.environ.get("ROS2_BASE_FRAME", "rslidar"),
        map_frame=os.environ.get("ROS2_MAP_FRAME", "map"),
        odom_frame=os.environ.get("ROS2_ODOM_FRAME", "odom"),
    )
    set_map_runtime(map_runtime)

    # Wait for first TF + map to arrive (otherwise pose lookup throws "missing transform")
    print("waiting up to 5s for TF cache to populate ...", file=sys.stderr)
    deadline = time.time() + 5.0
    pose = None
    while time.time() < deadline:
        pose = get_pose()
        if pose is not None:
            break
        time.sleep(0.2)
    if pose is None:
        print("ERROR: get_pose() returned None after 5s. SLAM may not be publishing map -> base_link.",
              file=sys.stderr)
        try:
            print("  freshness:", map_runtime.freshness(), file=sys.stderr)
        except Exception as e:
            print(f"  freshness check failed: {e}", file=sys.stderr)
        return 2
    print(f"  got pose: x={pose.x:+.3f}  y={pose.y:+.3f}  yaw={pose.yaw:+.3f}rad ({math.degrees(pose.yaw):+.1f}deg)",
          file=sys.stderr)

    if args.watch_pose:
        end = time.time() + args.watch_pose
        while time.time() < end:
            p2 = get_pose()
            if p2 is None:
                print("  pose=None")
            else:
                print(f"  pose: x={p2.x:+.3f}  y={p2.y:+.3f}  yaw={math.degrees(p2.yaw):+.1f}deg")
            time.sleep(0.5)
        return 0

    if args.pose_only:
        return 0

    if args.x is not None and args.y is not None:
        gx, gy = args.x, args.y
        kind = "absolute"
    else:
        gx, gy = pose.x + args.dx, pose.y + args.dy
        kind = "relative"

    print(f"\ndispatching {kind} goto: x={gx:+.3f} y={gy:+.3f} (timeout={args.timeout}s) ...",
          file=sys.stderr)
    t0 = time.time()
    result = goto(gx, gy, 0.0, timeout_s=args.timeout)
    dt = time.time() - t0
    print(f"\n  goto returned after {dt:.2f}s:")
    print(f"    status={result.status}")
    print(f"    detail={result.detail}")
    if result.final_pose:
        fp = result.final_pose
        print(f"    final_pose: x={fp.x:+.3f}  y={fp.y:+.3f}  yaw={math.degrees(fp.yaw):+.1f}deg")
    after = get_pose()
    if after:
        print(f"    pose now:   x={after.x:+.3f}  y={after.y:+.3f}  yaw={math.degrees(after.yaw):+.1f}deg")

    return 0 if result.status == "reached" else 1


if __name__ == "__main__":
    sys.exit(main())
