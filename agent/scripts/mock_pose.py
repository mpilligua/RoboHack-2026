"""Inspect or set the mock robot pose used by perception/external_pose.py.

Run from the agent/ directory:

    # show current mock pose
    python scripts/mock_pose.py

    # set pose
    python scripts/mock_pose.py 1.0 0.5 0.0 0.0
    python scripts/mock_pose.py --x 1.0 --y 0.5 --yaw 1.57

    # nudge from current pose
    python scripts/mock_pose.py --dx 0.5 --dyaw 0.78
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from perception.external_pose import _read_mock_pose, _write_mock_pose, Pose, _MOCK_POSE_PATH


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("positional", nargs="*", type=float, metavar=("X", "Y", "Z", "YAW"),
                   help="positional set: x y [z [yaw]]")
    p.add_argument("--x", type=float)
    p.add_argument("--y", type=float)
    p.add_argument("--z", type=float)
    p.add_argument("--yaw", type=float, help="yaw in radians")
    p.add_argument("--yaw-deg", type=float, help="yaw in degrees (converted to radians)")
    p.add_argument("--dx", type=float, default=0.0)
    p.add_argument("--dy", type=float, default=0.0)
    p.add_argument("--dz", type=float, default=0.0)
    p.add_argument("--dyaw", type=float, default=0.0)
    args = p.parse_args()

    cur = _read_mock_pose()
    print(f"current: x={cur.x:+.3f}  y={cur.y:+.3f}  z={cur.z:+.3f}  yaw={cur.yaw:+.3f}rad ({math.degrees(cur.yaw):+.1f}deg)")
    print(f"file:    {_MOCK_POSE_PATH}")

    new_x, new_y, new_z, new_yaw = cur.x, cur.y, cur.z, cur.yaw

    pos = list(args.positional)
    if pos:
        new_x = pos[0]
        if len(pos) >= 2: new_y = pos[1]
        if len(pos) >= 3: new_z = pos[2]
        if len(pos) >= 4: new_yaw = pos[3]

    if args.x is not None: new_x = args.x
    if args.y is not None: new_y = args.y
    if args.z is not None: new_z = args.z
    if args.yaw is not None: new_yaw = args.yaw
    if args.yaw_deg is not None: new_yaw = math.radians(args.yaw_deg)

    new_x += args.dx
    new_y += args.dy
    new_z += args.dz
    new_yaw += args.dyaw

    if (new_x, new_y, new_z, new_yaw) == (cur.x, cur.y, cur.z, cur.yaw):
        print("(no change)")
        return 0

    _write_mock_pose(Pose(new_x, new_y, new_z, new_yaw))
    print(f"new:     x={new_x:+.3f}  y={new_y:+.3f}  z={new_z:+.3f}  yaw={new_yaw:+.3f}rad ({math.degrees(new_yaw):+.1f}deg)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
