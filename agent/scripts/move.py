"""Manual motion test — send one short command to the dog.

Usage:
    python scripts/move.py forward [duration_s] [speed]
    python scripts/move.py backward [duration_s] [speed]
    python scripts/move.py left [duration_s] [omega_rad_s]   # turn in place
    python scripts/move.py right [duration_s] [omega_rad_s]
    python scripts/move.py strafe_left [duration_s] [speed]
    python scripts/move.py strafe_right [duration_s] [speed]
    python scripts/move.py stop

Defaults: duration 0.5 s, linear 0.15 m/s, angular 0.4 rad/s.

Safety:
    HOLD THE HARNESS the first time you run this. The script clamps speeds and
    durations to MAX_LINEAR / MAX_ANGULAR / MAX_DURATION in robot/motion.py.
"""

from __future__ import annotations

import os
import sys
import time

from dotenv import load_dotenv

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot import Lite3Motion  # noqa: E402


def main() -> int:
    load_dotenv()
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    cmd = sys.argv[1]

    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    port = int(os.environ.get("ROS2_BRIDGE_PORT", "9091"))

    print(f"connecting to foxy rosbridge ws://{host}:{port} …", file=sys.stderr)
    with Lite3Motion(host=host, port=port) as m:
        if cmd == "stop":
            m.stop()
            print("sent stop")
            return 0

        duration = float(sys.argv[2]) if len(sys.argv) >= 3 else 0.5
        speed_or_omega = float(sys.argv[3]) if len(sys.argv) >= 4 else None

        actions = {
            "forward":      lambda: m.forward(speed_or_omega or 0.15, duration),
            "backward":     lambda: m.backward(speed_or_omega or 0.15, duration),
            "left":         lambda: m.turn_left(speed_or_omega or 0.4, duration),
            "right":        lambda: m.turn_right(speed_or_omega or 0.4, duration),
            "strafe_left":  lambda: m.strafe_left(speed_or_omega or 0.15, duration),
            "strafe_right": lambda: m.strafe_right(speed_or_omega or 0.15, duration),
        }
        if cmd not in actions:
            print(f"unknown command: {cmd}", file=sys.stderr)
            print(__doc__)
            return 1

        print(f"running {cmd} for {duration:.2f}s…", file=sys.stderr)
        t0 = time.time()
        actions[cmd]()
        print(f"done in {time.time()-t0:.2f}s", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
