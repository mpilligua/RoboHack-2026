"""Quick test: print depth at the center RGB pixel.

Run:
    cd .../agent
    python -m scripts.test_depth_center
    python scripts/test_depth_center.py 5

Optional arg: window_radius (default 3).
"""

from __future__ import annotations

import json
import os
import sys
import time

from dotenv import load_dotenv

# Allow running as `python scripts/test_depth_center.py` from the agent/ folder.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot import Lite3Robot  # noqa: E402


def main() -> int:
    load_dotenv()
    window_radius = 3
    if len(sys.argv) >= 2:
        try:
            window_radius = int(sys.argv[1])
        except ValueError:
            print("window_radius must be an integer", file=sys.stderr)
            return 1

    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    port = int(os.environ.get("ROS_BRIDGE_PORT", "9090"))

    print(f"connecting to rosbridge ws://{host}:{port} ...", file=sys.stderr)
    t0 = time.time()
    with Lite3Robot(host=host, port=port) as robot:
        print(f"connected in {time.time() - t0:.2f}s", file=sys.stderr)
        rgb = robot.get_rgb(timeout_s=10.0)
        u = rgb.width // 2
        v = rgb.height // 2
        out = robot.depth_at_rgb_pixel_naive(u, v, window_radius=window_radius, timeout_s=10.0)
        print(json.dumps(out, indent=2))

    return 0 if "error" not in out else 2


if __name__ == "__main__":
    sys.exit(main())
