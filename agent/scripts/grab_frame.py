"""Smallest possible test: connect to rosbridge, grab one RGB frame, save it.

Run:
    cd /Users/maria/Desktop/RoboHack/agent
    python -m scripts.grab_frame
"""

from __future__ import annotations

import os
import sys
import time

from dotenv import load_dotenv

# Allow running as `python scripts/grab_frame.py` from the agent/ folder.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot import Lite3Robot  # noqa: E402


def main() -> int:
    load_dotenv()
    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    port = int(os.environ.get("ROS_BRIDGE_PORT", "9090"))

    print(f"connecting to rosbridge ws://{host}:{port} …", file=sys.stderr)
    t0 = time.time()
    with Lite3Robot(host=host, port=port) as robot:
        print(f"connected in {time.time()-t0:.2f}s", file=sys.stderr)

        print("waiting for first RGB frame (up to 10s) …", file=sys.stderr)
        t0 = time.time()
        rgb = robot.get_rgb(timeout_s=10.0)
        print(
            f"got RGB {rgb.width}x{rgb.height} in {time.time()-t0:.2f}s",
            file=sys.stderr,
        )
        rgb.image.save("frame_rgb.jpg", quality=90)
        print("saved frame_rgb.jpg")

        print("waiting for first depth frame (up to 10s) …", file=sys.stderr)
        t0 = time.time()
        try:
            depth = robot.get_depth(timeout_s=10.0)
            print(
                f"got depth {depth.width}x{depth.height} in {time.time()-t0:.2f}s",
                file=sys.stderr,
            )
            print(f"depth summary: {robot.depth_summary()}")

            import numpy as np
            from PIL import Image

            # Raw 16-bit PNG, values are millimeters (preserves true depth).
            Image.fromarray(depth.depth_mm, mode="I;16").save("frame_depth_raw.png")
            print("saved frame_depth_raw.png (16-bit, mm)")

            # Colorized 8-bit visualization — clip to 0.1–4 m, invert so close=bright.
            d = depth.depth_mm.astype(np.float32)
            valid = (d > 100) & (d < 4000)
            d_clipped = np.clip(d, 100, 4000)
            norm = ((4000 - d_clipped) / 3900.0 * 255).astype(np.uint8)
            norm[~valid] = 0
            # Simple turbo-ish gradient with three channels.
            r = np.clip(1.5 * norm - 64, 0, 255).astype(np.uint8)
            g = np.clip(2.0 * np.abs(norm.astype(int) - 128), 0, 255).astype(np.uint8)
            g = (255 - g).astype(np.uint8)
            b = np.clip(255 - 1.5 * norm, 0, 255).astype(np.uint8)
            rgb = np.stack([r, g, b], axis=-1)
            rgb[~valid] = 0
            Image.fromarray(rgb, mode="RGB").save("frame_depth_vis.jpg", quality=90)
            print("saved frame_depth_vis.jpg (colorized: close=red, far=blue, black=invalid)")
        except TimeoutError as e:
            print(f"depth: {e}", file=sys.stderr)

        pose = robot.get_pose(timeout_s=2.0)
        print(f"pose: {pose}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
