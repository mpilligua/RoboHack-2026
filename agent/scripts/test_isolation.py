"""Isolation test: which part of cli.py kills the camera?

Run sub-tests one at a time:
    python scripts/test_isolation.py rgb     # only Lite3Robot, get one RGB frame
    python scripts/test_isolation.py rgbd    # RGB + depth + pose subs
    python scripts/test_isolation.py motion  # add Lite3Motion (no publish)
    python scripts/test_isolation.py follow  # add Lite3Follow
    python scripts/test_isolation.py all     # everything cli.py creates
    python scripts/test_isolation.py vlm     # one VLM call on a fetched RGB

Between each test, on the robot run:
    timeout 3 rostopic hz /camera/color/image_raw

If the test passes (gets a frame, no error) and then `rostopic hz` shows 0 Hz,
that test killed the camera. We've found it.
"""

from __future__ import annotations

import os
import sys
import time

from dotenv import load_dotenv

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot import Lite3Follow, Lite3Motion, Lite3Robot, connect_ros2_rosbridge  # noqa: E402


def setup():
    load_dotenv()
    return (
        os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103"),
        int(os.environ.get("ROS_BRIDGE_PORT", "9090")),
        int(os.environ.get("ROS2_BRIDGE_PORT", "9091")),
    )


def t_rgb():
    host, p1, _ = setup()
    print(f"[rgb] connect noetic bridge :{p1}", file=sys.stderr)
    with Lite3Robot(host=host, port=p1) as r:
        f = r.get_rgb(timeout_s=5.0)
        print(f"[rgb] got {f.width}x{f.height} ok")
        time.sleep(2)
        print("[rgb] sleeping 2s, then closing")


def t_rgbd():
    host, p1, _ = setup()
    print(f"[rgbd] connect noetic bridge :{p1}", file=sys.stderr)
    with Lite3Robot(host=host, port=p1) as r:
        rgb = r.get_rgb(timeout_s=5.0)
        depth = r.get_depth(timeout_s=5.0)
        print(f"[rgbd] rgb {rgb.width}x{rgb.height}, depth {depth.width}x{depth.height}")
        for i in range(5):
            time.sleep(1)
            print(f"[rgbd] tick {i}")


def t_motion():
    host, _, p2 = setup()
    print(f"[motion] connect foxy bridge :{p2}", file=sys.stderr)
    with Lite3Motion(host=host, port=p2):
        print("[motion] connected, no publish")
        time.sleep(3)


def t_follow():
    host, _, p2 = setup()
    print(f"[follow] connect foxy bridge :{p2}", file=sys.stderr)
    with Lite3Follow(host=host, port=p2) as f:
        print("[follow] connected; will read detections for 5s")
        for i in range(5):
            time.sleep(1)
            print(f"[follow] tick {i}: {len(f.get_detections())} detections")


def t_all():
    host, p1, p2 = setup()
    print("[all] connect everything (one ROS2 socket)", file=sys.stderr)
    robot = Lite3Robot(host=host, port=p1)
    ros2 = connect_ros2_rosbridge(host, p2)
    motion = Lite3Motion(ros_client=ros2)
    follow = Lite3Follow(ros_client=ros2)
    try:
        rgb = robot.get_rgb(timeout_s=5.0)
        depth = robot.get_depth(timeout_s=5.0)
        print(f"[all] rgb {rgb.width}x{rgb.height}, depth {depth.width}x{depth.height}")
        for i in range(5):
            time.sleep(1)
            print(f"[all] tick {i}: dets={len(follow.get_detections())}")
    finally:
        follow.close()
        motion.close()
        try:
            ros2.terminate()
        except Exception:
            pass
        robot.close()


def t_vlm():
    host, p1, _ = setup()
    print(f"[vlm] fetch rgb + send to bedrock", file=sys.stderr)
    from vlm import make_client, vlm_describe  # noqa: E402

    with Lite3Robot(host=host, port=p1) as r:
        jpeg = r.rgb_jpeg_b64()
        client = make_client()
        text = vlm_describe(client, jpeg, "One sentence: what's in this image?")
        print(f"[vlm] reply: {text}")


SUBS = {"rgb": t_rgb, "rgbd": t_rgbd, "motion": t_motion, "follow": t_follow, "all": t_all, "vlm": t_vlm}


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] not in SUBS:
        print(__doc__)
        return 1
    SUBS[sys.argv[1]]()
    print("[done] now check `rostopic hz /camera/color/image_raw` on the robot")
    return 0


if __name__ == "__main__":
    sys.exit(main())
