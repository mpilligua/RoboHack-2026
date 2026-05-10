"""Bare-metal motion test: just turn the robot. No LLM, no tracker, no agent.

Prereqs: ROS 2 foxy bridge running on the robot at port 9091, Auto Mode ON.

Run:
    cd /Users/maria/Desktop/RoboHack/agent
    source .venv/bin/activate
    python scripts/test_turn.py
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dotenv import load_dotenv
load_dotenv()

from robot import Lite3Motion, connect_ros2_rosbridge


def banner(msg: str) -> None:
    print(f"\n=== {msg} ===", flush=True)


def main() -> None:
    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    port = int(os.environ.get("ROS2_BRIDGE_PORT", "9091"))

    print(f"connecting to ROS 2 bridge ws://{host}:{port} …")
    ros2 = connect_ros2_rosbridge(host, port)
    motion = Lite3Motion(ros_client=ros2)
    print("motion adapter connected. starting tests in 3 s — clear the area.")
    time.sleep(3)

    try:
        banner("TEST 1: forward 1.5 s @ 0.15 m/s — baseline, expect dog walks forward")
        motion.forward(speed=0.15, duration_s=1.5)
        time.sleep(1.5)

        banner("TEST 2: backward 1.5 s @ 0.15 m/s — expect dog walks backward")
        motion.backward(speed=0.15, duration_s=1.5)
        time.sleep(1.5)

        banner("TEST 3: turn_left 2.0 s @ 0.5 rad/s — expect dog rotates LEFT in place (CCW from above)")
        motion.turn_left(omega=0.5, duration_s=2.0)
        time.sleep(2.0)

        banner("TEST 4: turn_right 2.0 s @ 0.5 rad/s — expect dog rotates RIGHT in place (CW from above)")
        motion.turn_right(omega=0.5, duration_s=2.0)
        time.sleep(2.0)

        banner("TEST 5: turn_right 1.31 s @ 0.6 rad/s — exact same call as the sweep uses")
        motion.turn_right(omega=0.6, duration_s=1.31)
        time.sleep(1.31)

        banner("TEST 6: stop")
        motion.stop()
        time.sleep(2.0)

        banner("TEST 7: drive_continuous turn_right for 5 s @ 0.5 rad/s — expect SMOOTH continuous rotation, no stops")
        elapsed = motion.drive_continuous(vx=0.0, vy=0.0, omega=-0.5, max_duration_s=5.0)
        motion.stop()
        print(f"   actual elapsed: {elapsed:.2f}s")

    finally:
        try:
            motion.stop()
        except Exception:
            pass
        motion.close()
        try:
            ros2.terminate()
        except Exception:
            pass
        print("\ndone. report what the dog actually did for each test.")


if __name__ == "__main__":
    main()
