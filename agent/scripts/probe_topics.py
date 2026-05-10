"""Step 1 probe: confirm what the robot is publishing to both rosbridges.

For each topic the future map-merge layer needs, try to grab one message and
report ok/timeout/error. No state mutation; safe to run any time.

Run:
    cd .../agent
    python scripts/probe_topics.py
    python scripts/probe_topics.py --timeout 5
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import roslibpy
from dotenv import load_dotenv


@dataclass
class Probe:
    bridge: str  # "noetic-9090" or "foxy-9091"
    topic: str
    msg_type: str
    note: str = ""


PROBES = [
    Probe("noetic-9090", "/camera/color/image_raw", "sensor_msgs/Image", "RGB"),
    Probe("noetic-9090", "/camera/depth/image_rect_raw", "sensor_msgs/Image", "depth (unaligned)"),
    Probe("noetic-9090", "/camera/color/camera_info", "sensor_msgs/CameraInfo", "RGB intrinsics K (optional)"),
    Probe("noetic-9090", "/camera/depth/camera_info", "sensor_msgs/CameraInfo", "depth intrinsics K (optional)"),
    Probe("noetic-9090", "/leg_odom", "nav_msgs/Odometry", "robot pose in odom frame"),
    Probe("foxy-9091", "/agent/yolo_detections", "std_msgs/String", "YOLO bbox+class JSON"),
    Probe("foxy-9091", "/tf", "tf2_msgs/TFMessage", "TF (only if SLAM/nav2 up)"),
    Probe("foxy-9091", "/map", "nav_msgs/OccupancyGrid", "static map (only if nav2/SLAM up)"),
]


def fetch_one(client: roslibpy.Ros, topic: str, msg_type: str, timeout_s: float):
    sub = roslibpy.Topic(client, topic, msg_type)
    evt = threading.Event()
    holder: dict = {}

    def cb(msg):
        if not evt.is_set():
            holder["msg"] = msg
            evt.set()

    sub.subscribe(cb)
    try:
        if not evt.wait(timeout=timeout_s):
            return None
        return holder["msg"]
    finally:
        try:
            sub.unsubscribe()
        except Exception:
            pass


def summarize(topic: str, msg) -> str:
    """One-line summary of payload — just enough to confirm it's plausible."""
    if msg is None:
        return ""
    try:
        if topic.endswith("/image_raw") or topic.endswith("/image_rect_raw"):
            return f"{msg['width']}x{msg['height']} {msg.get('encoding','?')}"
        if topic.endswith("/camera_info"):
            K = msg.get("K") or msg.get("k")
            if K and len(K) >= 9:
                return f"fx={K[0]:.1f} fy={K[4]:.1f} cx={K[2]:.1f} cy={K[5]:.1f}"
            return "no K"
        if topic == "/leg_odom":
            p = msg["pose"]["pose"]["position"]
            return f"pos=({p['x']:.2f}, {p['y']:.2f}, {p['z']:.2f})"
        if topic == "/agent/yolo_detections":
            data = msg.get("data", "")
            return f"{len(data)} chars"
        if topic == "/tf":
            tfs = msg.get("transforms", [])
            frames = sorted({(t["header"]["frame_id"], t["child_frame_id"]) for t in tfs})
            return f"{len(tfs)} transforms: {frames[:3]}{'...' if len(frames) > 3 else ''}"
        if topic == "/map":
            info = msg.get("info", {})
            return f"{info.get('width','?')}x{info.get('height','?')} @ {info.get('resolution','?')}m/cell"
    except Exception as e:
        return f"summary failed: {e}"
    return "ok"


def probe_bridge(bridge_name: str, host: str, port: int, probes: list[Probe], timeout_s: float):
    print(f"\n=== {bridge_name} (ws://{host}:{port}) ===")
    client = roslibpy.Ros(host=host, port=port)
    try:
        client.run(timeout=5.0)
    except Exception as e:
        print(f"  CONNECT FAIL: {e}")
        return
    if not client.is_connected:
        print("  CONNECT FAIL: not connected after run()")
        return
    print("  connected")

    for p in probes:
        t0 = time.time()
        try:
            msg = fetch_one(client, p.topic, p.msg_type, timeout_s)
        except Exception as e:
            print(f"  [ERR ] {p.topic:40s}  ({p.note}): {e}")
            continue
        dt = time.time() - t0
        if msg is None:
            print(f"  [TIMEOUT {timeout_s:.1f}s] {p.topic:40s}  ({p.note})")
        else:
            print(f"  [OK  {dt:5.2f}s   ] {p.topic:40s}  {summarize(p.topic, msg)}  -- {p.note}")

    try:
        client.terminate()
    except Exception:
        pass


def main() -> int:
    load_dotenv()
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103"))
    ap.add_argument("--noetic-port", type=int, default=int(os.environ.get("ROS_BRIDGE_PORT", "9090")))
    ap.add_argument("--foxy-port", type=int, default=int(os.environ.get("FOXY_BRIDGE_PORT", "9091")))
    ap.add_argument("--timeout", type=float, default=3.0, help="per-topic wait")
    # Internal: when set, this process probes only one bridge then exits.
    # Lets us run each bridge in a fresh interpreter (Twisted's reactor can't be
    # re-run in the same process, which breaks back-to-back roslibpy.Ros.run()).
    ap.add_argument("--only", choices=["noetic-9090", "foxy-9091"], default=None)
    args = ap.parse_args()

    if args.only == "noetic-9090":
        probes = [p for p in PROBES if p.bridge == "noetic-9090"]
        probe_bridge("noetic-9090", args.host, args.noetic_port, probes, args.timeout)
        return 0
    if args.only == "foxy-9091":
        probes = [p for p in PROBES if p.bridge == "foxy-9091"]
        probe_bridge("foxy-9091", args.host, args.foxy_port, probes, args.timeout)
        return 0

    # Default: spawn one subprocess per bridge so each gets a fresh reactor.
    import subprocess
    here = os.path.abspath(__file__)
    common = [sys.executable, here, "--host", args.host, "--timeout", str(args.timeout)]
    for which, port_arg in (("noetic-9090", f"--noetic-port={args.noetic_port}"),
                            ("foxy-9091", f"--foxy-port={args.foxy_port}")):
        rc = subprocess.call(common + [port_arg, "--only", which])
        if rc != 0:
            print(f"  (subprocess for {which} exited rc={rc})")

    print("\nDone. Topics marked TIMEOUT either aren't published or are too slow for the wait.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
