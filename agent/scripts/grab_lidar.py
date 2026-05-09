"""Grab one Livox/RSLidar PointCloud2 frame over the foxy rosbridge.

Run:
    python scripts/grab_lidar.py
"""

from __future__ import annotations

import base64
import os
import struct
import sys
import threading
import time

import numpy as np
import roslibpy
from dotenv import load_dotenv

LIDAR_TOPIC = "/livox/lidar"


def decode_pointcloud2(msg: dict) -> np.ndarray:
    """Decode a sensor_msgs/PointCloud2 into an Nx3 float32 array (XYZ)."""
    fields = {f["name"]: f for f in msg["fields"]}
    if not all(k in fields for k in ("x", "y", "z")):
        raise RuntimeError(f"unexpected fields: {list(fields)}")

    point_step = msg["point_step"]
    n_points = msg["width"] * msg["height"]
    data = base64.b64decode(msg["data"]) if isinstance(msg["data"], str) else bytes(msg["data"])

    # Build numpy structured access from the field offsets.
    ox = fields["x"]["offset"]
    oy = fields["y"]["offset"]
    oz = fields["z"]["offset"]

    arr = np.frombuffer(data, dtype=np.uint8).reshape(n_points, point_step)
    xyz = np.empty((n_points, 3), dtype=np.float32)
    xyz[:, 0] = arr[:, ox:ox + 4].copy().view(np.float32).ravel()
    xyz[:, 1] = arr[:, oy:oy + 4].copy().view(np.float32).ravel()
    xyz[:, 2] = arr[:, oz:oz + 4].copy().view(np.float32).ravel()

    valid = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 1e-6)
    return xyz[valid]


def write_ply(path: str, points: np.ndarray) -> None:
    with open(path, "wb") as f:
        f.write(b"ply\nformat binary_little_endian 1.0\n")
        f.write(f"element vertex {len(points)}\n".encode())
        f.write(b"property float x\nproperty float y\nproperty float z\nend_header\n")
        f.write(points.astype("<f4").tobytes())


def main() -> int:
    load_dotenv()
    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    # Livox driver publishes on the ROS 1 noetic graph (port 9090).
    port = int(os.environ.get("ROS_BRIDGE_PORT", "9090"))

    print(f"connecting to ws://{host}:{port} …", file=sys.stderr)
    client = roslibpy.Ros(host=host, port=port)
    client.run(timeout=10.0)
    if not client.is_connected:
        print("could not connect to rosbridge", file=sys.stderr)
        return 1

    got = threading.Event()
    holder: dict = {}

    def cb(msg: dict) -> None:
        if not got.is_set():
            holder["msg"] = msg
            got.set()

    sub = roslibpy.Topic(client, LIDAR_TOPIC, "sensor_msgs/PointCloud2")
    sub.subscribe(cb)

    print(f"waiting for first {LIDAR_TOPIC} (up to 15s) …", file=sys.stderr)
    if not got.wait(timeout=15.0):
        print("timeout: no lidar message received", file=sys.stderr)
        sub.unsubscribe()
        client.terminate()
        return 1

    msg = holder["msg"]
    t = time.time()
    pts = decode_pointcloud2(msg)
    print(
        f"got {len(pts)} valid points  "
        f"x[{pts[:,0].min():.2f},{pts[:,0].max():.2f}]  "
        f"y[{pts[:,1].min():.2f},{pts[:,1].max():.2f}]  "
        f"z[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]  "
        f"(decoded in {time.time()-t:.2f}s)"
    )

    write_ply("frame_lidar.ply", pts)
    print("saved frame_lidar.ply (Z-up, ROS convention)")

    # Try a few orientations — open whichever looks right in 3dviewer.net.
    # ROS REP-103: x forward, y left, z up. Viewers typically want y up.
    pts_a = np.column_stack([-pts[:, 1], pts[:, 2], -pts[:, 0]])
    write_ply("frame_lidar_a.ply", pts_a)
    pts_b = np.column_stack([pts[:, 0], pts[:, 2], -pts[:, 1]])
    write_ply("frame_lidar_b.ply", pts_b)
    pts_c = np.column_stack([pts[:, 1], pts[:, 2], pts[:, 0]])
    write_ply("frame_lidar_c.ply", pts_c)
    print("saved a/b/c — open each in viewer, pick the one with horizontal floor")

    sub.unsubscribe()
    client.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
