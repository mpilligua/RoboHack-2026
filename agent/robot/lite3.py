"""Lite3 robot adapter over rosbridge_websocket.

Subscribes to RealSense D435i topics and motion-host telemetry, exposes a
small synchronous API the agent tools can call.

Run rosbridge on the robot first:
    ssh ysc@192.168.1.103
    roslaunch rosbridge_server rosbridge_websocket.launch
"""

from __future__ import annotations

import base64
import io
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import roslibpy
from PIL import Image


RGB_TOPIC = "/camera/color/image_raw"
# Note: this Lite3 publishes depth in the camera's own frame, not aligned to
# RGB. Pixel-perfect overlay would need /camera/aligned_depth_to_color, which
# isn't published here. Good enough for distance summaries.
DEPTH_TOPIC = "/camera/depth/image_rect_raw"
IMU_TOPIC = "/imu/data"
ODOM_TOPIC = "/leg_odom"
JOINT_TOPIC = "/joint_states"
CMD_VEL_TOPIC = "/cmd_vel"


@dataclass
class RGBFrame:
    image: Image.Image
    width: int
    height: int
    stamp: float


@dataclass
class DepthFrame:
    # Depth in millimeters (uint16, RealSense convention).
    depth_mm: np.ndarray
    width: int
    height: int
    stamp: float


@dataclass
class Pose:
    x: float
    y: float
    z: float
    yaw: float
    stamp: float


class Lite3Robot:
    """Thin synchronous wrapper around rosbridge subscriptions.

    Each topic is cached as the most-recent message; `get_*()` returns the
    cached frame or blocks briefly waiting for the first one.
    """

    def __init__(
        self,
        host: str = "192.168.1.103",
        port: int = 9090,
        connect_timeout_s: float = 10.0,
    ) -> None:
        self._client = roslibpy.Ros(host=host, port=port)
        self._lock = threading.Lock()
        self._rgb: Optional[RGBFrame] = None
        self._depth: Optional[DepthFrame] = None
        self._pose: Optional[Pose] = None
        self._battery: Optional[float] = None

        self._client.run(timeout=connect_timeout_s)
        if not self._client.is_connected:
            raise RuntimeError(
                f"Could not connect to rosbridge at ws://{host}:{port}. "
                "Is rosbridge_websocket running on the robot?"
            )

        self._subs = [
            self._subscribe(RGB_TOPIC, "sensor_msgs/Image", self._on_rgb),
            self._subscribe(DEPTH_TOPIC, "sensor_msgs/Image", self._on_depth),
            self._subscribe(ODOM_TOPIC, "nav_msgs/Odometry", self._on_odom),
        ]

        self._cmd_vel = roslibpy.Topic(
            self._client, CMD_VEL_TOPIC, "geometry_msgs/Twist"
        )
        self._cmd_vel.advertise()

    # ------------------------------------------------------------------ subs

    def _subscribe(
        self,
        topic: str,
        msg_type: str,
        cb,
        throttle_rate: int = 100,
        queue_length: int = 1,
    ):
        # throttle_rate: min ms between messages pushed over the WS.
        # queue_length=1: rosbridge drops all but the newest message when the
        # socket is congested, so we always see the latest frame.
        sub = roslibpy.Topic(
            self._client,
            topic,
            msg_type,
            throttle_rate=throttle_rate,
            queue_length=queue_length,
        )
        sub.subscribe(cb)
        return sub

    def _on_rgb(self, msg: dict) -> None:
        # sensor_msgs/Image with encoding rgb8 or bgr8; data is base64.
        data = base64.b64decode(msg["data"])
        h, w = msg["height"], msg["width"]
        encoding = msg.get("encoding", "rgb8")
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
        if encoding == "bgr8":
            arr = arr[:, :, ::-1]
        img = Image.fromarray(arr, mode="RGB")
        with self._lock:
            self._rgb = RGBFrame(image=img, width=w, height=h, stamp=time.time())

    def _on_depth(self, msg: dict) -> None:
        # 16UC1, depth in millimeters.
        data = base64.b64decode(msg["data"])
        h, w = msg["height"], msg["width"]
        depth = np.frombuffer(data, dtype=np.uint16).reshape(h, w).copy()
        with self._lock:
            self._depth = DepthFrame(
                depth_mm=depth, width=w, height=h, stamp=time.time()
            )

    def _on_odom(self, msg: dict) -> None:
        p = msg["pose"]["pose"]
        q = p["orientation"]
        # yaw from quaternion (z-axis rotation).
        siny_cosp = 2 * (q["w"] * q["z"] + q["x"] * q["y"])
        cosy_cosp = 1 - 2 * (q["y"] ** 2 + q["z"] ** 2)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))
        with self._lock:
            self._pose = Pose(
                x=float(p["position"]["x"]),
                y=float(p["position"]["y"]),
                z=float(p["position"]["z"]),
                yaw=yaw,
                stamp=time.time(),
            )

    # ------------------------------------------------------------- public API

    def _wait_for(self, attr: str, timeout_s: float):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self._lock:
                val = getattr(self, attr)
            if val is not None:
                return val
            time.sleep(0.05)
        return None

    def _wait_for_fresh(self, attr: str, timeout_s: float, max_age_s: float):
        """Block until the cached frame is younger than `max_age_s`.

        Guards against returning a stale frame that was buffered while the
        WS queue was backed up. A frame captured before this call started
        is always considered stale regardless of `max_age_s`.
        """
        call_start = time.time()
        deadline = call_start + timeout_s
        while time.time() < deadline:
            with self._lock:
                val = getattr(self, attr)
            if val is not None:
                age = time.time() - val.stamp
                if age <= max_age_s and val.stamp >= call_start - max_age_s:
                    return val
            time.sleep(0.05)
        return None

    def get_rgb(self, timeout_s: float = 3.0, max_age_s: Optional[float] = None) -> RGBFrame:
        if max_age_s is None:
            frame = self._wait_for("_rgb", timeout_s)
        else:
            frame = self._wait_for_fresh("_rgb", timeout_s, max_age_s)
        if frame is None:
            raise TimeoutError(
                f"No RGB frame on {RGB_TOPIC} within {timeout_s}s"
                + (f" (max_age_s={max_age_s})" if max_age_s is not None else "")
            )
        return frame

    def get_depth(self, timeout_s: float = 3.0, max_age_s: Optional[float] = None) -> DepthFrame:
        if max_age_s is None:
            frame = self._wait_for("_depth", timeout_s)
        else:
            frame = self._wait_for_fresh("_depth", timeout_s, max_age_s)
        if frame is None:
            raise TimeoutError(
                f"No depth frame on {DEPTH_TOPIC} within {timeout_s}s"
                + (f" (max_age_s={max_age_s})" if max_age_s is not None else "")
            )
        return frame

    def get_rgbd(
        self,
        timeout_s: float = 3.0,
        max_age_s: Optional[float] = None,
    ) -> tuple[RGBFrame, DepthFrame]:
        return (
            self.get_rgb(timeout_s, max_age_s),
            self.get_depth(timeout_s, max_age_s),
        )

    def get_pose(self, timeout_s: float = 2.0) -> Optional[Pose]:
        return self._wait_for("_pose", timeout_s)

    def rgb_jpeg_b64(
        self,
        quality: int = 85,
        timeout_s: float = 3.0,
        max_age_s: Optional[float] = None,
    ) -> str:
        """RGB frame encoded as base64 JPEG — feed straight to a VLM."""
        frame = self.get_rgb(timeout_s, max_age_s)
        buf = io.BytesIO()
        frame.image.save(buf, format="JPEG", quality=quality)
        return base64.b64encode(buf.getvalue()).decode("ascii")

    def depth_summary(
        self,
        timeout_s: float = 3.0,
        max_age_s: Optional[float] = None,
    ) -> dict:
        """Compact depth stats — closest object, center distance, etc."""
        frame = self.get_depth(timeout_s, max_age_s)
        d = frame.depth_mm
        valid = d[(d > 0) & (d < 10_000)]
        cx, cy = frame.width // 2, frame.height // 2
        center_mm = int(d[cy, cx]) if d[cy, cx] > 0 else None
        return {
            "width": frame.width,
            "height": frame.height,
            "min_mm": int(valid.min()) if valid.size else None,
            "median_mm": int(np.median(valid)) if valid.size else None,
            "center_mm": center_mm,
            "valid_fraction": float(valid.size / d.size),
        }

    # ---------------------------------------------------------------- motion
    # Locomotion is Person A's territory; these are stubs that publish to
    # /cmd_vel for the agent to call. Off by default for safety.

    def set_velocity(self, vx: float = 0.0, vy: float = 0.0, omega: float = 0.0) -> None:
        msg = roslibpy.Message(
            {
                "linear": {"x": float(vx), "y": float(vy), "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": float(omega)},
            }
        )
        self._cmd_vel.publish(msg)

    def stop(self) -> None:
        self.set_velocity(0.0, 0.0, 0.0)

    # ----------------------------------------------------------------- close

    def close(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
        for sub in self._subs:
            try:
                sub.unsubscribe()
            except Exception:
                pass
        try:
            self._cmd_vel.unadvertise()
        except Exception:
            pass
        self._client.terminate()

    def __enter__(self) -> "Lite3Robot":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
