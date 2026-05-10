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

from .rgbd_naive import compute_depth_at_rgb_pixel_naive


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
    """rosbridge wrapper that fetches frames on demand only.

    Camera streams are large at full rate (~35 MB/s raw at 30 Hz over WiFi).
    By default each ``get_rgb`` / ``get_depth`` does subscribe → one message →
    unsubscribe. Optional ``max_rgb_hz`` / ``max_depth_hz`` add client-side
    throttling: reuse recent cached frames or sleep between fetches. Pose
    fetches are not throttled.
    """

    def __init__(
        self,
        host: str = "192.168.1.103",
        port: int = 9090,
        connect_timeout_s: float = 10.0,
        *,
        max_rgb_hz: Optional[float] = None,
        max_depth_hz: Optional[float] = None,
    ) -> None:
        self._client = roslibpy.Ros(host=host, port=port)
        self._lock = threading.Lock()
        # Max fetch rates for RGB/depth (pose is uncapped). None or <=0 = unlimited.
        self._max_rgb_hz = max_rgb_hz
        self._max_depth_hz = max_depth_hz
        self._last_rgb_fetch_mono: float = 0.0
        self._last_depth_fetch_mono: float = 0.0
        # Last fetched frames are cached for status reporting only — we don't
        # auto-update them, so `get_status` can show "rgb_age_s = 5s ago" etc.
        self._rgb: Optional[RGBFrame] = None
        self._depth: Optional[DepthFrame] = None
        self._pose: Optional[Pose] = None

        self._client.run(timeout=connect_timeout_s)
        if not self._client.is_connected:
            raise RuntimeError(
                f"Could not connect to rosbridge at ws://{host}:{port}. "
                "Is rosbridge_websocket running on the robot?"
            )

        self._cmd_vel = roslibpy.Topic(
            self._client, CMD_VEL_TOPIC, "geometry_msgs/Twist"
        )
        self._cmd_vel.advertise()

    def _respect_rgb_rate_limit(self) -> Optional[RGBFrame]:
        """Return cached RGB if we must not fetch yet; otherwise sleep until ok."""
        if self._max_rgb_hz is None or self._max_rgb_hz <= 0:
            return None
        interval = 1.0 / float(self._max_rgb_hz)
        with self._lock:
            cached = self._rgb
            last = self._last_rgb_fetch_mono
        now = time.monotonic()
        if cached is not None and (now - last) < interval:
            return cached
        wait = interval - (now - last)
        if wait > 0:
            time.sleep(wait)
        return None

    def _respect_depth_rate_limit(self) -> Optional[DepthFrame]:
        if self._max_depth_hz is None or self._max_depth_hz <= 0:
            return None
        interval = 1.0 / float(self._max_depth_hz)
        with self._lock:
            cached = self._depth
            last = self._last_depth_fetch_mono
        now = time.monotonic()
        if cached is not None and (now - last) < interval:
            return cached
        wait = interval - (now - last)
        if wait > 0:
            time.sleep(wait)
        return None

    def _fetch_one(self, topic_name: str, msg_type: str, timeout_s: float):
        """Subscribe, wait for one message, unsubscribe. Returns the dict or
        raises TimeoutError."""
        sub = roslibpy.Topic(self._client, topic_name, msg_type)
        evt = threading.Event()
        holder: dict = {}

        def cb(msg: dict) -> None:
            if not evt.is_set():
                holder["msg"] = msg
                evt.set()

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
        try:
            if not evt.wait(timeout=timeout_s):
                raise TimeoutError(
                    f"No message on {topic_name} within {timeout_s}s"
                )
            return holder["msg"]
        finally:
            try:
                sub.unsubscribe()
            except Exception:
                pass

    # ------------------------------------------------------------- public API

    def get_rgb(self, timeout_s: float = 5.0) -> RGBFrame:
        cached = self._respect_rgb_rate_limit()
        if cached is not None:
            return cached
        msg = self._fetch_one(RGB_TOPIC, "sensor_msgs/Image", timeout_s)
        data = base64.b64decode(msg["data"])
        h, w = msg["height"], msg["width"]
        encoding = msg.get("encoding", "rgb8")
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
        if encoding == "bgr8":
            arr = arr[:, :, ::-1]
        img = Image.fromarray(arr, mode="RGB")
        frame = RGBFrame(image=img, width=w, height=h, stamp=time.time())
        with self._lock:
            self._rgb = frame
            self._last_rgb_fetch_mono = time.monotonic()
        return frame

    def get_depth(self, timeout_s: float = 5.0) -> DepthFrame:
        cached = self._respect_depth_rate_limit()
        if cached is not None:
            return cached
        msg = self._fetch_one(DEPTH_TOPIC, "sensor_msgs/Image", timeout_s)
        data = base64.b64decode(msg["data"])
        h, w = msg["height"], msg["width"]
        depth = np.frombuffer(data, dtype=np.uint16).reshape(h, w).copy()
        frame = DepthFrame(depth_mm=depth, width=w, height=h, stamp=time.time())
        with self._lock:
            self._depth = frame
            self._last_depth_fetch_mono = time.monotonic()
        return frame

    def get_rgbd(self, timeout_s: float = 5.0) -> tuple[RGBFrame, DepthFrame]:
        return self.get_rgb(timeout_s), self.get_depth(timeout_s)

    def get_pose(self, timeout_s: float = 2.0) -> Optional[Pose]:
        try:
            msg = self._fetch_one(ODOM_TOPIC, "nav_msgs/Odometry", timeout_s)
        except TimeoutError:
            return None
        p = msg["pose"]["pose"]
        q = p["orientation"]
        siny_cosp = 2 * (q["w"] * q["z"] + q["x"] * q["y"])
        cosy_cosp = 1 - 2 * (q["y"] ** 2 + q["z"] ** 2)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))
        pose = Pose(
            x=float(p["position"]["x"]),
            y=float(p["position"]["y"]),
            z=float(p["position"]["z"]),
            yaw=yaw,
            stamp=time.time(),
        )
        with self._lock:
            self._pose = pose
        return pose

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

    def depth_at_rgb_pixel_naive(
        self,
        u_rgb: int,
        v_rgb: int,
        *,
        window_radius: int = 3,
        timeout_s: float = 3.0,
    ) -> dict:
        """Approximate depth at an RGB pixel.

        Captures the latest RGB and depth frames, scales the RGB pixel into
        depth image coordinates, and falls back to the nearest valid depth in
        a local window when the direct depth sample is invalid.
        """
        rgb_frame, depth_frame = self.get_rgbd(timeout_s)
        return compute_depth_at_rgb_pixel_naive(
            rgb_width=rgb_frame.width,
            rgb_height=rgb_frame.height,
            depth_mm=depth_frame.depth_mm,
            u_rgb=int(u_rgb),
            v_rgb=int(v_rgb),
            window_radius=int(window_radius),
        )

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
        try:
            self._cmd_vel.unadvertise()
        except Exception:
            pass
        try:
            self._client.terminate()
        except Exception:
            pass  # roslibpy 2.0 cleanup bug; harmless

    def __enter__(self) -> "Lite3Robot":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
