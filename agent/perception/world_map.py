"""Laptop-side world object map (Plan B).

Pulls YOLO detections + RGB + depth + pose from existing rosbridge adapters
(Lite3Robot, Lite3Follow), projects bbox centers into the /leg_odom frame,
and updates ObjectRecord entries in MemoryStore with x_odom, y_odom.

Why laptop-side: ROS-on-the-robot QoS / TF / launch debugging was eating all
our time. Everything we need is already plumbed through rosbridge for the
agent. Tradeoff: drift (no SLAM correction) and latency (~200-500ms per
tick over WiFi). Acceptable for short tasks.

Frame conventions:
- Camera optical frame (REP-103): +X right, +Y down, +Z forward.
- /leg_odom is a 2D world frame: +X forward, +Y left, +Z up. Yaw rotates +X.
- Camera offset relative to base: hardcoded below (camera mounted forward
  and slightly above center, looking forward). Edit if your robot differs.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from memory.schemas import ObjectRecord
from memory.store import MemoryStore
from perception.external_pose import get_pose as external_get_pose
from robot.follow import Lite3Follow
from robot.lite3 import Lite3Robot
from robot.rgbd_naive import compute_depth_at_rgb_pixel_naive


# Hardcoded RealSense intrinsics — read live from /camera/color/camera_info on
# 2026-05-10. Update if the camera config changes (e.g. resolution).
@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


D435I_640x480 = CameraIntrinsics(
    fx=607.8, fy=607.2, cx=329.3, cy=239.9, width=640, height=480
)

# Camera mount, in base_link / odom-aligned frame:
# +x_base forward, +y_base left, +z_base up.
# Lidar is "behind/above" the camera; we approximate camera position relative
# to base_link from the static TFs the robot publishes (base_link -> camera_link
# was 0.255 fwd, 0 left, 0.072 up, pitched ~20° down).
CAMERA_OFFSET_X = 0.255
CAMERA_OFFSET_Y = 0.0
CAMERA_OFFSET_Z = 0.072
CAMERA_PITCH_DOWN_RAD = math.radians(20.0)


def pixel_depth_to_camera_xyz(u: float, v: float, depth_m: float,
                              K: CameraIntrinsics) -> tuple[float, float, float]:
    """Pinhole back-projection. Returns XYZ in camera optical frame."""
    X = (u - K.cx) * depth_m / K.fx
    Y = (v - K.cy) * depth_m / K.fy
    Z = depth_m
    return X, Y, Z


def camera_optical_to_base(xc: float, yc: float, zc: float,
                           pitch_down_rad: float = CAMERA_PITCH_DOWN_RAD,
                           ox: float = CAMERA_OFFSET_X,
                           oy: float = CAMERA_OFFSET_Y,
                           oz: float = CAMERA_OFFSET_Z) -> tuple[float, float, float]:
    """Convert from camera optical frame to robot base frame.

    Optical (REP-103): +X right, +Y down, +Z forward.
    Base: +X forward, +Y left, +Z up.

    Without pitch, the conversion would be:
        x_base = +Zc          (forward in optical = forward in base)
        y_base = -Xc          (right in optical = -y in base)
        z_base = -Yc          (down in optical = -z in base)

    With camera pitched down by `pitch_down_rad` (camera nose tilts toward
    the floor), we rotate the optical-frame point about the camera's X axis
    by +pitch_down_rad before applying the swap.
    """
    # The camera's optical axis is tilted DOWN by pitch_down_rad. To express
    # the same point in an *un-tilted* optical frame (so the swap to base
    # works), we apply the inverse: rotate about optical +X by -pitch_down.
    # That moves a point straight along the optical axis (0, 0, +Z) to
    # (0, +Z*sin(p), +Z*cos(p)) — i.e. "below center, slightly less forward",
    # which after the swap below puts it physically below the camera, as
    # expected for a camera tilted down looking at the floor straight ahead.
    cos_p = math.cos(-pitch_down_rad)
    sin_p = math.sin(-pitch_down_rad)
    yc_r = yc * cos_p - zc * sin_p
    zc_r = yc * sin_p + zc * cos_p

    x_base = zc_r + ox
    y_base = -xc + oy
    z_base = -yc_r + oz
    return x_base, y_base, z_base


def base_to_odom(x_base: float, y_base: float, z_base: float,
                 robot_x: float, robot_y: float, robot_z: float,
                 robot_yaw: float) -> tuple[float, float, float]:
    """Apply the robot's pose in odom to a point in base frame -> point in odom.

    Standard 2D rotation about Z by yaw, then translate.
    """
    cos_y = math.cos(robot_yaw)
    sin_y = math.sin(robot_yaw)
    x_odom = robot_x + x_base * cos_y - y_base * sin_y
    y_odom = robot_y + x_base * sin_y + y_base * cos_y
    z_odom = robot_z + z_base
    return x_odom, y_odom, z_odom


@dataclass
class TickStats:
    n_detections: int
    n_projected: int
    n_skipped_no_depth: int
    n_skipped_no_pose: int
    fetch_ms: float


class WorldObjectUpdater:
    """One tick = pull frames + detections + pose, project, upsert into memory.

    Designed to be called from a background thread (~1-2 Hz). Each tick is
    bounded — does at most one RGB fetch, one depth fetch, one pose fetch.
    Failures are logged via the supplied logger and don't raise.
    """

    def __init__(self,
                 robot: Lite3Robot,
                 follow: Lite3Follow,
                 memory: MemoryStore,
                 K: CameraIntrinsics = D435I_640x480,
                 logger=None) -> None:
        self._robot = robot
        self._follow = follow
        self._memory = memory
        self._K = K
        self._log = logger

    def _info(self, msg: str) -> None:
        if self._log:
            self._log.info(msg)

    def _warn(self, msg: str) -> None:
        if self._log:
            self._log.warning(msg)

    def tick(self) -> TickStats:
        t0 = time.monotonic()
        dets = self._follow.get_detections()
        if not dets:
            return TickStats(0, 0, 0, 0, (time.monotonic() - t0) * 1000)

        # One snapshot of the world per tick.
        try:
            rgb = self._robot.get_rgb(timeout_s=2.0)
            depth = self._robot.get_depth(timeout_s=2.0)
        except Exception as e:
            self._warn(f"frame fetch failed: {e}")
            return TickStats(len(dets), 0, len(dets), 0, (time.monotonic() - t0) * 1000)

        # Pose comes from the externally-injected function (perception/external_pose.py),
        # not Lite3Robot anymore — no rosbridge odom dependency.
        try:
            pose = external_get_pose()
        except Exception as e:
            self._warn(f"external get_pose failed: {e}")
            pose = None
        fetch_ms = (time.monotonic() - t0) * 1000

        n_no_depth = 0
        n_no_pose = 0
        n_proj = 0
        for det in dets:
            try:
                x1, y1, x2, y2 = det.bbox
            except (TypeError, ValueError):
                continue
            u = int(round(0.5 * (x1 + x2)))
            v = int(round(0.5 * (y1 + y2)))

            depth_info = compute_depth_at_rgb_pixel_naive(
                rgb_width=rgb.width, rgb_height=rgb.height,
                depth_mm=depth.depth_mm,
                u_rgb=u, v_rgb=v, window_radius=8,
            )
            if "error" in depth_info or "depth_m" not in depth_info:
                n_no_depth += 1
                # Still upsert the bbox-only record so the agent sees the
                # detection even without a position.
                self._memory.upsert_object(ObjectRecord(
                    yolo_id=int(det.id),
                    label=det.label or "?",
                    description="",
                    bbox=list(det.bbox),
                    confidence=det.conf,
                    last_seen_ts=time.time(),
                ))
                continue
            depth_m = float(depth_info["depth_m"])

            xc, yc, zc = pixel_depth_to_camera_xyz(u, v, depth_m, self._K)
            xb, yb, zb = camera_optical_to_base(xc, yc, zc)

            x_odom = y_odom = z_odom = None
            pose_stamp = None
            if pose is not None:
                x_odom, y_odom, z_odom = base_to_odom(
                    xb, yb, zb, pose.x, pose.y, pose.z, pose.yaw
                )
                pose_stamp = pose.stamp
            else:
                n_no_pose += 1

            self._memory.upsert_object(ObjectRecord(
                yolo_id=int(det.id),
                label=det.label or "?",
                description="",
                bbox=list(det.bbox),
                confidence=det.conf,
                depth_m=depth_m,
                last_seen_ts=time.time(),
                x_odom=x_odom,
                y_odom=y_odom,
                z_odom=z_odom,
                pose_stamp=pose_stamp,
            ))
            n_proj += 1

        return TickStats(
            n_detections=len(dets),
            n_projected=n_proj,
            n_skipped_no_depth=n_no_depth,
            n_skipped_no_pose=n_no_pose,
            fetch_ms=fetch_ms,
        )
