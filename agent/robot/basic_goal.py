"""Basic goal controller adapter over ROS 2 rosbridge."""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import roslibpy

BASIC_GOAL_TOPIC = "/basic_goal"
BASIC_GOAL_CANCEL_TOPIC = "/basic_goal_cancel"
BASIC_GOAL_STATUS_TOPIC = "/basic_goal_status"
ODOM_TOPIC = "/odom"
SIMPLE_CMD_TOPIC = "/simple_cmd"
COMPLEX_CMD_TOPIC = "/complex_cmd"


def _yaw_from_quat(q: dict) -> float:
    siny_cosp = 2.0 * (float(q["w"]) * float(q["z"]) + float(q["x"]) * float(q["y"]))
    cosy_cosp = 1.0 - 2.0 * (float(q["y"]) ** 2 + float(q["z"]) ** 2)
    return math.atan2(siny_cosp, cosy_cosp)


class Lite3BasicGoal:
    """Publish basic goals and low-level motion commands via ROS 2 rosbridge."""

    def __init__(
        self,
        host: str = "192.168.1.103",
        port: int = 9091,
        connect_timeout_s: float = 10.0,
        *,
        ros_client: Optional[roslibpy.Ros] = None,
    ) -> None:
        self._lock = threading.Lock()
        self._status_cv = threading.Condition(self._lock)
        self._status: Optional[str] = None
        self._status_ts: Optional[float] = None
        self._odom: Optional[dict] = None
        self._odom_ts: Optional[float] = None

        if ros_client is not None:
            self._client = ros_client
            self._owns_client = False
        else:
            self._client = roslibpy.Ros(host=host, port=port)
            self._client.run(timeout=connect_timeout_s)
            if not self._client.is_connected:
                raise RuntimeError(
                    f"Could not connect to foxy rosbridge at ws://{host}:{port}"
                )
            self._owns_client = True

        self._goal_pub = roslibpy.Topic(self._client, BASIC_GOAL_TOPIC, "geometry_msgs/Pose2D")
        self._goal_pub.advertise()
        self._cancel_pub = roslibpy.Topic(
            self._client, BASIC_GOAL_CANCEL_TOPIC, "std_msgs/String"
        )
        self._cancel_pub.advertise()
        self._simple_pub = roslibpy.Topic(
            self._client, SIMPLE_CMD_TOPIC, "transfer_interfaces/msg/MotionSimpleCMD"
        )
        self._simple_pub.advertise()
        self._complex_pub = roslibpy.Topic(
            self._client, COMPLEX_CMD_TOPIC, "transfer_interfaces/msg/MotionComplexCMD"
        )
        self._complex_pub.advertise()

        self._status_sub = roslibpy.Topic(
            self._client, BASIC_GOAL_STATUS_TOPIC, "std_msgs/String"
        )
        self._status_sub.subscribe(self._on_status)
        self._odom_sub = roslibpy.Topic(self._client, ODOM_TOPIC, "nav_msgs/Odometry")
        self._odom_sub.subscribe(self._on_odom)

    def _on_status(self, msg: dict) -> None:
        try:
            status = str(msg.get("data", ""))
        except Exception:
            return
        now = time.time()
        with self._status_cv:
            self._status = status
            self._status_ts = now
            self._status_cv.notify_all()

    def _on_odom(self, msg: dict) -> None:
        try:
            pose = msg["pose"]["pose"]
            position = pose["position"]
            orientation = pose["orientation"]
            odom = {
                "x": float(position["x"]),
                "y": float(position["y"]),
                "z": float(position.get("z", 0.0)),
                "yaw": _yaw_from_quat(orientation),
            }
        except Exception:
            return
        with self._lock:
            self._odom = odom
            self._odom_ts = time.time()

    def send_goal(self, x: float, y: float, theta: float) -> None:
        msg = roslibpy.Message({"x": float(x), "y": float(y), "theta": float(theta)})
        self._goal_pub.publish(msg)

    def cancel_goal(self, reason: str = "stop") -> None:
        self._cancel_pub.publish(roslibpy.Message({"data": str(reason)}))

    def send_simple_cmd(self, cmd_code: int, size: int, msg_type: int) -> None:
        msg = roslibpy.Message(
            {"cmd_code": int(cmd_code), "size": int(size), "type": int(msg_type)}
        )
        self._simple_pub.publish(msg)

    def send_complex_cmd(self, cmd_code: int, size: int, msg_type: int, data: float) -> None:
        msg = roslibpy.Message(
            {
                "cmd_code": int(cmd_code),
                "size": int(size),
                "type": int(msg_type),
                "data": float(data),
            }
        )
        self._complex_pub.publish(msg)

    def get_status(self, wait_for: Optional[str] = None, timeout_s: Optional[float] = None) -> dict:
        if wait_for is not None:
            timeout = float(timeout_s or 0.0)
            deadline = time.time() + timeout
            with self._status_cv:
                while True:
                    if self._status is not None and self._status == wait_for:
                        break
                    remaining = deadline - time.time()
                    if remaining <= 0:
                        break
                    self._status_cv.wait(timeout=remaining)
                status = self._status
                stamp = self._status_ts
        else:
            with self._status_cv:
                status = self._status
                stamp = self._status_ts
        age_s = None if stamp is None else max(0.0, time.time() - stamp)
        return {
            "status": status,
            "updated_at": stamp,
            "age_s": age_s,
            "wait_for": wait_for,
            "matched": bool(wait_for) and status == wait_for,
        }

    def get_odom(self) -> Optional[dict]:
        with self._lock:
            odom = None if self._odom is None else dict(self._odom)
            stamp = self._odom_ts
        if odom is None:
            return None
        odom["updated_at"] = stamp
        odom["age_s"] = None if stamp is None else max(0.0, time.time() - stamp)
        return odom

    def close(self) -> None:
        for unsub in (self._status_sub, self._odom_sub):
            try:
                unsub.unsubscribe()
            except Exception:
                pass
        for pub in (self._goal_pub, self._cancel_pub, self._simple_pub, self._complex_pub):
            try:
                pub.unadvertise()
            except Exception:
                pass
        if self._owns_client:
            try:
                self._client.terminate()
            except Exception:
                pass

    def __enter__(self) -> "Lite3BasicGoal":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
