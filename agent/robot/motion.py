"""Motion adapter: publishes /cmd_vel on the ROS 2 foxy graph via rosbridge.

The Lite3 motion stack runs on ROS 2 (foxy), so this connects to a separate
rosbridge instance on a different port from the perception (noetic) one.

Run the foxy bridge on the robot first:
    source /opt/ros/foxy/setup.bash
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091

Conservative defaults are enforced. Anything bigger is clamped.
"""

from __future__ import annotations

import threading
import time

import roslibpy


CMD_VEL_TOPIC = "/cmd_vel"

MAX_LINEAR = 0.3   # m/s
MAX_ANGULAR = 0.5  # rad/s
MAX_DURATION = 2.0  # seconds per command


def _clip(v: float, hi: float) -> float:
    return max(-hi, min(hi, float(v)))


class Lite3Motion:
    def __init__(self, host: str = "192.168.1.103", port: int = 9091, connect_timeout_s: float = 10.0):
        self._client = roslibpy.Ros(host=host, port=port)
        self._client.run(timeout=connect_timeout_s)
        if not self._client.is_connected:
            raise RuntimeError(
                f"Could not connect to foxy rosbridge at ws://{host}:{port}. "
                "Is `ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091` running?"
            )
        self._cmd_vel = roslibpy.Topic(self._client, CMD_VEL_TOPIC, "geometry_msgs/Twist")
        self._cmd_vel.advertise()
        self._stop_flag = threading.Event()

    def _publish(self, vx: float, vy: float, omega: float) -> None:
        msg = roslibpy.Message(
            {
                "linear": {"x": float(vx), "y": float(vy), "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": float(omega)},
            }
        )
        self._cmd_vel.publish(msg)

    def stop(self) -> None:
        self._stop_flag.set()
        # Send a few zero commands to make sure the controller hears it.
        for _ in range(3):
            self._publish(0.0, 0.0, 0.0)
            time.sleep(0.05)

    def _drive(self, vx: float, vy: float, omega: float, duration_s: float, hz: float = 20.0) -> None:
        vx = _clip(vx, MAX_LINEAR)
        vy = _clip(vy, MAX_LINEAR)
        omega = _clip(omega, MAX_ANGULAR)
        duration_s = max(0.0, min(MAX_DURATION, float(duration_s)))

        self._stop_flag.clear()
        period = 1.0 / hz
        deadline = time.time() + duration_s
        while time.time() < deadline and not self._stop_flag.is_set():
            self._publish(vx, vy, omega)
            time.sleep(period)
        # Always end with a zero — important so the dog doesn't keep coasting.
        self._publish(0.0, 0.0, 0.0)

    def forward(self, speed: float = 0.15, duration_s: float = 1.0) -> None:
        self._drive(speed, 0.0, 0.0, duration_s)

    def backward(self, speed: float = 0.15, duration_s: float = 1.0) -> None:
        self._drive(-abs(speed), 0.0, 0.0, duration_s)

    def turn_left(self, omega: float = 0.4, duration_s: float = 1.0) -> None:
        self._drive(0.0, 0.0, abs(omega), duration_s)

    def turn_right(self, omega: float = 0.4, duration_s: float = 1.0) -> None:
        self._drive(0.0, 0.0, -abs(omega), duration_s)

    def strafe_left(self, speed: float = 0.15, duration_s: float = 1.0) -> None:
        self._drive(0.0, abs(speed), 0.0, duration_s)

    def strafe_right(self, speed: float = 0.15, duration_s: float = 1.0) -> None:
        self._drive(0.0, -abs(speed), 0.0, duration_s)

    def close(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
        try:
            self._cmd_vel.unadvertise()
        except Exception:
            pass
        self._client.terminate()

    def __enter__(self) -> "Lite3Motion":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
