"""Shared rosbridge websocket client helpers."""

from __future__ import annotations

import roslibpy


def connect_ros2_rosbridge(
    host: str,
    port: int,
    *,
    connect_timeout_s: float = 10.0,
) -> roslibpy.Ros:
    """Open one ROS 2 (foxy) rosbridge websocket session.

    Pass the returned client into Lite3Motion / Lite3Follow so both share a
    single socket to ``ROS2_BRIDGE_PORT``.
    """
    client = roslibpy.Ros(host=host, port=port)
    client.run(timeout=connect_timeout_s)
    if not client.is_connected:
        raise RuntimeError(
            f"Could not connect to foxy rosbridge at ws://{host}:{port}. "
            "Is ros2 launch rosbridge_server rosbridge_websocket_launch.xml running?"
        )
    return client
