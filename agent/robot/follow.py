"""Lite3 follow-mode adapter.

Talks to the patched run_tracker.py over the ROS 2 foxy rosbridge:
  /agent/yolo_detections (std_msgs/String, JSON list of {id, bbox, conf})
  /agent/follow_target   (std_msgs/String, "<id>" or "stop")

The frame for VLM matching comes from the regular RealSense topic via
Lite3Robot (noetic bridge); we don't double-publish the image here.
"""

from __future__ import annotations

import json
import threading
from dataclasses import dataclass
from typing import Optional

import roslibpy


DET_TOPIC = "/agent/yolo_detections"
TARGET_TOPIC = "/agent/follow_target"


@dataclass
class Detection:
    id: int
    bbox: list[float]  # [x1, y1, x2, y2]
    conf: Optional[float]
    label: Optional[str] = None  # YOLO COCO class name, e.g. "chair", "person"
    cls: Optional[int] = None    # YOLO class index


class Lite3Follow:
    def __init__(self, host: str = "192.168.1.103", port: int = 9091, connect_timeout_s: float = 10.0):
        self._client = roslibpy.Ros(host=host, port=port)
        self._lock = threading.Lock()
        self._dets: list[Detection] = []

        self._client.run(timeout=connect_timeout_s)
        if not self._client.is_connected:
            raise RuntimeError(
                f"Could not connect to foxy rosbridge at ws://{host}:{port}"
            )

        self._det_sub = roslibpy.Topic(self._client, DET_TOPIC, "std_msgs/String")
        self._det_sub.subscribe(self._on_dets)

        self._target_pub = roslibpy.Topic(self._client, TARGET_TOPIC, "std_msgs/String")
        self._target_pub.advertise()

    def _on_dets(self, msg: dict) -> None:
        try:
            payload = json.loads(msg["data"])
            dets = [
                Detection(
                    id=int(d["id"]),
                    bbox=list(d["bbox"]),
                    conf=d.get("conf"),
                    label=d.get("label"),
                    cls=d.get("cls"),
                )
                for d in payload
            ]
        except Exception:
            return
        with self._lock:
            self._dets = dets

    def get_detections(self) -> list[Detection]:
        with self._lock:
            return list(self._dets)

    def follow(self, target_id: int) -> None:
        """Follow a target indefinitely (won't auto-stop)."""
        self._target_pub.publish(roslibpy.Message({"data": str(int(target_id))}))

    def goto(self, target_id: int) -> None:
        """Drive toward a target and auto-stop when close (bbox ≥ 20% of frame)."""
        self._target_pub.publish(
            roslibpy.Message({"data": f"goto:{int(target_id)}"})
        )

    def stop(self) -> None:
        self._target_pub.publish(roslibpy.Message({"data": "stop"}))

    def close(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
        try:
            self._det_sub.unsubscribe()
        except Exception:
            pass
        try:
            self._target_pub.unadvertise()
        except Exception:
            pass
        try:
            self._client.terminate()
        except Exception:
            pass  # roslibpy 2.0 cleanup bug; harmless

    def __enter__(self) -> "Lite3Follow":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
