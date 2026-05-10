from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class WaypointRecord:
    name: str
    type: str
    pose: dict
    frame_id: str
    base_frame: str
    created_at: float
    map_signature: str


class WaypointStore:
    """Session-scoped in-memory waypoint store."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._waypoints: dict[str, WaypointRecord] = {}

    def save(
        self,
        *,
        name: str,
        waypoint_type: str,
        pose: dict,
        frame_id: str,
        base_frame: str,
        map_signature: str,
    ) -> WaypointRecord:
        with self._lock:
            if name in self._waypoints:
                raise ValueError(f"waypoint {name!r} already exists")
            record = WaypointRecord(
                name=name,
                type=waypoint_type,
                pose=dict(pose),
                frame_id=frame_id,
                base_frame=base_frame,
                created_at=time.time(),
                map_signature=map_signature,
            )
            self._waypoints[name] = record
            return record

    def list(self) -> list[WaypointRecord]:
        with self._lock:
            return [self._copy(r) for r in sorted(self._waypoints.values(), key=lambda r: r.name.lower())]

    def get(self, name: str) -> Optional[WaypointRecord]:
        with self._lock:
            record = self._waypoints.get(name)
            return None if record is None else self._copy(record)

    def _copy(self, record: WaypointRecord) -> WaypointRecord:
        return WaypointRecord(
            name=record.name,
            type=record.type,
            pose=dict(record.pose),
            frame_id=record.frame_id,
            base_frame=record.base_frame,
            created_at=record.created_at,
            map_signature=record.map_signature,
        )
