from __future__ import annotations

import threading
import time
from typing import Optional

from .schemas import ActiveGoal, Event, ObjectRecord, RobotStateSnapshot

_MAX_EVENTS = 200


class MemoryStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.robot: RobotStateSnapshot = RobotStateSnapshot()
        self._objects: dict[int, ObjectRecord] = {}
        self.goal: Optional[ActiveGoal] = None
        self._events: list[Event] = []
        self.current_plan: Optional[str] = None

    # --- robot state ---

    def update_robot_state(self, **kwargs) -> None:
        with self._lock:
            for k, v in kwargs.items():
                setattr(self.robot, k, v)
            self.robot.last_updated = time.time()

    def get_robot_state(self) -> RobotStateSnapshot:
        with self._lock:
            return self.robot

    # --- objects ---

    def upsert_object(self, record: ObjectRecord) -> None:
        with self._lock:
            existing = self._objects.get(record.yolo_id)
            if existing is not None:
                existing.label = record.label
                existing.description = record.description
                existing.bbox = record.bbox
                if record.confidence is not None:
                    existing.confidence = record.confidence
                if record.depth_m is not None:
                    existing.depth_m = record.depth_m
                if record.position_text is not None:
                    existing.position_text = record.position_text
                existing.last_seen_ts = record.last_seen_ts
                existing.seen_count += 1
            else:
                self._objects[record.yolo_id] = record

    def get_objects(self) -> list[ObjectRecord]:
        with self._lock:
            return sorted(self._objects.values(), key=lambda o: o.last_seen_ts, reverse=True)

    def get_object(self, yolo_id: int) -> Optional[ObjectRecord]:
        with self._lock:
            return self._objects.get(yolo_id)

    def find_objects_by_label(self, label: str) -> list[ObjectRecord]:
        label_lower = label.lower()
        with self._lock:
            return [o for o in self._objects.values() if label_lower in o.label.lower()]

    def resolve_reference(self, ref: str) -> Optional[ObjectRecord]:
        ref_lower = ref.strip().lower()
        with self._lock:
            # "it" / "that" → use selected_object_id from active goal
            if ref_lower in ("it", "that", "this") and self.goal and self.goal.selected_object_id is not None:
                return self._objects.get(self.goal.selected_object_id)
            # Try partial label match, prefer most recently seen
            matches = [o for o in self._objects.values() if ref_lower in o.label.lower()]
            if matches:
                return max(matches, key=lambda o: o.last_seen_ts)
            return None

    def find_objects_matching_constraints(self, constraints: dict) -> list[ObjectRecord]:
        with self._lock:
            results = list(self._objects.values())
        label = constraints.get("label", "").lower()
        position = constraints.get("position", "").lower()
        max_depth = constraints.get("max_depth_m")
        min_depth = constraints.get("min_depth_m")
        if label:
            results = [o for o in results if label in o.label.lower()]
        if position:
            results = [o for o in results if o.position_text == position]
        if max_depth is not None:
            results = [o for o in results if o.depth_m is not None and o.depth_m <= max_depth]
        if min_depth is not None:
            results = [o for o in results if o.depth_m is not None and o.depth_m >= min_depth]
        return sorted(results, key=lambda o: o.last_seen_ts, reverse=True)

    # --- goal ---

    def set_goal(self, goal: ActiveGoal) -> None:
        with self._lock:
            self.goal = goal

    def get_goal(self) -> Optional[ActiveGoal]:
        with self._lock:
            return self.goal

    def clear_goal(self) -> None:
        with self._lock:
            self.goal = None

    def set_selected_object_id(self, yolo_id: int | None) -> None:
        with self._lock:
            if self.goal is not None:
                self.goal.selected_object_id = yolo_id

    # --- events ---

    def add_event(self, event: Event) -> None:
        with self._lock:
            self._events.append(event)
            if len(self._events) > _MAX_EVENTS:
                self._events = self._events[-_MAX_EVENTS:]

    def last_events(self, n: int = 20) -> list[Event]:
        with self._lock:
            return list(self._events[-n:])

    # --- snapshot for agent context injection ---

    def snapshot(self) -> dict:
        with self._lock:
            robot = self.robot
            robot_dict = {
                "rosbridge_connected": robot.rosbridge_connected,
                "motion_connected": robot.motion_connected,
                "follow_connected": robot.follow_connected,
                "nearest_obstacle_mm": robot.nearest_obstacle_mm,
                "depth_center_mm": robot.depth_center_mm,
                "depth_fresh": (
                    robot.depth_stamp is not None
                    and (time.time() - robot.depth_stamp) < 5.0
                ),
            }
            objects_list = [
                {
                    "id": o.yolo_id,
                    "label": o.label,
                    "description": o.description,
                    "position": o.position_text,
                    "depth_m": o.depth_m,
                    "seen_count": o.seen_count,
                    "last_seen_s_ago": round(max(0.0, time.time() - o.last_seen_ts), 2),
                }
                for o in sorted(self._objects.values(), key=lambda x: x.last_seen_ts, reverse=True)
            ]
            goal_dict = None
            if self.goal:
                goal_dict = {
                    "description": self.goal.description,
                    "status": self.goal.status,
                    "selected_object_id": self.goal.selected_object_id,
                }
            recent = [
                {"type": e.type, "source": e.source, "message": e.message}
                for e in self._events[-10:]
            ]
        return {
            "robot": robot_dict,
            "objects": objects_list,
            "goal": goal_dict,
            "recent_events": recent,
        }
