from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class RobotStateSnapshot:
    depth_stamp: Optional[float] = None
    depth_min_mm: Optional[int] = None
    depth_center_mm: Optional[int] = None
    nearest_obstacle_mm: Optional[int] = None
    rosbridge_connected: bool = False
    motion_connected: bool = False
    follow_connected: bool = False
    last_updated: float = field(default_factory=time.time)


@dataclass
class ObjectRecord:
    yolo_id: int
    label: str
    description: str
    bbox: list  # [x1, y1, x2, y2] in RGB pixels
    confidence: Optional[float] = None
    depth_m: Optional[float] = None
    position_text: Optional[str] = None  # "left" | "center" | "right"
    last_seen_ts: float = field(default_factory=time.time)
    seen_count: int = 1


@dataclass
class ActiveGoal:
    description: str
    status: str = "active"  # active | complete | failed | cancelled
    selected_object_id: Optional[int] = None
    created_ts: float = field(default_factory=time.time)


@dataclass
class Event:
    timestamp: float
    type: str   # tool_call | tool_result | safety_block | goal_update | tool_error | tool_blocked
    source: str
    message: str
    data: Optional[dict] = None
