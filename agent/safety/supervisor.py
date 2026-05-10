from __future__ import annotations

import threading
import time
from dataclasses import dataclass

OBSTACLE_BLOCK_MM = 400
DEPTH_STALE_S = 5.0


@dataclass
class SafetyResult:
    safe: bool
    reason: str


class SafetySupervisor:
    def __init__(self, memory) -> None:
        self._memory = memory
        self._emergency_latched = False
        self._lock = threading.Lock()

    def check_forward_motion(self) -> SafetyResult:
        """Forward-motion policy.

        Emergency stop remains a hard block. Depth-derived obstacle data is
        advisory only because this robot's depth is noisy enough to create
        false positives.
        """
        with self._lock:
            if self._emergency_latched:
                return SafetyResult(False, "emergency stop is latched — call reset_emergency_stop to resume")

        state = self._memory.get_robot_state()
        now = time.time()

        if state.depth_stamp is None or (now - state.depth_stamp) > DEPTH_STALE_S:
            return SafetyResult(True, "depth data stale or missing — advisory only")

        if state.nearest_obstacle_mm is not None and state.nearest_obstacle_mm < OBSTACLE_BLOCK_MM:
            return SafetyResult(
                True,
                f"depth reports obstacle {state.nearest_obstacle_mm}mm ahead (threshold {OBSTACLE_BLOCK_MM}mm) — advisory only"
            )

        return SafetyResult(True, "ok")

    def check_any_motion(self) -> SafetyResult:
        """Used for turn/backward — only emergency latch blocks these."""
        with self._lock:
            if self._emergency_latched:
                return SafetyResult(False, "emergency stop is latched — call reset_emergency_stop to resume")
        return SafetyResult(True, "ok")

    def trigger_emergency_stop(self) -> None:
        with self._lock:
            self._emergency_latched = True

    def reset_emergency_stop(self) -> None:
        with self._lock:
            self._emergency_latched = False

    def is_emergency_latched(self) -> bool:
        with self._lock:
            return self._emergency_latched
