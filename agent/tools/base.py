from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class ToolResult:
    ok: bool
    tool: str
    result: Optional[dict] = None
    error: Optional[str] = None
    events: list[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)

    def to_str(self) -> str:
        payload: dict[str, Any] = {"ok": self.ok, "tool": self.tool}
        if self.result is not None:
            payload["result"] = self.result
        if self.error is not None:
            payload["error"] = self.error
        if self.events:
            payload["events"] = self.events
        return json.dumps(payload)


@dataclass
class ToolContext:
    memory: Any    # MemoryStore
    robot: Any     # Lite3Robot
    motion: Any    # Lite3Motion | None
    follow: Any    # Lite3Follow | None
    basic_goal: Any  # Lite3BasicGoal | None
    vlm: Any       # VLMClientProtocol
    safety: Any    # SafetySupervisor
    map_runtime: Any = None
    waypoints: Any = None
