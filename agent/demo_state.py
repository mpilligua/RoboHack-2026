"""Shared state file for the demo: demo.py writes, demo_screen.py reads.

A simple JSON file in /tmp avoids any IPC complexity. The screen polls every
~150 ms; the demo writes whenever it advances a step or speaks a line.
"""

from __future__ import annotations

import json
import os
import tempfile
import threading
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path

STATE_PATH = Path(os.environ.get("DEMO_STATE_PATH") or
                  Path(tempfile.gettempdir()) / "robohack_demo_state.json")


@dataclass
class DemoState:
    """Mirrors the four-panel screen layout."""
    # Panel 1: conversation
    conversation: list[dict] = field(default_factory=list)  # [{role, text}]
    # Panel 2: plan checklist
    plan: list[dict] = field(default_factory=list)  # [{text, status: pending|active|done}]
    # Panel 3: spatial memory
    memory: list[dict] = field(default_factory=list)  # [{label, position, depth_m}]
    # Panel 4: current tool + monitoring
    tool: str = ""
    monitor: dict = field(default_factory=dict)
    # Bookkeeping
    title: str = "RoboHack — agentic guide-dog demo"
    updated_ts: float = 0.0


_state = DemoState()
_lock = threading.Lock()


def _write_locked() -> None:
    _state.updated_ts = time.time()
    tmp = STATE_PATH.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(asdict(_state), indent=2))
    tmp.replace(STATE_PATH)


def reset_plan(steps: list[str]) -> None:
    with _lock:
        _state.plan = [{"text": s, "status": "pending"} for s in steps]
        _state.conversation.clear()
        _state.memory.clear()
        _state.tool = ""
        _state.monitor = {}
        _write_locked()


def set_step(idx: int, status: str) -> None:
    """status: pending | active | done"""
    with _lock:
        if 0 <= idx < len(_state.plan):
            _state.plan[idx]["status"] = status
            _write_locked()


def advance(idx: int) -> None:
    """Mark idx as active and everything before it as done."""
    with _lock:
        for i, step in enumerate(_state.plan):
            if i < idx:
                step["status"] = "done"
            elif i == idx:
                step["status"] = "active"
            else:
                step["status"] = "pending"
        _write_locked()


def say_user(text: str) -> None:
    with _lock:
        _state.conversation.append({"role": "user", "text": text})
        _write_locked()


def say_agent(text: str) -> None:
    with _lock:
        _state.conversation.append({"role": "agent", "text": text})
        _write_locked()


def set_tool(label: str, **monitor_kwargs) -> None:
    with _lock:
        _state.tool = label
        _state.monitor = monitor_kwargs
        _write_locked()


def set_memory(items: list[dict]) -> None:
    """items: [{label, position, depth_m}]"""
    with _lock:
        _state.memory = items
        _write_locked()


def clear_state() -> None:
    """Wipe the state file on demo exit so the screen shows nothing stale."""
    try:
        STATE_PATH.unlink(missing_ok=True)
    except OSError:
        pass
