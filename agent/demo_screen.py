"""Four-panel rolling display for the live demo. Run in a SECOND terminal.

Polls the JSON state file written by demo.py and re-renders every ~150 ms.
Designed to be readable from the back row at full-screen, large font.

Usage:
    python demo_screen.py             # default: poll the standard state path
    DEMO_STATE_PATH=/tmp/x.json python demo_screen.py
"""

from __future__ import annotations

import json
import os
import shutil
import sys
import time
from pathlib import Path

STATE_PATH = Path(os.environ.get("DEMO_STATE_PATH") or
                  "/tmp/robohack_demo_state.json")

# ANSI colors — plenty for a stage demo.
RESET = "\x1b[0m"
BOLD = "\x1b[1m"
DIM = "\x1b[2m"
RED = "\x1b[31m"
GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"
BLUE = "\x1b[34m"
MAGENTA = "\x1b[35m"
CYAN = "\x1b[36m"
WHITE = "\x1b[37m"
CLEAR = "\x1b[2J\x1b[H"


def term_size() -> tuple[int, int]:
    sz = shutil.get_terminal_size((100, 30))
    return sz.columns, sz.lines


def truncate(text: str, width: int) -> str:
    if len(text) <= width:
        return text
    return text[: max(0, width - 1)] + "…"


def render(state: dict | None) -> str:
    cols, rows = term_size()
    half_w = (cols - 3) // 2  # 2 vertical borders + middle separator
    panel_h = max(6, (rows - 4) // 2)  # 2 horizontal panels + borders + title

    out: list[str] = []
    title = state.get("title", "RoboHack demo") if state else "(waiting for demo to start)"
    out.append(BOLD + CYAN + f" {title} ".center(cols, "═") + RESET)

    if state is None:
        empty = "no state file yet — start demo.py in the other terminal."
        out.append("")
        out.append(DIM + empty.center(cols) + RESET)
        return "\n".join(out)

    # Panel 1 (top-left): conversation
    convo_lines = render_conversation(state.get("conversation", []), half_w, panel_h - 2)
    # Panel 2 (top-right): plan
    plan_lines = render_plan(state.get("plan", []), half_w, panel_h - 2)
    # Panel 3 (bottom-left): memory
    memory_lines = render_memory(state.get("memory", []), half_w, panel_h - 2)
    # Panel 4 (bottom-right): tools + monitor
    tools_lines = render_tools(state.get("tool", ""), state.get("monitor", {}), half_w, panel_h - 2)

    # Top row
    out.append(panel_top(half_w))
    out.append(panel_title_row("Conversation", "Plan", half_w))
    out.append(panel_separator(half_w))
    for left, right in zip_pad(convo_lines, plan_lines, panel_h - 2):
        out.append(f"│ {pad(left, half_w - 2)} │ {pad(right, half_w - 2)} │")

    # Mid separator joins both rows
    out.append(panel_separator(half_w))
    out.append(panel_title_row("Spatial memory", "Tool & monitoring", half_w))
    out.append(panel_separator(half_w))
    for left, right in zip_pad(memory_lines, tools_lines, panel_h - 2):
        out.append(f"│ {pad(left, half_w - 2)} │ {pad(right, half_w - 2)} │")
    out.append(panel_footer(half_w))

    return "\n".join(out)


def panel_top(half_w: int) -> str:
    return f"┌{'─' * half_w}┬{'─' * half_w}┐"


def panel_title_row(left: str, right: str, half_w: int) -> str:
    l = BOLD + truncate(left, half_w - 2) + RESET
    r = BOLD + truncate(right, half_w - 2) + RESET
    return f"│ {pad(l, half_w - 2)} │ {pad(r, half_w - 2)} │"


def panel_separator(half_w: int) -> str:
    return f"├{'─' * (half_w)}┼{'─' * (half_w)}┤"


def panel_footer(half_w: int) -> str:
    return f"└{'─' * (half_w)}┴{'─' * (half_w)}┘"


def pad(s: str, width: int) -> str:
    visible = strip_ansi(s)
    pad_len = max(0, width - len(visible))
    return s + " " * pad_len


def strip_ansi(s: str) -> str:
    out = []
    i = 0
    while i < len(s):
        if s[i] == "\x1b" and i + 1 < len(s) and s[i + 1] == "[":
            j = s.find("m", i + 2)
            if j == -1:
                break
            i = j + 1
        else:
            out.append(s[i])
            i += 1
    return "".join(out)


def zip_pad(a: list[str], b: list[str], n: int) -> list[tuple[str, str]]:
    out = []
    for i in range(n):
        out.append((a[i] if i < len(a) else "", b[i] if i < len(b) else ""))
    return out


def render_conversation(convo: list[dict], width: int, height: int) -> list[str]:
    lines: list[str] = []
    for entry in convo[-height:]:
        role = entry.get("role", "")
        text = entry.get("text", "")
        if role == "user":
            tag = MAGENTA + "you:" + RESET
        elif role == "agent":
            tag = GREEN + "🤖:" + RESET
        else:
            tag = role + ":"
        # Wrap manually to width
        wrapped = wrap(text, width - 6)
        for i, w in enumerate(wrapped):
            prefix = tag + " " if i == 0 else "    "
            lines.append(prefix + w)
    return lines[-height:]


def render_plan(plan: list[dict], width: int, height: int) -> list[str]:
    lines: list[str] = []
    for step in plan:
        status = step.get("status", "pending")
        text = step.get("text", "")
        if status == "done":
            mark = GREEN + "✓" + RESET
            text_color = DIM + text + RESET
        elif status == "active":
            mark = YELLOW + "→" + RESET
            text_color = BOLD + YELLOW + text + RESET
        else:
            mark = DIM + "·" + RESET
            text_color = DIM + text + RESET
        lines.append(f" {mark} {truncate(text_color + ' ' * 0, width)}")
    return lines[:height]


def render_memory(memory: list[dict], width: int, height: int) -> list[str]:
    if not memory:
        return [DIM + "(empty — robot hasn't scanned yet)" + RESET]
    lines: list[str] = []
    for item in memory[:height]:
        label = item.get("label", "?")
        position = item.get("position") or ""
        depth = item.get("depth_m")
        depth_s = f"{depth:.1f}m" if isinstance(depth, (int, float)) else "?"
        lines.append(f" {CYAN}{label}{RESET}  {position}  {DIM}{depth_s}{RESET}")
    return lines


def render_tools(tool: str, monitor: dict, width: int, height: int) -> list[str]:
    lines: list[str] = []
    if tool:
        lines.append(YELLOW + "active: " + RESET + tool)
    if monitor:
        for k, v in monitor.items():
            lines.append(f"  {DIM}{k}{RESET}: {v}")
    if not lines:
        lines = [DIM + "(idle)" + RESET]
    return lines[:height]


def wrap(text: str, width: int) -> list[str]:
    if width <= 0:
        return [text]
    out: list[str] = []
    line = ""
    for word in text.split():
        if len(line) + len(word) + 1 > width:
            if line:
                out.append(line)
            line = word
        else:
            line = (line + " " + word).strip()
    if line:
        out.append(line)
    return out or [""]


def main() -> None:
    last_mtime = -1.0
    last_render = ""
    print(CLEAR + "[demo screen] waiting for state at " + str(STATE_PATH))
    try:
        while True:
            try:
                mtime = STATE_PATH.stat().st_mtime
            except FileNotFoundError:
                mtime = -1.0
                state = None
            else:
                if mtime != last_mtime:
                    try:
                        state = json.loads(STATE_PATH.read_text())
                    except (json.JSONDecodeError, OSError):
                        state = None
                else:
                    state = None
            if state is not None or mtime != last_mtime:
                last_mtime = mtime
                if state is not None:
                    rendered = render(state)
                    if rendered != last_render:
                        sys.stdout.write(CLEAR + rendered)
                        sys.stdout.flush()
                        last_render = rendered
            time.sleep(0.15)
    except KeyboardInterrupt:
        sys.stdout.write(RESET + "\n")


if __name__ == "__main__":
    main()
