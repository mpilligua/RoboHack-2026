"""Live agentic demo: complex task → plan → execute → monitor → complete.

The user gives ONE high-level request: "Help me present, find me a seat, then
follow the speaker off stage." The robot narrates a plan and executes it across
five scenes. Real motion + real perception. Spoken lines are canned, sized for
TTS.

A second terminal running `demo_screen.py` shows the four-panel state
(conversation, plan, spatial memory, current tool).

Usage:
    cd agent && source .venv/bin/activate
    # Window 1:
    python demo_screen.py
    # Window 2:
    python demo.py             # full run with the robot
    python demo.py --dry       # rehearsal, no robot connection
    python demo.py --skip 3    # start at scene 3 (1-indexed)
"""

from __future__ import annotations

import argparse
import os
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Callable

from dotenv import load_dotenv

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import demo_state  # noqa: E402
import tts_engine  # noqa: E402
from memory.store import MemoryStore  # noqa: E402
from robot import (  # noqa: E402
    Lite3BasicGoal,
    Lite3Follow,
    Lite3Motion,
    Lite3Robot,
    connect_ros2_rosbridge,
)
from safety.supervisor import SafetySupervisor  # noqa: E402
from tools import follow_tools, motion_tools, perception_tools, safety_tools  # noqa: E402
from tools.base import ToolContext  # noqa: E402
from vlm_client import make_vlm_client  # noqa: E402


USER_NAME = (os.environ.get("USER_NAME") or "Maria").strip() or "Maria"


# ---------------------------------------------------------------------------
# Plan shown on the screen
# ---------------------------------------------------------------------------

PLAN_STEPS = [
    "Localize on stage",
    "Walk to center waypoint",
    "Scan surroundings",
    "Store landmarks in spatial memory",
    "Answer 'what's around me?'",
    "Find a chair on user request",
    "Guide user to the chair",
    "Track speaker entering stage",
    "Follow speaker to the exit",
    "Confirm safe arrival",
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


_SILENT = False  # set True by --silent to skip TTS playback (rehearsal only)


def speak(text: str) -> None:
    """Print + speak via the shared tts_engine (same voice as voice_server)."""
    print(f"\n  🤖 {text}", flush=True)
    demo_state.say_agent(text)
    if _SILENT:
        return
    tts_engine.speak(text)


def user_line(text: str) -> None:
    """Log a line for the conversation panel without speaking."""
    print(f"\n  🎤 {USER_NAME}: {text}", flush=True)
    demo_state.say_user(text)


def cue(label: str) -> None:
    print(f"\n── {label} " + "─" * max(0, 60 - len(label)), file=sys.stderr)


_AUTO_DELAY_S = 0.0  # set >0 by --auto to skip the PTT gate


_POST_KEY_PAUSE_S = 2.0  # natural beat between presenter finishing + robot replying


def wait_for_ptt(prompt: str) -> None:
    """Press-to-advance gate. Presenter speaks aloud (their own voice — no mic
    capture). When finished, they press any key. We then pause for ~2s so the
    transition feels conversational, then return so the robot speaks back."""
    if _AUTO_DELAY_S > 0:
        print(f"\n🎤 [auto-advance after {_AUTO_DELAY_S:.1f}s] {prompt!r}", flush=True)
        time.sleep(_AUTO_DELAY_S)
        return
    print(f"\n🎤 SAY: {prompt}", flush=True)
    print("   [press any key when you've finished speaking]", flush=True)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ch = sys.stdin.read(1)
        if ch in ("q", "\x03"):
            raise KeyboardInterrupt
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print(f"   [pause {_POST_KEY_PAUSE_S:.1f}s, then robot replies...]", flush=True)
    time.sleep(_POST_KEY_PAUSE_S)


def beat(seconds: float) -> None:
    time.sleep(seconds)


@dataclass
class DemoCtx:
    tool_ctx: ToolContext
    dry: bool

    def call(self, label: str, fn: Callable[[], object], **monitor) -> object | None:
        """Run a real tool call; show on the panel; survive errors."""
        cue(f"tool: {label}")
        demo_state.set_tool(label, **monitor)
        if self.dry:
            print(f"   [dry-run: would call {label}]", file=sys.stderr)
            return None
        try:
            result = fn()
            print(f"   {label} -> {str(result)[:200]}", file=sys.stderr)
            return result
        except Exception as exc:
            print(f"   {label} FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
            return None

    def memory_snapshot(self) -> list[dict]:
        """Pull the current spatial-memory items for the panel."""
        try:
            snap = self.tool_ctx.memory.snapshot()
            objs = snap.get("objects", []) or []
            return [
                {
                    "label": o.get("label", "?"),
                    "position": o.get("position") or "",
                    "depth_m": o.get("depth_m"),
                }
                for o in objs[:6]
            ]
        except Exception:
            return []


# ---------------------------------------------------------------------------
# Scenes
# ---------------------------------------------------------------------------


def scene_1_complex_task(_d: DemoCtx) -> None:
    cue("SCENE 1 — complex task setup")
    demo_state.advance(0)
    wait_for_ptt(f"{USER_NAME}: 'Help me present: take me to the center, "
                 "remember what's around us, find me somewhere to sit afterward, "
                 "and then follow the speaker off stage.'")
    user_line("Help me present: take me to the center, remember what's around us, "
              "find me somewhere to sit afterward, and then follow the speaker off stage.")
    speak(f"Got it, {USER_NAME}. I'll handle that as a multi-step task: first "
          "I'll walk you to the center, then scan and remember the stage, "
          "then find you a seat when you're done, and finally follow the speaker out.")


def scene_2_walk_to_center(d: DemoCtx) -> None:
    cue("SCENE 2 — walk to center")
    demo_state.advance(1)
    speak("Heading to the center now.")
    d.call("walk_forward(distance_m=1.0)",
           lambda: motion_tools.handle_walk_forward(d.tool_ctx, {"distance_m": 1.0}),
           remaining_m=1.0, path_clear=True)
    # Monitoring beat — splits the walk so the audience hears progress narration.
    beat(0.2)
    speak("Halfway there, the path's clear.")
    d.call("walk_forward(distance_m=1.0)",
           lambda: motion_tools.handle_walk_forward(d.tool_ctx, {"distance_m": 1.0}),
           remaining_m=0.0, path_clear=True)
    beat(0.2)
    speak("Turning to face you.")
    d.call("turn_right(angle_deg=90)",
           lambda: motion_tools.handle_turn_right(d.tool_ctx, {"angle_deg": 90}))
    beat(0.2)
    speak("I'm at the center.")


def scene_3_scan_and_remember(d: DemoCtx) -> None:
    cue("SCENE 3 — scan + remember")
    demo_state.advance(2)
    speak("Now scanning the stage so I can remember what's around us.")
    d.call("describe_scene()",
           lambda: perception_tools.handle_describe_scene(d.tool_ctx, {}))
    # Slow scan sweeps so YOLO + world_tick have a chance to populate world memory.
    d.call("turn_left(angle_deg=30)",
           lambda: motion_tools.handle_turn_left(d.tool_ctx, {"angle_deg": 30}))
    beat(0.6)
    d.call("turn_right(angle_deg=60)",
           lambda: motion_tools.handle_turn_right(d.tool_ctx, {"angle_deg": 60}))
    beat(0.6)
    d.call("turn_left(angle_deg=30)",
           lambda: motion_tools.handle_turn_left(d.tool_ctx, {"angle_deg": 30}))
    demo_state.advance(3)
    # Pull the actual memory contents into the panel.
    items = d.memory_snapshot()
    demo_state.set_memory(items)
    if items:
        # Read out the first 2 things we actually remember.
        bits = ", ".join(
            f"a {it['label']} {it['position'] or 'nearby'}"
            for it in items[:2]
            if it.get("label")
        )
        speak(f"Done. I've stored what I saw — {bits}." if bits else
              "Done. I've stored what I saw.")
    else:
        # Fallback canned line (vision pipeline silent).
        speak("Done. I've stored what I saw — a chair to your left, the audience in front, "
              "and an exit path behind us.")


def scene_4_whats_around(d: DemoCtx) -> None:
    cue("SCENE 4 — query memory")
    demo_state.advance(4)
    wait_for_ptt(f"{USER_NAME}: 'What's around me?'")
    user_line("What's around me?")
    items = d.memory_snapshot()
    demo_state.set_memory(items)
    d.call("query_spatial_memory(radius=5m)", lambda: items, count=len(items))
    if items:
        # Build a natural sentence from the top 2-3 items.
        parts = []
        for it in items[:3]:
            label = it.get("label") or "something"
            pos = it.get("position") or ""
            depth = it.get("depth_m")
            depth_phrase = f" about {depth:.0f} meters" if isinstance(depth, (int, float)) and depth > 0 else ""
            if pos:
                parts.append(f"a {label} to your {pos}{depth_phrase}")
            else:
                parts.append(f"a {label}{depth_phrase}")
        speak("You're at the center. I can see " + ", and ".join(parts) + ".")
    else:
        speak(f"You're at the center, {USER_NAME}. The audience is in front of you, "
              "a chair is about three meters to your left, and the exit is behind us.")


def scene_5_find_a_seat(d: DemoCtx) -> None:
    cue("SCENE 5 — find a seat")
    demo_state.advance(5)
    wait_for_ptt(f"{USER_NAME}: 'I'm done. Find me somewhere to sit.'")
    user_line("I'm done. Find me somewhere to sit.")
    speak("I remember a chair to your left. Let me guide you there.")
    demo_state.advance(6)
    # find_and_go_to is one blocking call: rotates until the chair is in view,
    # then drives toward it with depth-based auto-stop. No hardcoded distance,
    # no second walk_forward needed. The call itself takes a few seconds, so
    # we narrate before AND after rather than mid-action.
    speak("Walking over now.")
    # stop_distance_m is the depth threshold at which the robot halts:
    # the bbox median depth reaching this value triggers stop. Larger value
    # = stop SOONER (further from the chair). Default 0.8 m had the robot
    # touching the chair, so we use 1.2 m to leave roughly a 0.3 m gap once
    # the seat/armrest depth + low-angle perspective are accounted for.
    d.call("find_and_go_to(label='chair', stop_distance_m=1.2)",
           lambda: follow_tools.handle_find_and_go_to(d.tool_ctx, {
               "label": "chair", "stop_distance_m": 1.2,
           }),
           goal="chair", stop_distance_m=1.2)
    beat(0.3)
    speak("We've arrived. The chair is directly in front of you. You can reach forward slowly.")


def scene_6_follow_speaker(d: DemoCtx) -> None:
    cue("SCENE 6 — follow speaker off stage")
    demo_state.advance(7)
    wait_for_ptt('Pau (entering): "Hi Maria, follow me off stage."')
    user_line("(speaker) Hi Maria, follow me off stage.")
    speak("Got it. Tracking the speaker now and using the exit path I remembered.")
    demo_state.advance(8)
    d.call("track_person('speaker') + maintain_distance(1.0m)",
           lambda: follow_tools.handle_find_person_and_follow(d.tool_ctx, {}),
           target="speaker", distance_m="1.0")
    beat(8.0)  # Pau walks out; robot follows.
    demo_state.advance(9)
    d.call("stop()",
           lambda: safety_tools.handle_stop(d.tool_ctx, {}))
    speak("We've exited the stage safely.")


SCENES = [
    scene_1_complex_task,
    scene_2_walk_to_center,
    scene_3_scan_and_remember,
    scene_4_whats_around,
    scene_5_find_a_seat,
    scene_6_follow_speaker,
]


# ---------------------------------------------------------------------------
# Bring-up / teardown
# ---------------------------------------------------------------------------


def build_tool_ctx(dry: bool) -> tuple[ToolContext, list[Callable[[], None]]]:
    cleanups: list[Callable[[], None]] = []

    if dry:
        memory = MemoryStore()
        ctx = ToolContext(
            memory=memory,
            robot=None, motion=None, follow=None, basic_goal=None,
            vlm=None, safety=SafetySupervisor(memory),
        )
        return ctx, cleanups

    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    port = int(os.environ.get("ROS2_BRIDGE_PORT", "9091"))

    print(f"[demo] connecting to ROS 2 bridge ws://{host}:{port} ...", file=sys.stderr)
    ros2 = connect_ros2_rosbridge(host, port)
    cleanups.append(lambda: ros2.terminate())

    robot = Lite3Robot(host=host, port=port, ros_client=ros2)
    cleanups.append(robot.close)

    motion = Lite3Motion(ros_client=ros2)
    cleanups.append(motion.close)
    cleanups.append(motion.stop)

    follow = Lite3Follow(ros_client=ros2)
    cleanups.append(follow.close)
    cleanups.append(follow.stop)

    basic_goal = Lite3BasicGoal(ros_client=ros2)
    cleanups.append(basic_goal.close)

    memory = MemoryStore()
    safety = SafetySupervisor(memory)
    vlm = make_vlm_client()

    ctx = ToolContext(
        memory=memory, robot=robot, motion=motion, follow=follow,
        basic_goal=basic_goal, vlm=vlm, safety=safety,
    )

    # Background world-tick is intentionally disabled for the demo: it spams
    # frame-fetch errors in the demo terminal under flaky wifi and isn't
    # needed since the spatial memory panel uses canned/scripted content.
    # Set ENABLE_WORLD_TICK=1 to opt back in.
    if os.environ.get("ENABLE_WORLD_TICK") == "1":
        try:
            from perception.world_tick import WorldTickDriver
            world_tick = WorldTickDriver(robot, follow, memory, period_s=1.0)
            world_tick.start()
            cleanups.append(world_tick.stop)
        except Exception as exc:
            print(f"[demo] world tick disabled: {exc}", file=sys.stderr)

    return ctx, cleanups


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry", action="store_true",
                        help="Skip robot connection — TTS + console only.")
    parser.add_argument("--skip", type=int, default=1,
                        help="Start from scene N (1-indexed).")
    parser.add_argument("--auto", type=float, nargs="?", const=1.5, default=0.0,
                        metavar="DELAY_S",
                        help="Auto-advance through PTT prompts (rehearsal mode). "
                             "Default delay 1.5s when flag given without value.")
    parser.add_argument("--silent", action="store_true",
                        help="Skip TTS playback (for non-interactive testing).")
    parser.add_argument("--pause", type=float, default=2.0,
                        metavar="SECONDS",
                        help="Pause after you press a key, before the robot replies. Default 2s.")
    args = parser.parse_args()

    global _AUTO_DELAY_S, _SILENT, _POST_KEY_PAUSE_S
    _AUTO_DELAY_S = args.auto
    _SILENT = args.silent
    _POST_KEY_PAUSE_S = args.pause

    load_dotenv()
    tool_ctx, cleanups = build_tool_ctx(args.dry)
    demo_ctx = DemoCtx(tool_ctx=tool_ctx, dry=args.dry)

    demo_state.reset_plan(PLAN_STEPS)
    cleanups.append(demo_state.clear_state)

    print("\n" + "=" * 70)
    print("  RoboHack — agentic guide-dog demo")
    print(f"  {len(SCENES)} scenes. Hold SPACE for each line, tap to advance.")
    print(f"  State file: {demo_state.STATE_PATH}")
    print("  (Run `python demo_screen.py` in a second terminal for the panel.)")
    print("  Ctrl-C aborts and stops the robot.")
    print("=" * 70)

    try:
        for idx, scene in enumerate(SCENES, start=1):
            if idx < max(args.skip, 1):
                continue
            scene(demo_ctx)
        print("\n[demo] complete.")
    except KeyboardInterrupt:
        print("\n[demo] interrupted — stopping robot.", file=sys.stderr)
    finally:
        for fn in reversed(cleanups):
            try:
                fn()
            except Exception:
                pass


if __name__ == "__main__":
    main()
