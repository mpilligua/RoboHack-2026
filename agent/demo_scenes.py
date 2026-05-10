"""6 stage-demo scenes with prompt + reply + motion.

Each phone press in voice_server's --demo mode advances one scene. Each scene:
- prompt: what the presenter will say (shown as transcript in the phone UI)
- reply:  what the robot speaks back (TTS on phone + laptop)
- motion: a function that fires real tool handlers against the ToolContext.
          It runs on a worker thread, so the HTTP response is not blocked
          while the robot drives.

This is decoupled from demo.py's SCENES_CONVO + scene_*() functions: voice_server
uses these 6 scenes (one tap each); demo.py keeps its own 6 scenes for the
key-driven REPL with the four-panel UI. The two paths can share or diverge.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

from tools import follow_tools, motion_tools, perception_tools, safety_tools
from tools.base import ToolContext


USER_NAME = "Maria"


def _walk_forward(ctx: ToolContext, distance_m: float) -> None:
    motion_tools.handle_walk_forward(ctx, {"distance_m": distance_m})


def _turn_right(ctx: ToolContext, angle_deg: float) -> None:
    motion_tools.handle_turn_right(ctx, {"angle_deg": angle_deg})


def _turn_left(ctx: ToolContext, angle_deg: float) -> None:
    motion_tools.handle_turn_left(ctx, {"angle_deg": angle_deg})


# --- Scene motion functions. Each one is what the robot DOES on a phone tap.

def _motion_scene1(_ctx: ToolContext) -> None:
    """Setup. Robot doesn't move — just speaks the plan."""
    return


def _motion_scene2(ctx: ToolContext) -> None:
    """Walk to center, turn to face audience."""
    _walk_forward(ctx, 1.0)
    time.sleep(0.2)
    _walk_forward(ctx, 1.0)
    time.sleep(0.2)
    _turn_right(ctx, 90.0)


def _motion_scene3(ctx: ToolContext) -> None:
    """Scan the stage."""
    perception_tools.handle_describe_scene(ctx, {})
    _turn_left(ctx, 30.0)
    time.sleep(0.4)
    _turn_right(ctx, 60.0)
    time.sleep(0.4)
    _turn_left(ctx, 30.0)


def _motion_scene4(_ctx: ToolContext) -> None:
    """Query memory. Speech only."""
    return


def _motion_scene5(ctx: ToolContext) -> None:
    """Find the chair and walk to it; then back off so the person can sit,
    wait, and turn 180° to face the same direction the seated person faces.

    `stop_distance_m=1.5` is generous — the bbox-median depth heuristic tends
    to underestimate how close the chair edge actually is, so we stop early
    and add a small explicit back-step before the user sits."""
    follow_tools.handle_find_and_go_to(ctx, {
        "label": "chair",
        "stop_distance_m": 1.2,
    })
    # Defensive nudge backward in case auto-stop overshot. ~15 cm.
    motion_tools.handle_walk_backward(ctx, {"distance_m": 0.3})
    # Pause while the user sits.
    time.sleep(1.0)
    # About-face: now looking the same direction as the seated user.
    motion_tools.handle_turn_right(ctx, {"angle_deg": 180.0})


def _motion_scene6(ctx: ToolContext) -> None:
    """Find the speaker, then follow indefinitely until manually stopped.

    Uses a long sleep (10 min) as a safety timeout so the robot doesn't
    follow forever if we forget. In practice the operator stops it via the
    phone's red 'stop' button (POST /stop) once Maria is offstage."""
    follow_tools.handle_find_person_and_follow(ctx, {})
    time.sleep(600.0)  # 10 min safety timeout
    safety_tools.handle_stop(ctx, {})


@dataclass
class Scene:
    name: str
    prompt: str
    reply: str
    motion: Callable[[ToolContext], None]


SCENES: list[Scene] = [
    Scene(
        name="1. complex task",
        prompt=(
            "Help me present: take me to the center, remember what's around us, "
            "find me somewhere to sit afterward, and then follow the speaker off stage."
        ),
        reply=(
            f"Got it, {USER_NAME}. I'll handle that as a multi-step task: first "
            "I'll walk you to the center, then scan and remember the stage, "
            "then find you a seat when you're done, and finally follow the speaker out."
        ),
        motion=_motion_scene1,
    ),
    Scene(
        name="2. walk to center",
        prompt="(robot walks to the center, turning to face the audience)",
        reply="Heading to the center now. I'm at the center.",
        motion=_motion_scene2,
    ),
    Scene(
        name="3. scan + remember",
        prompt="(robot scans the stage)",
        reply=(
            "Now scanning the stage so I can remember what's around us. "
            "Done. I've stored a chair to your left, the audience in front, "
            "and an exit path behind us."
        ),
        motion=_motion_scene3,
    ),
    Scene(
        name="4. what's around me",
        prompt="What's around me?",
        reply=(
            f"You're at the center, {USER_NAME}. The audience is in front of you, "
            "a chair is about three meters to your left, and the exit is behind us."
        ),
        motion=_motion_scene4,
    ),
    Scene(
        name="5. find a seat",
        prompt="I'm done. Find me somewhere to sit.",
        reply=(
            "I remember a chair to your left. Let me guide you there. "
            "We've arrived. The chair is right in front of you."
        ),
        motion=_motion_scene5,
    ),
    Scene(
        name="6. follow speaker",
        prompt="Hi Maria, follow me off stage.",
        reply="Got it. Tracking the speaker now and using the exit path I remembered.",
        motion=_motion_scene6,
    ),
]
