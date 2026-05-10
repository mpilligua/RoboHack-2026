"""Canned scene conversation shared by demo.py and voice_server.py --demo.

`SCENES_CONVO` is a list of (user_line, agent_line) pairs in the order they
should fire. demo.py uses the conversational lines for the four-panel UI;
voice_server's --demo mode uses them to spoof the /talk endpoint so the
phone shows real-looking transcripts + replies without ever hitting the LLM
or whisper.
"""

from __future__ import annotations

USER_NAME = "Maria"


# Each tuple is (what the presenter says, what the robot replies).
# Index N in this list = scene N+1 in the on-stage flow.
SCENES_CONVO: list[tuple[str, str]] = [
    # 1. Complex task setup
    (
        "Help me present: take me to the center, remember what's around us, "
        "find me somewhere to sit afterward, and then follow the speaker off stage.",
        f"Got it, {USER_NAME}. I'll handle that as a multi-step task: first "
        "I'll walk you to the center, then scan and remember the stage, "
        "then find you a seat when you're done, and finally follow the speaker out.",
    ),
    # 2. Walk to center (no user line — robot just narrates as it moves)
    (
        "(takes the first step toward the center)",
        "Heading to the center now.",
    ),
    (
        "(continues walking)",
        "Halfway there, the path's clear.",
    ),
    (
        "(arrives, turns to face the audience)",
        "I'm at the center.",
    ),
    # 3. Scan + remember
    (
        "(scanning the stage)",
        "Now scanning the stage so I can remember what's around us.",
    ),
    (
        "(finishing the scan)",
        "Done. I've stored what I saw — a chair to your left, the audience in front, "
        "and an exit path behind us.",
    ),
    # 4. Query memory
    (
        "What's around me?",
        f"You're at the center, {USER_NAME}. The audience is in front of you, "
        "a chair is about three meters to your left, and the exit is behind us.",
    ),
    # 5. Find a seat
    (
        "I'm done. Find me somewhere to sit.",
        "I remember a chair to your left. Let me guide you there.",
    ),
    (
        "(walking toward the chair)",
        "Walking over now.",
    ),
    (
        "(closing in on the chair)",
        "Almost there, the chair's right in front of you.",
    ),
    (
        "(arrived at the chair)",
        "We've arrived. The chair is directly in front of you. You can reach forward slowly.",
    ),
    # 6. Follow speaker off stage
    (
        "Hi Maria, follow me off stage.",
        "Got it. Tracking the speaker now and using the exit path I remembered.",
    ),
    (
        "(walking out together)",
        "We've exited the stage safely.",
    ),
]
