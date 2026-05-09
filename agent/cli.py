"""Agent loop driving the Lite3 via Bedrock Converse + tool use.

Run:
    cd /Users/maria/Desktop/RoboHack/agent
    pip install -r requirements.txt
    # set AWS_BEARER_TOKEN_BEDROCK and AWS_REGION in .env
    python cli.py
"""

from __future__ import annotations

import json
import os
import sys

from dotenv import load_dotenv

# Allow running as `python cli.py` from the agent/ folder.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot import Lite3Follow, Lite3Motion, Lite3Robot  # noqa: E402
from tools import TOOL_SCHEMAS, dispatch  # noqa: E402
from vlm import make_client, _model_id  # noqa: E402


SYSTEM_PROMPT = """\
You are the assistant brain of a quadruped guide-dog robot helping a blind \
or low-vision user. The robot has an RGBD camera at knee height. It can \
walk/turn (cmd_vel) and follow a specific person (YOLO tracker). Always:

- Call tools to look at the world; never guess what is in front of the robot.
- Before driving, describe_scene or get_rgbd_summary if there's any chance of obstacles.
- Use small motion durations: 0.5–2 s. Speeds default to gentle (0.15 m/s, 0.4 rad/s).
- After moving, briefly say what you just did.
- If the user says "stop", call stop_motion or stop_following immediately, then ask what they need.

Follow flow:
- When the user wants to follow someone (e.g. 'follow me', 'follow the person in red'):
  1. Call list_people. The result has yolo_ids + a free-form VLM description per id.
  2. Match the user's description to one yolo_id. If only one person is detected and the
     user said 'follow me', that's the one. If ambiguous, ask the user briefly.
  3. Call follow_person with that yolo_id.
- To stop following, call stop_following. The tracker also keeps moving the dog while a
  target is set, so you usually don't need walk/turn while following.

Be concise. Name objects, give rough positions (left / centered / right).
If a tool fails or returns 'error', tell the user plainly.
"""


def run_once(user_text: str, robot: Lite3Robot, motion, follow, max_steps: int = 6) -> str:
    bedrock = make_client()
    model_id = _model_id()
    tool_config = {"tools": TOOL_SCHEMAS}
    system = [{"text": SYSTEM_PROMPT}]

    messages: list[dict] = [
        {"role": "user", "content": [{"text": user_text}]}
    ]

    for _ in range(max_steps):
        resp = bedrock.converse(
            modelId=model_id,
            system=system,
            messages=messages,
            toolConfig=tool_config,
            inferenceConfig={"maxTokens": 1024},
        )

        out_msg = resp["output"]["message"]
        messages.append(out_msg)
        stop = resp.get("stopReason")

        if stop != "tool_use":
            text = "".join(p.get("text", "") for p in out_msg["content"] if "text" in p)
            return text

        tool_results = []
        for part in out_msg["content"]:
            if "toolUse" not in part:
                continue
            tu = part["toolUse"]
            name = tu["name"]
            args = tu.get("input") or {}
            print(f"  → tool: {name}({args})", file=sys.stderr)
            result = dispatch(name, args, robot, motion, follow, bedrock)
            print(f"  ← {result[:200]}", file=sys.stderr)
            tool_results.append(
                {
                    "toolResult": {
                        "toolUseId": tu["toolUseId"],
                        "content": [{"text": result}],
                    }
                }
            )

        messages.append({"role": "user", "content": tool_results})

    return "(agent: hit max_steps without final answer)"


def main() -> None:
    load_dotenv()
    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    cam_port = int(os.environ.get("ROS_BRIDGE_PORT", "9090"))
    motion_port = int(os.environ.get("ROS2_BRIDGE_PORT", "9091"))

    print(f"connecting to camera bridge ws://{host}:{cam_port} …", file=sys.stderr)
    robot = Lite3Robot(host=host, port=cam_port)

    motion = None
    follow = None
    try:
        print(f"connecting to motion bridge ws://{host}:{motion_port} …", file=sys.stderr)
        motion = Lite3Motion(host=host, port=motion_port)
    except Exception as e:
        print(f"motion bridge unavailable: {e}", file=sys.stderr)
    try:
        print(f"connecting to follow tracker ws://{host}:{motion_port} …", file=sys.stderr)
        follow = Lite3Follow(host=host, port=motion_port)
    except Exception as e:
        print(f"follow tracker unavailable: {e}", file=sys.stderr)

    try:
        print("connected. type a question (Ctrl-D to exit).", file=sys.stderr)
        while True:
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print(file=sys.stderr)
                return
            if not line:
                continue
            try:
                answer = run_once(line, robot, motion, follow)
                print(answer)
            except KeyboardInterrupt:
                # User hit Ctrl-C mid-tool. Stop motion and following, ask again.
                print("\n[interrupted — stopping motion + follow]", file=sys.stderr)
                if follow is not None:
                    try: follow.stop()
                    except Exception: pass
                if motion is not None:
                    try: motion.stop()
                    except Exception: pass
    finally:
        # Always tell the robot to stand still on shutdown.
        if follow is not None:
            try: follow.stop()
            except Exception: pass
            follow.close()
        if motion is not None:
            try: motion.stop()
            except Exception: pass
            motion.close()
        robot.close()


if __name__ == "__main__":
    main()
