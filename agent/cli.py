"""Agent loop driving the Lite3 via Bedrock Converse + tool use.

Run:
    cd /Users/maria/Desktop/RoboHack/agent
    pip install -r requirements.txt
    # set AWS_BEARER_TOKEN_BEDROCK and AWS_REGION in .env
    python cli.py
"""

from __future__ import annotations

import atexit
import json
import os
import sys
from pathlib import Path

from dotenv import load_dotenv

# Allow running as `python cli.py` from the agent/ folder.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot import (  # noqa: E402
    Lite3BasicGoal,
    Lite3Follow,
    Lite3Motion,
    Lite3Robot,
    connect_ros2_rosbridge,
)
from tools import TOOL_SCHEMAS, dispatch  # noqa: E402
from vlm import make_client, _model_id  # noqa: E402

_USE_NEW_PIPELINE = os.environ.get("LEGACY_LOOP", "0") != "1"
if _USE_NEW_PIPELINE:
    from vlm_client import make_vlm_client
    from memory.store import MemoryStore
    from safety.supervisor import SafetySupervisor
    from tools.base import ToolContext
    from tools.setup import build_registry
    from agents_app.sdk_agents import DialogueAgent, PlannerAgent, _make_agent_client, _use_openai_compat
    from agents_app.orchestrator import Orchestrator


SYSTEM_PROMPT = """\
You are the assistant brain of a quadruped guide-dog robot helping a blind \
or low-vision user. The robot has an RGBD camera at knee height. It can \
walk/turn (cmd_vel) and follow a specific person (YOLO tracker). Always:

- Call tools to look at the world; never guess what is in front of the robot.
- Before driving, describe_scene or get_rgbd_summary if there's any chance of obstacles.
- Use small motion durations: 0.5–2 s. Speeds default to gentle (0.15 m/s, 0.4 rad/s).
- After moving, briefly say what you just did.
- If the user says "stop", call stop_motion or stop_following immediately, then ask what they need.

Tracking flow:
- Always call list_visible_objects first to find the right yolo id (it has
  label + VLM description per id). If only one matching candidate, that's it;
  if multiple (two chairs), ask the user briefly.
- Then pick ONE of:
  - follow_person(yolo_id) — open-ended follow. Dog walks toward target as long
    as it's tracked. Use for 'follow me', 'follow that person'. Call stop_tracking
    when the user says stop.
  - go_to_object(yolo_id) — one-shot approach. Dog walks to it and auto-stops
    when close. Use for 'go to the chair', 'walk to the door'. Don't call
    stop_tracking afterwards — it stops on its own.
- stop_tracking halts both modes. Safe to call at any time.

Be concise. Name objects, give rough positions (left / centered / right).
If a tool fails or returns 'error', tell the user plainly.
"""


AGENT_DIR = Path(__file__).resolve().parent
CLI_LOCK_PATH = AGENT_DIR / ".cli.pid.lock"


def _parse_max_hz(env_key: str, default: float) -> float | None:
    """Positive limit in Hz; ``None`` = unlimited. Env ``<= 0`` disables cap."""
    raw = os.environ.get(env_key)
    if raw is None or str(raw).strip() == "":
        v = default
    else:
        v = float(raw)
    if v <= 0:
        return None
    return v


def _clear_stale_cli_lock(lock_path: Path) -> bool:
    """Return True if lock removed and caller should retry exclusive create."""
    try:
        pid = int(lock_path.read_text().strip())
    except Exception:
        try:
            lock_path.unlink(missing_ok=True)
        except OSError:
            pass
        return True
    if sys.platform == "win32":
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        try:
            lock_path.unlink(missing_ok=True)
        except OSError:
            pass
        return True
    except PermissionError:
        return False
    return False


def acquire_cli_lock(lock_path: Path = CLI_LOCK_PATH) -> None:
    """Ensure a single agent REPL; stale locks cleared when PID is dead (Unix)."""
    while True:
        try:
            fd = os.open(str(lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            try:
                os.write(fd, str(os.getpid()).encode())
            finally:
                os.close(fd)
            break
        except FileExistsError:
            if _clear_stale_cli_lock(lock_path):
                continue
            try:
                other = lock_path.read_text().strip()
            except OSError:
                other = "?"
            print(
                f"Another agent appears to be running (lock {lock_path}, pid {other}). "
                "Stop it or delete the lock file if stale.",
                file=sys.stderr,
            )
            raise SystemExit(1)


def release_cli_lock(lock_path: Path = CLI_LOCK_PATH) -> None:
    try:
        if lock_path.exists() and lock_path.read_text().strip() == str(os.getpid()):
            lock_path.unlink()
    except OSError:
        pass


def run_once(user_text: str, robot: Lite3Robot, motion, follow, basic_goal, max_steps: int = 6) -> str:
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
            result = dispatch(name, args, robot, motion, follow, basic_goal, bedrock)
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
    acquire_cli_lock()
    atexit.register(release_cli_lock)

    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    cam_port = int(os.environ.get("ROS_BRIDGE_PORT", "9090"))
    motion_port = int(os.environ.get("ROS2_BRIDGE_PORT", "9091"))

    rgb_hz = _parse_max_hz("RGB_MAX_HZ", 2.0)
    depth_hz = _parse_max_hz("DEPTH_MAX_HZ", 2.0)

    print(f"connecting to camera bridge ws://{host}:{cam_port} …", file=sys.stderr)
    robot = Lite3Robot(
        host=host,
        port=cam_port,
        max_rgb_hz=rgb_hz,
        max_depth_hz=depth_hz,
    )

    ros2_client = None
    motion = None
    follow = None
    basic_goal = None

    try:
        print(f"connecting to ROS 2 bridge ws://{host}:{motion_port} …", file=sys.stderr)
        ros2_client = connect_ros2_rosbridge(host, motion_port)
        print("ROS 2 bridge OK (shared by motion + follow + basic goal)", file=sys.stderr)
    except Exception as e:
        print(f"ROS 2 bridge unavailable: {e}", file=sys.stderr)

    if ros2_client is not None:
        try:
            motion = Lite3Motion(ros_client=ros2_client)
            print("motion adapter connected", file=sys.stderr)
        except Exception as e:
            print(f"motion adapter failed: {e}", file=sys.stderr)
        try:
            follow = Lite3Follow(ros_client=ros2_client)
            print("follow adapter connected", file=sys.stderr)
        except Exception as e:
            print(f"follow adapter failed: {e}", file=sys.stderr)
        try:
            basic_goal = Lite3BasicGoal(ros_client=ros2_client)
            print("basic goal adapter connected", file=sys.stderr)
        except Exception as e:
            print(f"basic goal adapter failed: {e}", file=sys.stderr)

    orchestrator = None
    if _USE_NEW_PIPELINE:
        try:
            memory = MemoryStore()
            safety = SafetySupervisor(memory)
            vlm = make_vlm_client()
            ctx = ToolContext(
                memory=memory,
                robot=robot,
                motion=motion,
                follow=follow,
                basic_goal=basic_goal,
                vlm=vlm,
                safety=safety,
            )
            registry = build_registry()
            oa_client = _make_agent_client() if _use_openai_compat() else None
            dialogue = DialogueAgent(oa_client)
            planner = PlannerAgent(registry, ctx, client=oa_client)
            orchestrator = Orchestrator(dialogue, planner, memory)
            backend = "openai-compat" if oa_client else "bedrock-boto3"
            print(f"new pipeline ready [{backend}] (LEGACY_LOOP=1 to use legacy loop)", file=sys.stderr)
        except Exception as e:
            print(f"new pipeline init failed: {e} — falling back to legacy loop", file=sys.stderr)
            orchestrator = None

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
                if orchestrator is not None:
                    answer = orchestrator.run(line)
                else:
                    answer = run_once(line, robot, motion, follow, basic_goal)
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
        if basic_goal is not None:
            basic_goal.close()
        if ros2_client is not None:
            try:
                ros2_client.terminate()
            except Exception:
                pass
        robot.close()
        release_cli_lock()


if __name__ == "__main__":
    main()
