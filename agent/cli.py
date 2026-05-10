"""CLI for the Bedrock-native agent with the full current tool registry."""

from __future__ import annotations

import atexit
import os
import sys
from pathlib import Path

from dotenv import load_dotenv

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from agents_app.orchestrator import Orchestrator  # noqa: E402
from agents_app.sdk_agents import PlannerAgent  # noqa: E402
from memory.store import MemoryStore  # noqa: E402
from robot import (  # noqa: E402
    Lite3BasicGoal,
    Lite3Follow,
    Lite3Motion,
    Lite3Robot,
    MapRuntime,
    connect_ros2_rosbridge,
)
from robot.map_runtime import Nav2NavigateClient  # noqa: E402
from safety.supervisor import SafetySupervisor  # noqa: E402
from tools.base import ToolContext  # noqa: E402
from tools.setup import build_registry  # noqa: E402
from tools.waypoint_store import WaypointStore  # noqa: E402
from vlm_client import make_vlm_client  # noqa: E402


AGENT_DIR = Path(__file__).resolve().parent
CLI_LOCK_PATH = AGENT_DIR / ".cli.pid.lock"


def _parse_max_hz(env_key: str, default: float) -> float | None:
    raw = os.environ.get(env_key)
    if raw is None or str(raw).strip() == "":
        value = default
    else:
        value = float(raw)
    if value <= 0:
        return None
    return value


def _clear_stale_cli_lock(lock_path: Path) -> bool:
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
    while True:
        try:
            fd = os.open(str(lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            try:
                os.write(fd, str(os.getpid()).encode())
            finally:
                os.close(fd)
            return
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


def main() -> None:
    load_dotenv()
    acquire_cli_lock()
    atexit.register(release_cli_lock)

    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    motion_port = int(os.environ.get("ROS2_BRIDGE_PORT", "9091"))
    rgb_hz = _parse_max_hz("RGB_MAX_HZ", 2.0)
    depth_hz = _parse_max_hz("DEPTH_MAX_HZ", 2.0)

    ros2_client = None
    motion = None
    follow = None
    basic_goal = None
    map_runtime = None
    waypoints = WaypointStore()

    try:
        print(f"connecting to ROS 2 bridge ws://{host}:{motion_port} ...", file=sys.stderr)
        ros2_client = connect_ros2_rosbridge(host, motion_port)
        print("ROS 2 bridge OK", file=sys.stderr)
    except Exception as exc:
        print(f"ROS 2 bridge unavailable: {exc}", file=sys.stderr)

    print(f"connecting to ROS 2 perception bridge ws://{host}:{motion_port} ...", file=sys.stderr)
    robot = Lite3Robot(
        host=host,
        port=motion_port,
        max_rgb_hz=rgb_hz,
        max_depth_hz=depth_hz,
        ros_client=ros2_client,
    )

    if ros2_client is not None:
        try:
            map_frame = os.environ.get("ROS2_MAP_FRAME", "map")
            goal_topic = (os.environ.get("NAV2_GOAL_POSE_TOPIC") or "").strip() or None
            goal_msg_type = os.environ.get("NAV2_GOAL_MESSAGE_TYPE", "PoseStamped")
            nav_goal_client = (
                Nav2NavigateClient(
                    ros2_client,
                    goal_pose_topic=goal_topic,
                    goal_message_type=goal_msg_type,
                    goal_frame_id=map_frame,
                )
                if goal_topic
                else None
            )
            map_runtime = MapRuntime(
                ros_client=ros2_client,
                base_frame=os.environ.get("ROS2_BASE_FRAME", "rslidar"),
                map_frame=map_frame,
                odom_frame=os.environ.get("ROS2_ODOM_FRAME", "odom"),
                nav2_navigate_client=nav_goal_client,
            )
            print("map runtime connected", file=sys.stderr)
            if goal_topic:
                print(f"Nav2 navigation via {goal_msg_type} topic {goal_topic!r}", file=sys.stderr)
            elif not os.environ.get("NAV_SUPPRESS_TOPIC_HINT"):
                print(
                    "Nav2 hint: Foxy rosbridge 1.x has no ROS 2 actions — if the robot ignores "
                    "go_to_map_pose, add NAV2_GOAL_POSE_TOPIC=/goal_pose to agent/.env (see .env.example).",
                    file=sys.stderr,
                )
        except Exception as exc:
            print(f"map runtime failed: {exc}", file=sys.stderr)
        try:
            motion = Lite3Motion(ros_client=ros2_client)
            print("motion adapter connected", file=sys.stderr)
        except Exception as exc:
            print(f"motion adapter failed: {exc}", file=sys.stderr)
        try:
            follow = Lite3Follow(ros_client=ros2_client)
            print("follow adapter connected", file=sys.stderr)
        except Exception as exc:
            print(f"follow adapter failed: {exc}", file=sys.stderr)
        try:
            basic_goal = Lite3BasicGoal(ros_client=ros2_client)
            print("basic goal adapter connected", file=sys.stderr)
        except Exception as exc:
            print(f"basic goal adapter failed: {exc}", file=sys.stderr)

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
            map_runtime=map_runtime,
            waypoints=waypoints,
        )
        registry = build_registry()
        planner = PlannerAgent(registry, ctx, client=None)
        orchestrator = Orchestrator(planner, memory)
        print("bedrock agent ready [native converse + current tools]", file=sys.stderr)
    except Exception as exc:
        print(f"bedrock agent init failed: {exc}", file=sys.stderr)
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
            if orchestrator is None:
                print("The Bedrock agent failed to initialize.", flush=True)
                continue
            try:
                parts: list[str] = []
                for chunk in orchestrator.run_stream(line):
                    parts.append(chunk)
                    print(chunk, flush=True)
                if not parts:
                    print("(no response)")
            except KeyboardInterrupt:
                print("\n[interrupted - stopping motion + follow]", file=sys.stderr)
                if follow is not None:
                    try:
                        follow.stop()
                    except Exception:
                        pass
                if motion is not None:
                    try:
                        motion.stop()
                    except Exception:
                        pass
    finally:
        if follow is not None:
            try:
                follow.stop()
            except Exception:
                pass
            follow.close()
        if motion is not None:
            try:
                motion.stop()
            except Exception:
                pass
            motion.close()
        if basic_goal is not None:
            basic_goal.close()
        if map_runtime is not None:
            map_runtime.close()
        if ros2_client is not None:
            try:
                ros2_client.terminate()
            except Exception:
                pass
        robot.close()
        release_cli_lock()


if __name__ == "__main__":
    main()
