"""Run ONE demo tool in isolation, with the exact args demo.py uses.

Connects to the same robot the demo connects to (ROS_BRIDGE_HOST + 9091),
builds a real ToolContext, dispatches the named handler, prints the
ToolResult, and tears down. No scenes, no PTT, no panel, no orchestrator.

Usage:
    python scripts/test_tool.py list
    python scripts/test_tool.py walk_forward            # default args
    python scripts/test_tool.py walk_forward --distance 0.5
    python scripts/test_tool.py turn_right --angle 30
    python scripts/test_tool.py describe_scene
    python scripts/test_tool.py find_object --label chair
    python scripts/test_tool.py find_person_and_follow
    python scripts/test_tool.py stop                    # safety stop

Tools list (matches demo.py scenes 1-6):
    walk_forward            scene 2 / 5    forward motion
    turn_right              scene 2 / 3    rotate clockwise
    turn_left               scene 3        rotate counter-clockwise
    describe_scene          scene 3        VLM scene description
    find_object             scene 5        rotate until label seen
    find_person_and_follow  scene 6        find biggest person + follow
    stop                    scene 6        emergency stop
"""

from __future__ import annotations

import argparse
import os
import sys
import time

from dotenv import load_dotenv

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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


# Each entry: (handler, default args dict, list of (cli_flag, kwarg_key, type)).
TOOLS = {
    "walk_forward": (
        motion_tools.handle_walk_forward,
        {"distance_m": 0.5},
        [("--distance", "distance_m", float)],
    ),
    "walk_backward": (
        motion_tools.handle_walk_backward,
        {"distance_m": 0.3},
        [("--distance", "distance_m", float)],
    ),
    "turn_left": (
        motion_tools.handle_turn_left,
        {"angle_deg": 30.0},
        [("--angle", "angle_deg", float)],
    ),
    "turn_right": (
        motion_tools.handle_turn_right,
        {"angle_deg": 30.0},
        [("--angle", "angle_deg", float)],
    ),
    "describe_scene": (
        perception_tools.handle_describe_scene,
        {},
        [],
    ),
    "find_object": (
        follow_tools.handle_find_object,
        {"label": "chair"},
        [("--label", "label", str)],
    ),
    "find_and_go_to": (
        follow_tools.handle_find_and_go_to,
        {"label": "chair"},
        [("--label", "label", str)],
    ),
    "find_person_and_follow": (
        follow_tools.handle_find_person_and_follow,
        {},
        [],
    ),
    "stop": (
        safety_tools.handle_stop,
        {},
        [],
    ),
}


def build_context() -> tuple[ToolContext, list]:
    """Same connections demo.py establishes (build_tool_ctx without dry-run)."""
    cleanups: list = []
    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    port = int(os.environ.get("ROS2_BRIDGE_PORT", "9091"))

    print(f"[test_tool] connecting to ws://{host}:{port} ...", file=sys.stderr)
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
    return ctx, cleanups


def main() -> int:
    load_dotenv()

    parser = argparse.ArgumentParser(description="Test one demo tool in isolation.")
    parser.add_argument("tool", help=f"tool name. Use 'list' to see available tools.")
    # Pre-known flags for any tool. Argparse ignores unknowns per-tool.
    parser.add_argument("--distance", type=float, help="meters (motion tools)")
    parser.add_argument("--angle", type=float, help="degrees (turn tools)")
    parser.add_argument("--label", type=str, help="object label (find tools)")
    parser.add_argument("--hold", type=float, default=0.0, metavar="SECONDS",
                        help="Hold the connection open for N seconds AFTER the tool returns "
                             "before tearing down. Required for follow tools — otherwise the "
                             "cleanup's auto-stop cancels the follow before the robot moves. "
                             "Default 8s for follow tools, 0 otherwise.")
    args = parser.parse_args()

    if args.tool == "list":
        print("Available tools:\n")
        for name, (_, defaults, _) in TOOLS.items():
            print(f"  {name:25}  defaults: {defaults}")
        return 0

    if args.tool not in TOOLS:
        print(f"[test_tool] unknown tool: {args.tool!r}", file=sys.stderr)
        print(f"[test_tool] available: {', '.join(TOOLS.keys())}", file=sys.stderr)
        return 1

    handler, defaults, flag_specs = TOOLS[args.tool]
    call_args = dict(defaults)
    for flag, key, _ty in flag_specs:
        cli_value = getattr(args, flag.lstrip("-"), None)
        if cli_value is not None:
            call_args[key] = cli_value

    ctx, cleanups = build_context()
    try:
        print(f"[test_tool] -> {args.tool}({call_args})", file=sys.stderr)
        t0 = time.time()
        result = handler(ctx, call_args)
        elapsed = time.time() - t0
        print(f"[test_tool] result ({elapsed:.2f}s):", file=sys.stderr)
        print(f"  ok:     {result.ok}")
        print(f"  tool:   {result.tool}")
        if result.result is not None:
            print(f"  result: {result.result}")
        if result.error is not None:
            print(f"  error:  {result.error}")
        rc = 0 if result.ok else 2

        # Decide hold duration. Follow / approach tools need time to actually
        # drive the robot before cleanup yanks the connection (which publishes
        # "stop" on the follow topic). Motion tools complete synchronously so
        # they don't need a hold.
        FOLLOW_LIKE = {
            "find_person_and_follow", "follow_person",
            "find_and_go_to", "go_to_object",
        }
        hold_s = args.hold if args.hold > 0 else (8.0 if args.tool in FOLLOW_LIKE else 0.0)
        if hold_s > 0:
            print(f"[test_tool] holding {hold_s:.1f}s before teardown so the "
                  f"robot can act (Ctrl-C to abort sooner)...", file=sys.stderr)
            try:
                time.sleep(hold_s)
            except KeyboardInterrupt:
                print("[test_tool] hold interrupted, tearing down now.", file=sys.stderr)
    finally:
        # Twisted's reactor gets multiple stop attempts as adapters tear down,
        # spamming ReactorNotRunning tracebacks from background threads.
        # contextlib.redirect_stderr only catches the current thread; daemon
        # threads bypass it. So we redirect at the file-descriptor level.
        sys.stderr.flush()
        saved_fd = os.dup(2)
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, 2)
        try:
            for fn in reversed(cleanups):
                try:
                    fn()
                except Exception:
                    pass
            # Give daemon threads a beat to emit their tracebacks while
            # stderr is still sinkholed.
            time.sleep(0.2)
        finally:
            os.dup2(saved_fd, 2)
            os.close(saved_fd)
            os.close(devnull)
    return rc


if __name__ == "__main__":
    sys.exit(main())
