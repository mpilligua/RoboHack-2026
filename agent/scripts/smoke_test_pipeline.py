"""Smoke test for the new agent pipeline.

Requires a live robot connection and AGENT_BASE_URL / AGENT_API_KEY / AGENT_MODEL
env vars set. Run from the agent/ directory:

    python scripts/smoke_test_pipeline.py

Tests three interactions through the full pipeline and checks basic sanity.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dotenv import load_dotenv
load_dotenv()

from robot import Lite3BasicGoal, Lite3Follow, Lite3Motion, Lite3Robot, connect_ros2_rosbridge
from vlm_client import make_vlm_client
from memory.store import MemoryStore
from safety.supervisor import SafetySupervisor
from tools.base import ToolContext
from tools.setup import build_registry
from agents_app.sdk_agents import DialogueAgent, PlannerAgent, _make_agent_client
from agents_app.orchestrator import Orchestrator


def _connect_robot():
    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    cam_port = int(os.environ.get("ROS_BRIDGE_PORT", "9090"))
    motion_port = int(os.environ.get("ROS2_BRIDGE_PORT", "9091"))

    print(f"connecting to camera bridge ws://{host}:{cam_port} …")
    robot = Lite3Robot(host=host, port=cam_port, max_rgb_hz=2.0, max_depth_hz=2.0)

    ros2_client = motion = follow = basic_goal = None
    try:
        ros2_client = connect_ros2_rosbridge(host, motion_port)
        motion = Lite3Motion(ros_client=ros2_client)
        follow = Lite3Follow(ros_client=ros2_client)
        basic_goal = Lite3BasicGoal(ros_client=ros2_client)
        print("ROS 2 bridge connected (motion + follow + basic goal)")
    except Exception as e:
        print(f"ROS 2 bridge unavailable: {e}")

    return robot, motion, follow, basic_goal, ros2_client


def _build_orchestrator(robot, motion, follow, basic_goal):
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
    oa_client = _make_agent_client()
    dialogue = DialogueAgent(oa_client)
    planner = PlannerAgent(oa_client, registry, ctx)
    return Orchestrator(dialogue, planner, memory)


def run_tests(orchestrator):
    tests = [
        {
            "name": "describe scene",
            "query": "what do you see?",
            "check": lambda r: len(r) > 10,
        },
        {
            "name": "safety check",
            "query": "check if the path ahead is safe",
            "check": lambda r: any(w in r.lower() for w in ["safe", "obstacle", "depth", "path", "clear", "block"]),
        },
        {
            "name": "stop",
            "query": "stop everything",
            "check": lambda r: len(r) > 0,
        },
    ]

    passed = 0
    failed = 0
    for t in tests:
        print(f"\n--- {t['name']} ---")
        print(f"  query: {t['query']!r}")
        try:
            response = orchestrator.run(t["query"])
            print(f"  response: {response[:300]}")
            if t["check"](response):
                print(f"  [PASS]")
                passed += 1
            else:
                print(f"  [FAIL] response did not match expected pattern")
                failed += 1
        except Exception as e:
            print(f"  [FAIL] exception: {e}")
            failed += 1

    print(f"\n{'='*40}")
    print(f"results: {passed} passed, {failed} failed")
    return failed == 0


def main():
    robot, motion, follow, basic_goal, ros2_client = _connect_robot()
    try:
        orchestrator = _build_orchestrator(robot, motion, follow, basic_goal)
        ok = run_tests(orchestrator)
    finally:
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
            try: ros2_client.terminate()
            except Exception: pass
        robot.close()

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
