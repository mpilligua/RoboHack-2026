"""DialogueAgent and PlannerAgent using AWS Bedrock Converse only."""

from __future__ import annotations

import json
import os
import queue
import sys
from concurrent.futures import ThreadPoolExecutor
from typing import Iterator

from .sdk_tools import PLANNER_TOOLS_BEDROCK, dispatch_tool
from tools.base import ToolContext
from tools.registry import ToolRegistry


_READ_ONLY_TOOLS = frozenset({
    "get_robot_pose_in_map",
    "get_map_summary",
    "get_local_map_context",
    "get_local_occupancy_grid",
    "list_waypoints",
    "get_waypoint",
    "check_waypoint_reachable",
    "get_route_summary_to_waypoint",
    "compare_map_vs_live_scan",
    "get_navigation_status",
    "describe_scene",
    "list_visible_objects",
    "read_label",
    "get_rgbd_summary",
    "get_depth_at_pixel",
    "get_visible_objects",
    "find_object",
    "find_objects_matching_constraints",
    "resolve_reference",
    "get_robot_status",
    "get_basic_goal_status",
    "get_ros2_odom",
    # World map (read-only memory queries):
    "list_world_objects",
    "find_object_in_world",
})

_TOOL_POOL = ThreadPoolExecutor(max_workers=4, thread_name_prefix="tool")


def _can_parallelize(tool_names: list[str]) -> bool:
    return len(tool_names) >= 2 and all(name in _READ_ONLY_TOOLS for name in tool_names)


DIALOGUE_SYSTEM = """\
You are the dialogue layer of a quadruped guide-dog robot assistant.
Understand the user's intent and extract a clear goal.
Return ONLY a JSON object with two fields:
  "intent": one of [describe_scene, read_label, move, follow, stop, query_memory, unknown]
  "goal": one plain English sentence describing what the robot should achieve.
No extra text, no markdown, no explanation."""


def _user_name() -> str:
    return (os.environ.get("USER_NAME") or "Maria").strip() or "Maria"


PLANNER_SYSTEM_TEMPLATE = """\
You are the planning layer of a quadruped guide-dog robot assistant.
The user's name is {user_name}. Address them by name when greeting or getting their attention; otherwise no need to repeat it.
You receive a goal and a memory snapshot, then call tools to fulfill it.

Planning style:
- Be action-oriented. If one tool can safely make progress, use it instead of gathering extra context first.
- Trust built-in tool safety. walk_forward, go_to_object, and find_and_go_to already do their own depth/safety handling.
  Do NOT add manual pre-flight calls just to be cautious unless the user explicitly asked for analysis or a previous action failed.
- Visible objects or noisy depth readings are not, by themselves, a reason to refuse simple forward motion.
- Reuse memory aggressively. If the needed object or navigation outcome is already in Current robot memory, or can be read with a
  cheap memory tool, use that instead of refreshing perception.
- Prefer a single action tool over multi-step decompositions whenever possible.

Tool selection rules:
- Pick the SMALLEST set of tools that gets the job done. Each call costs ~1s of latency.
- For simple relative motion requests like "go forward a bit" or "turn right", call the motion tool directly.
  Do NOT call describe_scene, list_visible_objects, or get_robot_status first.
- Motion tools (walk_forward, go_to_object) run their own depth-based safety check
  internally. Do NOT call get_rgbd_summary as a manual pre-flight.
- For absolute navigation requests like "go to 0,0" or "go to the waypoint desk", use
  go_to_map_pose or go_to_waypoint, not legacy basic-goal tools.
- Trust Nav2 for obstacle handling during absolute navigation. Do NOT add manual local obstacle
  pre-checks before go_to_map_pose or go_to_waypoint unless the user explicitly asked for route analysis
  or a previous navigation attempt failed.
- Use check_waypoint_reachable, get_route_summary_to_waypoint, get_local_map_context, or
  compare_map_vs_live_scan when the user asks for route analysis, nearby obstacle discussion,
  or diagnosis of why navigation might fail.
- Stale scan or costmap data is a reason to avoid strong route-analysis claims, not a reason by itself
  to avoid dispatching a Nav2 navigation goal.
- Prefer get_visible_objects or find_objects_matching_constraints before fresh perception when recent memory is likely enough.
- Use resolve_reference for pronouns or references like "it", "that chair", "the one on the left". If it returns an id, act on it directly.
- For perception, choose ONE of describe_scene OR list_visible_objects per turn:
    * describe_scene - open-vocabulary narration. Use for situational questions,
      hazards, layout, things YOLO can't name (cables, puddles, signs).
    * list_visible_objects - YOLO + per-object VLM. Use when the user wants to
      act on something and you truly need a fresh yolo_id,
      or when filtering for a specific COCO class via label_filter.
- For "go to / where is / come back to / find" requests, ALWAYS check the world
  map FIRST before any visual search:
    1. Call list_world_objects (or find_object_in_world for a specific label).
    2. If a matching object has a world position (x_odom and y_odom not null,
       has_world_position true), call go_to_world_object — DO NOT call
       find_and_go_to. The position field "left/center/right" being null is
       NORMAL for remembered objects; only x_odom/y_odom matter.
    3. Only fall back to find_and_go_to / find_object / find_person_and_follow
       if the world map has no matching object with coordinates.
- list_visible_objects accepts label_filter - use it for fresh yolo_ids when
  acting on something currently in view.
- go_to_object returns immediately and auto-stops. Do NOT poll get_basic_goal_status.
- go_to_world_object BLOCKS until arrival/timeout/blocked; the result tells you
  what happened. Do NOT poll afterwards.
- After go_to_object / find_and_go_to / find_person_and_follow runs, the navigation
  outcome (reached / lost_target / timeout) is written to the memory snapshot's
  recent events with type='nav_outcome'. If the user asks 'did you reach it?' or
  'where are you now?', check recent events first instead of issuing new perception calls.
- When a request genuinely needs multiple INDEPENDENT read-only tools (e.g. describe_scene
  AND list_visible_objects for an unusual question that needs both), emit them as multiple
  tool_use blocks in the SAME response. They will run in parallel. Do NOT batch motion
  tools or any tool that changes robot state.
- USER COMMUNICATION — read this carefully:
    * The user is VISUALLY IMPAIRED and only hears you via the `speak_to_user` tool.
    * Your assistant text output is an INTERNAL LOG — the user will NOT see or hear it.
    * To say ANYTHING to the user (confirmations, errors, answers, questions), you MUST call `speak_to_user`.
    * VOICE & PERSONA: speak like a friendly, calm guide-dog companion. Casual, warm, first-person. NOT like a status report or robot.
        - Talk like a person, not a robot. "Found a door, heading there now." NOT "I found a door after rotating 35° to the left. The robot is now facing it."
        - Use "I" / "we", contractions ("I'm", "let's", "got it"). Refer to yourself in first person — never "the robot".
        - Avoid technical jargon: no degrees, IDs, sensor names, percentages, or status verbs like "successfully", "engaged", "executing". Distances in meters ARE fine when the user is being guided to something — "two meters ahead", "about a meter to your right" — but say it the way a person would, not "2.0 m" or "x_odom 1.83".
        - Never say "Waiting for your next instruction" or any sign-off — silence is the sign-off.
        - Examples of GOOD voice: "Got it.", "Found the door, about two meters ahead.", "Walking over now.", "Almost there.", "Reached the chair.", "There's a chair to your right.", "Hmm, I can't see one — want me to look around?", "I'm here.", "Stopped."
        - Examples of GOOD navigation flow: "Looking for the door." → "Found it, just a couple of meters ahead." → "Walking over." → "I'm here."
        - Examples of BAD voice: "Action complete.", "I have successfully located the target.", "Rotating 35 degrees.", "The robot is now facing the door.", "Goal status: reached.", "Approaching object yolo_id 42."
    * GREETINGS: when the user just says "hi", "hello", "hey", or similar, reply with ONLY a warm "Hi {user_name}." or "Hey {user_name}." — no self-introduction, no role explanation, no "I am your guide dog", no offers of help. Just the greeting. Do not call any other tool.
    * Each `speak_to_user` call: one short sentence, under 15 words for actions/confirmations/errors/questions; up to ~25 words for scene descriptions. Plain spoken English. No markdown, IDs, coordinates, symbols, or units.
    * You may call `speak_to_user` mid-task for short progress updates ("Looking around.", "Found it, walking over.") — spoken immediately.
    * NAVIGATION & FIND ACTIONS — give the user real guidance, not just start/end pings. For find_and_go_to, go_to_object, go_to_world_object, go_to_map_pose, go_to_waypoint, follow_person, walk_forward, and similar:
        1. Acknowledge briefly when you start ("Looking for the door.").
        2. When you spot or know the target, share what you found and roughly where: "Found it, about two meters ahead." or "It's just to your right, a couple of steps away." Use the distance from the tool result if available (round to natural numbers — "about a meter", "a couple of meters", "right in front of you" for under a meter).
        3. Confirm arrival: "I'm here." or "Reached the chair." If close but stopped short for safety, say so: "Stopped just short of it."
        4. Skip the middle if the action is fast (under ~2 seconds) — just start + end.
    * Do NOT narrate every tool call. Speak only when the user benefits: starting a long action, key milestones during it, finishing one, an error, or an answer.
    * After success: one terse, casual confirmation ("Done.", "Got it.", "I'm at the chair.").
    * On tool failure (ok=false): one short, plain-language reason, 5-10 words. Not "Error:" — say what went wrong like a person would.
    * Ambiguous request: one short speak_to_user question, under 10 words.
    * Scene description: 2-3 most relevant items in natural phrasing. Spatial cues ("on your left", "ahead", "a couple of meters out") are welcome when they help the user form a picture; skip compass headings and exact coordinates.
    * If you have nothing useful to say, do not call speak_to_user. Do not announce that you are waiting or done waiting.
- Your final assistant text may be empty, or a brief internal log of what you did. It is not user-facing."""


def _planner_system() -> str:
    return PLANNER_SYSTEM_TEMPLATE.format(user_name=_user_name())


def _bedrock_model() -> str:
    return os.environ.get("BEDROCK_MODEL_ID", "us.anthropic.claude-sonnet-4-6")


class DialogueAgent:
    """Classify intent and extract a goal sentence from user input."""

    def __init__(self, client=None) -> None:
        self._client = client

    def run(self, user_text: str) -> dict:
        return self._run_boto3(user_text)

    def _run_boto3(self, user_text: str) -> dict:
        from vlm import make_client

        bedrock = make_client()
        try:
            resp = bedrock.converse(
                modelId=_bedrock_model(),
                system=[{"text": DIALOGUE_SYSTEM}],
                messages=[{"role": "user", "content": [{"text": user_text}]}],
                inferenceConfig={"maxTokens": 200},
            )
            raw = "".join(part.get("text", "") for part in resp["output"]["message"]["content"])
            return json.loads(raw)
        except (json.JSONDecodeError, Exception):
            return {"intent": "unknown", "goal": user_text}


class PlannerAgent:
    """Multi-turn tool loop: receives a goal, calls tools, returns response."""

    def __init__(
        self,
        registry: ToolRegistry,
        ctx: ToolContext,
        client=None,
        max_steps: int = 6,
    ) -> None:
        self._registry = registry
        self._ctx = ctx
        self._client = client
        self._max_steps = max_steps

    def run(self, goal: str, memory_snapshot: dict) -> str:
        return self._run_boto3(goal, memory_snapshot)

    def run_stream(self, goal: str, memory_snapshot: dict) -> Iterator[str]:
        yield from self._run_boto3_stream(goal, memory_snapshot)

    def _run_boto3(self, goal: str, memory_snapshot: dict) -> str:
        from vlm import make_client

        bedrock = make_client()
        system_text = _planner_system() + "\n\nCurrent robot memory:\n" + json.dumps(memory_snapshot, indent=2)
        system = [{"text": system_text}]
        messages: list[dict] = [{"role": "user", "content": [{"text": goal}]}]
        tool_config = {"tools": PLANNER_TOOLS_BEDROCK}

        speak_q: queue.Queue[str] = queue.Queue()
        prev_q = getattr(self._ctx, "speak_queue", None)
        self._ctx.speak_queue = speak_q
        spoken: list[str] = []

        def collect_speech() -> None:
            while True:
                try:
                    spoken.append(speak_q.get_nowait())
                except queue.Empty:
                    return

        try:
            for _ in range(self._max_steps):
                resp = bedrock.converse(
                    modelId=_bedrock_model(),
                    system=system,
                    messages=messages,
                    toolConfig=tool_config,
                    inferenceConfig={"maxTokens": 256},
                )
                out_msg = resp["output"]["message"]
                messages.append(out_msg)
                stop = resp.get("stopReason")

                internal_text = "".join(part.get("text", "") for part in out_msg["content"] if "text" in part).strip()
                if internal_text:
                    print(f"  [planner:internal] {internal_text}", file=sys.stderr)

                if stop != "tool_use":
                    collect_speech()
                    return " ".join(spoken)

                tool_uses = [part["toolUse"] for part in out_msg["content"] if "toolUse" in part]
                tool_results = self._dispatch_tool_uses(tool_uses)
                messages.append({"role": "user", "content": tool_results})
                collect_speech()

            collect_speech()
            return " ".join(spoken) if spoken else "(agent: reached max steps)"
        finally:
            self._ctx.speak_queue = prev_q

    def _run_boto3_stream(self, goal: str, memory_snapshot: dict) -> Iterator[str]:
        from vlm import make_client

        bedrock = make_client()
        system_text = _planner_system() + "\n\nCurrent robot memory:\n" + json.dumps(memory_snapshot, indent=2)
        system = [{"text": system_text}]
        messages: list[dict] = [{"role": "user", "content": [{"text": goal}]}]
        tool_config = {"tools": PLANNER_TOOLS_BEDROCK}

        speak_q: queue.Queue[str] = queue.Queue()
        prev_q = getattr(self._ctx, "speak_queue", None)
        self._ctx.speak_queue = speak_q

        def drain_speak() -> Iterator[str]:
            while True:
                try:
                    yield speak_q.get_nowait()
                except queue.Empty:
                    return

        try:
            for _ in range(self._max_steps):
                resp = bedrock.converse_stream(
                    modelId=_bedrock_model(),
                    system=system,
                    messages=messages,
                    toolConfig=tool_config,
                    inferenceConfig={"maxTokens": 256},
                )

                assembled_content: list[dict] = []
                current_text = ""
                current_tool: dict | None = None
                current_tool_input = ""
                stop_reason = None

                for event in resp["stream"]:
                    if "contentBlockStart" in event:
                        start = event["contentBlockStart"].get("start", {})
                        if "toolUse" in start:
                            current_tool = {
                                "toolUseId": start["toolUse"]["toolUseId"],
                                "name": start["toolUse"]["name"],
                            }
                            current_tool_input = ""
                    elif "contentBlockDelta" in event:
                        delta = event["contentBlockDelta"]["delta"]
                        if "text" in delta:
                            current_text += delta["text"]
                        elif "toolUse" in delta:
                            current_tool_input += delta["toolUse"].get("input", "")
                    elif "contentBlockStop" in event:
                        if current_tool is not None:
                            try:
                                parsed_input = json.loads(current_tool_input or "{}")
                            except json.JSONDecodeError:
                                parsed_input = {}
                            assembled_content.append({
                                "toolUse": {
                                    "toolUseId": current_tool["toolUseId"],
                                    "name": current_tool["name"],
                                    "input": parsed_input,
                                }
                            })
                            current_tool = None
                            current_tool_input = ""
                        elif current_text:
                            assembled_content.append({"text": current_text})
                            if current_text.strip():
                                print(f"  [planner:internal] {current_text.strip()}", file=sys.stderr)
                            current_text = ""
                    elif "messageStop" in event:
                        stop_reason = event["messageStop"].get("stopReason")

                messages.append({"role": "assistant", "content": assembled_content})

                if stop_reason != "tool_use":
                    yield from drain_speak()
                    return

                tool_uses = [part["toolUse"] for part in assembled_content if "toolUse" in part]
                tool_results = self._dispatch_tool_uses(tool_uses)
                messages.append({"role": "user", "content": tool_results})

                yield from drain_speak()

            yield from drain_speak()
        finally:
            self._ctx.speak_queue = prev_q

    def _dispatch_tool_uses(self, tool_uses: list[dict]) -> list[dict]:
        names = [tool_use["name"] for tool_use in tool_uses]
        parallel = _can_parallelize(names)
        if parallel:
            print(f"  [planner] parallel batch: {names}", file=sys.stderr)

        def run_one(tool_use: dict) -> tuple[str, str]:
            name = tool_use["name"]
            args = tool_use.get("input") or {}
            print(f"  [planner] -> {name}({args})", file=sys.stderr)
            result_str = dispatch_tool(name, args, self._registry, self._ctx)
            print(f"  [planner] <- {result_str[:200]}", file=sys.stderr)
            return tool_use["toolUseId"], result_str

        if parallel:
            results = list(_TOOL_POOL.map(run_one, tool_uses))
        else:
            results = [run_one(tool_use) for tool_use in tool_uses]

        return [
            {"toolResult": {"toolUseId": tool_use_id, "content": [{"text": result_str}]}}
            for tool_use_id, result_str in results
        ]

