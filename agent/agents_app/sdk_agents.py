"""DialogueAgent and PlannerAgent using AWS Bedrock Converse only."""

from __future__ import annotations

import json
import os
import re
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
_SENTENCE_SPLIT = re.compile(r"(?<=[.!?])\s+|\n+")


def _can_parallelize(tool_names: list[str]) -> bool:
    return len(tool_names) >= 2 and all(name in _READ_ONLY_TOOLS for name in tool_names)


def _sentence_chunks(buf: str) -> tuple[list[str], str]:
    parts = _SENTENCE_SPLIT.split(buf)
    if len(parts) == 1:
        return [], buf
    return [part for part in parts[:-1] if part.strip()], parts[-1]


DIALOGUE_SYSTEM = """\
You are the dialogue layer of a quadruped guide-dog robot assistant.
Understand the user's intent and extract a clear goal.
Return ONLY a JSON object with two fields:
  "intent": one of [describe_scene, read_label, move, follow, stop, query_memory, unknown]
  "goal": one plain English sentence describing what the robot should achieve.
No extra text, no markdown, no explanation."""


PLANNER_SYSTEM = """\
You are the planning layer of a quadruped guide-dog robot assistant.
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
- For "find/go to/follow" requests where the target may be out of view, prefer:
    * find_and_go_to instead of list_visible_objects + go_to_object
    * find_person_and_follow instead of list_visible_objects + follow_person
    * find_object instead of manually scanning with repeated perception calls
- list_visible_objects accepts label_filter - use it instead of listing-then-filtering.
- go_to_object returns immediately and auto-stops. Do NOT poll get_basic_goal_status.
- World-map tools (list_world_objects, find_object_in_world, go_to_world_object) operate
  on REMEMBERED objects with stored world coordinates. Prefer them when the user
  references an object 'you saw earlier', asks to 'come back to', or wants positions
  in world coordinates. list_visible_objects only sees what's in the current frame.
- go_to_world_object navigates by stored coordinates and works EVEN IF the object
  isn't currently visible — use this instead of find_and_go_to when you have a
  remembered position.
- After go_to_object / find_and_go_to / find_person_and_follow runs, the navigation
  outcome (reached / lost_target / timeout) is written to the memory snapshot's
  recent events with type='nav_outcome'. If the user asks 'did you reach it?' or
  'where are you now?', check recent events first instead of issuing new perception calls.
- When a request genuinely needs multiple INDEPENDENT read-only tools (e.g. describe_scene
  AND list_visible_objects for an unusual question that needs both), emit them as multiple
  tool_use blocks in the SAME response. They will run in parallel. Do NOT batch motion
  tools or any tool that changes robot state.
- Final message to the user is spoken or shown verbatim — keep it minimal:
    * Default to one short sentence; use two only for a necessary error or safety caveat.
    * No markdown, bullets, numbered lists, pleasantries, or "I'll..." preamble.
    * Do not restate the user's request or summarize internal steps.
- If a tool returns ok=false, give only the essential reason in as few words as safe.
- Ask a follow-up only when the goal is genuinely ambiguous and the cheapest safe tool path cannot disambiguate it; one short question only.
- After success, one terse confirmation or outcome — not a recap.
- Do NOT describe tool calls, reasoning, or planning details to the user."""


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
        system_text = PLANNER_SYSTEM + "\n\nCurrent robot memory:\n" + json.dumps(memory_snapshot, indent=2)
        system = [{"text": system_text}]
        messages: list[dict] = [{"role": "user", "content": [{"text": goal}]}]
        tool_config = {"tools": PLANNER_TOOLS_BEDROCK}

        for _ in range(self._max_steps):
            resp = bedrock.converse(
                modelId=_bedrock_model(),
                system=system,
                messages=messages,
                toolConfig=tool_config,
                inferenceConfig={"maxTokens": 1024},
            )
            out_msg = resp["output"]["message"]
            messages.append(out_msg)
            stop = resp.get("stopReason")

            if stop != "tool_use":
                return "".join(part.get("text", "") for part in out_msg["content"] if "text" in part)

            tool_uses = [part["toolUse"] for part in out_msg["content"] if "toolUse" in part]
            tool_results = self._dispatch_tool_uses(tool_uses)
            messages.append({"role": "user", "content": tool_results})

        return "(agent: reached max steps)"

    def _run_boto3_stream(self, goal: str, memory_snapshot: dict) -> Iterator[str]:
        from vlm import make_client

        bedrock = make_client()
        system_text = PLANNER_SYSTEM + "\n\nCurrent robot memory:\n" + json.dumps(memory_snapshot, indent=2)
        system = [{"text": system_text}]
        messages: list[dict] = [{"role": "user", "content": [{"text": goal}]}]
        tool_config = {"tools": PLANNER_TOOLS_BEDROCK}

        for _ in range(self._max_steps):
            resp = bedrock.converse_stream(
                modelId=_bedrock_model(),
                system=system,
                messages=messages,
                toolConfig=tool_config,
                inferenceConfig={"maxTokens": 1024},
            )

            assembled_content: list[dict] = []
            current_text = ""
            current_tool: dict | None = None
            current_tool_input = ""
            stop_reason = None
            text_buffer = ""

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
                        text_buffer += delta["text"]
                        chunks, text_buffer = _sentence_chunks(text_buffer)
                        for chunk in chunks:
                            yield chunk
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
                        current_text = ""
                elif "messageStop" in event:
                    stop_reason = event["messageStop"].get("stopReason")

            if text_buffer.strip():
                yield text_buffer

            messages.append({"role": "assistant", "content": assembled_content})

            if stop_reason != "tool_use":
                return

            tool_uses = [part["toolUse"] for part in assembled_content if "toolUse" in part]
            tool_results = self._dispatch_tool_uses(tool_uses)
            messages.append({"role": "user", "content": tool_results})

        yield "(agent: reached max steps)"

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

