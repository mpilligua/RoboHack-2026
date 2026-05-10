"""Dry-run the planner against fabricated tool results.

Goal: study the planner's tool-calling chain without the robot.

How it works:
- We build a fake ToolRegistry whose .call() asks Bedrock to fabricate a
  plausible JSON result for the given (tool, args). The PlannerAgent doesn't
  notice — it sees the same dispatch interface as the real registry.
- Every call is recorded. After each test prompt we dump the full chain.

Run:
    cd agent && source .venv/bin/activate
    python scripts/dry_run_planner.py
    # or pick specific prompts:
    python scripts/dry_run_planner.py --only "follow me"

Env: same Bedrock setup as cli.py (.env with AWS_BEARER_TOKEN_BEDROCK,
AWS_REGION, BEDROCK_MODEL_ID).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, field

from dotenv import load_dotenv

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

load_dotenv()

from agents_app.sdk_agents import PlannerAgent  # noqa: E402
from memory.store import MemoryStore  # noqa: E402
from tools.base import ToolContext, ToolResult  # noqa: E402
from tools.setup import build_registry  # noqa: E402
from vlm import make_client  # noqa: E402


FAKE_TOOL_SYSTEM = """\
You are simulating the result of a robot tool call. The robot is a quadruped
guide-dog with an RGBD camera at knee height. Given a tool name, its arguments,
and the scenario, fabricate a *plausible* result the real tool might return.

Rules:
- Return ONLY a JSON object matching the format expected by that tool.
- Be concrete: invent specific objects, distances, ids, etc.
- Stay consistent with prior fake results in the same scenario (passed below).
- For perception tools, imagine a typical indoor scene unless context suggests otherwise.
- For motion tools, return success unless an obstacle was previously reported.
- Numbers in SI units (meters, seconds, radians, mm where the schema says so).
- Keep descriptions short."""


# Cheat-sheet so the fake LLM knows the rough output shape per tool. Keeps
# results well-typed without us having to pre-write every stub.
TOOL_OUTPUT_HINTS = {
    "describe_scene":   '{"description": "<one or two sentences>"}',
    "read_label":       '{"text": "<verbatim text on item, then short summary>"}',
    "get_rgbd_summary": '{"min_mm": <int>, "center_mm": <int>, "median_mm": <int>, "valid_fraction": <0..1>}',
    "get_depth_at_pixel": '{"depth_mm": <int>, "valid": true}',
    "list_visible_objects": '{"objects": [{"id": <int>, "label": "<coco>", "description": "..."}]}',
    "get_visible_objects": '{"objects": [{"id": <int>, "label": "...", "description": "...", "position": "left|center|right", "depth_m": <float>, "seen_count": <int>}]}',
    "find_object":      '{"objects": [...], "count": <int>}',
    "resolve_reference":'{"id": <int>, "label": "...", "description": "...", "position": "left|center|right", "depth_m": <float>}',
    "find_objects_matching_constraints": '{"objects": [...], "count": <int>}',
    "walk_forward":     '{"action": "walk_forward", "distance_m": <float>, "duration_s": <float>}',
    "walk_backward":    '{"action": "walk_backward", "distance_m": <float>, "duration_s": <float>}',
    "turn_left":        '{"action": "turn_left", "angle_deg": <float>}',
    "turn_right":       '{"action": "turn_right", "angle_deg": <float>}',
    "stop_motion":      '{"action": "stop_motion"}',
    "stop":             '{"action": "stop"}',
    "stop_tracking":    '{"action": "stop_tracking"}',
    "get_robot_status": '{"robot": {"rosbridge_connected": true, "motion_connected": true, "follow_connected": true, "depth_min_mm": <int>, "depth_center_mm": <int>}}',
    "follow_person":    '{"action": "follow_person", "yolo_id": <int>, "status": "tracking"}',
    "go_to_object":     '{"action": "go_to_object", "yolo_id": <int>, "status": "approaching"}',
    "find_and_go_to":   '{"label": "<str>", "found_yolo_id": <int>, "found_label": "<str>", "swept_deg": <float>, "direction": "left|right", "go_to_object": {"action": "go_to_object", "yolo_id": <int>, "status": "approaching"}}',
    "find_object":      '{"label": "<str>", "yolo_id": <int>, "found_label": "<str>", "swept_deg": <float>, "direction": "left|right"}',
    "find_person_and_follow": '{"label": "person", "found_yolo_id": <int>, "found_label": "person", "swept_deg": <float>, "direction": "left|right", "follow_person": {"action": "follow_person", "yolo_id": <int>}}',
    "send_basic_goal":  '{"action": "send_basic_goal", "x": <float>, "y": <float>, "theta": <float>, "status": "accepted"}',
    "cancel_basic_goal":'{"action": "cancel_basic_goal", "status": "cancelled"}',
    "get_basic_goal_status": '{"status": "goal_reached"}',
    "get_ros2_odom":    '{"x": <float>, "y": <float>, "z": <float>, "yaw": <float>}',
    "send_simple_cmd":  '{"action": "send_simple_cmd", "ack": true}',
    "send_complex_cmd": '{"action": "send_complex_cmd", "ack": true}',
}


@dataclass
class CallRecord:
    step: int
    tool: str
    args: dict
    result: str   # JSON string
    elapsed_ms: float


@dataclass
class Trace:
    prompt: str
    calls: list[CallRecord] = field(default_factory=list)
    final_reply: str = ""
    total_elapsed_s: float = 0.0


def _fabricate_result(
    bedrock,
    model_id: str,
    tool: str,
    args: dict,
    scenario_context: str,
    prior_calls: list[CallRecord],
) -> str:
    hint = TOOL_OUTPUT_HINTS.get(tool, '{"ok": true}')
    prior_summary = "\n".join(
        f"- {c.tool}({json.dumps(c.args)}) -> {c.result[:150]}"
        for c in prior_calls[-6:]  # last 6 to keep prompt small
    ) or "(none)"
    user_prompt = (
        f"Scenario: {scenario_context}\n\n"
        f"Prior tool calls in this scenario:\n{prior_summary}\n\n"
        f"Now simulate this call:\n"
        f"  tool: {tool}\n"
        f"  args: {json.dumps(args)}\n\n"
        f"Expected JSON shape: {hint}\n\n"
        f"Return ONLY the inner result JSON (what the tool's `result` field would contain). "
        f"No prose, no code fences."
    )
    resp = bedrock.converse(
        modelId=model_id,
        system=[{"text": FAKE_TOOL_SYSTEM}],
        messages=[{"role": "user", "content": [{"text": user_prompt}]}],
        inferenceConfig={"maxTokens": 350, "temperature": 0.4},
    )
    raw = "".join(p.get("text", "") for p in resp["output"]["message"]["content"]).strip()
    # Strip code fences if the model added them anyway.
    if raw.startswith("```"):
        raw = raw.strip("`").lstrip("json").strip()
    try:
        result_dict = json.loads(raw)
    except json.JSONDecodeError:
        result_dict = {"raw": raw, "parse_failed": True}
    return json.dumps({"ok": True, "tool": tool, "result": result_dict})


class FakeRegistry:
    """Drop-in for ToolRegistry. Same .call() and .names() signatures, but
    delegates to a fabricator instead of real handlers."""

    def __init__(self, real_names: list[str], scenario_context: str):
        self._names = real_names
        self._bedrock = make_client()
        self._model = os.environ.get(
            "BEDROCK_MODEL_ID", "us.anthropic.claude-sonnet-4-6"
        )
        self.scenario_context = scenario_context
        self.trace = Trace(prompt=scenario_context)

    def call(self, name: str, args: dict, ctx: ToolContext, caller: str) -> ToolResult:
        if name not in self._names:
            return ToolResult(ok=False, tool=name, error=f"unknown tool: {name!r}")
        t0 = time.time()
        result_json = _fabricate_result(
            self._bedrock,
            self._model,
            name,
            args,
            self.scenario_context,
            self.trace.calls,
        )
        elapsed_ms = (time.time() - t0) * 1000
        self.trace.calls.append(CallRecord(
            step=len(self.trace.calls) + 1,
            tool=name,
            args=args,
            result=result_json,
            elapsed_ms=elapsed_ms,
        ))
        # ToolResult expects a dict for `result`; we cheat and stuff the JSON
        # blob in via a parsed dict so .to_str() round-trips cleanly.
        try:
            parsed = json.loads(result_json)
            return ToolResult(
                ok=parsed.get("ok", True),
                tool=name,
                result=parsed.get("result"),
                error=parsed.get("error"),
            )
        except json.JSONDecodeError:
            return ToolResult(ok=True, tool=name, result={"raw": result_json})

    def names(self) -> list[str]:
        return list(self._names)


# Test prompts — representative of what the user might say.
DEFAULT_PROMPTS = [
    "what do you see?",
    "walk forward a little",
    "is there a chair nearby?",
    "follow me",
    "go to the chair",
    "stop",
    "describe the scene then walk forward 1 meter if it's safe",
    "read the label on the bottle in front of you",
    "turn left 90 degrees",
    "find the door and walk to it",
]


def _make_dummy_ctx(memory: MemoryStore) -> ToolContext:
    """A ctx where every robot/vlm/etc handle is None. Real tool handlers
    would crash, but FakeRegistry never invokes them."""
    return ToolContext(
        memory=memory,
        robot=None,
        motion=None,
        follow=None,
        basic_goal=None,
        vlm=None,
        safety=None,
    )


def _print_trace(trace: Trace) -> None:
    print()
    print("=" * 78)
    print(f"PROMPT: {trace.prompt}")
    print("=" * 78)
    for c in trace.calls:
        print(f"\n  step {c.step:>2} ({c.elapsed_ms:>5.0f} ms)  {c.tool}({json.dumps(c.args)})")
        # Pretty-print the fake result.
        try:
            parsed = json.loads(c.result)
            inner = parsed.get("result", parsed)
            print(f"           -> {json.dumps(inner)[:300]}")
        except json.JSONDecodeError:
            print(f"           -> {c.result[:300]}")

    print()
    print(f"  FINAL REPLY: {trace.final_reply}")
    print()
    # Summary stats.
    counts: dict[str, int] = {}
    for c in trace.calls:
        counts[c.tool] = counts.get(c.tool, 0) + 1
    repeats = {k: v for k, v in counts.items() if v > 1}
    print(f"  total calls: {len(trace.calls)}   unique tools: {len(counts)}   "
          f"total time: {trace.total_elapsed_s:.1f}s")
    if repeats:
        print(f"  REPEATED:    " + ", ".join(f"{k}×{v}" for k, v in repeats.items()))
    seq = " -> ".join(c.tool for c in trace.calls)
    print(f"  sequence:    {seq}")


def run_one(prompt: str, registry_names: list[str]) -> Trace:
    memory = MemoryStore()
    ctx = _make_dummy_ctx(memory)
    fake_reg = FakeRegistry(registry_names, scenario_context=prompt)
    planner = PlannerAgent(fake_reg, ctx, client=None, max_steps=8)
    snapshot = memory.snapshot()
    t0 = time.time()
    reply = planner.run(prompt, snapshot)
    fake_reg.trace.total_elapsed_s = time.time() - t0
    fake_reg.trace.final_reply = reply
    return fake_reg.trace


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--only",
        action="append",
        help="Run only prompts containing this substring (repeatable). Default: all.",
    )
    parser.add_argument(
        "--prompt",
        action="append",
        help="Add a custom prompt (repeatable).",
    )
    args = parser.parse_args()

    real_reg = build_registry()
    names = real_reg.names()
    print(f"[harness] {len(names)} tools available: {', '.join(names)}")

    prompts = list(DEFAULT_PROMPTS)
    if args.prompt:
        prompts.extend(args.prompt)
    if args.only:
        prompts = [p for p in prompts if any(s.lower() in p.lower() for s in args.only)]
        if not prompts:
            print("[harness] no prompts matched --only filters")
            return

    traces: list[Trace] = []
    for prompt in prompts:
        try:
            trace = run_one(prompt, names)
        except Exception as e:
            print(f"\n[harness] prompt {prompt!r} crashed: {type(e).__name__}: {e}")
            continue
        _print_trace(trace)
        traces.append(trace)

    # Aggregate report
    print()
    print("#" * 78)
    print("AGGREGATE")
    print("#" * 78)
    total_calls = sum(len(t.calls) for t in traces)
    print(f"  prompts run:      {len(traces)}")
    print(f"  total tool calls: {total_calls}")
    print(f"  avg per prompt:   {total_calls / max(len(traces), 1):.1f}")
    overall_counts: dict[str, int] = {}
    for t in traces:
        for c in t.calls:
            overall_counts[c.tool] = overall_counts.get(c.tool, 0) + 1
    print()
    print("  tool usage across all prompts:")
    for tool, n in sorted(overall_counts.items(), key=lambda kv: -kv[1]):
        bar = "█" * n
        print(f"    {tool:38} {n:>3}  {bar}")


if __name__ == "__main__":
    main()
