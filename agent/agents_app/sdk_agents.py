"""DialogueAgent and PlannerAgent.

Default backend: AWS Bedrock native Converse API (boto3) — works with
BEDROCK_MODEL_ID + AWS_REGION + AWS_BEARER_TOKEN_BEDROCK.

Optional override: set AGENT_BASE_URL + AGENT_API_KEY + AGENT_MODEL to use
any OpenAI-compatible endpoint instead (Qwen, OpenAI, future Bedrock compat).
"""

from __future__ import annotations

import json
import os
import re
import sys
from typing import Iterator

from .sdk_tools import PLANNER_TOOLS, PLANNER_TOOLS_BEDROCK, dispatch_tool
from tools.base import ToolContext
from tools.registry import ToolRegistry
from vlm import make_client


# Cut on `. `, `! `, `? ` or newline so each chunk is a self-contained clause
# that `say` can synthesize without sounding clipped.
_SENTENCE_SPLIT = re.compile(r'(?<=[.!?])\s+|\n+')


def _sentence_chunks(buf: str) -> tuple[list[str], str]:
    """Split buf into completed sentences + remaining tail. The tail is
    whatever didn't reach a terminator yet; caller keeps it for the next delta."""
    parts = _SENTENCE_SPLIT.split(buf)
    if len(parts) == 1:
        return [], buf
    return [p for p in parts[:-1] if p.strip()], parts[-1]

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

Rules:
- Before walking forward, ALWAYS call get_rgbd_summary or check_safety first.
- Use short, time-based commands for motion (walk_forward/backward/turn_left/turn_right).
- Before follow_person or go_to_object, ALWAYS call list_visible_objects first.
- If a tool returns ok=false, report why to the user concisely.
- After completing the goal, return a brief natural-language summary.
- Do NOT describe tool call details to the user."""


def _use_openai_compat() -> bool:
    return bool(os.environ.get("AGENT_BASE_URL", "").strip())


def _make_agent_client():
    """Return an openai.OpenAI client. Only used when AGENT_BASE_URL is set."""
    import openai
    return openai.OpenAI(
        base_url=os.environ["AGENT_BASE_URL"],
        api_key=os.environ.get("AGENT_API_KEY", "unused"),
    )


def _agent_model() -> str:
    model = os.environ.get("AGENT_MODEL", "").strip()
    if not model:
        raise RuntimeError("AGENT_MODEL is not set")
    return model


def _bedrock_model() -> str:
    return os.environ.get("BEDROCK_MODEL_ID", "us.anthropic.claude-sonnet-4-6")


class DialogueAgent:
    """Classify intent and extract a goal sentence from user input."""

    def __init__(self, client=None) -> None:
        self._client = client  # openai.OpenAI or None

    def run(self, user_text: str) -> dict:
        if _use_openai_compat() and self._client is not None:
            return self._run_openai(user_text)
        return self._run_boto3(user_text)

    def _run_openai(self, user_text: str) -> dict:
        try:
            resp = self._client.chat.completions.create(
                model=_agent_model(),
                messages=[
                    {"role": "system", "content": DIALOGUE_SYSTEM},
                    {"role": "user", "content": user_text},
                ],
                max_tokens=200,
                temperature=0.0,
            )
            raw = resp.choices[0].message.content or ""
            return json.loads(raw)
        except (json.JSONDecodeError, Exception):
            return {"intent": "unknown", "goal": user_text}

    def _run_boto3(self, user_text: str) -> dict:
        bedrock = make_client()
        try:
            resp = bedrock.converse(
                modelId=_bedrock_model(),
                system=[{"text": DIALOGUE_SYSTEM}],
                messages=[{"role": "user", "content": [{"text": user_text}]}],
                inferenceConfig={"maxTokens": 200},
            )
            raw = "".join(
                p.get("text", "") for p in resp["output"]["message"]["content"]
            )
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
        max_steps: int = 8,
    ) -> None:
        self._registry = registry
        self._ctx = ctx
        self._client = client  # openai.OpenAI or None
        self._max_steps = max_steps

    def run(self, goal: str, memory_snapshot: dict) -> str:
        if _use_openai_compat() and self._client is not None:
            return self._run_openai(goal, memory_snapshot)
        return self._run_boto3(goal, memory_snapshot)

    def run_stream(self, goal: str, memory_snapshot: dict) -> Iterator[str]:
        """Yield text chunks (sentence-sized) as soon as they arrive from
        the model. Tool-loop steps don't emit; only the final answer streams."""
        if _use_openai_compat() and self._client is not None:
            # OpenAI-compat streaming not implemented — fall back to one-shot.
            yield self._run_openai(goal, memory_snapshot)
            return
        yield from self._run_boto3_stream(goal, memory_snapshot)

    def _run_openai(self, goal: str, memory_snapshot: dict) -> str:
        system_content = PLANNER_SYSTEM + "\n\nCurrent robot memory:\n" + json.dumps(memory_snapshot, indent=2)
        messages: list[dict] = [
            {"role": "system", "content": system_content},
            {"role": "user", "content": goal},
        ]
        for _ in range(self._max_steps):
            resp = self._client.chat.completions.create(
                model=_agent_model(),
                messages=messages,
                tools=PLANNER_TOOLS,
                tool_choice="auto",
                max_tokens=1024,
                temperature=0.1,
            )
            choice = resp.choices[0]
            msg = choice.message
            messages.append(msg.model_dump(exclude_unset=True))

            if choice.finish_reason == "stop" or not msg.tool_calls:
                return msg.content or "(no response)"

            for tc in msg.tool_calls:
                name = tc.function.name
                try:
                    args = json.loads(tc.function.arguments or "{}")
                except json.JSONDecodeError:
                    args = {}
                print(f"  [planner] -> {name}({args})", file=sys.stderr)
                result_str = dispatch_tool(name, args, self._registry, self._ctx)
                print(f"  [planner] <- {result_str[:200]}", file=sys.stderr)
                messages.append({"role": "tool", "tool_call_id": tc.id, "content": result_str})

        return "(agent: reached max steps)"

    def _run_boto3(self, goal: str, memory_snapshot: dict) -> str:
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
                return "".join(p.get("text", "") for p in out_msg["content"] if "text" in p)

            tool_results = []
            for part in out_msg["content"]:
                if "toolUse" not in part:
                    continue
                tu = part["toolUse"]
                name = tu["name"]
                args = tu.get("input") or {}
                print(f"  [planner] -> {name}({args})", file=sys.stderr)
                result_str = dispatch_tool(name, args, self._registry, self._ctx)
                print(f"  [planner] <- {result_str[:200]}", file=sys.stderr)
                tool_results.append({
                    "toolResult": {
                        "toolUseId": tu["toolUseId"],
                        "content": [{"text": result_str}],
                    }
                })

            messages.append({"role": "user", "content": tool_results})

        return "(agent: reached max steps)"

    def _run_boto3_stream(self, goal: str, memory_snapshot: dict) -> Iterator[str]:
        """Same loop as _run_boto3, but the FINAL turn (no more tool_use) is
        opened with converse_stream so caller can start TTS-ing while the
        model is still generating."""
        bedrock = make_client()
        system_text = PLANNER_SYSTEM + "\n\nCurrent robot memory:\n" + json.dumps(memory_snapshot, indent=2)
        system = [{"text": system_text}]
        messages: list[dict] = [{"role": "user", "content": [{"text": goal}]}]
        tool_config = {"tools": PLANNER_TOOLS_BEDROCK}

        for _ in range(self._max_steps):
            # We can't know if this turn will be the final one without asking
            # the model, so we always use the streaming API and decide what to
            # do based on the stop reason as the stream drains.
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
            text_buffer = ""  # for sentence-chunk yielding

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

            # Flush trailing text that didn't end with a sentence terminator.
            if text_buffer.strip():
                yield text_buffer

            messages.append({"role": "assistant", "content": assembled_content})

            if stop_reason != "tool_use":
                return

            tool_results = []
            for part in assembled_content:
                if "toolUse" not in part:
                    continue
                tu = part["toolUse"]
                name = tu["name"]
                args = tu.get("input") or {}
                print(f"  [planner] -> {name}({args})", file=sys.stderr)
                result_str = dispatch_tool(name, args, self._registry, self._ctx)
                print(f"  [planner] <- {result_str[:200]}", file=sys.stderr)
                tool_results.append({
                    "toolResult": {
                        "toolUseId": tu["toolUseId"],
                        "content": [{"text": result_str}],
                    }
                })

            messages.append({"role": "user", "content": tool_results})

        yield "(agent: reached max steps)"
