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

from robot import Lite3Robot  # noqa: E402
from tools import TOOL_SCHEMAS, dispatch  # noqa: E402
from vlm import make_client, _model_id  # noqa: E402


SYSTEM_PROMPT = """\
You are the assistant brain of a quadruped guide-dog robot helping a blind \
or low-vision user. The robot has an RGBD camera at knee height. You can \
inspect what it sees and report what is around. Always:

- Call tools to look at the world; never guess what is in front of the robot.
- Be concise and concrete. Name objects, give rough positions (left / centered / right).
- If a tool fails or returns 'error', tell the user plainly.
- If the user asks something the available tools cannot answer, say so.
"""


def run_once(user_text: str, robot: Lite3Robot, max_steps: int = 6) -> str:
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
            result = dispatch(name, args, robot, bedrock)
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
    port = int(os.environ.get("ROS_BRIDGE_PORT", "9090"))

    print(f"connecting to rosbridge ws://{host}:{port} …", file=sys.stderr)
    with Lite3Robot(host=host, port=port) as robot:
        print("connected. type a question (Ctrl-D to exit).", file=sys.stderr)
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                print(file=sys.stderr)
                return
            if not line:
                continue
            answer = run_once(line, robot)
            print(answer)


if __name__ == "__main__":
    main()
