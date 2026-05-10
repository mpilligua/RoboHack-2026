# Agent & LLM abstraction overview

Reference for how the current laptop-side agent talks to Bedrock (Claude) and to the Lite3 over rosbridge. Snapshot of the code as of 2026-05-10.

## Big picture

Laptop runs the agent loop and an LLM (Claude on Bedrock). The robot (Lite3 at `192.168.1.103`) runs `rosbridge_websocket` and publishes RealSense + odometry topics. Everything flows over a WebSocket.

```
User text ─▶ cli.run_once ─▶ Bedrock Converse (Claude)
                 ▲                    │
                 │ tool results       │ toolUse requests
                 │                    ▼
           tools.dispatch ─▶ perception handlers
                                      │
                                      ├─ Lite3Robot (roslibpy)  ◀──WS──▶  rosbridge on robot
                                      └─ vlm_describe (Bedrock again, with image)
```

## Entry point: `agent/cli.py`

`main()` loads `.env`, opens a `Lite3Robot` context manager (connects to rosbridge), then loops on stdin. Each line calls `run_once(user_text, robot)`.

`run_once` is the agent loop:

1. Build a Bedrock Converse request with `SYSTEM_PROMPT` (guide-dog persona, "call tools, don't guess"), the user's message, and `toolConfig={"tools": TOOL_SCHEMAS}`.
2. Call `bedrock.converse(...)`. Inspect `stopReason`.
3. If not `tool_use`, concatenate text parts and return.
4. If `tool_use`, for every `toolUse` block in the model's output:
   - `dispatch(name, args, robot, bedrock)` runs the handler.
   - The result is appended as a `toolResult` block in a new `user` message.
5. Loop, up to `max_steps=6`.

Tool calls and the first 200 chars of each result are echoed to stderr so you can watch the agent reason.

## Tool layer: `agent/tools/perception.py`

`TOOL_SCHEMAS` is a list of `toolSpec` entries (Bedrock Converse format). Five tools are exposed:

| Tool | Handler | What it touches |
|---|---|---|
| `describe_scene(focus?)` | `_describe_scene` | `robot.rgb_jpeg_b64()` → `vlm_describe` |
| `read_label(item_description)` | `_read_label` | same, stricter prompt |
| `get_rgbd_summary()` | `_get_rgbd_summary` | `robot.depth_summary()` (local numpy) |
| `get_pose()` | `_get_pose` | `robot.get_pose()` |
| `get_status()` | `_get_status` | cached frame ages + connection flag |

`dispatch(name, args, robot, vlm)` looks the name up in `_HANDLERS` and wraps exceptions so a failure comes back as `{"error": "..."}` JSON instead of crashing the loop.

The "VLM" passed to tools is the same Bedrock client — a visual tool just calls Converse again but with the JPEG inline (see `vlm.vlm_describe`). So inside a single turn you can get Bedrock calling itself: outer Claude decides `describe_scene`, handler captures a frame, inner Converse call sends that JPEG to Claude, text comes back as the tool result, outer Claude summarizes for the user.

## Robot adapter: `agent/robot/lite3.py`

`Lite3Robot` wraps `roslibpy.Ros` and subscribes to:

- `/camera/color/image_raw` → decoded (base64 → numpy → PIL) into `_rgb`
- `/camera/depth/image_rect_raw` → uint16 depth in mm into `_depth`
- `/leg_odom` → yaw pulled from quaternion into `_pose`

Each callback caches the latest frame under a lock. `get_rgb` / `get_depth` / `get_pose` poll with a small timeout. Helpers on top:

- `rgb_jpeg_b64(quality=85)` — what the VLM handlers feed into Converse.
- `depth_summary()` — `min_mm`, `median_mm`, `center_mm`, `valid_fraction`; pure local numpy, no model call.

It also advertises `/cmd_vel` and exposes `set_velocity` / `stop`, but those aren't wired as tools yet.

## Motion adapter: `agent/robot/motion.py` (not yet in the loop)

`Lite3Motion` is a separate class that would connect to a second rosbridge on port 9091 (ROS 2 foxy, because Lite3's motion stack is on foxy while perception is on noetic). It exposes `forward`, `backward`, `turn_left` / `right`, `strafe_left` / `right`, all clamped by `MAX_LINEAR=0.3 m/s`, `MAX_ANGULAR=0.5 rad/s`, `MAX_DURATION=2.0 s`, and always emits a final zero Twist. Per `RUNBOOK.md`, the foxy bridge isn't running yet, so no motion tools are registered in `TOOL_SCHEMAS`.

## LLM client: `agent/vlm.py`

Two tiny functions:

- `make_client()` → `boto3.client("bedrock-runtime")`, region from `AWS_REGION`.
- `_model_id()` → `BEDROCK_MODEL_ID` env, default `us.anthropic.claude-opus-4-7-20251101-v1:0`.
- `vlm_describe(client, jpeg_b64, prompt)` → single-shot Converse with `[{text}, {image}]` content.

Auth is the new Bedrock bearer token (`AWS_BEARER_TOKEN_BEDROCK`), picked up automatically by boto3.

## One end-to-end example

User: `what do you see in front of you?`

1. `cli.run_once` sends the message plus 5 tool schemas to Claude.
2. Claude returns `stopReason=tool_use` with `toolUse{name="describe_scene", input={"focus":"obstacles ahead"}}`.
3. `dispatch` → `_describe_scene` → `robot.rgb_jpeg_b64()` pulls the latest cached RealSense frame, encodes JPEG.
4. `vlm_describe` sends prompt + JPEG back to Bedrock (same model), returns a paragraph.
5. That paragraph is posted as a `toolResult` in the next user message.
6. Claude runs again, this time returns plain text (`stopReason=end_turn`), which `run_once` prints.

---

# LLM abstraction, deeper dive

There are essentially two layers: the **outer agent loop** (Converse + tool use) and the **inner VLM call** (Converse with an image). Both hit the same Bedrock Converse endpoint, just with different message shapes.

## 1. The Converse API in one paragraph

Bedrock's `converse(modelId, system, messages, toolConfig, inferenceConfig)` is a unified chat API across providers (Anthropic, Llama, Titan, etc.). Messages are an ordered list of `{"role": "user"|"assistant", "content": [blocks]}`. Content blocks are typed: `{"text": ...}`, `{"image": {...}}`, `{"toolUse": {...}}`, `{"toolResult": {...}}`. The response has `output.message` (always `role=assistant`) and a top-level `stopReason` that tells you *why* the model stopped: `end_turn`, `tool_use`, `max_tokens`, etc. This is the pivot the loop turns on.

## 2. Tool schemas: a thin wrapper over `toolSpec`

`tools/perception.py` defines tools via `_spec(...)` which produces:

```python
{
  "toolSpec": {
    "name": "describe_scene",
    "description": "Capture the current RGB frame ...",
    "inputSchema": {"json": {
      "type": "object",
      "properties": {"focus": {"type": "string", "description": "..."}},
      # "required": [...] only added when non-empty
    }}
  }
}
```

Three conventions worth noting:

- **`inputSchema.json` is literal JSON Schema** — Bedrock validates the model's tool args against it before dispatching. Bad args surface as a client error, not as garbage into your handler.
- **Descriptions are the prompt.** The model picks a tool based on `description`, not `name`. The guide-dog system prompt says "call tools to look at the world, never guess," and each tool description tells Claude *when* to pick it ("Use when the user asks what the robot sees"). That wording is the primary lever for routing behavior.
- **No `required` when `properties` is empty.** Converse rejects `"required": []` on some providers, so `_spec` only adds it when non-empty. That's why `get_pose` / `get_rgbd_summary` / `get_status` schemas look bare.

`TOOL_SCHEMAS` is then passed through as `toolConfig={"tools": TOOL_SCHEMAS}` — a plain list, no tool routing or filtering logic on the client side.

## 3. The agent loop as a state machine

`cli.run_once` is deceptively simple. Abstractly:

```
append user message
loop up to max_steps:
    resp = converse(system, messages, toolConfig)
    append resp.output.message                   # role=assistant
    if resp.stopReason != "tool_use": return text
    run every toolUse in resp.output.message
    append a single user message whose content
      is the list of toolResult blocks
```

Key invariants that keep this from corrupting:

- **Assistant messages are appended verbatim.** The assistant's `content` may contain a mix of `text` blocks (model's running commentary) and `toolUse` blocks. You append the whole thing. You don't re-serialize or pick pieces out — Bedrock cares about exact echo.
- **Every `toolUse` must be answered in the next user turn.** If Claude emits three `toolUse` blocks, the next user message must carry three `toolResult` blocks with matching `toolUseId`s. `run_once` handles this by collecting them in a list and posting one user message with all of them:

  ```python
  tool_results.append({
    "toolResult": {
      "toolUseId": tu["toolUseId"],
      "content": [{"text": result}],
    }
  })
  messages.append({"role": "user", "content": tool_results})
  ```

  Miss one and you get a 400 on the next turn.
- **Tool results are always stringified.** The handlers return `str` (either prose from the VLM or `json.dumps(...)` for structured data). Bedrock also accepts `{"toolResult": {..., "content": [{"json": {...}}]}}`, which would skip a `json.dumps` / `json.loads` roundtrip, but the code picks the string form uniformly. This is a fine call — easier to log, works across models — just know it's a choice.
- **`stopReason` is the only exit condition** besides `max_steps`. The code doesn't try to detect "final answer" in text; if the model emits any tool call it loops, otherwise it returns. Max steps is a hard safety bound (6 means at most 6 Bedrock calls — still cheap, but caps blast radius on an agent that decides to keep probing).
- **Errors become tool results, not exceptions.** `dispatch` catches every `Exception` and returns `json.dumps({"error": "..."})`. That JSON flows back into Claude's context, which is what lets the system prompt's "If a tool fails or returns 'error', tell the user plainly" work without the loop crashing.

## 4. Model selection and auth

`vlm._model_id()` reads `BEDROCK_MODEL_ID` with a default of `us.anthropic.claude-opus-4-7-20251101-v1:0`. RUNBOOK notes `us.anthropic.claude-sonnet-4-6` as the currently-entitled fallback — i.e. model is env-swappable without touching code, which matters because Bedrock's entitlement model ("explicit deny" without the right inference profile) bites often.

Auth is entirely implicit: `boto3.client("bedrock-runtime", region_name=...)` picks up `AWS_BEARER_TOKEN_BEDROCK` automatically. No SigV4 setup, no explicit credentials in code. The `make_client()` helper exists to centralize the region env read, and the same client is reused both for the outer loop and for the inner vision call in handlers.

## 5. The inner VLM call: same API, different shape

`vlm.vlm_describe(client, jpeg_b64, prompt, model=None, max_tokens=512)` is the second LLM abstraction. It:

- Takes a base64 string, decodes it once (`image_bytes = base64.b64decode(...)`), and passes raw bytes into `{"image": {"format": "jpeg", "source": {"bytes": image_bytes}}}`. Converse's Python SDK accepts bytes here, not base64 — encoding twice would silently fail.
- Uses a *single* user message: `[{"text": prompt}, {"image": ...}]`. No `system` block, no tools, no history.
- Reads every `text` block from the response and concatenates: `"".join(p.get("text", "") for p in parts)`. That's robust to the model sometimes splitting a response across multiple text blocks.

This is deliberately a separate function from the agent loop — it's stateless, single-shot, no tool use. When a handler like `_describe_scene` calls it, the outer agent's conversation never sees the image, only the returned text. That keeps outer-loop tokens small: raw JPEGs don't balloon the message history, only the VLM's paragraph does.

## 6. Nested call topology

One turn of `describe_scene` therefore hits Bedrock *twice*:

```
outer converse (text+tools) ─▶ tool_use: describe_scene
                                    │
                                    ▼
                               rgb_jpeg_b64()        # robot
                                    │
                                    ▼
                    inner converse (text+image)      # vlm_describe
                                    │
                                    ▼
                              toolResult text
                                    │
                                    ▼
outer converse (text+tools) ─▶ end_turn: final answer
```

Two consequences:

- **Latency stacks.** A scene question is ~`2 * bedrock_rtt + rosbridge_frame_wait`. If you later want the outer agent to ask follow-ups on the same image, caching the VLM's description as context (or passing the image directly into the outer turn) would cut latency in half — at the cost of fatter outer messages.
- **The outer model can't "see."** It only gets the VLM's prose. If the description is ambiguous, the outer model has no way to re-examine the pixels, only to call `describe_scene` again with a different `focus`. This is a real limitation of the current abstraction and is the main reason the outer model occasionally double-calls with different focus hints.

## 7. What's explicitly outside the abstraction

- **No streaming.** `converse` is blocking; `converse_stream` isn't used. Fine for CLI, would need replacing for a voice/websocket UI.
- **No token accounting.** The response's `usage` field isn't inspected. Bedrock returns input/output tokens per call — easy win if cost tracking matters.
- **No retries.** A transient 5xx from Bedrock crashes `run_once`. Boto3's default retry config covers *some* of this, but tool-use turns with large images benefit from explicit `botocore.config.Config(retries={"mode": "adaptive"})`.
- **No tool-choice control.** `toolConfig` only carries `tools`, not `toolChoice`. Claude could be forced to call a specific tool via `{"tool": {"name": "describe_scene"}}` if you wanted a "must look first" mode. Right now routing is pure prompt engineering.
- **Single system prompt, static.** `SYSTEM_PROMPT` is a module-level constant, identity + tool contract baked in. If you wanted per-session personas (e.g. "verbose debugging mode"), you'd wire them through `run_once`.

## 8. How to extend without breaking the abstraction

Adding a tool means three additions, nothing else:

1. A `_spec(...)` entry in `TOOL_SCHEMAS`.
2. A handler `def _foo(robot, vlm, args: dict) -> str:` — return a string, JSON or prose.
3. A `"foo": _foo` entry in `_HANDLERS`.

Because `dispatch` takes `(name, args, robot, vlm)` as the whole contract, new tools automatically get both the robot and the Bedrock client. That's how `Lite3Motion` would plug in: pass a second object into `dispatch`, or attach it to `robot`, and add `walk_forward` / `turn` / `stop` tools with clamped numeric schemas. No loop changes needed — `run_once` is tool-agnostic, it just pipes `toolUse` to `dispatch` and `toolResult` back.

## Current gaps (visible in code + RUNBOOK)

- No motion tools exposed yet — `Lite3Motion` exists but isn't in `_HANDLERS`, and the foxy rosbridge on 9091 isn't running.
- Depth isn't aligned to RGB (explicit comment in `lite3.py`); pixel-accurate "how far is that object" needs `/camera/aligned_depth_to_color`.
- No LiDAR tool in the agent despite `frame_lidar*.ply` files on disk and `scripts/grab_lidar.py` — it's still offline experimentation.
- `get_status` reaches into `robot._lock` / `_rgb` / `_depth` directly (marked `# noqa: SLF001`), so that private shape is load-bearing for tooling.
