# Hackathon migration plan

**Goal:** get from the working POC (`agent/cli.py`) to a demo-ready system with semantic memory, multi-turn shelf reasoning (Beat 5), and a phone UI.

**Rule:** `python agent/cli.py` stays runnable at every step.

---

## What the POC already does (don't touch)

- `robot/lite3.py` — connects to rosbridge :9090, streams RGB + depth + leg_odom. **Works.**
- `robot/motion.py` — `Lite3Motion` over foxy :9091, publishes `/cmd_vel`. **Works** (tested via `scripts/move.py`).
- `tools/perception.py` — 5 Bedrock tools: `describe_scene`, `read_label`, `get_rgbd_summary`, `get_pose`, `get_status`. **Works.**
- `vlm.py` — Bedrock Converse single-shot VLM. **Works.**
- `cli.py` — Bedrock ReAct loop. **Works.**
- `scripts/grab_lidar.py` — standalone lidar decoder. **Works.**

**Two bugs to fix immediately (5 min each):**
1. `cli.py::run_once` rebuilds `messages = []` every turn → no cross-turn memory. Hoist `messages` out of `run_once` into `main()`.
2. `Lite3Robot.__init__` advertises `/cmd_vel` on the noetic bridge — wrong bridge, dead code. Delete those 3 lines.

---

## The three things that actually need building

### 1. Object memory (Beat 5 centerpiece) — Person B, ~3 h

Beat 5 needs the robot to remember what it scanned and answer follow-ups without re-scanning. Right now there is no memory at all.

**Build a minimal `memory.py` at repo root:**

```python
# memory.py
import time, uuid
from dataclasses import dataclass, field

@dataclass
class Obj:
    id: str
    label: str
    depth_m: float | None = None
    bearing_deg: float | None = None
    position: str | None = None        # "leftmost", "center", etc.
    visible_text: list[str] = field(default_factory=list)
    risks: list[str] = field(default_factory=list)
    attributes: dict = field(default_factory=dict)
    recommended: bool = False

class ObjectStore:
    def __init__(self):
        self._objects: dict[str, Obj] = {}
        self._last_recommended: str | None = None

    def upsert(self, obj: Obj) -> Obj:
        self._objects[obj.id] = obj
        return obj

    def all(self) -> list[Obj]:
        return list(self._objects.values())

    def get(self, oid: str) -> Obj | None:
        return self._objects.get(oid)

    def find(self, query: str) -> list[Obj]:
        q = query.lower()
        return [o for o in self._objects.values()
                if q in o.label.lower() or q in " ".join(o.visible_text).lower()]

    def mark_recommended(self, oid: str):
        self._last_recommended = oid

    def last_recommended(self) -> Obj | None:
        return self._objects.get(self._last_recommended) if self._last_recommended else None

    def make_id(self, label: str) -> str:
        return f"obj_{label.lower().replace(' ','_')[:15]}_{uuid.uuid4().hex[:4]}"

    def clear(self):
        self._objects.clear()
        self._last_recommended = None
```

**Add `scan_shelf` and `resolve_reference` to `tools/perception.py`:**

`scan_shelf` — capture frame, call VLM with this prompt, parse JSON, upsert into `ObjectStore`:

```
Prompt:
You are the perception system of a guide-dog robot at knee height facing a shelf.
Return a JSON array. Each element:
{"label":"...", "position":"leftmost|second from left|center|second from right|rightmost",
 "bearing_deg":<float>, "depth_m":<float>, "visible_text":["..."],
 "risks":["contains nuts","..."], "attributes":{"healthy_candidate":true}}
Return ONLY the JSON array.
```

`resolve_reference(ref, store)` — pure Python, no VLM:
- "it" / "that" → `store.last_recommended()`
- "left" / "leftmost" → min bearing_deg
- "right" / "rightmost" → max bearing_deg
- "closest" / "nearest" → min depth_m
- "healthy" → first with `healthy_candidate=True` and no risks
- fallback → `store.find(ref)[0]`

**Update system prompt in `cli.py`:**
```
Call scan_shelf once. Answer follow-ups from memory using get_visible_objects,
resolve_reference, find_objects_matching_constraints. Do not re-scan.
```

**Beat 5 test (do this before moving on):**
```
> what's on the shelf?          → scan_shelf → lists items
> anything with nuts?           → read_label on candidates → answers from memory
> which is healthiest?          → find_objects_matching_constraints → recommends
> take me to it                 → resolve_reference("it") → returns object_id
```

---

### 2. Motion wired to the agent — Person A, ~2 h

`Lite3Motion` works but isn't exposed as a Bedrock tool. Add three tools to `tools/perception.py`:

```python
_spec("move_forward",  "Move forward N meters (max 1.0m, max 0.25 m/s).",
      {"distance_m": {"type": "number"}}, required=["distance_m"]),
_spec("rotate",        "Rotate in place. Positive=left, negative=right.",
      {"degrees": {"type": "number"}}, required=["degrees"]),
_spec("stop",          "Stop all motion immediately.", {}),
```

Handlers call `Lite3Motion` directly. Keep it simple — no safety supervisor needed for the hackathon, just clamp values:

```python
def _move_forward(robot, vlm, args):
    from robot import Lite3Motion
    import os
    m = Lite3Motion(host=os.environ.get("ROS_BRIDGE_HOST","192.168.1.103"),
                    port=int(os.environ.get("ROS2_BRIDGE_PORT","9091")))
    dist = min(float(args["distance_m"]), 1.0)
    speed = 0.15
    m.forward(speed, dist / speed)
    m.close()
    return json.dumps({"ok": True, "moved_m": dist})
```

**Safety note:** the existing `MAX_LINEAR=0.3`, `MAX_ANGULAR=0.5`, `MAX_DURATION=2.0` clamps in `Lite3Motion._drive` are sufficient for the demo. Don't over-engineer this.

---

### 3. Phone UI — Person C, ~3 h

Minimal FastAPI + WebSocket + single HTML file. No React, no build step.

**`server.py` at repo root:**

```python
# server.py — run with: python server.py
import asyncio, json, os
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
import uvicorn
from dotenv import load_dotenv

load_dotenv()

# Import the existing agent loop
import sys; sys.path.insert(0, "agent")
from robot.lite3 import Lite3Robot
from tools import TOOL_SCHEMAS, dispatch
from vlm import make_client, _model_id

app = FastAPI()
_connections: list[WebSocket] = []
_robot: Lite3Robot | None = None
_messages: list[dict] = []   # session memory

class Msg(BaseModel):
    text: str

@app.on_event("startup")
async def startup():
    global _robot
    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    port = int(os.environ.get("ROS_BRIDGE_PORT", "9090"))
    _robot = Lite3Robot(host=host, port=port)

@app.post("/message")
async def message(msg: Msg):
    global _messages
    response = await asyncio.to_thread(_run_once, msg.text, _robot, _messages)
    await _broadcast({"type": "speech", "text": response})
    return {"response": response}

@app.post("/stop")
async def stop():
    if _robot:
        _robot.stop()
    return {"ok": True}

@app.get("/objects")
async def objects():
    # Return objects from the global store (set up in tools/perception.py)
    from tools.perception import _store
    return {"objects": [vars(o) for o in _store.all()]}

@app.websocket("/ws")
async def ws(websocket: WebSocket):
    await websocket.accept()
    _connections.append(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        _connections.remove(websocket)

async def _broadcast(data: dict):
    for ws in list(_connections):
        try:
            await ws.send_text(json.dumps(data))
        except Exception:
            _connections.remove(ws)

# Serve the phone UI
app.mount("/", StaticFiles(directory="frontend", html=True), name="static")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

**`frontend/index.html`** — single file, no build:

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Guide Dog</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font: 16px system-ui; background: #111; color: #eee; padding: 1rem; max-width: 480px; margin: auto; }
    h1 { color: #7dd3fc; margin-bottom: 1rem; font-size: 1.1rem; }
    #log { background: #1a1a1a; border-radius: 8px; padding: 1rem; height: 220px; overflow-y: auto; margin-bottom: 1rem; font-size: 0.9rem; }
    .u { color: #93c5fd; } .r { color: #86efac; } .t { color: #fbbf24; font-size: 0.8rem; }
    #objs { background: #1a1a1a; border-radius: 8px; padding: 0.75rem; margin-bottom: 1rem; font-size: 0.85rem; min-height: 2rem; }
    input { width: 100%; background: #222; border: 1px solid #444; border-radius: 6px; padding: 0.6rem; color: #eee; font-size: 1rem; margin-bottom: 0.5rem; }
    .btn { width: 100%; padding: 0.7rem; border: none; border-radius: 8px; font-size: 1rem; cursor: pointer; margin-bottom: 0.5rem; }
    #send { background: #2563eb; color: #fff; }
    #stop { background: #dc2626; color: #fff; font-weight: bold; font-size: 1.1rem; }
  </style>
</head>
<body>
  <h1>🐕 Guide Dog Assistant</h1>
  <div id="log"></div>
  <div style="font-size:0.7rem;color:#6b7280;margin-bottom:0.25rem">OBJECTS IN MEMORY</div>
  <div id="objs">—</div>
  <input id="inp" placeholder="Ask something..." />
  <button class="btn" id="send" onclick="send()">Send</button>
  <button class="btn" id="stop" onclick="estop()">⛔ STOP</button>
  <script>
    const log = document.getElementById('log');
    const objs = document.getElementById('objs');
    function add(text, cls) {
      const d = document.createElement('div');
      d.className = cls; d.textContent = text;
      log.appendChild(d); log.scrollTop = log.scrollHeight;
    }
    const ws = new WebSocket(`ws://${location.hostname}:8000/ws`);
    ws.onmessage = e => {
      const d = JSON.parse(e.data);
      if (d.type === 'speech') add('🐕 ' + d.text, 'r');
      if (d.type === 'tool') add('⚙ ' + d.text, 't');
    };
    async function send() {
      const inp = document.getElementById('inp');
      const text = inp.value.trim(); if (!text) return;
      add('👤 ' + text, 'u'); inp.value = '';
      const r = await fetch('/message', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({text})});
      await refreshObjs();
    }
    async function estop() {
      await fetch('/stop', {method:'POST'});
      add('🚨 STOP sent', 't');
    }
    async function refreshObjs() {
      const r = await fetch('/objects');
      const d = await r.json();
      objs.innerHTML = d.objects.length
        ? d.objects.map(o => `• <b>${o.label}</b> ${o.position||''} ${o.depth_m ? o.depth_m.toFixed(1)+'m' : ''} ${o.risks.length ? '⚠ '+o.risks.join(', ') : ''}`).join('<br>')
        : '—';
    }
    document.getElementById('inp').addEventListener('keydown', e => { if (e.key==='Enter') send(); });
  </script>
</body>
</html>
```

---

## File map — what goes where

```
agent/
  cli.py              fix: hoist messages list, add --server flag to call server.py instead
  robot/lite3.py      fix: remove dead /cmd_vel advertise on noetic bridge
  tools/perception.py add: scan_shelf, resolve_reference, move_forward, rotate, stop tools
                      add: module-level _store = ObjectStore()
  vlm.py              no change

memory.py             NEW — ObjectStore (flat file at repo root for simplicity)
server.py             NEW — FastAPI + WebSocket + static file server
frontend/
  index.html          NEW — phone UI

configs/
  topics.yaml         NEW — connection params (optional but nice)
```

That's it. No `agents_app/`, no `backend/`, no `perception/`, no `tools/base.py`. Those are the right abstractions for a production system — not for 12 hours.

---

## Build order

| # | What | Who | Time | Gate |
|---|---|---|---|---|
| 1 | Fix `messages` hoist + dead `/cmd_vel` | any | 10 min | cli.py still works |
| 2 | `memory.py` ObjectStore | B | 30 min | imports cleanly |
| 3 | `scan_shelf` tool + system prompt update | B | 2 h | Beat 5 works in CLI |
| 4 | `resolve_reference` + `find_objects_matching_constraints` | B | 1 h | "take me to it" resolves |
| 5 | `move_forward` / `rotate` / `stop` tools | A | 1 h | dog moves from CLI |
| 6 | `server.py` + `frontend/index.html` | C | 2 h | phone shows transcript + objects |
| 7 | Wire tool events to WebSocket broadcast | B+C | 30 min | phone shows ⚙ tool calls live |
| 8 | Full demo run-through | all | 1 h | Beat 5 end-to-end on phone |

**If time runs out after step 4:** the CLI demo is already Beat 5-complete. Steps 5–8 are additive.

---

## Cached scan fallback (30 min, do Day 2 morning)

After a successful scan, save it:

```python
# scripts/save_scan.py
import json, sys
sys.path.insert(0, ".")
from tools.perception import _store
json.dump([vars(o) for o in _store.all()], open(sys.argv[1], "w"), indent=2)
print(f"saved {len(_store.all())} objects")
```

Load it if the camera dies:

```python
# scripts/load_scan.py
import json, sys
sys.path.insert(0, ".")
from tools.perception import _store
from memory import Obj
for d in json.load(open(sys.argv[1])):
    _store.upsert(Obj(**d))
print(f"loaded {len(_store.all())} objects")
```

---

## What to skip entirely

- OpenAI Agents SDK — the Bedrock Converse loop works fine for the demo
- ToolRegistry with access control — overkill for 3 tools
- SafetySupervisor as a separate class — the clamps in `Lite3Motion._drive` are enough
- FrameBuffer — single-slot in `Lite3Robot` is fine
- Separate `agents_app/`, `backend/`, `perception/` packages — one `server.py` is enough
- FakeRobot — just test with the real robot or mock at the function level

The architecture doc describes the right long-term design. For the hackathon, the goal is Beat 5 working on a phone. Ship that first.
