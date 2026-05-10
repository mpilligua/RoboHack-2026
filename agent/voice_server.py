"""Voice UI server using the Bedrock-native agent."""

from __future__ import annotations

import argparse
import atexit
import io
import os
import signal
import sys
import tempfile
import threading
import time
import uuid
from concurrent.futures import Future, ThreadPoolExecutor

from dotenv import load_dotenv
from flask import Flask, jsonify, request, send_file

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


load_dotenv()

WHISPER_MODEL_NAME = os.environ.get("WHISPER_MODEL", "small")
WHISPER_LANGUAGE = os.environ.get("WHISPER_LANGUAGE", "en")
TTS_VOICE = os.environ.get("TTS_VOICE", "Samantha")
LISTEN_HOST = os.environ.get("VOICE_HOST", "0.0.0.0")
LISTEN_PORT = int(os.environ.get("VOICE_PORT", "5050"))


def _parse_max_hz(env_key: str, default: float) -> float | None:
    raw = os.environ.get(env_key)
    if raw is None or str(raw).strip() == "":
        value = default
    else:
        value = float(raw)
    if value <= 0:
        return None
    return value


_robot_state = {
    "robot": None,
    "motion": None,
    "follow": None,
    "basic_goal": None,
    "map_runtime": None,
    "ros2_client": None,
    "orchestrator": None,
    "world_tick": None,
    "lock": threading.Lock(),
}


def _connect_robot():
    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    motion_port = int(os.environ.get("ROS2_BRIDGE_PORT", "9091"))
    rgb_hz = _parse_max_hz("RGB_MAX_HZ", 2.0)
    depth_hz = _parse_max_hz("DEPTH_MAX_HZ", 2.0)

    try:
        print(f"[voice] connecting to ROS 2 bridge ws://{host}:{motion_port} ...", file=sys.stderr)
        _robot_state["ros2_client"] = connect_ros2_rosbridge(host, motion_port)
    except Exception as exc:
        print(f"[voice] ROS 2 bridge unavailable: {exc}", file=sys.stderr)

    ros2 = _robot_state["ros2_client"]
    print(f"[voice] connecting to ROS 2 perception bridge ws://{host}:{motion_port} ...", file=sys.stderr)
    _robot_state["robot"] = Lite3Robot(
        host=host,
        port=motion_port,
        max_rgb_hz=rgb_hz,
        max_depth_hz=depth_hz,
        ros_client=ros2,
    )

    if ros2 is not None:
        try:
            map_frame = os.environ.get("ROS2_MAP_FRAME", "map")
            goal_topic = (os.environ.get("NAV2_GOAL_POSE_TOPIC") or "").strip() or None
            goal_msg_type = os.environ.get("NAV2_GOAL_MESSAGE_TYPE", "PoseStamped")
            nav_goal_client = (
                Nav2NavigateClient(
                    ros2,
                    goal_pose_topic=goal_topic,
                    goal_message_type=goal_msg_type,
                    goal_frame_id=map_frame,
                )
                if goal_topic
                else None
            )
            _robot_state["map_runtime"] = MapRuntime(
                ros_client=ros2,
                base_frame=os.environ.get("ROS2_BASE_FRAME", "rslidar"),
                map_frame=map_frame,
                odom_frame=os.environ.get("ROS2_ODOM_FRAME", "odom"),
                nav2_navigate_client=nav_goal_client,
            )
            from perception.external_pose import set_map_runtime
            set_map_runtime(_robot_state["map_runtime"])
            print("[voice] map runtime connected", file=sys.stderr)
            if goal_topic:
                print(f"[voice] Nav2 via {goal_msg_type} topic {goal_topic!r}", file=sys.stderr)
            elif not os.environ.get("NAV_SUPPRESS_TOPIC_HINT"):
                print(
                    "[voice] Nav2 hint: add NAV2_GOAL_POSE_TOPIC=/goal_pose to .env if "
                    "go_to_map_pose does not move the robot (Foxy rosbridge has no actions).",
                    file=sys.stderr,
                )
        except Exception as exc:
            print(f"[voice] map runtime failed: {exc}", file=sys.stderr)
        for name, cls in (("motion", Lite3Motion), ("follow", Lite3Follow), ("basic_goal", Lite3BasicGoal)):
            try:
                _robot_state[name] = cls(ros_client=ros2)
                print(f"[voice] {name} adapter connected", file=sys.stderr)
            except Exception as exc:
                print(f"[voice] {name} adapter failed: {exc}", file=sys.stderr)

    try:
        memory = MemoryStore()
        safety = SafetySupervisor(memory)
        vlm = make_vlm_client()
        ctx = ToolContext(
            memory=memory,
            robot=_robot_state["robot"],
            motion=_robot_state["motion"],
            follow=_robot_state["follow"],
            basic_goal=_robot_state["basic_goal"],
            vlm=vlm,
            safety=safety,
            map_runtime=_robot_state["map_runtime"],
            waypoints=WaypointStore(),
        )
        registry = build_registry()
        planner = PlannerAgent(registry, ctx, client=None)
        _robot_state["orchestrator"] = Orchestrator(planner, memory)
        _robot_state["safety"] = safety
        print("[voice] bedrock agent ready [native converse + current tools]", file=sys.stderr)

        follow = _robot_state.get("follow")
        if follow is not None and os.environ.get("DISABLE_WORLD_TICK") != "1":
            try:
                from perception.world_tick import WorldTickDriver
                period = float(os.environ.get("WORLD_TICK_PERIOD_S", "1.0"))
                world_tick = WorldTickDriver(_robot_state["robot"], follow, memory, period_s=period)
                world_tick.start()
                _robot_state["world_tick"] = world_tick
            except Exception as e:
                print(f"[voice] world tick failed to start: {e}", file=sys.stderr)
    except Exception as exc:
        print(f"[voice] bedrock agent init failed: {exc}", file=sys.stderr)
        _robot_state["orchestrator"] = None
        _robot_state["safety"] = None


def _connect_dry_run() -> None:
    """Robot-less init: same orchestrator/planner/tools, but FakeRegistry
    fabricates tool results via Bedrock instead of dispatching to real handlers.
    Mirrors scripts/dry_run_planner.py's setup."""
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "scripts"))
    from dry_run_planner import FakeRegistry  # noqa: E402

    try:
        memory = MemoryStore()
        ctx = ToolContext(
            memory=memory,
            robot=None,
            motion=None,
            follow=None,
            basic_goal=None,
            vlm=None,
            safety=None,
        )
        real_reg = build_registry()
        fake_reg = FakeRegistry(real_reg.names(), scenario_context="voice session")
        planner = PlannerAgent(fake_reg, ctx, client=None)
        _robot_state["orchestrator"] = Orchestrator(planner, memory)
        _robot_state["safety"] = None
        _robot_state["fake_reg"] = fake_reg
        print("[voice] dry-run agent ready [FakeRegistry — no robot]", file=sys.stderr)
    except Exception as exc:
        print(f"[voice] dry-run agent init failed: {exc}", file=sys.stderr)
        _robot_state["orchestrator"] = None


def _shutdown_robot() -> None:
    """Mirrors cli.py's finally-block cleanup. Idempotent."""
    if _robot_state.get("_shutdown_done"):
        return
    _robot_state["_shutdown_done"] = True
    print("[voice] shutting down robot connections ...", file=sys.stderr)
    world_tick = _robot_state.get("world_tick")
    follow = _robot_state.get("follow")
    motion = _robot_state.get("motion")
    basic_goal = _robot_state.get("basic_goal")
    map_runtime = _robot_state.get("map_runtime")
    ros2 = _robot_state.get("ros2_client")
    robot = _robot_state.get("robot")
    if world_tick is not None:
        try:
            world_tick.stop()
        except Exception:
            pass
    for adapter in (follow, motion):
        if adapter is None:
            continue
        try:
            adapter.stop()
        except Exception:
            pass
        try:
            adapter.close()
        except Exception:
            pass
    if basic_goal is not None:
        try:
            basic_goal.close()
        except Exception:
            pass
    if map_runtime is not None:
        try:
            map_runtime.close()
        except Exception:
            pass
    if ros2 is not None:
        try:
            ros2.terminate()
        except Exception:
            pass
    if robot is not None:
        try:
            robot.close()
        except Exception:
            pass


_whisper = {"model": None, "lock": threading.Lock()}


def preload_whisper() -> None:
    import whisper as _w

    print(f"[voice] preloading whisper model={WHISPER_MODEL_NAME} ...", file=sys.stderr)
    t0 = time.time()
    with _whisper["lock"]:
        _whisper["model"] = _w.load_model(WHISPER_MODEL_NAME)
    print(f"[voice] whisper ready in {time.time()-t0:.1f}s", file=sys.stderr)


def whisper_transcribe(wav_path: str) -> str:
    with _whisper["lock"]:
        if _whisper["model"] is None:
            import whisper as _w

            _whisper["model"] = _w.load_model(WHISPER_MODEL_NAME)
        model = _whisper["model"]
    result = model.transcribe(wav_path, fp16=False, language=WHISPER_LANGUAGE)
    return (result.get("text") or "").strip()


from tts_engine import tts_to_wav, concat_wavs as _concat_wavs  # noqa: E402

_tts_pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix="tts")

# Stage-demo mode: bypass whisper + LLM, return canned scenes from
# demo_script.SCENES_CONVO. Toggled via `voice_server.py --demo`.
_DEMO_MODE = False
_demo_idx = 0
_demo_lock = threading.Lock()


def _next_demo_scene() -> tuple[str, str]:
    """Return (user_transcript, agent_reply) for the next scene; wrap to 0
    once we run off the end so a demo run can loop without a restart."""
    from demo_script import SCENES_CONVO
    global _demo_idx
    with _demo_lock:
        idx = _demo_idx % len(SCENES_CONVO)
        _demo_idx += 1
    return SCENES_CONVO[idx]


_tts_cache: dict[str, bytes] = {}
_TTS_CACHE_MAX = 16


def _tts_cache_put(tts_id: str, data: bytes) -> None:
    _tts_cache[tts_id] = data
    while len(_tts_cache) > _TTS_CACHE_MAX:
        oldest = next(iter(_tts_cache))
        _tts_cache.pop(oldest, None)


app = Flask(__name__)


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Lite3 guide-dog</title>
<style>
  :root {
    color-scheme: dark;
    --bg: #0b0d12;
    --panel: #151925;
    --panel-2: #1c2132;
    --accent: #4f8cff;
    --accent-2: #6ea8ff;
    --danger: #ff4d5e;
    --danger-2: #ff6b7a;
    --good: #3ecf8e;
    --text: #e8ecf5;
    --muted: #8892a6;
  }
  * { box-sizing: border-box; }
  html, body { height: 100%; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, "SF Pro Text", system-ui, sans-serif;
    background: radial-gradient(1200px 600px at 20% -10%, #1a2240 0%, var(--bg) 60%);
    color: var(--text);
    margin: 0;
    padding: 20px 16px 28px;
    -webkit-font-smoothing: antialiased;
  }
  .wrap { max-width: 520px; margin: 0 auto; }
  header {
    display: flex; align-items: center; justify-content: space-between;
    margin: 4px 2px 18px;
  }
  .brand {
    display: flex; align-items: center; gap: 10px;
    font-size: 15px; font-weight: 600; letter-spacing: .02em;
  }
  .brand .dot {
    width: 10px; height: 10px; border-radius: 50%;
    background: var(--good); box-shadow: 0 0 0 4px rgba(62,207,142,.18);
  }
  .brand .dot.busy { background: #ffc857; box-shadow: 0 0 0 4px rgba(255,200,87,.18); }
  .brand .dot.err  { background: var(--danger); box-shadow: 0 0 0 4px rgba(255,77,94,.18); }
  .sub { color: var(--muted); font-size: 12px; font-weight: 500; }

  #status {
    font-size: 14px;
    padding: 10px 14px;
    border-radius: 12px;
    margin-bottom: 14px;
    background: var(--panel);
    border: 1px solid #222839;
    transition: background .2s, color .2s;
  }
  #status[data-kind="busy"]  { background: rgba(255,200,87,.08);  color: #ffdd8a; border-color: rgba(255,200,87,.25); }
  #status[data-kind="rec"]   { background: rgba(255,77,94,.10);   color: #ffb0b8; border-color: rgba(255,77,94,.30); }
  #status[data-kind="error"] { background: rgba(255,77,94,.12);   color: #ff98a2; border-color: rgba(255,77,94,.35); }
  #status[data-kind="ok"]    { background: rgba(62,207,142,.10);  color: #8ee9c0; border-color: rgba(62,207,142,.30); }

  button {
    font-family: inherit;
    border: none;
    border-radius: 18px;
    color: white;
    font-weight: 600;
    -webkit-tap-highlight-color: transparent;
    touch-action: manipulation;
    cursor: pointer;
    transition: transform .05s, background .15s, box-shadow .15s;
  }
  button:active { transform: scale(.985); }

  #enableAudio, #ptt {
    display: block;
    width: 100%;
    height: 200px;
    font-size: 26px;
    letter-spacing: .01em;
    background: linear-gradient(180deg, var(--accent) 0%, #3a6fe0 100%);
    box-shadow: 0 10px 30px -12px rgba(79,140,255,.55), inset 0 1px 0 rgba(255,255,255,.12);
  }
  #enableAudio { background: linear-gradient(180deg, #3ecf8e 0%, #2fae77 100%);
                 box-shadow: 0 10px 30px -12px rgba(62,207,142,.5), inset 0 1px 0 rgba(255,255,255,.12); }
  #ptt.recording {
    background: linear-gradient(180deg, var(--danger) 0%, #cc3445 100%);
    box-shadow: 0 10px 30px -10px rgba(255,77,94,.6), inset 0 1px 0 rgba(255,255,255,.12);
    animation: pulse 1s ease-in-out infinite;
  }
  @keyframes pulse {
    0%, 100% { box-shadow: 0 10px 30px -10px rgba(255,77,94,.6), 0 0 0 0 rgba(255,77,94,.35); }
    50%      { box-shadow: 0 10px 30px -10px rgba(255,77,94,.6), 0 0 0 14px rgba(255,77,94,0); }
  }

  #stopBtn {
    width: 100%;
    height: 76px;
    margin-top: 12px;
    font-size: 20px;
    letter-spacing: .22em;
    background: linear-gradient(180deg, var(--danger) 0%, #c23140 100%);
    box-shadow: 0 8px 24px -10px rgba(255,77,94,.55), inset 0 1px 0 rgba(255,255,255,.12);
  }
  #stopBtn:active { background: linear-gradient(180deg, var(--danger-2) 0%, #cc3445 100%); }

  .log { margin-top: 22px; display: flex; flex-direction: column; gap: 10px; }
  .turn {
    padding: 12px 14px;
    border-radius: 14px;
    line-height: 1.4;
    font-size: 15px;
    border: 1px solid transparent;
    animation: slideIn .18s ease-out;
  }
  @keyframes slideIn { from { opacity: 0; transform: translateY(-4px); } to { opacity: 1; transform: none; } }
  .me  { background: linear-gradient(180deg, #1f3a6e 0%, #1a2f5a 100%); border-color: #2a4985; align-self: flex-end; max-width: 90%; }
  .bot { background: var(--panel-2); border-color: #242a3d; align-self: flex-start; max-width: 95%; }
  .meta { font-size: 11px; color: var(--muted); margin-top: 6px; letter-spacing: .04em; text-transform: uppercase; }

  .hint {
    text-align: center;
    color: var(--muted);
    font-size: 12px;
    margin-top: 10px;
    letter-spacing: .02em;
  }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <div class="brand"><span class="dot" id="statusDot"></span><span>guide-dog voice</span></div>
    <div class="sub">hold to speak</div>
  </header>
  <div id="status">idle</div>
  <button id="ptt">hold to talk</button>
  <button id="stopBtn">STOP</button>
  <div class="hint" id="hint">hold the blue button while speaking</div>
  <div class="log" id="log"></div>
</div>
<script>
const ptt = document.getElementById('ptt');
const stopBtn = document.getElementById('stopBtn');
const hint = document.getElementById('hint');
const log = document.getElementById('log');
const statusEl = document.getElementById('status');
const statusDot = document.getElementById('statusDot');
let mediaRecorder = null;
let chunks = [];
let replyAudio = null;

const SILENT_WAV =
  'data:audio/wav;base64,UklGRiQAAABXQVZFZm10IBAAAAABAAEARKwAAIhYAQACABAAZGF0YQAAAAA=';

// On the first PTT press we prime <audio> with a silent clip so iOS/Safari
// will allow programmatic play() later when the reply audio arrives.
let audioPrimed = false;
function primeAudio() {
  if (audioPrimed) return;
  audioPrimed = true;
  try {
    replyAudio = new Audio();
    replyAudio.preload = 'auto';
    replyAudio.playsInline = true;
    replyAudio.setAttribute('playsinline', '');
    replyAudio.onended = () => setStatus('idle', 'idle');
    replyAudio.src = SILENT_WAV;
    const p = replyAudio.play();
    if (p && p.catch) p.catch(() => {});
  } catch (e) {}
}
setStatus('ready', 'ok');

stopBtn.addEventListener('click', async () => {
  // Immediately pause speech playback...
  if (replyAudio) {
    try { replyAudio.pause(); replyAudio.currentTime = 0; } catch (e) {}
  }
  // ...and hard-stop the robot without waiting on the LLM path.
  setStatus('stopping robot...', 'busy');
  try {
    const res = await fetch('/stop', { method: 'POST' });
    const data = await res.json().catch(() => ({}));
    if (data.ok) setStatus('stopped', 'ok');
    else setStatus('stop reported errors: ' + (data.errors || []).join('; '), 'error');
  } catch (err) {
    setStatus('stop request failed: ' + (err.message || err), 'error');
  }
});

function setStatus(text, kind) {
  statusEl.textContent = text;
  statusEl.dataset.kind = kind || 'idle';
  statusDot.className = 'dot' + (kind === 'busy' || kind === 'rec' ? ' busy' : kind === 'error' ? ' err' : '');
}

function add(role, text, meta) {
  const div = document.createElement('div');
  div.className = 'turn ' + (role === 'me' ? 'me' : 'bot');
  div.textContent = text;
  if (meta) {
    const m = document.createElement('div');
    m.className = 'meta';
    m.textContent = meta;
    div.appendChild(m);
  }
  log.prepend(div);
  return div;
}

function pickRecordMime() {
  const candidates = [
    'audio/webm;codecs=opus',
    'audio/webm',
    'audio/mp4;codecs=mp4a.40.2',
    'audio/mp4',
  ];
  for (const t of candidates) {
    if (window.MediaRecorder && MediaRecorder.isTypeSupported(t)) return t;
  }
  return '';
}

async function startRecording() {
  // Demo mode: skip mic entirely. The server ignores audio content and returns
  // the next scripted scene. We POST a tiny silent blob to satisfy the
  // multipart upload + the > 2KB size check.
  if (window.DEMO_MODE) {
    primeAudio();
    setStatus('recording...', 'rec');
    ptt.classList.add('recording');
    ptt.textContent = 'release to send';
    return;
  }
  if (!navigator.mediaDevices) {
    setStatus('browser has no audio recording', 'error');
    return;
  }
  // Re-prime the audio element inside this user gesture so iOS Safari keeps
  // letting us auto-play the eventual TTS reply.
  if (replyAudio) {
    try {
      replyAudio.src = SILENT_WAV;
      const p = replyAudio.play();
      if (p && p.catch) p.catch(() => {});
    } catch (e) {}
  }
  try {
    setStatus('asking for mic permission...', 'busy');
    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    const mimeType = pickRecordMime();
    mediaRecorder = mimeType
      ? new MediaRecorder(stream, { mimeType })
      : new MediaRecorder(stream);
    chunks = [];
    mediaRecorder.ondataavailable = (e) => chunks.push(e.data);
    mediaRecorder.onstop = async () => {
      const type = mediaRecorder.mimeType || 'audio/webm';
      const ext = type.includes('mp4') ? 'm4a' : type.includes('ogg') ? 'ogg' : 'webm';
      const blob = new Blob(chunks, { type });
      stream.getTracks().forEach(t => t.stop());
      await sendAudio(blob, ext);
    };
    mediaRecorder.start();
    ptt.classList.add('recording');
    ptt.textContent = 'release to send';
    setStatus('listening...', 'rec');
  } catch (err) {
    setStatus('mic blocked: ' + (err.message || err.name), 'error');
  }
}

function stopRecording() {
  if (window.DEMO_MODE) {
    ptt.classList.remove('recording');
    ptt.textContent = 'hold to talk';
    // 4KB of zeros, labeled as audio/webm — satisfies size & MIME checks.
    const dummy = new Blob([new Uint8Array(4096)], { type: 'audio/webm' });
    sendAudio(dummy, 'webm');
    return;
  }
  if (mediaRecorder && mediaRecorder.state !== 'inactive') mediaRecorder.stop();
  ptt.classList.remove('recording');
  ptt.textContent = 'hold to talk';
}

async function sendAudio(blob, ext) {
  setStatus('transcribing...', 'busy');
  const fd = new FormData();
  fd.append('audio', blob, 'clip.' + (ext || 'webm'));
  const res = await fetch('/talk', { method: 'POST', body: fd });
  if (!res.ok) {
    setStatus('error ' + res.status, 'error');
    add('bot', 'error ' + res.status + ': ' + (await res.text()));
    return;
  }
  const data = await res.json();
  if (data.transcript) add('me', data.transcript);
  const botDiv = add('bot', data.reply || '(no reply)');
  if (data.tts_id) {
    const url = '/tts?id=' + encodeURIComponent(data.tts_id);
    setStatus('speaking...', 'busy');
    if (replyAudio) {
      replyAudio.src = url;
      replyAudio.play().catch((err) => {
        console.warn('autoplay blocked', err);
        setStatus('autoplay blocked, tap to play', 'error');
        addPlayButton(botDiv, url, false);
      });
    } else {
      addPlayButton(botDiv, url, false);
      setStatus('idle', 'idle');
    }
  } else {
    setStatus('idle', 'idle');
  }
}

function addPlayButton(parentDiv, url, tryAutoplay) {
  const btn = document.createElement('button');
  btn.textContent = '▶ tap to hear reply';
  btn.style.cssText = 'margin-top:10px;padding:14px 20px;border-radius:14px;background:var(--accent);color:#fff;font-size:18px;font-weight:600;width:100%;';
  let played = false;
  btn.onclick = () => {
    if (played) return;
    played = true;
    btn.textContent = '▶ playing...';
    const a = new Audio(url);
    a.playsInline = true;
    a.setAttribute('playsinline', '');
    a.onended = () => { btn.textContent = '✓ played'; setStatus('idle', 'idle'); };
    setStatus('speaking...', 'busy');
    a.play().catch((err) => {
      btn.textContent = '▶ tap to hear reply';
      played = false;
      setStatus('tap the button to hear: ' + (err.message || err.name), 'error');
    });
  };
  parentDiv.appendChild(btn);
  if (tryAutoplay) {
    const a = new Audio(url);
    a.playsInline = true;
    a.setAttribute('playsinline', '');
    a.onended = () => { btn.textContent = '✓ played'; setStatus('idle', 'idle'); };
    setStatus('speaking...', 'busy');
    a.play().then(() => {
      played = true;
      btn.textContent = '▶ playing...';
    }).catch(() => {});
  }
}

ptt.addEventListener('mousedown', () => { primeAudio(); startRecording(); });
ptt.addEventListener('mouseup', stopRecording);
ptt.addEventListener('mouseleave', () => { if (mediaRecorder?.state === 'recording') stopRecording(); });
ptt.addEventListener('touchstart', (e) => { e.preventDefault(); primeAudio(); startRecording(); }, { passive: false });
ptt.addEventListener('touchend', (e) => { e.preventDefault(); stopRecording(); }, { passive: false });
</script>
</body>
</html>
"""


@app.route("/")
def index():
    # Inject a tiny <script> right after <head> so the JS code below can
    # branch on DEMO_MODE without us having to template the whole page.
    flag_tag = f"<script>window.DEMO_MODE = {str(bool(_DEMO_MODE)).lower()};</script>"
    html = INDEX_HTML.replace("<head>", f"<head>\n{flag_tag}", 1)
    resp = app.make_response(html)
    resp.headers["Cache-Control"] = "no-store"
    return resp


_MIME_TO_SUFFIX = {
    "audio/webm": ".webm",
    "audio/ogg": ".ogg",
    "audio/mp4": ".m4a",
    "audio/mpeg": ".mp3",
    "audio/wav": ".wav",
    "audio/x-wav": ".wav",
    "audio/aac": ".aac",
}


def _guess_suffix(mime: str | None, filename: str | None) -> str:
    if mime:
        base = mime.split(";")[0].strip().lower()
        if base in _MIME_TO_SUFFIX:
            return _MIME_TO_SUFFIX[base]
    if filename:
        ext = os.path.splitext(filename)[1].lower()
        if ext:
            return ext
    return ".webm"


@app.route("/talk", methods=["POST"])
def talk():
    t_req = time.time()
    audio = request.files.get("audio")
    if audio is None:
        return ("missing audio", 400)

    # --- demo mode: ignore the audio entirely, advance the canned scene list.
    if _DEMO_MODE:
        transcript, reply = _next_demo_scene()
        tts_id = uuid.uuid4().hex
        try:
            wav = tts_to_wav(reply)
            _tts_cache_put(tts_id, wav)
        except Exception as exc:
            print(f"[voice/demo] tts failed: {exc}", file=sys.stderr)
            tts_id = None
        print(
            f"[voice/demo] scene -> transcript={transcript[:60]!r} "
            f"reply={reply[:60]!r} ({time.time()-t_req:.2f}s)",
            file=sys.stderr,
        )
        return jsonify({"transcript": transcript, "reply": reply, "tts_id": tts_id})

    suffix = _guess_suffix(audio.mimetype, audio.filename)
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as file_obj:
        audio.save(file_obj.name)
        wav_path = file_obj.name
    upload_kb = os.path.getsize(wav_path) / 1024
    print(
        f"[voice] upload mime={audio.mimetype!r} filename={audio.filename!r} "
        f"suffix={suffix} size={upload_kb:.1f}KB",
        file=sys.stderr,
    )
    if upload_kb < 2:
        try:
            os.unlink(wav_path)
        except Exception:
            pass
        return jsonify({
            "transcript": "",
            "reply": "(too short — hold the button for at least half a second)",
        }), 200
    t_recv = time.time()
    try:
        transcript = whisper_transcribe(wav_path)
    except Exception as exc:
        try:
            os.unlink(wav_path)
        except Exception:
            pass
        return jsonify({"transcript": "", "reply": f"(STT failed: {exc})"}), 500
    else:
        try:
            os.unlink(wav_path)
        except Exception:
            pass
    t_stt = time.time()

    if not transcript:
        return jsonify({"transcript": "", "reply": "(no speech detected)"}), 200

    reply_parts: list[str] = []
    tts_futures: list[Future] = []
    with _robot_state["lock"]:
        try:
            orch = _robot_state.get("orchestrator")
            if orch is None:
                raise RuntimeError("bedrock agent not initialized")
            fake_reg = _robot_state.get("fake_reg")
            if fake_reg is not None:
                fake_reg.scenario_context = transcript
            for chunk in orch.run_stream(transcript):
                reply_parts.append(chunk)
                tts_futures.append(_tts_pool.submit(tts_to_wav, chunk))
        except Exception as exc:
            err = f"(agent error: {type(exc).__name__}: {exc})"
            reply_parts.append(err)
            tts_futures.append(_tts_pool.submit(tts_to_wav, err))
    reply = " ".join(part.strip() for part in reply_parts if part.strip())
    t_agent = time.time()

    tts_id = uuid.uuid4().hex
    try:
        wav_blobs = [future.result() for future in tts_futures]
        _tts_cache_put(tts_id, _concat_wavs(wav_blobs))
    except Exception as exc:
        print(f"[voice] tts failed: {exc}", file=sys.stderr)
        tts_id = None
    t_tts = time.time()

    print(
        f"[voice] {upload_kb:.0f}KB recv {t_recv-t_req:.2f}s "
        f"stt {t_stt-t_recv:.2f}s agent {t_agent-t_stt:.2f}s "
        f"tts {t_tts-t_agent:.2f}s total {t_tts-t_req:.2f}s",
        file=sys.stderr,
    )

    return jsonify({"transcript": transcript, "reply": reply, "tts_id": tts_id})


@app.route("/stop", methods=["POST"])
def stop():
    """Hard-stop the robot without going through the LLM.

    Wired to the big red button in the UI so a blind user gets an immediate
    halt — no Whisper, no Bedrock, no tool dispatch in the loop.
    """
    errors: list[str] = []
    safety = _robot_state.get("safety")
    if safety is not None:
        try:
            safety.trigger_emergency_stop()
        except Exception as exc:
            errors.append(f"safety: {exc}")
    for key in ("follow", "motion", "robot", "basic_goal"):
        obj = _robot_state.get(key)
        if obj is None:
            continue
        for method in ("stop", "cancel"):
            fn = getattr(obj, method, None)
            if fn is None:
                continue
            try:
                fn()
            except Exception as exc:
                errors.append(f"{key}.{method}: {exc}")
    return jsonify({"ok": not errors, "errors": errors})


@app.route("/resume", methods=["POST"])
def resume():
    """Clear the latched emergency-stop flag."""
    safety = _robot_state.get("safety")
    if safety is None:
        return jsonify({"ok": False, "reason": "safety not initialized"}), 503
    try:
        safety.reset_emergency_stop()
        return jsonify({"ok": True})
    except Exception as exc:
        return jsonify({"ok": False, "reason": str(exc)}), 500


@app.route("/demo/reset", methods=["POST", "GET"])
def demo_reset():
    """Rewind the canned scene index to 0 (demo mode only)."""
    global _demo_idx
    with _demo_lock:
        _demo_idx = 0
    return jsonify({"ok": True, "index": 0, "demo_mode": _DEMO_MODE})


@app.route("/demo/state", methods=["GET"])
def demo_state_route():
    """Inspect the current scene index (demo mode only)."""
    from demo_script import SCENES_CONVO
    with _demo_lock:
        idx = _demo_idx
    return jsonify({
        "demo_mode": _DEMO_MODE,
        "next_index": idx % len(SCENES_CONVO),
        "scenes_total": len(SCENES_CONVO),
    })


@app.route("/tts_test")
def tts_test():
    audio = tts_to_wav("hello, this is a test of the audio path.")
    return send_file(
        io.BytesIO(audio),
        mimetype="audio/wav",
        as_attachment=False,
        download_name="test.wav",
    )


@app.route("/tts")
def tts():
    tts_id = request.args.get("id", "")
    audio = _tts_cache.get(tts_id)
    if audio is None:
        return ("not found", 404)
    return send_file(io.BytesIO(audio), mimetype="audio/wav", as_attachment=False, download_name="reply.wav")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run without a robot: FakeRegistry fabricates tool results via Bedrock.",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Stage-demo mode: ignore mic audio + LLM, return canned scenes from "
             "demo_script.SCENES_CONVO. Skips whisper preload and robot bring-up.",
    )
    args = parser.parse_args()

    global _DEMO_MODE
    _DEMO_MODE = args.demo

    if args.demo:
        print("[voice] DEMO MODE — whisper + LLM + robot bypassed", file=sys.stderr)
    else:
        preload_whisper()
        if args.dry_run:
            _connect_dry_run()
        else:
            _connect_robot()
        atexit.register(_shutdown_robot)

    def _handle_signal(signum, _frame):
        print(f"[voice] received signal {signum}, shutting down", file=sys.stderr)
        if not args.demo:
            _shutdown_robot()
        sys.exit(0)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _handle_signal)
        except (ValueError, OSError):
            # Not on main thread or unsupported on this platform.
            pass

    print(
        f"[voice] serving on http://{LISTEN_HOST}:{LISTEN_PORT}/  "
        f"(open from your phone using your laptop's WiFi IP).",
        file=sys.stderr,
    )
    try:
        app.run(host=LISTEN_HOST, port=LISTEN_PORT, threaded=True)
    finally:
        if not args.demo:
            _shutdown_robot()


if __name__ == "__main__":
    main()

