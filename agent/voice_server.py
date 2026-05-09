"""Voice UI server.

Serves a phone web page with a push-to-talk button. Phone uploads audio,
laptop runs Whisper STT, feeds the text into the agent loop, replies with
text + TTS audio (macOS `say`).

Run:
    cd /Users/maria/Desktop/RoboHack/agent
    source .venv/bin/activate
    python voice_server.py

Then on your phone (same WiFi as laptop), open:
    http://<laptop-lan-ip>:5050/

Find the laptop IP with `ipconfig getifaddr en0` (Mac WiFi).

The agent's robot connections (camera bridge, motion bridge, follow tracker)
are opened once at startup and reused across requests.
"""

from __future__ import annotations

import io
import os
import subprocess
import sys
import tempfile
import threading
import time
import uuid

from dotenv import load_dotenv
from flask import Flask, jsonify, request, send_file

# Allow running as `python voice_server.py` from agent/.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from cli import run_once, SYSTEM_PROMPT  # noqa: E402  (legacy fallback)
from robot import (  # noqa: E402
    Lite3BasicGoal,
    Lite3Follow,
    Lite3Motion,
    Lite3Robot,
    connect_ros2_rosbridge,
)

# Same opt-out switch as cli.py: LEGACY_LOOP=1 forces the old run_once path.
_USE_NEW_PIPELINE = os.environ.get("LEGACY_LOOP", "0") != "1"
if _USE_NEW_PIPELINE:
    from agents_app.orchestrator import Orchestrator  # noqa: E402
    from agents_app.sdk_agents import (  # noqa: E402
        DialogueAgent,
        PlannerAgent,
        _make_agent_client,
        _use_openai_compat,
    )
    from memory.store import MemoryStore  # noqa: E402
    from safety.supervisor import SafetySupervisor  # noqa: E402
    from tools.base import ToolContext  # noqa: E402
    from tools.setup import build_registry  # noqa: E402
    from vlm_client import make_vlm_client  # noqa: E402


load_dotenv()

WHISPER_MODEL_NAME = os.environ.get("WHISPER_MODEL", "base")
TTS_VOICE = os.environ.get("TTS_VOICE", "Samantha")  # any macOS `say` voice
LISTEN_HOST = os.environ.get("VOICE_HOST", "0.0.0.0")
LISTEN_PORT = int(os.environ.get("VOICE_PORT", "5050"))


# -------------- robot connections (singleton) ---------------------------------

_robot_state = {
    "robot": None,
    "motion": None,
    "follow": None,
    "basic_goal": None,
    "ros2_client": None,
    "orchestrator": None,
    "lock": threading.Lock(),
}


def _connect_robot():
    """Mirror cli.py's connection + orchestrator setup so voice and CLI run the
    exact same agent (with safety supervisor, dialogue/planner split, etc.)."""
    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    cam_port = int(os.environ.get("ROS_BRIDGE_PORT", "9090"))
    motion_port = int(os.environ.get("ROS2_BRIDGE_PORT", "9091"))

    print(f"[voice] connecting to camera bridge ws://{host}:{cam_port} …", file=sys.stderr)
    _robot_state["robot"] = Lite3Robot(host=host, port=cam_port)

    try:
        print(f"[voice] connecting to ROS 2 bridge ws://{host}:{motion_port} …", file=sys.stderr)
        _robot_state["ros2_client"] = connect_ros2_rosbridge(host, motion_port)
    except Exception as e:
        print(f"[voice] ROS 2 bridge unavailable: {e}", file=sys.stderr)

    ros2 = _robot_state["ros2_client"]
    if ros2 is not None:
        for name, cls in (("motion", Lite3Motion), ("follow", Lite3Follow), ("basic_goal", Lite3BasicGoal)):
            try:
                _robot_state[name] = cls(ros_client=ros2)
                print(f"[voice] {name} adapter connected", file=sys.stderr)
            except Exception as e:
                print(f"[voice] {name} adapter failed: {e}", file=sys.stderr)

    if not _USE_NEW_PIPELINE:
        print("[voice] LEGACY_LOOP=1 — using old run_once agent", file=sys.stderr)
        return

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
        )
        registry = build_registry()
        oa_client = _make_agent_client() if _use_openai_compat() else None
        dialogue = DialogueAgent(oa_client)
        planner = PlannerAgent(registry, ctx, client=oa_client)
        _robot_state["orchestrator"] = Orchestrator(dialogue, planner, memory)
        backend = "openai-compat" if oa_client else "bedrock-boto3"
        print(f"[voice] new pipeline ready [{backend}]", file=sys.stderr)
    except Exception as e:
        print(f"[voice] new pipeline init failed: {e} — falling back to legacy run_once", file=sys.stderr)
        _robot_state["orchestrator"] = None


# -------------- whisper (lazy load) -------------------------------------------

_whisper = {"model": None, "lock": threading.Lock()}


def preload_whisper() -> None:
    """Load the Whisper model at server start so first request is fast."""
    import whisper as _w
    print(f"[voice] preloading whisper model={WHISPER_MODEL_NAME} …", file=sys.stderr)
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
    result = model.transcribe(wav_path, fp16=False)
    return (result.get("text") or "").strip()


# -------------- TTS (macOS `say`) ---------------------------------------------

def tts_to_wav(text: str) -> bytes:
    """Synthesize text via macOS `say` directly to WAV (LEI16, 22.05 kHz).

    `say` writes AIFF by default; specifying --data-format and a `.wav`
    suffix makes it write WAV — which every browser plays without fuss
    (AIFF/AIFC is unreliable in Chrome).
    """
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        out = f.name
    try:
        subprocess.run(
            [
                "say", "-v", TTS_VOICE,
                "--data-format=LEI16@22050",
                "--file-format=WAVE",
                "-o", out,
                text,
            ],
            check=True,
            capture_output=True,
            timeout=30,
        )
        with open(out, "rb") as f:
            return f.read()
    finally:
        try:
            os.unlink(out)
        except Exception:
            pass


# Cache TTS audio between /talk and /tts. Bytes-only, dict-ordered so we can
# trim the oldest entries cheaply.
_tts_cache: dict[str, bytes] = {}
_TTS_CACHE_MAX = 16


def _tts_cache_put(tts_id: str, data: bytes) -> None:
    _tts_cache[tts_id] = data
    while len(_tts_cache) > _TTS_CACHE_MAX:
        oldest = next(iter(_tts_cache))
        _tts_cache.pop(oldest, None)


# -------------- Flask app -----------------------------------------------------

app = Flask(__name__)


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Lite3 voice</title>
<style>
  :root { color-scheme: dark; }
  body { font-family: -apple-system, system-ui, sans-serif; background: #111; color: #eee; margin: 0; padding: 16px; }
  h1 { font-size: 18px; margin: 0 0 12px; opacity: .7; font-weight: 500; }
  #ptt, #enableAudio, #stopBtn {
    /* shared: full-width tap target */
    display: block; width: 100%; border: none; border-radius: 24px;
    background: #1f6feb; color: white; font-weight: 600;
    -webkit-tap-highlight-color: transparent; touch-action: manipulation;
  }
  #ptt, #enableAudio { height: 220px; font-size: 28px; }
  #stopBtn {
    height: 80px; font-size: 22px; margin-top: 12px;
    background: #d63838; letter-spacing: 0.15em;
  }
  #stopBtn:active { background: #ff4d4d; }
  #enableAudio { background: #2ea043; }
  #ptt.recording { background: #d63838; }
  #status { font-size: 14px; padding: 8px 12px; border-radius: 8px; margin-bottom: 10px; background: #222; }
  #status[data-kind="busy"]  { background: #3b3b1a; color: #ffe680; }
  #status[data-kind="rec"]   { background: #5a1a1a; color: #ffb3b3; }
  #status[data-kind="error"] { background: #5a1a1a; color: #ff8080; }
  .log { margin-top: 16px; }
  .turn { padding: 12px 14px; border-radius: 14px; margin-bottom: 10px; line-height: 1.35; }
  .me { background: #243559; }
  .bot { background: #2a2a2a; }
  .meta { font-size: 12px; opacity: .5; margin-top: 4px; }
</style>
</head>
<body>
  <h1>guide-dog voice</h1>
  <div id="status">idle</div>
  <button id="enableAudio">tap once to enable audio</button>
  <button id="ptt" hidden>hold to talk</button>
  <button id="stopBtn" hidden>stop audio</button>
  <div class="log" id="log"></div>
<script>
const ptt = document.getElementById('ptt');
const enableBtn = document.getElementById('enableAudio');
const log = document.getElementById('log');
const statusEl = document.getElementById('status');
let mediaRecorder = null;
let chunks = [];
// One persistent Audio element, unlocked by the user's first tap. iOS
// Safari requires this — it won't play anything created later otherwise.
let replyAudio = null;

const SILENT_WAV =
  'data:audio/wav;base64,UklGRiQAAABXQVZFZm10IBAAAAABAAEARKwAAIhYAQACABAAZGF0YQAAAAA=';

enableBtn.addEventListener('click', async () => {
  try {
    replyAudio = new Audio();
    replyAudio.preload = 'auto';
    replyAudio.onended = () => setStatus('idle', 'idle');
    replyAudio.src = SILENT_WAV;
    await replyAudio.play();
    enableBtn.hidden = true;
    ptt.hidden = false;
    document.getElementById('stopBtn').hidden = false;
    setStatus('audio enabled — ready', 'idle');
  } catch (err) {
    setStatus('could not enable audio: ' + (err.message || err.name), 'error');
  }
});

document.getElementById('stopBtn').addEventListener('click', () => {
  if (replyAudio) {
    replyAudio.pause();
    replyAudio.currentTime = 0;
  }
  setStatus('idle', 'idle');
});

function setStatus(text, kind) {
  statusEl.textContent = text;
  statusEl.dataset.kind = kind || 'idle';
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
  // iOS Safari only does audio/mp4. Chrome/Firefox do audio/webm + opus.
  // Try the most-compatible options in order; return the first the browser
  // says it supports. MediaRecorder will refuse to start with a bogus type.
  const candidates = [
    'audio/webm;codecs=opus',
    'audio/webm',
    'audio/mp4;codecs=mp4a.40.2',  // AAC-LC
    'audio/mp4',
  ];
  for (const t of candidates) {
    if (window.MediaRecorder && MediaRecorder.isTypeSupported(t)) return t;
  }
  return '';  // let the browser default
}

async function startRecording() {
  if (!navigator.mediaDevices) {
    setStatus('browser has no audio recording', 'error');
    return;
  }
  try {
    setStatus('asking for mic permission…', 'busy');
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
    setStatus('recording…', 'rec');
  } catch (err) {
    setStatus('mic blocked: ' + (err.message || err.name), 'error');
  }
}

function stopRecording() {
  if (mediaRecorder && mediaRecorder.state !== 'inactive') mediaRecorder.stop();
  ptt.classList.remove('recording');
  ptt.textContent = 'hold to talk';
}

async function sendAudio(blob, ext) {
  setStatus('transcribing…', 'busy');
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
  const botDiv = add('bot', data.reply || '(no reply)', data.tools_used ? 'tools: ' + data.tools_used.join(', ') : null);
  if (data.tts_id) {
    const url = '/tts?id=' + encodeURIComponent(data.tts_id);
    setStatus('speaking…', 'busy');
    if (replyAudio) {
      replyAudio.src = url;
      replyAudio.play().catch((err) => {
        console.warn('autoplay blocked', err);
        setStatus('autoplay blocked, tap to play', 'error');
        addPlayButton(botDiv, url);
      });
    } else {
      addPlayButton(botDiv, url);
      setStatus('idle', 'idle');
    }
  } else {
    setStatus('idle', 'idle');
  }
}

function addPlayButton(parentDiv, url) {
  const btn = document.createElement('button');
  btn.textContent = '▶ play reply';
  btn.style.cssText = 'margin-top:8px;padding:6px 14px;border-radius:8px;border:none;background:#1f6feb;color:#fff;font-size:14px;';
  btn.onclick = () => {
    const a = new Audio(url);
    a.onended = () => setStatus('idle', 'idle');
    setStatus('speaking…', 'busy');
    a.play();
  };
  parentDiv.appendChild(btn);
}

ptt.addEventListener('mousedown', startRecording);
ptt.addEventListener('mouseup', stopRecording);
ptt.addEventListener('mouseleave', () => { if (mediaRecorder?.state === 'recording') stopRecording(); });
ptt.addEventListener('touchstart', (e) => { e.preventDefault(); startRecording(); }, { passive: false });
ptt.addEventListener('touchend',   (e) => { e.preventDefault(); stopRecording(); }, { passive: false });
</script>
</body>
</html>
"""


@app.route("/")
def index():
    resp = app.make_response(INDEX_HTML)
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
    """Pick a file extension Whisper / ffmpeg will recognize.

    Browser tells us the real container in the MIME type. iOS Safari sends
    audio/mp4; desktop Chrome and Android send audio/webm. We honor that;
    the filename suffix is unreliable.
    """
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

    suffix = _guess_suffix(audio.mimetype, audio.filename)
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as f:
        audio.save(f.name)
        wav_path = f.name
    upload_kb = os.path.getsize(wav_path) / 1024
    print(
        f"[voice] upload mime={audio.mimetype!r} name={audio.filename!r} "
        f"saved-as {suffix} ({upload_kb:.0f}KB)",
        file=sys.stderr,
    )
    t_recv = time.time()
    try:
        transcript = whisper_transcribe(wav_path)
    except Exception as e:
        try:
            os.unlink(wav_path)
        except Exception:
            pass
        return jsonify({"transcript": "", "reply": f"(STT failed: {e})"}), 500
    else:
        try:
            os.unlink(wav_path)
        except Exception:
            pass
    t_stt = time.time()

    if not transcript:
        print(
            f"[voice] {upload_kb:.0f}KB recv {t_recv-t_req:.2f}s "
            f"stt {t_stt-t_recv:.2f}s — no speech",
            file=sys.stderr,
        )
        return jsonify({"transcript": "", "reply": "(no speech detected)"}), 200

    print(f"[voice] heard: {transcript!r}", file=sys.stderr)

    with _robot_state["lock"]:
        try:
            orch = _robot_state.get("orchestrator")
            if orch is not None:
                reply = orch.run(transcript)
            else:
                reply = run_once(
                    transcript,
                    _robot_state["robot"],
                    _robot_state["motion"],
                    _robot_state["follow"],
                )
        except Exception as e:
            reply = f"(agent error: {type(e).__name__}: {e})"
    t_agent = time.time()

    print(f"[voice] reply: {reply[:120]!r}", file=sys.stderr)

    tts_id = uuid.uuid4().hex
    try:
        _tts_cache_put(tts_id, tts_to_wav(reply or ""))
    except Exception as e:
        print(f"[voice] tts failed: {e}", file=sys.stderr)
        tts_id = None
    t_tts = time.time()

    print(
        f"[voice] {upload_kb:.0f}KB recv {t_recv-t_req:.2f}s "
        f"stt {t_stt-t_recv:.2f}s agent {t_agent-t_stt:.2f}s "
        f"tts {t_tts-t_agent:.2f}s total {t_tts-t_req:.2f}s",
        file=sys.stderr,
    )

    return jsonify(
        {"transcript": transcript, "reply": reply, "tts_id": tts_id}
    )


@app.route("/tts_test")
def tts_test():
    """Browser audio sanity check. Open /tts_test directly to confirm the
    browser plays our WAVs at all — independent of the talk → agent flow."""
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
    # Don't pop — Safari often re-fetches the same URL when seeking, retrying,
    # or for range requests. We let the entry expire when a new turn replaces it.
    audio = _tts_cache.get(tts_id)
    if audio is None:
        return ("not found", 404)
    return send_file(io.BytesIO(audio), mimetype="audio/wav", as_attachment=False, download_name="reply.wav")


def main() -> None:
    preload_whisper()
    _connect_robot()
    print(
        f"[voice] serving on http://{LISTEN_HOST}:{LISTEN_PORT}/  "
        f"(open from your phone using your laptop's WiFi IP).",
        file=sys.stderr,
    )
    # threaded=True so /talk can take its sweet time and not block /tts.
    app.run(host=LISTEN_HOST, port=LISTEN_PORT, threaded=True)


if __name__ == "__main__":
    main()
