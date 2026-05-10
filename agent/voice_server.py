"""Voice UI server using the Bedrock-native agent."""

from __future__ import annotations

import io
import os
import subprocess
import sys
import tempfile
import threading
import time
import uuid
import wave
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

_robot_state = {
    "robot": None,
    "motion": None,
    "follow": None,
    "basic_goal": None,
    "map_runtime": None,
    "ros2_client": None,
    "orchestrator": None,
    "lock": threading.Lock(),
}


def _connect_robot():
    host = os.environ.get("ROS_BRIDGE_HOST", "192.168.1.103")
    motion_port = int(os.environ.get("ROS2_BRIDGE_PORT", "9091"))

    try:
        print(f"[voice] connecting to ROS 2 bridge ws://{host}:{motion_port} ...", file=sys.stderr)
        _robot_state["ros2_client"] = connect_ros2_rosbridge(host, motion_port)
    except Exception as exc:
        print(f"[voice] ROS 2 bridge unavailable: {exc}", file=sys.stderr)

    ros2 = _robot_state["ros2_client"]
    print(f"[voice] connecting to ROS 2 perception bridge ws://{host}:{motion_port} ...", file=sys.stderr)
    _robot_state["robot"] = Lite3Robot(host=host, port=motion_port, ros_client=ros2)

    if ros2 is not None:
        try:
            _robot_state["map_runtime"] = MapRuntime(
                ros_client=ros2,
                base_frame=os.environ.get("ROS2_BASE_FRAME", "rslidar"),
                map_frame=os.environ.get("ROS2_MAP_FRAME", "map"),
                odom_frame=os.environ.get("ROS2_ODOM_FRAME", "odom"),
            )
            print("[voice] map runtime connected", file=sys.stderr)
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
        print("[voice] bedrock agent ready [native converse + current tools]", file=sys.stderr)
    except Exception as exc:
        print(f"[voice] bedrock agent init failed: {exc}", file=sys.stderr)
        _robot_state["orchestrator"] = None


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


def tts_to_wav(text: str) -> bytes:
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as file_obj:
        out = file_obj.name
    try:
        subprocess.run(
            [
                "say",
                "-v",
                TTS_VOICE,
                "--data-format=LEI16@22050",
                "--file-format=WAVE",
                "-o",
                out,
                text,
            ],
            check=True,
            capture_output=True,
            timeout=30,
        )
        with open(out, "rb") as file_obj:
            return file_obj.read()
    finally:
        try:
            os.unlink(out)
        except Exception:
            pass


_tts_pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix="tts")


def _concat_wavs(blobs: list[bytes]) -> bytes:
    if not blobs:
        return b""
    if len(blobs) == 1:
        return blobs[0]
    out_buf = io.BytesIO()
    with wave.open(out_buf, "wb") as w_out:
        with wave.open(io.BytesIO(blobs[0]), "rb") as w0:
            w_out.setnchannels(w0.getnchannels())
            w_out.setsampwidth(w0.getsampwidth())
            w_out.setframerate(w0.getframerate())
            w_out.writeframes(w0.readframes(w0.getnframes()))
        for blob in blobs[1:]:
            with wave.open(io.BytesIO(blob), "rb") as w_n:
                w_out.writeframes(w_n.readframes(w_n.getnframes()))
    return out_buf.getvalue()


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
<title>Lite3 voice</title>
<style>
  :root { color-scheme: dark; }
  body { font-family: -apple-system, system-ui, sans-serif; background: #111; color: #eee; margin: 0; padding: 16px; }
  h1 { font-size: 18px; margin: 0 0 12px; opacity: .7; font-weight: 500; }
  #ptt, #enableAudio, #stopBtn {
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
    setStatus('audio enabled - ready', 'idle');
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
  if (!navigator.mediaDevices) {
    setStatus('browser has no audio recording', 'error');
    return;
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
    setStatus('recording...', 'rec');
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
  btn.textContent = 'play reply';
  btn.style.cssText = 'margin-top:8px;padding:6px 14px;border-radius:8px;border:none;background:#1f6feb;color:#fff;font-size:14px;';
  btn.onclick = () => {
    const a = new Audio(url);
    a.onended = () => setStatus('idle', 'idle');
    setStatus('speaking...', 'busy');
    a.play();
  };
  parentDiv.appendChild(btn);
}

ptt.addEventListener('mousedown', startRecording);
ptt.addEventListener('mouseup', stopRecording);
ptt.addEventListener('mouseleave', () => { if (mediaRecorder?.state === 'recording') stopRecording(); });
ptt.addEventListener('touchstart', (e) => { e.preventDefault(); startRecording(); }, { passive: false });
ptt.addEventListener('touchend', (e) => { e.preventDefault(); stopRecording(); }, { passive: false });
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
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as file_obj:
        audio.save(file_obj.name)
        wav_path = file_obj.name
    upload_kb = os.path.getsize(wav_path) / 1024
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
    preload_whisper()
    _connect_robot()
    print(
        f"[voice] serving on http://{LISTEN_HOST}:{LISTEN_PORT}/  "
        f"(open from your phone using your laptop's WiFi IP).",
        file=sys.stderr,
    )
    app.run(host=LISTEN_HOST, port=LISTEN_PORT, threaded=True)


if __name__ == "__main__":
    main()

