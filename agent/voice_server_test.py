"""Voice loop test — no robot, no agent, no Bedrock.

Just: phone records audio → Whisper transcribes → server echoes the
transcript back as TTS. Use this to confirm the phone↔laptop audio path
works before plugging anything else in.

Run:
    python voice_server_test.py
Then open http://<laptop-ip>:5050/ on your phone (same WiFi).
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

from flask import Flask, jsonify, request, send_file


WHISPER_MODEL_NAME = os.environ.get("WHISPER_MODEL", "base")
TTS_VOICE = os.environ.get("TTS_VOICE", "Samantha")
LISTEN_HOST = os.environ.get("VOICE_HOST", "0.0.0.0")
LISTEN_PORT = int(os.environ.get("VOICE_PORT", "5050"))


_whisper = {"model": None, "lock": threading.Lock()}


def preload_whisper() -> None:
    """Load the Whisper model at server start so the first request is fast."""
    import whisper as _w
    print(f"[voice-test] preloading whisper={WHISPER_MODEL_NAME} …", file=sys.stderr)
    t0 = time.time()
    with _whisper["lock"]:
        _whisper["model"] = _w.load_model(WHISPER_MODEL_NAME)
    print(f"[voice-test] whisper ready in {time.time()-t0:.1f}s", file=sys.stderr)


def whisper_transcribe(path: str) -> str:
    with _whisper["lock"]:
        if _whisper["model"] is None:
            # Fallback if preload was skipped.
            import whisper as _w
            _whisper["model"] = _w.load_model(WHISPER_MODEL_NAME)
        m = _whisper["model"]
    return (m.transcribe(path, fp16=False).get("text") or "").strip()


def tts_to_aiff(text: str) -> bytes:
    with tempfile.NamedTemporaryFile(suffix=".aiff", delete=False) as f:
        out = f.name
    try:
        subprocess.run(
            ["say", "-v", TTS_VOICE, "-o", out, text],
            check=True, capture_output=True, timeout=30,
        )
        with open(out, "rb") as f:
            return f.read()
    finally:
        try: os.unlink(out)
        except Exception: pass


_tts_cache: dict[str, bytes] = {}
app = Flask(__name__)


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>voice test</title>
<style>
  :root { color-scheme: dark; }
  body { font-family: -apple-system, system-ui, sans-serif; background: #111; color: #eee; margin:0; padding:16px; }
  h1 { font-size: 18px; opacity:.7; margin: 0 0 12px; }
  #ptt { display:block; width:100%; height:220px; border:none; border-radius:24px; background:#1f6feb; color:#fff; font-size:28px; font-weight:600; -webkit-tap-highlight-color: transparent; touch-action: manipulation; }
  #ptt.recording { background:#d63838; }
  #status { font-size:14px; padding:8px 12px; border-radius:8px; margin-bottom:10px; background:#222; }
  #status[data-kind="busy"]  { background:#3b3b1a; color:#ffe680; }
  #status[data-kind="rec"]   { background:#5a1a1a; color:#ffb3b3; }
  #status[data-kind="error"] { background:#5a1a1a; color:#ff8080; }
  .turn { padding:12px 14px; border-radius:14px; margin:10px 0; line-height:1.35; }
  .me { background:#243559; }
  .bot { background:#2a2a2a; }
</style>
</head>
<body>
  <h1>voice test (echo, no agent)</h1>
  <div id="status">idle</div>
  <button id="ptt">hold to talk</button>
  <div id="log"></div>
<script>
const ptt = document.getElementById('ptt');
const log = document.getElementById('log');
const statusEl = document.getElementById('status');
let mediaRecorder = null, chunks = [];
function setStatus(text, kind) {
  statusEl.textContent = text;
  statusEl.dataset.kind = kind || 'idle';
}
function add(role, text) {
  const div = document.createElement('div');
  div.className = 'turn ' + (role === 'me' ? 'me' : 'bot');
  div.textContent = text;
  log.prepend(div);
}
async function startRec() {
  try {
    setStatus('asking for mic permission…', 'busy');
    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    mediaRecorder = new MediaRecorder(stream); chunks = [];
    mediaRecorder.ondataavailable = e => chunks.push(e.data);
    mediaRecorder.onstop = async () => {
      const blob = new Blob(chunks, { type: mediaRecorder.mimeType || 'audio/webm' });
      stream.getTracks().forEach(t => t.stop());
      setStatus('transcribing…', 'busy');
      const fd = new FormData(); fd.append('audio', blob, 'clip.webm');
      const res = await fetch('/echo', { method:'POST', body: fd });
      if (!res.ok) { setStatus('error ' + res.status, 'error'); return; }
      const data = await res.json();
      if (data.transcript) add('me', data.transcript);
      add('bot', data.reply);
      if (data.tts_id) {
        setStatus('speaking…', 'busy');
        const a = new Audio('/tts?id=' + encodeURIComponent(data.tts_id));
        a.onended = () => setStatus('idle', 'idle');
        a.play().catch(() => setStatus('idle', 'idle'));
      } else {
        setStatus('idle', 'idle');
      }
    };
    mediaRecorder.start();
    ptt.classList.add('recording'); ptt.textContent = 'release';
    setStatus('recording…', 'rec');
  } catch (err) {
    setStatus('mic blocked: ' + (err.message || err.name), 'error');
  }
}
function stopRec() {
  if (mediaRecorder && mediaRecorder.state !== 'inactive') mediaRecorder.stop();
  ptt.classList.remove('recording'); ptt.textContent = 'hold to talk';
}
ptt.addEventListener('mousedown', startRec);
ptt.addEventListener('mouseup', stopRec);
ptt.addEventListener('mouseleave', () => { if (mediaRecorder?.state==='recording') stopRec(); });
ptt.addEventListener('touchstart', e => { e.preventDefault(); startRec(); }, {passive:false});
ptt.addEventListener('touchend',   e => { e.preventDefault(); stopRec(); }, {passive:false});
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return INDEX_HTML


@app.route("/echo", methods=["POST"])
def echo():
    t_req = time.time()
    audio = request.files.get("audio")
    if audio is None:
        return ("missing audio", 400)
    suffix = os.path.splitext(audio.filename or "")[1] or ".webm"
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as f:
        audio.save(f.name)
        path = f.name
    upload_size_kb = os.path.getsize(path) / 1024
    t_recv = time.time()
    try:
        transcript = whisper_transcribe(path)
    finally:
        try: os.unlink(path)
        except Exception: pass
    t_stt = time.time()

    if not transcript:
        print(
            f"[voice-test] {upload_size_kb:.0f}KB  recv {t_recv-t_req:.2f}s  "
            f"stt {t_stt-t_recv:.2f}s  heard nothing",
            file=sys.stderr,
        )
        return jsonify({"transcript": "", "reply": "(no speech detected)"})

    reply = f"I heard you say: {transcript}"
    tts_id = uuid.uuid4().hex
    try:
        _tts_cache[tts_id] = tts_to_aiff(reply)
    except Exception as e:
        print(f"[voice-test] tts failed: {e}", file=sys.stderr)
        tts_id = None
    t_tts = time.time()
    print(
        f"[voice-test] {upload_size_kb:.0f}KB  recv {t_recv-t_req:.2f}s  "
        f"stt {t_stt-t_recv:.2f}s  tts {t_tts-t_stt:.2f}s  "
        f"total {t_tts-t_req:.2f}s  → {transcript!r}",
        file=sys.stderr,
    )
    return jsonify({"transcript": transcript, "reply": reply, "tts_id": tts_id})


@app.route("/tts")
def tts():
    audio = _tts_cache.pop(request.args.get("id", ""), None)
    if audio is None:
        return ("not found", 404)
    return send_file(io.BytesIO(audio), mimetype="audio/aiff")


def main() -> None:
    preload_whisper()
    print(f"[voice-test] http://{LISTEN_HOST}:{LISTEN_PORT}/  (open from your phone)", file=sys.stderr)
    app.run(host=LISTEN_HOST, port=LISTEN_PORT, threaded=True)


if __name__ == "__main__":
    main()
