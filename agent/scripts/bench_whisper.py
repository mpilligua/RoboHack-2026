"""Local Whisper benchmark.

Records 5 seconds from your Mac mic, transcribes with several Whisper
model sizes, prints how long each took. No phone, no upload, no agent.

Run:
    python scripts/bench_whisper.py [seconds] [model1] [model2] ...
Defaults: 5 seconds, models = tiny.en, base.en, small.en
"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time

import whisper


def record_with_say(seconds: float, out_path: str) -> None:
    """Record from the default mic via macOS `sox`/`afrecord`. Falls back to
    `ffmpeg` if neither is available."""
    # Prefer ffmpeg's avfoundation (works with Homebrew ffmpeg).
    if subprocess.call(["which", "ffmpeg"], stdout=subprocess.DEVNULL) == 0:
        cmd = [
            "ffmpeg", "-y", "-f", "avfoundation", "-i", ":0",
            "-ar", "16000", "-ac", "1", "-t", str(seconds), out_path,
        ]
        subprocess.run(cmd, check=True, capture_output=True)
        return
    raise RuntimeError(
        "ffmpeg not found. Install with `brew install ffmpeg` so we can record audio."
    )


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) >= 2 else 5.0
    models = sys.argv[2:] or ["tiny.en", "base.en", "small.en"]

    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        wav = f.name

    try:
        print(f"recording {seconds}s … speak now", flush=True)
        t0 = time.time()
        record_with_say(seconds, wav)
        rec_time = time.time() - t0
        size_kb = os.path.getsize(wav) / 1024
        print(f"  recorded in {rec_time:.2f}s ({size_kb:.0f} KB)\n")

        for name in models:
            print(f"loading {name} …", flush=True)
            t0 = time.time()
            model = whisper.load_model(name)
            load_t = time.time() - t0

            t0 = time.time()
            result = model.transcribe(wav, fp16=False)
            inf_t = time.time() - t0
            text = (result.get("text") or "").strip()

            print(
                f"  {name:>10s}  load {load_t:5.2f}s  infer {inf_t:5.2f}s  →  {text!r}\n",
                flush=True,
            )
    finally:
        try:
            os.unlink(wav)
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
