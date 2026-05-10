"""Shared TTS pipeline used by voice_server.py and demo.py.

`tts_to_wav` mirrors the original voice_server implementation: macOS `say`
with explicit PCM/WAVE format flags so callers can concatenate or stream.
`speak(text)` is a convenience for local playback (used by demo.py).

Honoring TTS_VOICE env var keeps the voice consistent across both surfaces.
"""

from __future__ import annotations

import io
import os
import subprocess
import sys
import tempfile
import wave

TTS_VOICE = os.environ.get("TTS_VOICE", "Samantha")


def tts_to_wav(text: str, voice: str | None = None) -> bytes:
    """Synthesize `text` to a WAV byte string. Used by voice_server's HTTP
    endpoint and any caller that needs the audio payload directly."""
    voice = voice or TTS_VOICE
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as file_obj:
        out = file_obj.name
    try:
        subprocess.run(
            [
                "say",
                "-v",
                voice,
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
        except OSError:
            pass


def concat_wavs(blobs: list[bytes]) -> bytes:
    """Stitch multiple WAVs into one. Assumes uniform format (true for our
    `say` outputs since we pin --data-format)."""
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


def speak(text: str, voice: str | None = None, blocking: bool = True) -> None:
    """Synthesize and play locally. Blocks until playback finishes by default
    so on-stage timing is predictable.

    Uses `say` directly for playback (one subprocess) when blocking, since
    that's the simplest and matches voice_server's voice config exactly."""
    if blocking:
        # Direct `say` is simpler than synthesizing-then-playing a wav.
        # Guarantees the same voice/cadence as voice_server's tts_to_wav.
        try:
            subprocess.run(
                ["say", "-v", voice or TTS_VOICE, text],
                check=False,
                timeout=60,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            print(f"[tts] playback unavailable: {exc}", file=sys.stderr)
        return

    # Non-blocking: synthesize to wav, hand to afplay in the background.
    try:
        wav = tts_to_wav(text, voice=voice)
    except Exception as exc:
        print(f"[tts] synthesis failed: {exc}", file=sys.stderr)
        return
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as file_obj:
        file_obj.write(wav)
        path = file_obj.name
    try:
        subprocess.Popen(["afplay", path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError as exc:
        print(f"[tts] afplay unavailable: {exc}", file=sys.stderr)
