"""Background tick driver for WorldObjectUpdater.

Runs in a daemon thread. Sleeps ``period_s`` between ticks. Catches and logs
all exceptions so a transient rosbridge hiccup doesn't crash the CLI.

Usage from cli.py / voice_server.py:
    from perception.world_tick import WorldTickDriver
    driver = WorldTickDriver(robot, follow, memory, period_s=1.0)
    driver.start()
    # ...
    driver.stop()  # at shutdown (optional; daemon thread exits with the process)
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from typing import Optional

from .world_map import WorldObjectUpdater, CameraIntrinsics, D435I_640x480


class WorldTickDriver:
    def __init__(self,
                 robot,
                 follow,
                 memory,
                 *,
                 K: CameraIntrinsics = D435I_640x480,
                 period_s: float = 1.0,
                 logger: Optional[logging.Logger] = None) -> None:
        if logger is None:
            logger = logging.getLogger("world_tick")
            if not logger.handlers:
                h = logging.StreamHandler(sys.stderr)
                h.setFormatter(logging.Formatter("[world_tick] %(message)s"))
                logger.addHandler(h)
                logger.setLevel(logging.INFO)
        self._log = logger
        self._updater = WorldObjectUpdater(robot, follow, memory, K=K, logger=logger)
        self._period = period_s
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="world_tick", daemon=True)
        self._thread.start()
        self._log.info(f"started (period={self._period:.1f}s)")

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        last_summary = 0.0
        ticks = 0
        projected = 0
        no_depth = 0
        no_pose = 0
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                stats = self._updater.tick()
                ticks += 1
                projected += stats.n_projected
                no_depth += stats.n_skipped_no_depth
                no_pose += stats.n_skipped_no_pose
            except Exception as e:
                self._log.warning(f"tick failed: {type(e).__name__}: {e}")
                stats = None

            # Periodic 1-line summary so it's clear it's alive without spamming
            now = time.monotonic()
            if now - last_summary >= 10.0 and ticks > 0:
                self._log.info(
                    f"summary: {ticks} ticks, {projected} projections, "
                    f"{no_depth} dets without depth, {no_pose} dets without pose"
                )
                last_summary = now
                ticks = projected = no_depth = no_pose = 0

            elapsed = time.monotonic() - t0
            wait = self._period - elapsed
            if wait > 0:
                self._stop.wait(timeout=wait)
