from __future__ import annotations

import threading
import time

import numpy as _np

from memory.schemas import Event

from .base import ToolContext, ToolResult


def _require_follow(ctx: ToolContext, tool: str):
    if ctx.follow is None:
        raise RuntimeError(f"follow adapter not connected (tool: {tool})")


# How long to keep seeing nothing of the target before declaring it lost.
# Short blips (target turns sideways, brief occlusion) shouldn't trigger this.
_LOST_TARGET_GRACE_S = 3.0


def handle_follow_person(ctx: ToolContext, args: dict) -> ToolResult:
    _require_follow(ctx, "follow_person")
    yolo_id = int(args["yolo_id"])
    ctx.follow.follow(yolo_id)
    return ToolResult(ok=True, tool="follow_person", result={"action": "follow_person", "yolo_id": yolo_id})


def handle_go_to_object(ctx: ToolContext, args: dict) -> ToolResult:
    _require_follow(ctx, "go_to_object")
    yolo_id = int(args["yolo_id"])
    stop_distance_m = float(args.get("stop_distance_m", 0.8))
    timeout_s = float(args.get("timeout_s", 30.0))

    ctx.follow.follow(yolo_id)

    def _record_outcome(outcome: str, detail: str, last_depth_m: float | None) -> None:
        """Write the navigation result to event memory so the next planner
        turn sees it in its snapshot. Without this, the watcher silently
        stops and the user has no feedback on whether the goal was reached."""
        ctx.memory.add_event(Event(
            timestamp=time.time(),
            type="nav_outcome",
            source="go_to_object",
            message=f"{outcome}: {detail}",
            data={
                "yolo_id": yolo_id,
                "outcome": outcome,
                "stop_distance_m": stop_distance_m,
                "last_depth_m": last_depth_m,
            },
        ))

    def watcher():
        deadline = time.time() + timeout_s
        last_depth_m: float | None = None
        last_seen_ts = time.time()
        while time.time() < deadline:
            time.sleep(0.5)
            dets = ctx.follow.get_detections()
            target = next((d for d in dets if d.id == yolo_id), None)
            if target is None:
                # Target not in current detections — could be a transient
                # occlusion or genuinely lost. Tolerate up to grace period.
                if time.time() - last_seen_ts > _LOST_TARGET_GRACE_S:
                    ctx.follow.stop()
                    _record_outcome(
                        "lost_target",
                        f"yolo_id {yolo_id} not seen for {_LOST_TARGET_GRACE_S}s",
                        last_depth_m,
                    )
                    return
                continue
            last_seen_ts = time.time()
            try:
                rgb = ctx.robot.get_rgb(timeout_s=2.0)
                depth = ctx.robot.get_depth(timeout_s=2.0)
            except TimeoutError:
                continue
            sx = depth.width / float(rgb.width)
            sy = depth.height / float(rgb.height)
            x1, y1, x2, y2 = target.bbox
            dx1 = max(0, int(x1 * sx))
            dy1 = max(0, int(y1 * sy))
            dx2 = min(depth.width, int(x2 * sx))
            dy2 = min(depth.height, int(y2 * sy))
            patch = depth.depth_mm[dy1:dy2, dx1:dx2]
            valid = patch[(patch > 100) & (patch < 8000)]
            if valid.size < 50:
                continue
            median_m = float(_np.median(valid)) / 1000.0
            last_depth_m = median_m
            if median_m <= stop_distance_m:
                ctx.follow.stop()
                _record_outcome("reached", f"stopped at {median_m:.2f} m", median_m)
                return
        # Loop exited because deadline passed.
        ctx.follow.stop()
        _record_outcome(
            "timeout",
            f"timed out after {timeout_s:.0f}s, last depth {last_depth_m}",
            last_depth_m,
        )

    threading.Thread(target=watcher, daemon=True).start()

    return ToolResult(
        ok=True,
        tool="go_to_object",
        result={
            "action": "go_to_object",
            "yolo_id": yolo_id,
            "stop_distance_m": stop_distance_m,
            "timeout_s": timeout_s,
            "note": "Walking toward target; will auto-stop when median depth in bbox <= stop_distance_m. Outcome (reached/lost_target/timeout) will appear in the next memory snapshot's recent events.",
        },
    )


# Search-and-go defaults — continuous rotation, sample YOLO in a background
# thread, interrupt rotation the moment a match appears (no stop-go-stop).
_SEARCH_OMEGA = 0.5           # rad/s. Bedrock-clipped to MAX_ANGULAR=0.5 anyway.
_SEARCH_POLL_INTERVAL_S = 0.15  # ~6 Hz YOLO polling during rotation
_SEARCH_MAX_TOTAL_DEG = 360.0


def _matches_label(det, label: str) -> bool:
    return label.lower() in (det.label or "").lower()


def _scan_for_label(ctx: ToolContext, label: str):
    """Return the first detection whose label matches, or None."""
    for det in ctx.follow.get_detections():
        if _matches_label(det, label):
            return det
    return None


def handle_find_and_go_to(ctx: ToolContext, args: dict) -> ToolResult:
    """Rotate in place sampling YOLO until an object with `label` is found,
    then auto-walk to it (same behavior as go_to_object). If a full sweep
    finds nothing, return ok=false."""
    _require_follow(ctx, "find_and_go_to")
    label = (args.get("label") or "").strip()
    if not label:
        return ToolResult(ok=False, tool="find_and_go_to", error="label is required")

    found, swept_deg, chunks, direction, err = _sweep_until_found(ctx, label, args)
    if err is not None:
        return ToolResult(ok=False, tool="find_and_go_to", error=err)
    if found is None:
        return ToolResult(
            ok=False,
            tool="find_and_go_to",
            error=f"no object matching {label!r} found after sweeping {swept_deg:.0f}° {direction}",
            result={"label": label, "swept_deg": swept_deg, "chunks": chunks},
        )

    stop_distance_m = float(args.get("stop_distance_m", 0.8))
    go_result = handle_go_to_object(
        ctx,
        {"yolo_id": found.id, "stop_distance_m": stop_distance_m},
    )
    return ToolResult(
        ok=go_result.ok,
        tool="find_and_go_to",
        result={
            "label": label,
            "found_yolo_id": found.id,
            "found_label": found.label,
            "swept_deg": swept_deg,
            "direction": direction,
            "go_to_object": go_result.result,
        },
        error=go_result.error,
    )


def _sweep_until_found(ctx: ToolContext, label: str, args: dict) -> tuple:
    """Continuous rotation while YOLO is sampled in a background thread.
    The moment a matching detection appears, we call motion.stop() which
    interrupts the long _drive loop within ~50 ms.

    Returns (detection_or_None, swept_deg, chunks_taken, direction, error_msg_or_None).
    `chunks_taken` is kept for backward-compat with callers; in continuous mode
    we report the number of YOLO polls instead. Hardware failures during the
    rotation are caught and reported as structured errors."""
    import sys
    if ctx.motion is None:
        return None, 0.0, 0, "right", "motion adapter not connected"

    direction = (args.get("direction") or "right").lower()
    if direction not in ("left", "right"):
        return None, 0.0, 0, direction, "direction must be 'left' or 'right'"
    turn_fn = ctx.motion.turn_left if direction == "left" else ctx.motion.turn_right

    max_sweep_deg = float(args.get("max_sweep_deg", _SEARCH_MAX_TOTAL_DEG))
    omega = float(args.get("omega", _SEARCH_OMEGA))
    sweep_duration_s = (max_sweep_deg * 3.14159 / 180.0) / omega

    # Check current view first — no rotation needed if target is already visible.
    found_in_view = _scan_for_label(ctx, label)
    if found_in_view is not None:
        print(f"[sweep] match in current view: id={found_in_view.id} "
              f"label={found_in_view.label!r} — skipping rotation",
              file=sys.stderr, flush=True)
        return found_in_view, 0.0, 0, direction, None

    print(f"[sweep] continuous rotation: looking for {label!r} dir={direction} "
          f"omega={omega} duration={sweep_duration_s:.1f}s ({max_sweep_deg}°)",
          file=sys.stderr, flush=True)

    # Background thread polls YOLO; on first match it captures the detection
    # and stops the motion. The main thread sits in the (blocking) turn call
    # and exits as soon as motion.stop() flips the _stop_flag.
    found_holder: list = [None]
    polls_holder = [0]
    poller_done = threading.Event()
    sweep_started_at = time.time()

    def poller():
        try:
            while not poller_done.is_set():
                time.sleep(_SEARCH_POLL_INTERVAL_S)
                polls_holder[0] += 1
                det = _scan_for_label(ctx, label)
                if det is not None:
                    found_holder[0] = det
                    print(f"[sweep] match after {polls_holder[0]} polls "
                          f"(~{(time.time()-sweep_started_at)*omega*180/3.14159:.0f}°): "
                          f"id={det.id} label={det.label!r}",
                          file=sys.stderr, flush=True)
                    try:
                        ctx.motion.stop()
                    except Exception:
                        pass
                    return
        except Exception as e:
            print(f"[sweep] poller error: {type(e).__name__}: {e}",
                  file=sys.stderr, flush=True)

    poller_thread = threading.Thread(target=poller, daemon=True)
    poller_thread.start()

    try:
        # drive_continuous bypasses the 2s per-call cap AND the trailing zero,
        # so the dog rotates smoothly the whole time. The poller thread
        # interrupts via ctx.motion.stop() when it spots a match.
        # Sign convention matches Lite3Motion: right = negative omega.
        signed_omega = -abs(omega) if direction == "right" else abs(omega)
        elapsed_s = ctx.motion.drive_continuous(
            vx=0.0, vy=0.0,
            omega=signed_omega,
            max_duration_s=sweep_duration_s,
        )
        # drive_continuous does not publish a zero; halt explicitly after the
        # rotation ends, whether by full sweep or by poller interrupt.
        try:
            ctx.motion.stop()
        except Exception:
            pass
        actual_swept_deg = (elapsed_s * abs(omega)) * 180.0 / 3.14159
    except Exception as e:
        poller_done.set()
        try:
            ctx.motion.stop()
        except Exception:
            pass
        return None, 0.0, polls_holder[0], direction, f"{type(e).__name__}: {e}"

    poller_done.set()
    poller_thread.join(timeout=1.0)

    found = found_holder[0]
    if found is None:
        print(f"[sweep] swept ~{actual_swept_deg:.0f}° in {elapsed:.1f}s "
              f"with {polls_holder[0]} polls — no match",
              file=sys.stderr, flush=True)
    return found, actual_swept_deg, polls_holder[0], direction, None


def handle_find_object(ctx: ToolContext, args: dict) -> ToolResult:
    """Locate-only sister of find_and_go_to: rotate until target appears,
    then return its yolo_id without walking. The robot ends the call facing
    the target so a subsequent go_to_object / follow_person works without
    re-searching."""
    _require_follow(ctx, "find_object")
    label = (args.get("label") or "").strip()
    if not label:
        return ToolResult(ok=False, tool="find_object", error="label is required")

    found, swept_deg, chunks, direction, err = _sweep_until_found(ctx, label, args)
    if err is not None:
        return ToolResult(ok=False, tool="find_object", error=err)
    if found is None:
        return ToolResult(
            ok=False,
            tool="find_object",
            error=f"no object matching {label!r} found after sweeping {swept_deg:.0f}° {direction}",
            result={"label": label, "swept_deg": swept_deg, "chunks": chunks},
        )
    return ToolResult(
        ok=True,
        tool="find_object",
        result={
            "label": label,
            "yolo_id": found.id,
            "found_label": found.label,
            "swept_deg": swept_deg,
            "direction": direction,
        },
    )


def handle_find_person_and_follow(ctx: ToolContext, args: dict) -> ToolResult:
    """Find-and-follow analogue of find_and_go_to. Rotates until a person is
    visible, then engages follow_person on them (continuous tracking)."""
    _require_follow(ctx, "find_person_and_follow")
    label = (args.get("label") or "person").strip()  # default to 'person'

    found, swept_deg, chunks, direction, err = _sweep_until_found(ctx, label, args)
    if err is not None:
        return ToolResult(ok=False, tool="find_person_and_follow", error=err)
    if found is None:
        return ToolResult(
            ok=False,
            tool="find_person_and_follow",
            error=f"no object matching {label!r} found after sweeping {swept_deg:.0f}° {direction}",
            result={"label": label, "swept_deg": swept_deg, "chunks": chunks},
        )

    follow_result = handle_follow_person(ctx, {"yolo_id": found.id})
    return ToolResult(
        ok=follow_result.ok,
        tool="find_person_and_follow",
        result={
            "label": label,
            "found_yolo_id": found.id,
            "found_label": found.label,
            "swept_deg": swept_deg,
            "direction": direction,
            "follow_person": follow_result.result,
        },
        error=follow_result.error,
    )


def handle_stop_tracking(ctx: ToolContext, args: dict) -> ToolResult:
    if ctx.follow is None:
        return ToolResult(ok=True, tool="stop_tracking", result={"action": "stop_tracking", "note": "no follow adapter"})
    try:
        ctx.follow.stop()
    except Exception as e:
        return ToolResult(ok=False, tool="stop_tracking", error=str(e))
    return ToolResult(ok=True, tool="stop_tracking", result={"action": "stop_tracking"})
