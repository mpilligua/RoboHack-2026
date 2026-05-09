import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import time

from memory.store import MemoryStore
from safety.supervisor import DEPTH_STALE_S, OBSTACLE_BLOCK_MM, SafetySupervisor


def _make(depth_stamp=None, nearest_mm=None):
    store = MemoryStore()
    kwargs = {}
    if depth_stamp is not None:
        kwargs["depth_stamp"] = depth_stamp
    if nearest_mm is not None:
        kwargs["nearest_obstacle_mm"] = nearest_mm
    if kwargs:
        store.update_robot_state(**kwargs)
    return SafetySupervisor(store)


def test_forward_ok_fresh_depth_no_obstacle():
    sup = _make(depth_stamp=time.time(), nearest_mm=800)
    result = sup.check_forward_motion()
    assert result.safe


def test_forward_blocked_depth_stamp_none():
    sup = _make()
    result = sup.check_forward_motion()
    assert not result.safe
    assert "stale" in result.reason.lower() or "missing" in result.reason.lower()


def test_forward_blocked_depth_stale():
    sup = _make(depth_stamp=time.time() - DEPTH_STALE_S - 1)
    result = sup.check_forward_motion()
    assert not result.safe


def test_forward_blocked_obstacle_close():
    sup = _make(depth_stamp=time.time(), nearest_mm=OBSTACLE_BLOCK_MM - 1)
    result = sup.check_forward_motion()
    assert not result.safe
    assert "obstacle" in result.reason.lower()


def test_forward_blocked_obstacle_exactly_at_threshold():
    # strictly less than threshold blocks
    sup = _make(depth_stamp=time.time(), nearest_mm=OBSTACLE_BLOCK_MM)
    result = sup.check_forward_motion()
    assert result.safe  # exactly at threshold is allowed (< not <=)


def test_forward_blocked_emergency_latch():
    sup = _make(depth_stamp=time.time(), nearest_mm=800)
    sup.trigger_emergency_stop()
    result = sup.check_forward_motion()
    assert not result.safe
    assert "emergency" in result.reason.lower()


def test_any_motion_ok_by_default():
    sup = _make()
    result = sup.check_any_motion()
    assert result.safe


def test_any_motion_passes_with_stale_depth():
    sup = _make(depth_stamp=time.time() - 100)
    result = sup.check_any_motion()
    assert result.safe  # stale depth only blocks forward motion


def test_any_motion_blocked_after_emergency():
    sup = _make()
    sup.trigger_emergency_stop()
    result = sup.check_any_motion()
    assert not result.safe


def test_reset_emergency_unblocks():
    sup = _make(depth_stamp=time.time(), nearest_mm=800)
    sup.trigger_emergency_stop()
    assert not sup.check_forward_motion().safe
    sup.reset_emergency_stop()
    assert sup.check_forward_motion().safe


def test_trigger_emergency_is_idempotent():
    sup = _make()
    sup.trigger_emergency_stop()
    sup.trigger_emergency_stop()
    assert sup.is_emergency_latched()


def test_is_emergency_latched():
    sup = _make()
    assert not sup.is_emergency_latched()
    sup.trigger_emergency_stop()
    assert sup.is_emergency_latched()
    sup.reset_emergency_stop()
    assert not sup.is_emergency_latched()
