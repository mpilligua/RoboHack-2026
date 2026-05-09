import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import time

from memory.store import MemoryStore
from safety.supervisor import SafetySupervisor
from tools.base import ToolContext, ToolResult
from tools.registry import CALLER_OPERATOR, CALLER_PLANNER, ToolRegistry


def _make_ctx(depth_stamp=None, nearest_mm=None):
    store = MemoryStore()
    if depth_stamp is not None:
        store.update_robot_state(depth_stamp=depth_stamp)
    if nearest_mm is not None:
        store.update_robot_state(nearest_obstacle_mm=nearest_mm)
    safety = SafetySupervisor(store)
    return ToolContext(
        memory=store,
        robot=None,
        motion=None,
        follow=None,
        basic_goal=None,
        vlm=None,
        safety=safety,
    )


def _ok_handler(ctx, args):
    return ToolResult(ok=True, tool="test_tool", result={"done": True})


def _err_handler(ctx, args):
    raise ValueError("boom")


def test_registered_tool_callable_by_allowed_caller():
    reg = ToolRegistry()
    reg.register("test_tool", _ok_handler, [CALLER_PLANNER])
    ctx = _make_ctx()
    result = reg.call("test_tool", {}, ctx, CALLER_PLANNER)
    assert result.ok


def test_unknown_tool_returns_error():
    reg = ToolRegistry()
    ctx = _make_ctx()
    result = reg.call("no_such_tool", {}, ctx, CALLER_PLANNER)
    assert not result.ok
    assert "unknown tool" in result.error.lower()


def test_disallowed_caller_returns_error():
    reg = ToolRegistry()
    reg.register("test_tool", _ok_handler, [CALLER_OPERATOR])
    ctx = _make_ctx()
    result = reg.call("test_tool", {}, ctx, CALLER_PLANNER)
    assert not result.ok
    assert "not allowed" in result.error.lower()


def test_forward_safety_blocks_when_depth_stale():
    reg = ToolRegistry()
    reg.register("move", _ok_handler, [CALLER_PLANNER], requires_forward_safety=True)
    ctx = _make_ctx()  # no depth_stamp → stale
    result = reg.call("move", {}, ctx, CALLER_PLANNER)
    assert not result.ok
    assert "safety block" in result.error.lower()
    assert "safety_blocked" in result.events


def test_forward_safety_passes_with_fresh_depth_and_clear_path():
    reg = ToolRegistry()
    reg.register("move", _ok_handler, [CALLER_PLANNER], requires_forward_safety=True)
    ctx = _make_ctx(depth_stamp=time.time(), nearest_mm=800)
    result = reg.call("move", {}, ctx, CALLER_PLANNER)
    assert result.ok


def test_motion_safety_passes_with_stale_depth():
    # turn/backward only block on emergency latch, not stale depth
    reg = ToolRegistry()
    reg.register("turn", _ok_handler, [CALLER_PLANNER], requires_motion_safety=True)
    ctx = _make_ctx()  # stale depth but no emergency
    result = reg.call("turn", {}, ctx, CALLER_PLANNER)
    assert result.ok


def test_motion_safety_blocks_on_emergency():
    reg = ToolRegistry()
    reg.register("turn", _ok_handler, [CALLER_PLANNER], requires_motion_safety=True)
    ctx = _make_ctx()
    ctx.safety.trigger_emergency_stop()
    result = reg.call("turn", {}, ctx, CALLER_PLANNER)
    assert not result.ok
    assert "safety_blocked" in result.events


def test_handler_exception_caught_as_error():
    reg = ToolRegistry()
    reg.register("boom_tool", _err_handler, [CALLER_PLANNER])
    ctx = _make_ctx()
    result = reg.call("boom_tool", {}, ctx, CALLER_PLANNER)
    assert not result.ok
    assert "ValueError" in result.error


def test_events_logged_on_each_call():
    reg = ToolRegistry()
    reg.register("test_tool", _ok_handler, [CALLER_PLANNER])
    ctx = _make_ctx()
    reg.call("test_tool", {"x": 1}, ctx, CALLER_PLANNER)
    events = ctx.memory.last_events(10)
    types = [e.type for e in events]
    assert "tool_call" in types
    assert "tool_result" in types


def test_operator_only_tool_blocked_from_planner():
    reg = ToolRegistry()
    reg.register("reset_emergency", _ok_handler, [CALLER_OPERATOR])
    ctx = _make_ctx()
    result = reg.call("reset_emergency", {}, ctx, CALLER_PLANNER)
    assert not result.ok


def test_operator_can_call_operator_tool():
    reg = ToolRegistry()
    reg.register("reset_emergency", _ok_handler, [CALLER_OPERATOR])
    ctx = _make_ctx()
    result = reg.call("reset_emergency", {}, ctx, CALLER_OPERATOR)
    assert result.ok
