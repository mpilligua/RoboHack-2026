import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from memory.store import MemoryStore
from safety.supervisor import SafetySupervisor
from tools.base import ToolContext, ToolResult
from tools.registry import CALLER_OPERATOR, CALLER_PLANNER, ToolRegistry


def _make_ctx():
    store = MemoryStore()
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
