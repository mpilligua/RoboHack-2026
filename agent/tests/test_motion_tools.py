import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from memory.store import MemoryStore
from safety.supervisor import SafetySupervisor
from tools.base import ToolContext
from tools.motion_tools import handle_walk_forward


class _FakeRobot:
    def __init__(self, summary=None, error=None):
        self._summary = summary or {"min_mm": 250, "center_mm": 400, "median_mm": 500, "valid_fraction": 0.8}
        self._error = error

    def depth_summary(self, timeout_s=12.0):
        if self._error is not None:
            raise RuntimeError(self._error)
        return self._summary


class _FakeMotion:
    def __init__(self):
        self.calls = []

    def forward(self, speed, duration_s):
        self.calls.append((speed, duration_s))


def _ctx(robot=None, motion=None):
    store = MemoryStore()
    safety = SafetySupervisor(store)
    return ToolContext(
        memory=store,
        robot=robot or _FakeRobot(),
        motion=motion or _FakeMotion(),
        follow=None,
        basic_goal=None,
        vlm=None,
        safety=safety,
    )


def test_walk_forward_allows_motion_when_depth_reports_close_obstacle():
    motion = _FakeMotion()
    ctx = _ctx(robot=_FakeRobot(summary={"min_mm": 200, "center_mm": 220, "median_mm": 300, "valid_fraction": 0.9}), motion=motion)

    result = handle_walk_forward(ctx, {"duration_s": 1.0})

    assert result.ok
    assert motion.calls
    assert "depth_advisory" in result.result


def test_walk_forward_allows_motion_when_depth_advisory_fails():
    motion = _FakeMotion()
    ctx = _ctx(robot=_FakeRobot(error="camera timeout"), motion=motion)

    result = handle_walk_forward(ctx, {"duration_s": 1.0})

    assert result.ok
    assert motion.calls
    assert "depth_advisory" in result.result
    assert "unavailable" in result.result["depth_advisory"]
