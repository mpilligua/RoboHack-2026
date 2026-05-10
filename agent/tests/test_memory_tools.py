import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from memory.schemas import ActiveGoal, ObjectRecord
from memory.store import MemoryStore
from safety.supervisor import SafetySupervisor
from tools.base import ToolContext
from tools.memory_tools import handle_resolve_reference


def _ctx():
    store = MemoryStore()
    return ToolContext(
        memory=store,
        robot=None,
        motion=None,
        follow=None,
        basic_goal=None,
        vlm=None,
        safety=SafetySupervisor(store),
    )


def test_resolve_reference_selects_object_on_active_goal():
    ctx = _ctx()
    ctx.memory.set_goal(ActiveGoal(description="go to it"))
    ctx.memory.upsert_object(ObjectRecord(
        yolo_id=4,
        label="chair",
        description="a red chair",
        bbox=[0, 0, 10, 10],
    ))

    result = handle_resolve_reference(ctx, {"ref": "chair"})

    assert result.ok
    assert ctx.memory.get_goal().selected_object_id == 4
    assert result.result["id"] == 4

