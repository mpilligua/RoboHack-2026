import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import threading
import time

import pytest

from memory.schemas import ActiveGoal, ObjectRecord
from memory.store import MemoryStore


def _obj(yolo_id=1, label="chair", description="a wooden chair", bbox=None, position_text="center"):
    return ObjectRecord(
        yolo_id=yolo_id,
        label=label,
        description=description,
        bbox=bbox or [10, 10, 100, 100],
        position_text=position_text,
    )


def test_upsert_inserts_new():
    store = MemoryStore()
    store.upsert_object(_obj(yolo_id=1))
    assert store.get_object(1) is not None
    assert store.get_object(1).label == "chair"


def test_upsert_merges_seen_count():
    store = MemoryStore()
    store.upsert_object(_obj(yolo_id=1))
    store.upsert_object(_obj(yolo_id=1, description="updated description"))
    obj = store.get_object(1)
    assert obj.seen_count == 2
    assert obj.description == "updated description"


def test_get_objects_sorted_by_last_seen():
    store = MemoryStore()
    store.upsert_object(_obj(yolo_id=1))
    time.sleep(0.01)
    store.upsert_object(_obj(yolo_id=2, label="table"))
    objs = store.get_objects()
    assert objs[0].yolo_id == 2


def test_find_by_label_case_insensitive():
    store = MemoryStore()
    store.upsert_object(_obj(label="Chair"))
    results = store.find_objects_by_label("chair")
    assert len(results) == 1
    results = store.find_objects_by_label("CHAIR")
    assert len(results) == 1


def test_resolve_reference_it_uses_goal():
    store = MemoryStore()
    store.upsert_object(_obj(yolo_id=5, label="bottle"))
    store.set_goal(ActiveGoal(description="grab the bottle", selected_object_id=5))
    result = store.resolve_reference("it")
    assert result is not None
    assert result.yolo_id == 5


def test_resolve_reference_label_match():
    store = MemoryStore()
    store.upsert_object(_obj(yolo_id=3, label="person"))
    result = store.resolve_reference("person")
    assert result is not None
    assert result.yolo_id == 3


def test_resolve_reference_no_match_returns_none():
    store = MemoryStore()
    result = store.resolve_reference("dragon")
    assert result is None


def test_find_objects_matching_constraints_label():
    store = MemoryStore()
    store.upsert_object(_obj(yolo_id=1, label="chair"))
    store.upsert_object(_obj(yolo_id=2, label="table"))
    results = store.find_objects_matching_constraints({"label": "chair"})
    assert len(results) == 1
    assert results[0].yolo_id == 1


def test_find_objects_matching_constraints_position():
    store = MemoryStore()
    store.upsert_object(_obj(yolo_id=1, position_text="left"))
    store.upsert_object(_obj(yolo_id=2, position_text="right"))
    results = store.find_objects_matching_constraints({"position": "left"})
    assert len(results) == 1 and results[0].yolo_id == 1


def test_find_objects_matching_constraints_depth():
    store = MemoryStore()
    o1 = _obj(yolo_id=1)
    o1.depth_m = 1.5
    store.upsert_object(o1)
    o2 = _obj(yolo_id=2)
    o2.depth_m = 3.0
    store.upsert_object(o2)
    results = store.find_objects_matching_constraints({"max_depth_m": 2.0})
    assert len(results) == 1 and results[0].yolo_id == 1


def test_event_cap_at_200():
    store = MemoryStore()
    from memory.schemas import Event
    for i in range(250):
        store.add_event(Event(time.time(), "tool_call", "test", f"msg {i}"))
    assert len(store.last_events(300)) == 200


def test_snapshot_has_required_keys():
    store = MemoryStore()
    snap = store.snapshot()
    assert "robot" in snap
    assert "objects" in snap
    assert "goal" in snap
    assert "recent_events" in snap


def test_thread_safety_concurrent_upsert():
    store = MemoryStore()
    errors = []

    def worker(obj_id):
        try:
            for _ in range(20):
                store.upsert_object(_obj(yolo_id=obj_id, label=f"obj{obj_id}"))
        except Exception as e:
            errors.append(e)

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(10)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert not errors
    assert len(store.get_objects()) == 10
