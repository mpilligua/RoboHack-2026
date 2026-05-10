import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import importlib.util
from pathlib import Path


_SDK_TOOLS_PATH = Path(__file__).resolve().parents[1] / "agents_app" / "sdk_tools.py"
_SPEC = importlib.util.spec_from_file_location("sdk_tools_direct", _SDK_TOOLS_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
assert _SPEC is not None and _SPEC.loader is not None
_SPEC.loader.exec_module(_MODULE)

PLANNER_TOOLS = _MODULE.PLANNER_TOOLS
PLANNER_TOOLS_BEDROCK = _MODULE.PLANNER_TOOLS_BEDROCK


def test_memory_tools_exposed_in_openai_schema():
    names = {tool["function"]["name"] for tool in PLANNER_TOOLS}
    assert "get_visible_objects" in names
    assert "find_objects_matching_constraints" in names


def test_memory_tools_exposed_in_bedrock_schema():
    names = {tool["toolSpec"]["name"] for tool in PLANNER_TOOLS_BEDROCK}
    assert "get_visible_objects" in names
    assert "find_objects_matching_constraints" in names
