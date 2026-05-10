# Legacy loop exports — wrapped so boto3 absence doesn't break the new pipeline
try:
    from .perception import TOOL_SCHEMAS, dispatch
except (ImportError, ModuleNotFoundError):
    TOOL_SCHEMAS = []

    def dispatch(*_args, **_kwargs):
        raise ImportError("boto3 is required for the legacy tool loop (pip install boto3)")

# New pipeline exports
from .setup import build_registry
from .base import ToolContext, ToolResult

__all__ = ["TOOL_SCHEMAS", "dispatch", "build_registry", "ToolContext", "ToolResult"]
