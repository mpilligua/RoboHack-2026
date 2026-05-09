"""Top-level coordinator: goal → planner → response."""

from __future__ import annotations

from typing import Iterator

from memory.schemas import ActiveGoal
from memory.store import MemoryStore

from .sdk_agents import PlannerAgent


class Orchestrator:
    def __init__(
        self,
        planner: PlannerAgent,
        memory: MemoryStore,
    ) -> None:
        self._planner = planner
        self._memory = memory

    def run(self, user_text: str) -> str:
        # Record active goal in memory
        self._memory.set_goal(ActiveGoal(description=user_text))

        # Run planner with current memory snapshot
        snapshot = self._memory.snapshot()
        response = self._planner.run(user_text, snapshot)

        # Mark goal complete
        goal = self._memory.get_goal()
        if goal is not None and goal.status == "active":
            goal.status = "complete"

        return response

    def run_stream(self, user_text: str) -> Iterator[str]:
        """Yield reply chunks as the planner produces them."""
        self._memory.set_goal(ActiveGoal(description=user_text))
        snapshot = self._memory.snapshot()
        try:
            yield from self._planner.run_stream(user_text, snapshot)
        finally:
            goal = self._memory.get_goal()
            if goal is not None and goal.status == "active":
                goal.status = "complete"
