"""Top-level coordinator: dialogue → goal → planner → response."""

from __future__ import annotations

import sys

from memory.schemas import ActiveGoal
from memory.store import MemoryStore

from .sdk_agents import DialogueAgent, PlannerAgent


class Orchestrator:
    def __init__(
        self,
        dialogue: DialogueAgent,
        planner: PlannerAgent,
        memory: MemoryStore,
    ) -> None:
        self._dialogue = dialogue
        self._planner = planner
        self._memory = memory

    def run(self, user_text: str) -> str:
        # Step 1: Classify intent and extract goal
        dialogue_result = self._dialogue.run(user_text)
        intent = dialogue_result.get("intent", "unknown")
        goal_text = dialogue_result.get("goal", user_text)
        print(f"  [dialogue] intent={intent!r}  goal={goal_text!r}", file=sys.stderr)

        # Step 2: Record active goal in memory
        self._memory.set_goal(ActiveGoal(description=goal_text))

        # Step 3: Run planner with current memory snapshot
        snapshot = self._memory.snapshot()
        response = self._planner.run(goal_text, snapshot)

        # Step 4: Mark goal complete
        goal = self._memory.get_goal()
        if goal is not None and goal.status == "active":
            goal.status = "complete"

        return response
