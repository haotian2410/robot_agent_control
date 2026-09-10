"""Deterministic Push/Pull trajectory selector."""
from typing import Any, Dict, Mapping


class StrategySelectionError(ValueError):
    pass


class StrategySelector:
    def select(self, request: Mapping[str, Any], context: Mapping[str, Any] | None = None) -> Dict[str, Any]:
        trajectory = request["manipulation"]["trajectory"]
        if trajectory not in {"linear", "circular"}:
            raise StrategySelectionError(f"Unsupported trajectory: {trajectory}")
        if request["resolved_interaction_mode"] == "free_object":
            return {
                "push_pull_strategy": "free_object_push",
                "selection_reason": "non-grasped free object with hard linear contact trajectory requested",
            }
        return {"push_pull_strategy": f"{trajectory}_push_pull", "selection_reason": f"hard {trajectory} contact trajectory requested"}
