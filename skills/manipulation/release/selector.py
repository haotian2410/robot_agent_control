"""Deterministic Release Strategy selector."""

from __future__ import annotations

from typing import Any, Dict, Mapping


class StrategySelectionError(ValueError):
    """Raised when a request cannot map to a Release Strategy."""


class StrategySelector:
    STRATEGIES = {"default_release"}

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def select(self, request: Mapping[str, Any], context: Mapping[str, Any] | None = None) -> Dict[str, Any]:
        requested = request["release"]["strategy"]
        strategy = "default_release" if requested == "auto" else requested
        if strategy not in self.STRATEGIES:
            raise StrategySelectionError(f"Unsupported release strategy: {strategy}")
        model = (context or {}).get("gripper_model", {}) or {}
        if model.get("type") and model["type"] not in {"parallel", "two_finger", "open_close"}:
            raise StrategySelectionError("Default Release requires a compatible open-close gripper.")
        return {
            "release_strategy": strategy,
            "selection_reason": "explicit release.strategy" if requested != "auto" else "default strategy for a compatible open-close gripper",
            "retry_used": False,
        }
