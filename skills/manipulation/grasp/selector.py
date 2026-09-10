"""Deterministic Grasp Strategy selector."""

from __future__ import annotations

from typing import Any, Dict, Mapping


class StrategySelectionError(ValueError):
    """Raised when a request cannot map to a valid grasp strategy."""


class StrategySelector:
    """Select grasp execution behavior; never generate a grasp pose."""

    STRATEGIES = {"default_grasp"}

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def select(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any] | None = None,
    ) -> Dict[str, Any]:
        context = context or {}
        grasp = request.get("grasp", {})
        requested = grasp.get("strategy", "auto")

        if requested != "auto":
            self._validate_explicit_strategy(requested, request, context)
            return self._result(requested, "explicit grasp.strategy")

        self._validate_default_compatibility(request, context)
        return self._result(
            "default_grasp",
            "default strategy for a compatible open-close gripper",
        )

    def _validate_explicit_strategy(
        self,
        strategy: str,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> None:
        if strategy not in self.STRATEGIES:
            raise StrategySelectionError(f"Unsupported grasp strategy: {strategy}")
        self._validate_default_compatibility(request, context)

    @staticmethod
    def _validate_default_compatibility(
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> None:
        operation = request.get("grasp", {}).get("operation", "grasp")
        if operation not in {"grasp", "regrasp", "verify_only"}:
            raise StrategySelectionError(
                f"Default Grasp does not support operation: {operation}"
            )

        model = context.get("gripper_model", {}) or {}
        gripper_type = model.get("type")
        if gripper_type and gripper_type not in {
            "parallel",
            "two_finger",
            "open_close",
        }:
            raise StrategySelectionError(
                "Default Grasp requires a compatible open-close gripper."
            )

    @staticmethod
    def _result(strategy: str, reason: str) -> Dict[str, Any]:
        return {
            "grasp_strategy": strategy,
            "selection_reason": reason,
            "retry_used": False,
        }
