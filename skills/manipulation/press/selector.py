"""Deterministic Press Strategy selector."""

from __future__ import annotations

from typing import Any, Dict, Mapping


class StrategySelectionError(ValueError):
    """Raised when a request cannot map to a valid press strategy."""


class StrategySelector:
    """Select contact-control behavior; never estimate the press point."""

    STRATEGIES = {"displacement_press", "force_controlled_press"}

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def select(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any] | None = None,
    ) -> Dict[str, Any]:
        context = context or {}
        press = request.get("press", {})
        params = request.get("strategy_params", {})
        requested = press.get("strategy", "auto")
        control_mode = press.get("control_mode", "auto")

        if press.get("operation") == "verify_only":
            strategy = "force_controlled_press" if control_mode == "force" or requested == "force_controlled_press" else "displacement_press"
            return self._result(strategy, "verify_only uses the compatible result validator")

        if requested != "auto":
            self._validate_explicit_strategy(requested, request, context)
            return self._result(requested, "explicit press.strategy")

        if control_mode == "force":
            self._require_force_strategy_inputs(request, context)
            return self._result(
                "force_controlled_press", "press.control_mode is force"
            )

        if control_mode == "displacement":
            self._require_displacement_inputs(request)
            return self._result(
                "displacement_press", "press.control_mode is displacement"
            )

        if params.get("target_force") is not None:
            self._require_force_strategy_inputs(request, context)
            return self._result(
                "force_controlled_press",
                "target_force and force feedback are available",
            )

        if (
            params.get("press_depth") is not None
            or params.get("travel_distance") is not None
        ):
            self._require_displacement_inputs(request)
            return self._result(
                "displacement_press", "press depth or travel distance is available"
            )

        raise StrategySelectionError(
            "Cannot select press strategy: provide target_force with force feedback, "
            "or provide press_depth/travel_distance."
        )

    def _validate_explicit_strategy(
        self,
        strategy: str,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> None:
        if strategy not in self.STRATEGIES:
            raise StrategySelectionError(f"Unsupported press strategy: {strategy}")
        if strategy == "force_controlled_press":
            self._require_force_strategy_inputs(request, context)
        else:
            self._require_displacement_inputs(request)

    @staticmethod
    def _require_displacement_inputs(request: Mapping[str, Any]) -> None:
        params = request.get("strategy_params", {})
        if params.get("press_depth") is None and params.get("travel_distance") is None:
            raise StrategySelectionError(
                "Displacement Press requires press_depth or travel_distance."
            )

    @staticmethod
    def _require_force_strategy_inputs(
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> None:
        params = request.get("strategy_params", {})
        if params.get("target_force") is None:
            raise StrategySelectionError(
                "Force-Controlled Press requires target_force."
            )
        force_state = context.get("force_torque_state", {}) or {}
        tactile_state = context.get("tactile_state", {}) or {}
        if not (force_state.get("available") and force_state.get("calibrated") or tactile_state.get("available")):
            raise StrategySelectionError(
                "Force-Controlled Press requires valid force or tactile feedback."
            )

    @staticmethod
    def _result(strategy: str, reason: str) -> Dict[str, Any]:
        return {
            "press_strategy": strategy,
            "selection_reason": reason,
            "retry_used": False,
        }
