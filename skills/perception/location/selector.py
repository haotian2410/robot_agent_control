"""Deterministic Locate Strategy selector."""

from __future__ import annotations

from typing import Any, Dict, Mapping


class StrategySelectionError(ValueError):
    """Raised when a request cannot map to a valid locate strategy."""


class StrategySelector:
    """Select semantic localization behavior; never estimate the physical pose."""

    STRATEGIES = {
        "known_model_locate",
        "known_category_locate",
        "grasp_pose_prediction",
        "unknown_object_locate",
        "approach_point_locate",
        "relative_point_locate",
    }

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def select(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any] | None = None,
    ) -> Dict[str, Any]:
        """Return a deterministic strategy selection result."""
        context = context or {}
        target = request.get("target", {})
        localization = request.get("localization", {})
        requested = localization.get("strategy", "auto")

        if requested != "auto":
            self._validate_explicit_strategy(requested, request, context)
            return {
                "locate_strategy": requested,
                "selection_reason": "explicit localization.strategy",
                "fallback_used": False,
            }

        target_type = target.get("type")
        if target_type == "relative":
            return self._result("relative_point_locate", "target.type is relative")
        if target_type == "approach":
            return self._result("approach_point_locate", "target.type is approach")

        prior = context.get("target_prior", {}) or {}
        model_id = target.get("model_id") or prior.get("model_id")
        category = target.get("category") or prior.get("category")

        if target_type == "grasp":
            operation_point = target.get("operation_point")
            has_predefined_grasp = (
                model_id
                and operation_point not in (None, "", "auto")
            )
            if has_predefined_grasp:
                return self._result(
                    "known_model_locate",
                    "grasp target has an explicit model-defined operation_point",
                )

            self._validate_grasp_prediction_request(request)
            return self._result(
                "grasp_pose_prediction",
                "target.type is grasp and no explicit predefined grasp pose was requested",
            )

        description = (
            target.get("instruction")
            or target.get("object")
            or request.get("strategy_params", {}).get("target_description")
        )

        if model_id:
            return self._result("known_model_locate", "model prior is available")
        if category:
            return self._result("known_category_locate", "category prior is available")
        if description:
            return self._result(
                "unknown_object_locate", "open-vocabulary target description is available"
            )

        raise StrategySelectionError(
            "Cannot select locate strategy: provide model_id, category, target "
            "description, or a relative/approach target definition."
        )

    def _validate_explicit_strategy(
        self,
        strategy: str,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> None:
        if strategy not in self.STRATEGIES:
            raise StrategySelectionError(f"Unsupported locate strategy: {strategy}")

        target = request.get("target", {})
        params = request.get("strategy_params", {})
        prior = context.get("target_prior", {}) or {}

        if strategy == "known_model_locate" and not (
            target.get("model_id") or prior.get("model_id") or params.get("model_id")
        ):
            raise StrategySelectionError("Known Model Locate requires model_id.")

        if strategy == "known_category_locate" and not (
            target.get("category") or prior.get("category") or params.get("category")
        ):
            raise StrategySelectionError("Known Category Locate requires category.")

        if strategy == "unknown_object_locate" and not (
            target.get("instruction")
            or target.get("object")
            or params.get("target_description")
        ):
            raise StrategySelectionError(
                "Unknown Object Locate requires a target description."
            )

        if strategy == "grasp_pose_prediction":
            if target.get("type") != "grasp":
                raise StrategySelectionError(
                    "Grasp Pose Prediction requires target.type=grasp."
                )
            self._validate_grasp_prediction_request(request)

        if strategy == "approach_point_locate" and not (
            request.get("reference", {}).get("pose") or params.get("target_pose")
        ):
            raise StrategySelectionError(
                "Approach Point Locate requires a final target pose."
            )

        if strategy == "relative_point_locate" and not (
            request.get("reference", {}).get("type")
            and request.get("offset", {}).get("mode")
        ):
            raise StrategySelectionError(
                "Relative Point Locate requires reference and offset."
            )

    @staticmethod
    def _validate_grasp_prediction_request(
        request: Mapping[str, Any],
    ) -> None:
        target = request.get("target", {})
        params = request.get("strategy_params", {})
        has_target_condition = any(
            target.get(key)
            for key in ("object", "instruction", "category", "instance_id", "model_id")
        )
        if not has_target_condition and not params.get("allow_scene_level", False):
            raise StrategySelectionError(
                "Grasp Pose Prediction requires target evidence "
                "(object/instruction/category/instance_id/model_id) or "
                "strategy_params.allow_scene_level=true."
            )

    @staticmethod
    def _result(strategy: str, reason: str) -> Dict[str, Any]:
        return {
            "locate_strategy": strategy,
            "selection_reason": reason,
            "fallback_used": False,
        }
