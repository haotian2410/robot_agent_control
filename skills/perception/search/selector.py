"""Deterministic Search Strategy selector."""

from __future__ import annotations

from typing import Any, Dict, Mapping


class StrategySelectionError(ValueError):
    """Raised when a request cannot map to a valid search strategy."""


class StrategySelector:
    """Select active-view behavior; never generate or execute the physical pose."""

    STRATEGIES = {
        "systematic_scan",
        "prior_guided_search",
        "occlusion_aware_view",
        "local_view_refinement",
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
        search = request.get("search", {})
        observation = request.get("observation", {})
        requested = search.get("strategy", "auto")

        if requested != "auto":
            self._validate_explicit_strategy(requested, request, context)
            return self._result(
                requested,
                "explicit search.strategy",
                search_required=True,
            )

        if self._observation_is_sufficient(request):
            return self._result(
                None,
                "current observation already satisfies requested thresholds",
                search_required=False,
            )

        visibility = observation.get("visibility")
        if visibility == "partial" and self._has_occlusion_evidence(request, context):
            return self._result(
                "occlusion_aware_view",
                "target is partially visible and occlusion evidence is available",
            )

        if visibility in {"visible", "partial"}:
            return self._result(
                "local_view_refinement",
                "target is visible but observation quality is insufficient",
            )

        if visibility in {"not_visible", "unknown"} and self._has_prior(request, context):
            return self._result(
                "prior_guided_search",
                "target position or direction prior is available",
            )

        if visibility in {"not_visible", "unknown"} and self._has_search_region(request, context):
            return self._result(
                "systematic_scan",
                "no reliable target prior; a valid search region is available",
            )

        raise StrategySelectionError(
            "Cannot select search strategy: provide occlusion evidence, a target "
            "prior/direction hint, or a valid search region."
        )

    def _validate_explicit_strategy(
        self,
        strategy: str,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> None:
        if strategy not in self.STRATEGIES:
            raise StrategySelectionError(f"Unsupported search strategy: {strategy}")

        if strategy == "systematic_scan" and not self._has_search_region(request, context):
            raise StrategySelectionError(
                "Systematic Scan requires search_region or a runtime viewpoint set."
            )

        if strategy == "prior_guided_search" and not self._has_prior(request, context):
            raise StrategySelectionError(
                "Prior-Guided Search requires a position, region, direction, edge, or tracking prior."
            )

        if strategy == "occlusion_aware_view":
            if not self._has_occlusion_evidence(request, context):
                raise StrategySelectionError(
                    "Occlusion-Aware View requires partial visibility or occlusion evidence."
                )
            observation = request.get("observation", {})
            target = request.get("target", {})
            if not (
                observation.get("coarse_target_pose")
                or observation.get("target_ray")
                or target.get("prior_pose")
                or (context.get("target_prior", {}) or {}).get("pose")
            ):
                raise StrategySelectionError(
                    "Occlusion-Aware View requires a coarse target pose, target ray, or target prior pose."
                )

        if strategy == "local_view_refinement":
            visibility = request.get("observation", {}).get("visibility")
            if visibility not in {"visible", "partial"}:
                raise StrategySelectionError(
                    "Local View Refinement requires a currently visible or partially visible target."
                )

    @staticmethod
    def _has_search_region(
        request: Mapping[str, Any], context: Mapping[str, Any]
    ) -> bool:
        region = request.get("search_region", {})
        return bool(region.get("type") or context.get("default_search_region"))

    @staticmethod
    def _has_prior(request: Mapping[str, Any], context: Mapping[str, Any]) -> bool:
        target = request.get("target", {})
        observation = request.get("observation", {})
        return bool(
            target.get("prior_pose")
            or target.get("prior_region")
            or observation.get("direction_hint")
            or observation.get("image_edge_hint") not in {None, "none"}
            or context.get("target_prior")
            or context.get("tracking_prediction")
        )

    @staticmethod
    def _has_occlusion_evidence(
        request: Mapping[str, Any], context: Mapping[str, Any]
    ) -> bool:
        observation = request.get("observation", {})
        return bool(
            observation.get("visibility") == "partial"
            or observation.get("occlusion_ratio") is not None
            or observation.get("occluder")
            or (context.get("perception_result", {}) or {}).get("occlusion")
            or (context.get("vlm_result", {}) or {}).get("occlusion")
        )

    @staticmethod
    def _observation_is_sufficient(request: Mapping[str, Any]) -> bool:
        observation = request.get("observation", {})
        constraints = request.get("constraints", {})
        if observation.get("visibility") != "visible":
            return False

        confidence = observation.get("confidence", 0.0)
        confidence_threshold = constraints.get("confidence_threshold", 0.7)
        if confidence < confidence_threshold:
            return False

        quality_values = [
            value
            for value in (observation.get("quality", {}) or {}).values()
            if isinstance(value, (int, float))
        ]
        if not quality_values:
            return False

        quality_threshold = constraints.get("quality_threshold", 0.6)
        return min(quality_values) >= quality_threshold

    @staticmethod
    def _result(
        strategy: str | None,
        reason: str,
        *,
        search_required: bool = True,
    ) -> Dict[str, Any]:
        return {
            "search_strategy": strategy,
            "search_required": search_required,
            "selection_reason": reason,
            "fallback_used": False,
        }
