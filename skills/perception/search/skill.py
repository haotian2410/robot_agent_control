"""Search Skill orchestration skeleton.

This file defines interfaces and execution order only. Integrate concrete
camera models, VLM/perception services, geometry, ray casting, transform
services, reachability checks and collision checks in the robot runtime.
"""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

from selector import StrategySelectionError, StrategySelector
from strategies import (
    LocalViewRefinement,
    OcclusionAwareView,
    PriorGuidedSearch,
    SystematicScan,
)
from utils.error_codes import ErrorCode, failure_result
from utils.validation import SearchValidationError, validate_search_request
from validators import ViewpointValidator


class SearchSkill:
    """Coordinate request validation, strategy selection and viewpoint validation."""

    def __init__(
        self,
        config: Optional[Mapping[str, Any]] = None,
        *,
        robot_runtime: Any = None,
    ) -> None:
        self.config: Mapping[str, Any] = config or {}
        self.robot_runtime = robot_runtime

        self.selector = StrategySelector(self.config)
        self.strategies = {
            "systematic_scan": SystematicScan(self.config),
            "prior_guided_search": PriorGuidedSearch(self.config),
            "occlusion_aware_view": OcclusionAwareView(self.config),
            "local_view_refinement": LocalViewRefinement(self.config),
        }
        self.viewpoint_validator = ViewpointValidator(self.config)

    def execute(self, request: Mapping[str, Any]) -> Dict[str, Any]:
        """Execute one Search request.

        Intended pipeline:
        1. Validate and normalize the external request.
        2. Read synchronized camera, perception, transform and robot context.
        3. Stop without movement when the current observation is sufficient.
        4. Select one Search Strategy using deterministic rules.
        5. Let the strategy generate candidate camera viewpoints.
        6. Apply legal fallback only in auto mode and only when configured.
        7. Transform, score and validate candidates against history and safety.
        8. Return one next viewpoint and an execution request template.
        """
        try:
            normalized = validate_search_request(request, self.config)
            context = self._get_runtime_context(normalized)
            selection = self.selector.select(normalized, context)

            if not selection["search_required"]:
                return {
                    "success": True,
                    "status": "completed",
                    "selection": selection,
                    "search_result": {
                        "next_viewpoint": None,
                        "candidates": [],
                        "search_state": self._build_stop_state(
                            normalized, "observation_sufficient"
                        ),
                        "execution_request": None,
                    },
                    "error": None,
                }

            raw_result = self._search_with_fallback(
                normalized, context, selection
            )
            search_result = self.viewpoint_validator.validate(
                raw_result, normalized, context
            )

            return {
                "success": True,
                "status": "completed",
                "selection": selection,
                "search_result": search_result,
                "error": None,
            }
        except SearchValidationError as exc:
            return failure_result(
                ErrorCode.INVALID_REQUEST,
                str(exc),
                failed_stage="validation",
                recoverable=True,
                recommended_action="Correct the request using schema.yaml.",
            )
        except StrategySelectionError as exc:
            return failure_result(
                ErrorCode.INVALID_REQUEST,
                str(exc),
                failed_stage="strategy_selection",
                recoverable=True,
                recommended_action=(
                    "Provide sufficient observation, target prior, direction hint, "
                    "or a valid search region."
                ),
            )
        except NotImplementedError as exc:
            return failure_result(
                ErrorCode.NOT_IMPLEMENTED,
                str(exc),
                failed_stage="implementation",
                recoverable=False,
            )
        except Exception as exc:  # Replace with typed runtime exceptions.
            return failure_result(
                ErrorCode.INTERNAL_ERROR,
                str(exc),
                failed_stage="unknown",
                recoverable=False,
            )

    def _get_runtime_context(
        self, request: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        """Read synchronized camera, perception, transform and robot context."""
        raise NotImplementedError(
            "Connect _get_runtime_context() to the robot runtime/context manager."
        )

    def _search_with_fallback(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
        selection: Dict[str, Any],
    ) -> Mapping[str, Any]:
        """Run the selected strategy and optionally apply a legal fallback.

        Required implementation behavior:
        - Explicit strategies must not silently fall back.
        - Fallback is permitted only when search.allow_fallback is true.
        - Every fallback candidate must satisfy its own input schema.
        - Update selection['search_strategy'], reason and fallback_used.
        - Preserve typed failures and evidence from each attempted strategy.
        - Stop when max_steps or no-improvement limits are reached.
        """
        strategy = self.strategies[selection["search_strategy"]]
        return strategy.generate(request, context)

    @staticmethod
    def _build_stop_state(
        request: Mapping[str, Any], stop_reason: str
    ) -> Dict[str, Any]:
        history = request.get("history", {})
        search = request.get("search", {})
        step_index = history.get("step_index", 0)
        max_steps = search.get("max_steps", 8)
        return {
            "step_index": step_index,
            "should_continue": False,
            "remaining_steps": max(0, max_steps - step_index),
            "stop_reason": stop_reason,
        }
