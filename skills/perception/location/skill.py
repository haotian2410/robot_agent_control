"""Locate Skill orchestration skeleton.

This file defines interfaces and execution order only. Integrate concrete
sensors, perception models, point-cloud processing, transform services,
reachability checks and collision checks in the robot runtime.
"""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

from selector import StrategySelectionError, StrategySelector
from strategies import (
    ApproachPointLocate,
    GraspPosePrediction,
    KnownCategoryLocate,
    KnownModelLocate,
    RelativePointLocate,
    UnknownObjectLocate,
)
from utils.error_codes import ErrorCode, failure_result
from utils.validation import LocateValidationError, validate_locate_request
from validators import PoseValidator


class LocateSkill:
    """Coordinate request validation, strategy selection and pose validation."""

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
            "known_model_locate": KnownModelLocate(self.config),
            "known_category_locate": KnownCategoryLocate(self.config),
            "grasp_pose_prediction": GraspPosePrediction(self.config),
            "unknown_object_locate": UnknownObjectLocate(self.config),
            "approach_point_locate": ApproachPointLocate(self.config),
            "relative_point_locate": RelativePointLocate(self.config),
        }
        self.pose_validator = PoseValidator(self.config)

    def execute(self, request: Mapping[str, Any]) -> Dict[str, Any]:
        """Execute one Locate request.

        Intended pipeline:
        1. Validate and normalize the external request.
        2. Read synchronized perception, transform and robot context.
        3. Select one Locate Strategy using deterministic rules.
        4. Let the strategy generate one or more candidate poses.
        5. Apply legal fallback only in auto mode and only when configured.
        6. Transform, rank and validate candidates.
        7. Return the selected target pose and structured evidence.
        """
        try:
            normalized = validate_locate_request(request, self.config)
            context = self._get_runtime_context(normalized)
            selection = self.selector.select(normalized, context)

            raw_result = self._locate_with_fallback(
                normalized, context, selection
            )
            localization_result = self.pose_validator.validate(
                raw_result, normalized, context
            )

            return {
                "success": True,
                "status": "completed",
                "selection": selection,
                "localization_result": localization_result,
                "error": None,
            }
        except LocateValidationError as exc:
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
                recommended_action="Provide sufficient target evidence or select a compatible strategy.",
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
        """Read synchronized sensor, transform, model and robot context."""
        raise NotImplementedError(
            "Connect _get_runtime_context() to the robot runtime/context manager."
        )

    def _locate_with_fallback(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
        selection: Dict[str, Any],
    ) -> Mapping[str, Any]:
        """Run the selected strategy and optionally apply a legal fallback.

        Required implementation behavior:
        - Explicit strategies must not silently fall back.
        - Fallback is permitted only when localization.allow_fallback is true.
        - Every fallback candidate must satisfy its own input schema.
        - Update selection['locate_strategy'], reason and fallback_used.
        - Preserve typed failures and evidence from each attempted strategy.
        """
        strategy = self.strategies[selection["locate_strategy"]]
        return strategy.locate(request, context)
