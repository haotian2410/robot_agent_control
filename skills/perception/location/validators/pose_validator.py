"""Unified Locate candidate validation skeleton."""

from __future__ import annotations

from typing import Any, Mapping


class PoseValidationError(RuntimeError):
    """Raised when no candidate satisfies the requested constraints."""


class PoseValidator:
    """Transform, rank and validate localization candidates."""

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def validate(
        self,
        raw_result: Mapping[str, Any],
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """Return the selected validated localization result.

        TODO:
        - Normalize candidate output and transform it to localization.output_frame.
        - Validate finite position and normalized orientation values.
        - Enforce confidence_threshold and orientation_required.
        - Check reachability when requested.
        - Check occupancy, collision and minimum_clearance when requested.
        - Rank valid candidates using candidate_selection.
        - Return structured validation evidence and typed failures.
        """
        raise NotImplementedError("Implement PoseValidator.validate().")
