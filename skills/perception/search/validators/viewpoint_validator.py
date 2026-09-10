"""Unified Search candidate viewpoint validation skeleton."""

from __future__ import annotations

from typing import Any, Mapping


class ViewpointValidationError(RuntimeError):
    """Raised when no candidate viewpoint satisfies requested constraints."""


class ViewpointValidator:
    """Transform, score and validate active-view candidates."""

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def validate(
        self,
        raw_result: Mapping[str, Any],
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """Return the selected validated Search result.

        TODO:
        - Normalize candidate camera poses and transform to search.output_frame.
        - Generate or validate look-at orientation.
        - Check target region against camera field of view.
        - Reject visited and failed viewpoints using Search History.
        - Convert wrist-camera poses to end-effector targets using hand-eye extrinsics.
        - Check reachability when requested.
        - Check occupancy, collision and minimum_clearance when requested.
        - Predict visibility/information gain and calculate motion cost.
        - Rank valid candidates using search.candidate_selection.
        - Build the Move or camera-controller execution request template.
        - Return next_viewpoint, candidates, validation and search_state.
        """
        raise NotImplementedError("Implement ViewpointValidator.validate().")
