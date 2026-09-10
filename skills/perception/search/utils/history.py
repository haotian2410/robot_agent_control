"""Search history comparison interfaces."""

from __future__ import annotations

from typing import Any, Mapping, Sequence


class SearchHistory:
    """Stateless helpers for comparing candidates with supplied Search History."""

    @staticmethod
    def is_duplicate(
        candidate: Mapping[str, Any],
        viewpoints: Sequence[Mapping[str, Any]],
        *,
        position_tolerance: float,
        orientation_tolerance: float,
    ) -> bool:
        """Return whether a candidate is equivalent to a visited viewpoint.

        TODO: compare positions and quaternion angular distance in a common frame.
        """
        raise NotImplementedError("Implement SearchHistory.is_duplicate().")

    @staticmethod
    def next_state(
        request: Mapping[str, Any],
        selected_viewpoint: Mapping[str, Any],
        *,
        improved: bool | None = None,
    ) -> Mapping[str, Any]:
        """Build the state that the Agent should pass into the next Search call."""
        raise NotImplementedError("Implement SearchHistory.next_state().")
