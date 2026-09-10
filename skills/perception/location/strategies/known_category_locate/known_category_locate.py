"""KnownCategoryLocate Strategy skeleton."""

from __future__ import annotations

from typing import Any, Mapping


class KnownCategoryLocate:
    """Generate localization candidates using known-category detection and pose estimation."""

    strategy_id = "known_category_locate"

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def locate(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """Return a backend-independent localization candidate result.

        TODO:
        - Merge request strategy_params with configured defaults.
        - Validate the selected strategy schema.
        - Resolve all required runtime dependencies.
        - Return target_pose, confidence, source, candidates and evidence.
        - Raise typed localization exceptions for expected failures.
        """
        raise NotImplementedError("Implement KnownCategoryLocate.locate().")
