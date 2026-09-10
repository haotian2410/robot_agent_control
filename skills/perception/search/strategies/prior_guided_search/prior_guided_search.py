"""PriorGuidedSearch Strategy skeleton."""

from __future__ import annotations

from typing import Any, Mapping


class PriorGuidedSearch:
    """Generate candidate camera viewpoints for Prior-Guided Search."""

    strategy_id = "prior_guided_search"

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def generate(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """Return a backend-independent candidate viewpoint result.

        TODO:
        - Merge request strategy_params with configured defaults.
        - Validate the selected strategy schema.
        - Resolve required camera, target, geometry and history context.
        - Return candidate_viewpoints, predicted gains, source and evidence.
        - Raise typed search exceptions for expected failures.
        """
        raise NotImplementedError("Implement PriorGuidedSearch.generate().")
