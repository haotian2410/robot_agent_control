"""Search request validation skeleton."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, Mapping


class SearchValidationError(ValueError):
    """Raised when a Search request violates the public or strategy schema."""


def validate_search_request(
    request: Mapping[str, Any],
    config: Mapping[str, Any] | None = None,
) -> Dict[str, Any]:
    """Validate, apply defaults and return a normalized request.

    This lightweight skeleton performs basic structural checks. Replace or
    extend it with the project's schema engine and strategy-schema dispatch.
    """
    if not isinstance(request, Mapping):
        raise SearchValidationError("Search request must be a mapping/object.")

    normalized = deepcopy(dict(request))
    target = normalized.get("target")
    if not isinstance(target, Mapping):
        raise SearchValidationError("target is required and must be an object.")
    if not any(
        target.get(key)
        for key in (
            "object",
            "instruction",
            "instance_id",
            "category",
            "model_id",
            "prior_pose",
            "prior_region",
        )
    ):
        raise SearchValidationError(
            "target requires a description, identifier, category, model, pose, or region prior."
        )

    observation = normalized.get("observation")
    if not isinstance(observation, Mapping):
        raise SearchValidationError(
            "observation is required and must be an object."
        )
    if observation.get("visibility") not in {
        "visible",
        "partial",
        "not_visible",
        "unknown",
    }:
        raise SearchValidationError(
            "observation.visibility must be visible, partial, not_visible, or unknown."
        )

    camera = normalized.get("camera")
    if not isinstance(camera, Mapping):
        raise SearchValidationError("camera is required and must be an object.")
    if not camera.get("id") or not camera.get("optical_frame"):
        raise SearchValidationError("camera.id and camera.optical_frame are required.")
    allowed_mounts = {
        "wrist",
        "head",
        "pan_tilt",
        "mobile",
        "movable_external",
        "fixed_external",
    }
    if camera.get("mount_type") not in allowed_mounts:
        raise SearchValidationError(
            "camera.mount_type must identify a supported camera mount."
        )
    if camera.get("mount_type") == "fixed_external" and camera.get("controlled_by") in {None, "none"}:
        raise SearchValidationError(
            "A fixed external camera cannot update viewpoint without another movable observation device."
        )

    normalized.setdefault("trigger", {})
    normalized.setdefault("search", {})
    normalized["search"].setdefault("strategy", "auto")
    normalized["search"].setdefault("goal", "locate")
    normalized["search"].setdefault("mode", "single_step")
    normalized["search"].setdefault("output_frame", "base")
    normalized["search"].setdefault("candidate_selection", "highest_score")
    normalized["search"].setdefault("max_candidates", 20)
    normalized["search"].setdefault("max_steps", 8)
    normalized["search"].setdefault("allow_fallback", True)

    normalized.setdefault("search_region", {})
    normalized.setdefault("constraints", {})
    normalized["constraints"].setdefault("confidence_threshold", 0.7)
    normalized["constraints"].setdefault("quality_threshold", 0.6)
    normalized["constraints"].setdefault("check_reachability", True)
    normalized["constraints"].setdefault("avoid_collision", True)
    normalized["constraints"].setdefault("minimum_clearance", 0.03)
    normalized["constraints"].setdefault("min_view_distance", 0.15)
    normalized["constraints"].setdefault("max_view_distance", 1.50)
    normalized["constraints"].setdefault("keep_target_in_view", True)
    normalized["constraints"].setdefault("require_look_at_target", True)
    normalized["constraints"].setdefault("timeout", 30.0)

    normalized.setdefault("history", {})
    normalized["history"].setdefault("step_index", 0)
    normalized["history"].setdefault("visited_viewpoints", [])
    normalized["history"].setdefault("failed_viewpoints", [])
    normalized["history"].setdefault("previous_observations", [])
    normalized["history"].setdefault("no_improvement_steps", 0)
    normalized.setdefault("strategy_params", {})

    confidence = normalized["observation"].get("confidence", 0.0)
    if not isinstance(confidence, (int, float)) or not 0.0 <= confidence <= 1.0:
        raise SearchValidationError(
            "observation.confidence must be between 0 and 1."
        )

    for key in ("confidence_threshold", "quality_threshold"):
        value = normalized["constraints"].get(key)
        if not isinstance(value, (int, float)) or not 0.0 <= value <= 1.0:
            raise SearchValidationError(
                f"constraints.{key} must be between 0 and 1."
            )

    if normalized["constraints"]["min_view_distance"] > normalized["constraints"]["max_view_distance"]:
        raise SearchValidationError(
            "constraints.min_view_distance cannot exceed max_view_distance."
        )

    if normalized["history"]["step_index"] >= normalized["search"]["max_steps"]:
        raise SearchValidationError(
            "Search history has reached search.max_steps; return SEARCH_EXHAUSTED."
        )

    timeout = normalized["constraints"].get("timeout")
    if not isinstance(timeout, (int, float)) or timeout <= 0:
        raise SearchValidationError("constraints.timeout must be positive.")

    # TODO: validate frames, poses, timestamps, strategy schema and history consistency.
    return normalized
