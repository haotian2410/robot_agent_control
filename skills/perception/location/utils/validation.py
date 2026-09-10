"""Locate request validation skeleton."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, Mapping


class LocateValidationError(ValueError):
    """Raised when a Locate request violates the public or strategy schema."""


def validate_locate_request(
    request: Mapping[str, Any],
    config: Mapping[str, Any] | None = None,
) -> Dict[str, Any]:
    """Validate, apply defaults and return a normalized request.

    This lightweight skeleton performs basic structural checks. Replace or
    extend it with the project's schema engine and strategy-schema dispatch.
    """
    if not isinstance(request, Mapping):
        raise LocateValidationError("Locate request must be a mapping/object.")

    normalized = deepcopy(dict(request))
    target = normalized.get("target")
    if not isinstance(target, Mapping):
        raise LocateValidationError("target is required and must be an object.")

    target_type = target.get("type")
    allowed_types = {
        "grasp",
        "place",
        "press",
        "insert",
        "extract",
        "approach",
        "relative",
        "object_pose",
    }
    if target_type not in allowed_types:
        raise LocateValidationError(
            "target.type must be grasp, place, press, insert, extract, "
            "approach, relative, or object_pose."
        )

    normalized.setdefault("reference", {})
    normalized.setdefault("offset", {})
    normalized.setdefault("localization", {})
    normalized["localization"].setdefault("strategy", "auto")
    normalized["localization"].setdefault("precision", "medium")
    normalized["localization"].setdefault("output_frame", "base")
    normalized["localization"].setdefault(
        "candidate_selection", "highest_confidence"
    )
    normalized["localization"].setdefault("max_candidates", 10)
    normalized["localization"].setdefault("allow_fallback", True)

    normalized.setdefault("constraints", {})
    normalized["constraints"].setdefault("confidence_threshold", 0.5)
    normalized["constraints"].setdefault("check_reachability", True)
    normalized["constraints"].setdefault("avoid_collision", True)
    normalized["constraints"].setdefault("minimum_clearance", 0.02)
    normalized["constraints"].setdefault("position_tolerance", 0.01)
    if "orientation_required" not in normalized["constraints"]:
        normalized["constraints"]["orientation_required"] = target_type == "grasp"
    normalized["constraints"].setdefault("timeout", 30.0)
    normalized.setdefault("strategy_params", {})

    if target_type == "relative":
        if not normalized["reference"].get("type"):
            raise LocateValidationError(
                "Relative target requires reference.type."
            )
        if normalized["offset"].get("mode") not in {"semantic", "cartesian"}:
            raise LocateValidationError(
                "Relative target requires offset.mode semantic or cartesian."
            )

    if target_type == "approach" and not (
        normalized["reference"].get("pose")
        or normalized["strategy_params"].get("target_pose")
    ):
        raise LocateValidationError(
            "Approach target requires reference.pose or strategy_params.target_pose."
        )

    threshold = normalized["constraints"].get("confidence_threshold")
    if not isinstance(threshold, (int, float)) or not 0.0 <= threshold <= 1.0:
        raise LocateValidationError(
            "constraints.confidence_threshold must be between 0 and 1."
        )

    timeout = normalized["constraints"].get("timeout")
    if not isinstance(timeout, (int, float)) or timeout <= 0:
        raise LocateValidationError("constraints.timeout must be positive.")

    # TODO: validate frames, pose values, numeric ranges and selected strategy schema.
    return normalized
