"""Grasp request validation and normalization."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, Mapping


class GraspValidationError(ValueError):
    """Raised when a Grasp request violates the public or strategy schema."""


def validate_grasp_request(
    request: Mapping[str, Any],
    config: Mapping[str, Any] | None = None,
) -> Dict[str, Any]:
    """Validate, apply defaults and return a normalized request."""
    if not isinstance(request, Mapping):
        raise GraspValidationError("Grasp request must be a mapping/object.")

    normalized = deepcopy(dict(request))
    target = normalized.get("target")
    if not isinstance(target, Mapping):
        raise GraspValidationError("target is required and must be an object.")

    target_type = target.get("type")
    if target_type not in {"object", "grasp_pose"}:
        raise GraspValidationError(
            "target.type must be object or grasp_pose."
        )
    if target_type == "object" and not (
        target.get("object_id") or target.get("category")
    ):
        raise GraspValidationError(
            "Object target requires target.object_id or target.category."
        )
    if target_type == "grasp_pose" and not target.get("grasp_pose"):
        raise GraspValidationError(
            "grasp_pose target requires target.grasp_pose."
        )

    normalized.setdefault("grasp", {})
    if not isinstance(normalized["grasp"], Mapping):
        raise GraspValidationError("grasp must be an object.")
    normalized["grasp"] = dict(normalized["grasp"])
    normalized["grasp"].setdefault("strategy", "auto")
    normalized["grasp"].setdefault("operation", "grasp")
    normalized["grasp"].setdefault("gripper_id", "default_gripper")
    normalized["grasp"].setdefault("verification_mode", "auto")
    if normalized["grasp"]["strategy"] not in {"auto", "default_grasp"}:
        raise GraspValidationError("grasp.strategy must be auto or default_grasp.")
    if normalized["grasp"]["operation"] not in {"grasp", "regrasp", "verify_only"}:
        raise GraspValidationError("grasp.operation must be grasp, regrasp, or verify_only.")
    if normalized["grasp"]["verification_mode"] not in {"auto", "position", "force", "tactile", "visual", "combined"}:
        raise GraspValidationError("Unsupported grasp.verification_mode.")

    normalized.setdefault("constraints", {})
    if not isinstance(normalized["constraints"], Mapping):
        raise GraspValidationError("constraints must be an object.")
    normalized["constraints"] = dict(normalized["constraints"])
    normalized["constraints"].setdefault("verify_grasp", True)
    normalized["constraints"].setdefault("retry_count", 0)
    normalized["constraints"].setdefault("hold_on_success", True)
    normalized["constraints"].setdefault("timeout", 10.0)
    normalized["constraints"].setdefault("maximum_force", 40.0)
    normalized["constraints"].setdefault("minimum_width", 0.0)
    normalized["constraints"].setdefault("contact_required", True)
    normalized["constraints"].setdefault("slip_check", True)
    normalized["constraints"].setdefault("require_pose_alignment", False)
    normalized["constraints"].setdefault("pose_position_tolerance", 0.01)
    normalized["constraints"].setdefault("pose_orientation_tolerance", 10.0)
    normalized.setdefault("strategy_params", {})
    if not isinstance(normalized["strategy_params"], Mapping):
        raise GraspValidationError("strategy_params must be an object.")
    normalized["strategy_params"] = dict(normalized["strategy_params"])

    constraints = normalized["constraints"]
    retry_count = constraints.get("retry_count")
    if not isinstance(retry_count, int) or retry_count < 0:
        raise GraspValidationError("constraints.retry_count must be a non-negative integer.")

    timeout = constraints.get("timeout")
    if not isinstance(timeout, (int, float)) or timeout <= 0:
        raise GraspValidationError("constraints.timeout must be positive.")

    maximum_force = constraints.get("maximum_force")
    if not isinstance(maximum_force, (int, float)) or maximum_force < 0:
        raise GraspValidationError("constraints.maximum_force must be non-negative.")

    minimum_width = constraints.get("minimum_width")
    maximum_width = constraints.get("maximum_width")
    if not isinstance(minimum_width, (int, float)) or minimum_width < 0:
        raise GraspValidationError("constraints.minimum_width must be non-negative.")
    if maximum_width is not None:
        if not isinstance(maximum_width, (int, float)) or maximum_width < minimum_width:
            raise GraspValidationError(
                "constraints.maximum_width must be >= minimum_width."
            )

    if retry_count > 10:
        raise GraspValidationError("constraints.retry_count must not exceed 10.")
    for key in ("verify_grasp", "hold_on_success", "contact_required", "slip_check", "require_pose_alignment"):
        if not isinstance(constraints[key], bool):
            raise GraspValidationError(f"constraints.{key} must be boolean.")
    for key in ("open_width", "close_width", "grasp_force", "hold_force", "contact_threshold", "position_tolerance", "force_tolerance", "settle_time"):
        if key in normalized["strategy_params"]:
            value = normalized["strategy_params"][key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0:
                raise GraspValidationError(f"strategy_params.{key} must be a non-negative number.")
            normalized["strategy_params"][key] = float(value)
    speed = normalized["strategy_params"].get("close_speed")
    if speed is not None and (isinstance(speed, bool) or not isinstance(speed, (int, float)) or not 0.0 < float(speed) <= 1.0):
        raise GraspValidationError("strategy_params.close_speed must be in (0, 1].")
    if speed is not None:
        normalized["strategy_params"]["close_speed"] = float(speed)
    if "expected_width" in target:
        width = target["expected_width"]
        if isinstance(width, bool) or not isinstance(width, (int, float)) or width < 0:
            raise GraspValidationError("target.expected_width must be non-negative.")
        normalized["target"] = dict(target)
        normalized["target"]["expected_width"] = float(width)
    return normalized
