"""Validation and normalization for Move requests."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, Mapping


class MoveValidationError(ValueError):
    """Raised when a Move request violates the public or strategy schema."""


def validate_move_request(
    request: Mapping[str, Any],
    config: Mapping[str, Any] | None = None,
) -> Dict[str, Any]:
    """Validate numeric ranges, dimensions, enums, and strategy requirements."""
    if not isinstance(request, Mapping):
        raise MoveValidationError("Move request must be a mapping/object.")

    normalized = deepcopy(dict(request))
    target = normalized.get("target")
    if not isinstance(target, Mapping):
        raise MoveValidationError("target is required and must be an object.")

    target_type = target.get("type")
    if target_type not in {"pose", "joint", "named_state"}:
        raise MoveValidationError(
            "target.type must be pose, joint, or named_state."
        )

    if target_type == "pose" and "position" not in target:
        raise MoveValidationError("Pose target requires target.position.")
    if target_type == "joint" and "joint_positions" not in target:
        raise MoveValidationError(
            "Joint target requires target.joint_positions."
        )
    if target_type == "named_state" and not target.get("state_name"):
        raise MoveValidationError(
            "Named-state target requires target.state_name."
        )

    normalized["target"] = dict(target)
    normalized.setdefault("motion", {})
    if not isinstance(normalized["motion"], Mapping):
        raise MoveValidationError("motion must be an object.")
    normalized["motion"] = dict(normalized["motion"])
    normalized["motion"].setdefault("path_type", "auto")
    normalized["motion"].setdefault("path_constraint", "soft")
    normalized["motion"].setdefault("phase", "transit")
    normalized["motion"].setdefault("optimization_goal", "balanced")

    normalized.setdefault("planning", {})
    if not isinstance(normalized["planning"], Mapping):
        raise MoveValidationError("planning must be an object.")
    normalized["planning"] = dict(normalized["planning"])
    normalized["planning"].setdefault("mode", "auto")
    normalized["planning"].setdefault("allow_replan", True)
    normalized["planning"].setdefault("planning_time", 5.0)
    normalized["planning"].setdefault("max_attempts", 3)

    normalized.setdefault("constraints", {})
    if not isinstance(normalized["constraints"], Mapping):
        raise MoveValidationError("constraints must be an object.")
    normalized["constraints"] = dict(normalized["constraints"])
    normalized["constraints"].setdefault("avoid_collision", True)
    normalized["constraints"].setdefault("velocity_scale", 0.5)
    normalized["constraints"].setdefault("acceleration_scale", 0.5)
    normalized["constraints"].setdefault("position_tolerance", 0.005)
    normalized["constraints"].setdefault("orientation_tolerance", 2.0)
    normalized["constraints"].setdefault("keep_end_effector_orientation", False)
    normalized["constraints"].setdefault("timeout", 30.0)
    normalized.setdefault("strategy_params", {})
    if not isinstance(normalized["strategy_params"], Mapping):
        raise MoveValidationError("strategy_params must be an object.")
    normalized["strategy_params"] = dict(normalized["strategy_params"])

    _enum(normalized["motion"], "path_type", {"auto", "joint", "linear", "circular"})
    _enum(normalized["motion"], "path_constraint", {"soft", "hard"})
    _enum(normalized["motion"], "phase", {"transit", "approach", "retreat", "contact"})
    _enum(normalized["motion"], "optimization_goal", {"balanced", "fastest", "shortest", "smoothest", "safest"})
    _enum(normalized["planning"], "mode", {"auto", "direct", "collision_free"})
    for key in ("allow_replan",):
        if not isinstance(normalized["planning"][key], bool):
            raise MoveValidationError(f"planning.{key} must be boolean.")
    _number_range(normalized["planning"], "planning_time", 0.1, 120.0)
    _number_range(normalized["planning"], "max_attempts", 1, 20, integer=True)
    if not isinstance(normalized["constraints"]["avoid_collision"], bool):
        raise MoveValidationError("constraints.avoid_collision must be boolean.")
    if not isinstance(normalized["constraints"]["keep_end_effector_orientation"], bool):
        raise MoveValidationError("constraints.keep_end_effector_orientation must be boolean.")
    for key in ("velocity_scale", "acceleration_scale"):
        _number_range(normalized["constraints"], key, 0.01, 1.0)
    _number_range(normalized["constraints"], "position_tolerance", 0.0, 0.1)
    _number_range(normalized["constraints"], "orientation_tolerance", 0.0, 180.0)
    _number_range(normalized["constraints"], "timeout", 0.1, 600.0)
    if "safety_distance" in normalized["constraints"]:
        _number_range(normalized["constraints"], "safety_distance", 0.0, 1.0)

    if target_type == "joint":
        joints = target["joint_positions"]
        if not isinstance(joints, (list, tuple)) or len(joints) != 6:
            raise MoveValidationError("target.joint_positions must contain 6 values.")
        normalized["target"]["joint_positions"] = [_finite(value, "target.joint_positions") for value in joints]
    elif target_type == "pose":
        normalized["target"]["position"] = _position(target["position"], "target.position")
        orientation = target.get("orientation")
        if orientation is not None:
            normalized["target"]["orientation"] = _orientation(orientation)
        frame = str(target.get("frame", "base"))
        if frame not in {"base", "world"}:
            raise MoveValidationError("Only base/world pose frames are currently supported.")
        normalized["target"]["frame"] = frame

    if normalized["motion"]["path_type"] == "circular":
        params = normalized["strategy_params"]
        if params.get("via_point") is None and not all(params.get(k) is not None for k in ("arc_center", "arc_axis", "arc_angle")):
            raise MoveValidationError("Circular motion requires via_point or arc_center/arc_axis/arc_angle.")
    ik_method = normalized["strategy_params"].get("ik_method", "auto")
    if ik_method not in {"auto", "ikfast", "trac_ik"}:
        raise MoveValidationError("strategy_params.ik_method must be auto, ikfast, or trac_ik.")
    if "cartesian_step" in normalized["strategy_params"]:
        _number_range(normalized["strategy_params"], "cartesian_step", 0.001, 0.1)
    if "ik_seed" in normalized["strategy_params"]:
        seed = normalized["strategy_params"]["ik_seed"]
        if not isinstance(seed, (list, tuple)) or len(seed) != 6:
            raise MoveValidationError("strategy_params.ik_seed must contain 6 values.")
        normalized["strategy_params"]["ik_seed"] = [_finite(value, "strategy_params.ik_seed") for value in seed]
    return normalized


def _finite(value: Any, field: str) -> float:
    import math
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise MoveValidationError(f"{field} must contain finite numbers.")
    return float(value)


def _position(value: Any, field: str) -> Dict[str, float]:
    if isinstance(value, Mapping) and all(key in value for key in ("x", "y", "z")):
        return {key: _finite(value[key], field) for key in ("x", "y", "z")}
    if isinstance(value, (list, tuple)) and len(value) == 3:
        return {key: _finite(item, field) for key, item in zip(("x", "y", "z"), value)}
    raise MoveValidationError(f"{field} must contain x, y, z.")


def _orientation(value: Any) -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        raise MoveValidationError("target.orientation must be an object.")
    representation = value.get("representation", "quaternion")
    if representation == "quaternion":
        result = {key: _finite(value.get(key), "target.orientation") for key in ("x", "y", "z", "w")}
        norm = sum(component * component for component in result.values()) ** 0.5
        if norm < 1e-9:
            raise MoveValidationError("Quaternion must have non-zero norm.")
        result = {key: component / norm for key, component in result.items()}
    elif representation == "rpy":
        result = {key: _finite(value.get(key), "target.orientation") for key in ("roll", "pitch", "yaw")}
    else:
        raise MoveValidationError("orientation.representation must be quaternion or rpy.")
    result["representation"] = representation
    return result


def _enum(container: Mapping[str, Any], key: str, allowed: set[str]) -> None:
    if container.get(key) not in allowed:
        raise MoveValidationError(f"{key} must be one of: {', '.join(sorted(allowed))}.")


def _number_range(container: Mapping[str, Any], key: str, lower: float, upper: float, *, integer: bool = False) -> None:
    value = container.get(key)
    if integer and (isinstance(value, bool) or not isinstance(value, int)):
        raise MoveValidationError(f"{key} must be an integer.")
    number = _finite(value, key)
    if not lower <= number <= upper:
        raise MoveValidationError(f"{key} must be between {lower} and {upper}.")
