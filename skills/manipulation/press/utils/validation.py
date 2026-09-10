"""Press request validation and normalization."""

from __future__ import annotations

import math
from copy import deepcopy
from typing import Any, Dict, Mapping

import numpy as np


class PressValidationError(ValueError):
    """Raised when a Press request violates the public or strategy schema."""


def validate_press_request(request: Mapping[str, Any], config: Mapping[str, Any] | None = None) -> Dict[str, Any]:
    if not isinstance(request, Mapping):
        raise PressValidationError("Press request must be a mapping/object.")
    normalized = deepcopy(dict(request))
    target = normalized.get("target")
    if not isinstance(target, Mapping):
        raise PressValidationError("target is required and must be an object.")
    normalized["target"] = dict(target)
    if target.get("type") not in {"press_pose", "surface_point", "button"}:
        raise PressValidationError("target.type must be press_pose, surface_point, or button.")

    press = normalized.setdefault("press", {})
    if not isinstance(press, Mapping):
        raise PressValidationError("press must be an object.")
    normalized["press"] = dict(press)
    defaults = {"strategy": "auto", "control_mode": "auto", "operation": "press", "direction_source": "target", "retract_after_press": True, "verification_mode": "auto"}
    for key, value in defaults.items():
        normalized["press"].setdefault(key, value)
    _enum(normalized["press"], "strategy", {"auto", "displacement_press", "force_controlled_press"})
    _enum(normalized["press"], "control_mode", {"auto", "displacement", "force"})
    _enum(normalized["press"], "operation", {"press", "verify_only"})
    _enum(normalized["press"], "direction_source", {"target", "surface_normal", "specified"})
    _enum(normalized["press"], "verification_mode", {"auto", "force_profile", "position", "digital_io", "visual", "combined"})
    if not isinstance(normalized["press"]["retract_after_press"], bool):
        raise PressValidationError("press.retract_after_press must be boolean.")

    constraints = normalized.setdefault("constraints", {})
    if not isinstance(constraints, Mapping):
        raise PressValidationError("constraints must be an object.")
    normalized["constraints"] = dict(constraints)
    constraint_defaults = {
        "maximum_force": 30.0, "maximum_travel": 0.03, "contact_threshold": 2.0,
        "approach_speed": 0.01, "press_speed": 0.01, "retract_speed": 0.02,
        "hold_time": 0.5, "verify_press": True, "retry_count": 0, "timeout": 10.0,
        "force_tolerance": 1.0, "position_tolerance": 0.001,
        "require_contact": True, "emergency_retract": True,
    }
    for key, value in constraint_defaults.items():
        normalized["constraints"].setdefault(key, value)
    for key in ("maximum_force", "maximum_travel", "contact_threshold", "hold_time", "force_tolerance", "position_tolerance"):
        normalized["constraints"][key] = _number(normalized["constraints"][key], f"constraints.{key}", minimum=0.0)
    for key in ("approach_speed", "press_speed", "retract_speed", "timeout"):
        normalized["constraints"][key] = _number(normalized["constraints"][key], f"constraints.{key}", minimum=0.0, exclusive=True)
    if normalized["constraints"]["contact_threshold"] > normalized["constraints"]["maximum_force"]:
        raise PressValidationError("constraints.contact_threshold must not exceed maximum_force.")
    for key in ("verify_press", "require_contact", "emergency_retract"):
        if not isinstance(normalized["constraints"][key], bool):
            raise PressValidationError(f"constraints.{key} must be boolean.")
    retry_count = normalized["constraints"]["retry_count"]
    if not isinstance(retry_count, int) or isinstance(retry_count, bool) or not 0 <= retry_count <= 10:
        raise PressValidationError("constraints.retry_count must be an integer in [0, 10].")
    if "retract_distance" in normalized["constraints"]:
        normalized["constraints"]["retract_distance"] = _number(normalized["constraints"]["retract_distance"], "constraints.retract_distance", minimum=0.0)

    params = normalized.setdefault("strategy_params", {})
    if not isinstance(params, Mapping):
        raise PressValidationError("strategy_params must be an object.")
    normalized["strategy_params"] = dict(params)
    for key in ("press_depth", "travel_distance", "post_contact_depth", "target_force", "contact_force", "force_ramp_rate", "force_settle_time", "overshoot_limit"):
        if key in normalized["strategy_params"]:
            normalized["strategy_params"][key] = _number(normalized["strategy_params"][key], f"strategy_params.{key}", minimum=0.0)
    for key in ("stop_on_contact", "allow_force_overshoot"):
        if key in normalized["strategy_params"] and not isinstance(normalized["strategy_params"][key], bool):
            raise PressValidationError(f"strategy_params.{key} must be boolean.")
    for key in ("press_depth", "travel_distance", "post_contact_depth"):
        if normalized["strategy_params"].get(key, 0.0) > normalized["constraints"]["maximum_travel"]:
            raise PressValidationError(f"strategy_params.{key} must not exceed constraints.maximum_travel.")
    target_force = normalized["strategy_params"].get("target_force")
    if target_force is not None and target_force > normalized["constraints"]["maximum_force"]:
        raise PressValidationError("strategy_params.target_force must not exceed constraints.maximum_force.")
    contact_force = normalized["strategy_params"].get("contact_force")
    if contact_force is not None and target_force is not None and contact_force > target_force:
        raise PressValidationError("strategy_params.contact_force must not exceed target_force.")

    source = normalized["press"]["direction_source"]
    raw_direction = target.get("press_direction") if source == "target" else target.get("surface_normal") if source == "surface_normal" else normalized["strategy_params"].get("press_direction")
    if raw_direction is None:
        raise PressValidationError(f"press.direction_source={source} requires a corresponding direction vector.")
    resolved_direction = _vector(raw_direction, "press direction")
    if source == "surface_normal":
        resolved_direction = -resolved_direction
    normalized["resolved_press_direction"] = resolved_direction.tolist()
    frame = str(target.get("direction_frame", "world"))
    if frame not in {"world", "base"}:
        raise PressValidationError("target.direction_frame must be world or base.")
    normalized["target"]["direction_frame"] = frame
    if normalized["press"]["operation"] != "verify_only":
        force_requested = normalized["press"]["strategy"] == "force_controlled_press" or normalized["press"]["control_mode"] == "force"
        displacement_requested = normalized["press"]["strategy"] == "displacement_press" or normalized["press"]["control_mode"] == "displacement"
        if force_requested and target_force is None:
            raise PressValidationError("Force-Controlled Press requires strategy_params.target_force.")
        if displacement_requested and normalized["strategy_params"].get("press_depth") is None and normalized["strategy_params"].get("travel_distance") is None:
            raise PressValidationError("Displacement Press requires press_depth or travel_distance.")
    return normalized


def _enum(value: Mapping[str, Any], key: str, allowed: set[str]) -> None:
    if value[key] not in allowed:
        raise PressValidationError(f"{key} must be one of {sorted(allowed)}.")


def _number(value: Any, field: str, *, minimum: float, exclusive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise PressValidationError(f"{field} must be a finite number.")
    number = float(value)
    if number < minimum or exclusive and number <= minimum:
        relation = "positive" if exclusive and minimum == 0 else f">= {minimum}"
        raise PressValidationError(f"{field} must be {relation}.")
    return number


def _vector(value: Any, field: str) -> np.ndarray:
    if isinstance(value, Mapping):
        vector = np.array([value.get(key) for key in ("x", "y", "z")], dtype=float)
    else:
        vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)) or np.linalg.norm(vector) < 1e-9:
        raise PressValidationError(f"{field} must contain three finite non-zero components.")
    return vector / np.linalg.norm(vector)
