"""Validation and normalization for Push/Pull requests."""
from __future__ import annotations

import math
from copy import deepcopy
from typing import Any, Dict, Mapping


class PushPullValidationError(ValueError):
    pass


def _number(value: Any, field: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise PushPullValidationError(f"{field} must be a finite number.")
    value = float(value)
    if value < 0.0 or positive and value <= 0.0:
        raise PushPullValidationError(f"{field} must be {'positive' if positive else 'non-negative'}.")
    return value


def _unit_vector(value: Any, field: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise PushPullValidationError(f"{field} must contain three numbers.")
    vector = [float(v) for v in value]
    if not all(math.isfinite(v) for v in vector):
        raise PushPullValidationError(f"{field} must be finite.")
    norm = math.sqrt(sum(v * v for v in vector))
    if norm <= 1e-12:
        raise PushPullValidationError(f"{field} must not be zero.")
    return [v / norm for v in vector]


def validate_push_pull_request(request: Mapping[str, Any], config: Mapping[str, Any] | None = None) -> Dict[str, Any]:
    if not isinstance(request, Mapping):
        raise PushPullValidationError("Push/Pull request must be an object.")
    result = deepcopy(dict(request))
    target = result.get("target")
    if not isinstance(target, Mapping) or target.get("type") not in {"object", "mechanism"} or not target.get("object_id"):
        raise PushPullValidationError("target.type and target.object_id are required.")
    result["target"] = dict(target)
    action = result.get("manipulation", {})
    if not isinstance(action, Mapping):
        raise PushPullValidationError("manipulation must be an object.")
    result["manipulation"] = {
        "operation": "push",
        "trajectory": "auto",
        "interaction_mode": "auto",
        "maintain_contact": True,
        "verification_mode": "auto",
        **action,
    }
    if result["manipulation"]["operation"] not in {"push", "pull", "verify_only"}:
        raise PushPullValidationError("manipulation.operation must be push, pull, or verify_only.")
    if result["manipulation"]["trajectory"] not in {"auto", "linear", "circular"}:
        raise PushPullValidationError("manipulation.trajectory must be auto, linear, or circular.")
    if result["manipulation"]["interaction_mode"] not in {"auto", "mechanism", "free_object"}:
        raise PushPullValidationError("manipulation.interaction_mode must be auto, mechanism, or free_object.")
    if not isinstance(result["manipulation"]["maintain_contact"], bool):
        raise PushPullValidationError("manipulation.maintain_contact must be boolean.")
    defaults = {"maximum_force": 40.0, "maximum_travel": 0.5, "maximum_angle": math.pi, "contact_threshold": 1.0, "speed": 0.03, "angular_speed": 0.2, "timeout": 30.0, "normal_alignment_tolerance": 1.0, "avoid_collision": True, "verify_motion": True, "stop_on_contact_loss": True, "emergency_retract": True}
    constraints = result.get("constraints", {})
    if not isinstance(constraints, Mapping):
        raise PushPullValidationError("constraints must be an object.")
    result["constraints"] = {**defaults, **constraints}
    for key in ("maximum_force", "maximum_travel", "maximum_angle", "contact_threshold"):
        result["constraints"][key] = _number(result["constraints"][key], f"constraints.{key}")
    for key in ("speed", "angular_speed", "timeout"):
        result["constraints"][key] = _number(result["constraints"][key], f"constraints.{key}", positive=True)
    result["constraints"]["normal_alignment_tolerance"] = _number(result["constraints"]["normal_alignment_tolerance"], "constraints.normal_alignment_tolerance")
    if result["constraints"]["normal_alignment_tolerance"] > 10.0:
        raise PushPullValidationError("normal_alignment_tolerance must not exceed 10 degrees.")
    if result["constraints"].get("avoid_collision") is not True:
        raise PushPullValidationError("Push/Pull requires constraints.avoid_collision=true.")
    if result["constraints"]["contact_threshold"] > result["constraints"]["maximum_force"]:
        raise PushPullValidationError("contact_threshold must not exceed maximum_force.")
    params = result.get("strategy_params", {})
    if not isinstance(params, Mapping):
        raise PushPullValidationError("strategy_params must be an object.")
    result["strategy_params"] = dict(params)
    carried_objects = result["strategy_params"].get("carried_objects", [])
    if not isinstance(carried_objects, list) or not all(isinstance(name, str) and name for name in carried_objects):
        raise PushPullValidationError("strategy_params.carried_objects must be a list of object names.")
    result["strategy_params"]["carried_objects"] = carried_objects
    interaction_mode = result["manipulation"]["interaction_mode"]
    if interaction_mode == "auto":
        interaction_mode = "free_object" if target["type"] == "object" else "mechanism"
    if interaction_mode == "free_object" and target["type"] != "object":
        raise PushPullValidationError("free_object interaction requires target.type=object.")
    if interaction_mode == "mechanism" and target["type"] != "mechanism":
        raise PushPullValidationError("mechanism interaction requires target.type=mechanism.")
    if interaction_mode == "free_object" and result["manipulation"]["operation"] == "pull":
        raise PushPullValidationError("A non-grasped free object supports push only; pull requires a grasped mechanism.")
    if interaction_mode == "free_object" and carried_objects:
        raise PushPullValidationError("carried_objects is only valid for mechanism interaction.")
    result["manipulation"]["interaction_mode"] = interaction_mode
    result["resolved_interaction_mode"] = interaction_mode
    trajectory = result["manipulation"]["trajectory"]
    if trajectory == "auto":
        trajectory = "circular" if any(k in params for k in ("arc_center", "arc_axis", "arc_angle")) else "linear"
        result["manipulation"]["trajectory"] = trajectory
    if interaction_mode == "free_object" and trajectory != "linear":
        raise PushPullValidationError("Free-object pushing supports a hard linear trajectory only.")
    if result["manipulation"]["operation"] != "verify_only":
        result["resolved_contact_normal"] = _unit_vector(target.get("contact_normal"), "target.contact_normal")
        normal_frame = str(target.get("normal_frame", "world"))
        if normal_frame not in {"world", "base"}:
            raise PushPullValidationError("target.normal_frame must be world or base.")
        result["target"]["normal_frame"] = normal_frame
        if trajectory == "linear":
            result["strategy_params"]["direction"] = _unit_vector(params.get("direction"), "strategy_params.direction")
            if interaction_mode == "free_object":
                direction = result["strategy_params"]["direction"]
                normal = result["resolved_contact_normal"]
                cosine = sum(a * b for a, b in zip(direction, normal))
                tolerance = math.cos(math.radians(result["constraints"]["normal_alignment_tolerance"]))
                if cosine > -tolerance:
                    raise PushPullValidationError(
                        "Free-object push direction must be opposite and collinear with the outward contact normal."
                    )
            distance = _number(params.get("distance"), "strategy_params.distance", positive=True)
            if distance > result["constraints"]["maximum_travel"]:
                raise PushPullValidationError("distance exceeds maximum_travel.")
            result["strategy_params"]["distance"] = distance
        else:
            if not isinstance(params.get("arc_center"), (list, tuple)) or len(params["arc_center"]) != 3:
                raise PushPullValidationError("circular motion requires a three-value arc_center.")
            result["strategy_params"]["arc_center"] = [float(v) for v in params["arc_center"]]
            result["strategy_params"]["arc_axis"] = _unit_vector(params.get("arc_axis"), "strategy_params.arc_axis")
            angle = _number(abs(params.get("arc_angle", 0.0)), "strategy_params.arc_angle", positive=True)
            if angle > result["constraints"]["maximum_angle"]:
                raise PushPullValidationError("arc_angle exceeds maximum_angle.")
            result["strategy_params"]["arc_angle"] = math.copysign(angle, float(params["arc_angle"]))
    return result
