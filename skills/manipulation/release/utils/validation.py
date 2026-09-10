"""Release request validation and normalization."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, Mapping


class ReleaseValidationError(ValueError):
    """Raised when a Release request violates its public schema."""


def validate_release_request(request: Mapping[str, Any], config: Mapping[str, Any] | None = None) -> Dict[str, Any]:
    if not isinstance(request, Mapping):
        raise ReleaseValidationError("Release request must be a mapping/object.")
    normalized = deepcopy(dict(request))
    target = normalized.get("target")
    if not isinstance(target, Mapping) or target.get("type") != "object" or not target.get("object_id"):
        raise ReleaseValidationError("target.type=object and target.object_id are required.")
    normalized["target"] = dict(target)

    release = normalized.setdefault("release", {})
    if not isinstance(release, Mapping):
        raise ReleaseValidationError("release must be an object.")
    normalized["release"] = dict(release)
    normalized["release"].setdefault("strategy", "auto")
    normalized["release"].setdefault("operation", "release")
    normalized["release"].setdefault("verification_mode", "auto")
    if normalized["release"]["strategy"] not in {"auto", "default_release"}:
        raise ReleaseValidationError("release.strategy must be auto or default_release.")
    if normalized["release"]["operation"] not in {"release", "open", "verify_only"}:
        raise ReleaseValidationError("release.operation must be release, open, or verify_only.")
    if normalized["release"]["verification_mode"] not in {"auto", "state", "disabled"}:
        raise ReleaseValidationError("release.verification_mode must be auto, state, or disabled.")

    constraints = normalized.setdefault("constraints", {})
    if not isinstance(constraints, Mapping):
        raise ReleaseValidationError("constraints must be an object.")
    normalized["constraints"] = dict(constraints)
    normalized["constraints"].setdefault("verify_release", True)
    normalized["constraints"].setdefault("retry_count", 0)
    normalized["constraints"].setdefault("timeout", 5.0)
    if not isinstance(normalized["constraints"]["verify_release"], bool):
        raise ReleaseValidationError("constraints.verify_release must be boolean.")
    retry_count = normalized["constraints"]["retry_count"]
    if not isinstance(retry_count, int) or isinstance(retry_count, bool) or not 0 <= retry_count <= 10:
        raise ReleaseValidationError("constraints.retry_count must be an integer in [0, 10].")
    timeout = normalized["constraints"]["timeout"]
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or timeout <= 0:
        raise ReleaseValidationError("constraints.timeout must be positive.")
    normalized["constraints"]["timeout"] = float(timeout)

    params = normalized.setdefault("strategy_params", {})
    if not isinstance(params, Mapping):
        raise ReleaseValidationError("strategy_params must be an object.")
    normalized["strategy_params"] = dict(params)
    # Preserve compatibility with the original public request shape.
    if "open_width" in normalized["constraints"] and "open_width" not in normalized["strategy_params"]:
        normalized["strategy_params"]["open_width"] = normalized["constraints"]["open_width"]
    for key in ("open_width", "position_tolerance", "settle_time"):
        if key in normalized["strategy_params"]:
            value = normalized["strategy_params"][key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0:
                raise ReleaseValidationError(f"strategy_params.{key} must be a non-negative number.")
            normalized["strategy_params"][key] = float(value)
    speed = normalized["strategy_params"].get("open_speed")
    if speed is not None and (isinstance(speed, bool) or not isinstance(speed, (int, float)) or not 0 < float(speed) <= 1):
        raise ReleaseValidationError("strategy_params.open_speed must be in (0, 1].")
    if speed is not None:
        normalized["strategy_params"]["open_speed"] = float(speed)
    return normalized
