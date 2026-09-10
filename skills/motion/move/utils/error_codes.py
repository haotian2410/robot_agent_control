"""Shared structured error codes for Move Skill."""

from __future__ import annotations

from enum import Enum
from typing import Any, Dict


class ErrorCode(str, Enum):
    INVALID_REQUEST = "INVALID_REQUEST"
    UNKNOWN_STRATEGY = "UNKNOWN_STRATEGY"
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"
    FRAME_TRANSFORM_FAILED = "FRAME_TRANSFORM_FAILED"
    TARGET_UNREACHABLE = "TARGET_UNREACHABLE"
    IK_FAILED = "IK_FAILED"
    INVALID_ARC_DEFINITION = "INVALID_ARC_DEFINITION"
    JOINT_LIMIT_VIOLATION = "JOINT_LIMIT_VIOLATION"
    SINGULARITY_RISK = "SINGULARITY_RISK"
    COLLISION_DETECTED = "COLLISION_DETECTED"
    PATH_CONSTRAINT_INFEASIBLE = "PATH_CONSTRAINT_INFEASIBLE"
    PLANNING_FAILED = "PLANNING_FAILED"
    PATH_NOT_FOUND = "PATH_NOT_FOUND"
    TRAJECTORY_INVALID = "TRAJECTORY_INVALID"
    CONTROLLER_NOT_READY = "CONTROLLER_NOT_READY"
    EXECUTION_FAILED = "EXECUTION_FAILED"
    EXECUTION_TIMEOUT = "EXECUTION_TIMEOUT"
    GOAL_TOLERANCE_EXCEEDED = "GOAL_TOLERANCE_EXCEEDED"
    SCENE_CHANGED = "SCENE_CHANGED"
    STOPPED_BY_SAFETY = "STOPPED_BY_SAFETY"
    INTERNAL_ERROR = "INTERNAL_ERROR"


def failure_result(
    code: ErrorCode | str,
    message: str,
    *,
    failed_stage: str,
    recoverable: bool,
    recommended_action: str = "Inspect the structured error and runtime state.",
    status: str = "failed",
    details: object = None,
) -> Dict[str, Any]:
    """Build a result that follows the public Move output schema."""
    value = code.value if isinstance(code, ErrorCode) else str(code)
    result = {
        "success": False,
        "status": status,
        "selection": None,
        "execution_result": None,
        "error": {
            "error_code": value,
            "error_message": message,
            "failed_stage": failed_stage,
            "recoverable": recoverable,
            "recommended_action": recommended_action,
        },
    }
    if details is not None:
        result["error"]["details"] = details
    return result
