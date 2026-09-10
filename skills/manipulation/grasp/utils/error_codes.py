"""Shared structured error codes for Grasp Skill."""

from __future__ import annotations

from enum import Enum
from typing import Any, Dict


class ErrorCode(str, Enum):
    INVALID_REQUEST = "INVALID_REQUEST"
    UNKNOWN_STRATEGY = "UNKNOWN_STRATEGY"
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"
    INVALID_GRIPPER_PARAMETERS = "INVALID_GRIPPER_PARAMETERS"
    GRIPPER_NOT_READY = "GRIPPER_NOT_READY"
    TARGET_NOT_GRASPABLE = "TARGET_NOT_GRASPABLE"
    POSE_ALIGNMENT_FAILED = "POSE_ALIGNMENT_FAILED"
    SENSOR_DATA_UNAVAILABLE = "SENSOR_DATA_UNAVAILABLE"
    GRIPPER_OPEN_FAILED = "GRIPPER_OPEN_FAILED"
    CONTACT_NOT_DETECTED = "CONTACT_NOT_DETECTED"
    FORCE_LIMIT_EXCEEDED = "FORCE_LIMIT_EXCEEDED"
    GRASP_FORCE_NOT_REACHED = "GRASP_FORCE_NOT_REACHED"
    GRASP_VERIFICATION_FAILED = "GRASP_VERIFICATION_FAILED"
    OBJECT_SLIPPED = "OBJECT_SLIPPED"
    GRASP_TIMEOUT = "GRASP_TIMEOUT"
    GRIPPER_EXECUTION_ERROR = "GRIPPER_EXECUTION_ERROR"
    INTERNAL_ERROR = "INTERNAL_ERROR"


def failure_result(
    code: ErrorCode | str,
    message: str,
    *,
    failed_stage: str,
    recoverable: bool,
    recommended_action: str = "Inspect the structured error and runtime context.",
    status: str = "failed",
) -> Dict[str, Any]:
    value = code.value if isinstance(code, ErrorCode) else str(code)
    return {
        "success": False,
        "status": status,
        "selection": None,
        "grasp_result": None,
        "error": {
            "error_code": value,
            "error_message": message,
            "failed_stage": failed_stage,
            "recoverable": recoverable,
            "recommended_action": recommended_action,
        },
    }
