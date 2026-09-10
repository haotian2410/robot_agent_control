"""Shared structured error codes for Press Skill."""

from __future__ import annotations

from enum import Enum
from typing import Any, Dict


class ErrorCode(str, Enum):
    INVALID_REQUEST = "INVALID_REQUEST"
    UNKNOWN_STRATEGY = "UNKNOWN_STRATEGY"
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"
    PRESS_CONTROLLER_NOT_READY = "PRESS_CONTROLLER_NOT_READY"
    INVALID_PRESS_DIRECTION = "INVALID_PRESS_DIRECTION"
    SENSOR_DATA_UNAVAILABLE = "SENSOR_DATA_UNAVAILABLE"
    FORCE_SENSOR_NOT_CALIBRATED = "FORCE_SENSOR_NOT_CALIBRATED"
    CONTACT_NOT_DETECTED = "CONTACT_NOT_DETECTED"
    TARGET_FORCE_NOT_REACHED = "TARGET_FORCE_NOT_REACHED"
    TARGET_DEPTH_NOT_REACHED = "TARGET_DEPTH_NOT_REACHED"
    FORCE_LIMIT_EXCEEDED = "FORCE_LIMIT_EXCEEDED"
    TRAVEL_LIMIT_EXCEEDED = "TRAVEL_LIMIT_EXCEEDED"
    FORCE_CONTROL_UNSTABLE = "FORCE_CONTROL_UNSTABLE"
    TARGET_NOT_ACTUATED = "TARGET_NOT_ACTUATED"
    PRESS_VERIFICATION_FAILED = "PRESS_VERIFICATION_FAILED"
    LOCAL_COLLISION_DETECTED = "LOCAL_COLLISION_DETECTED"
    RETRACT_FAILED = "RETRACT_FAILED"
    PRESS_TIMEOUT = "PRESS_TIMEOUT"
    PRESS_EXECUTION_ERROR = "PRESS_EXECUTION_ERROR"
    INTERNAL_ERROR = "INTERNAL_ERROR"


class PressExecutionError(RuntimeError):
    """Typed expected failure raised by press controllers and validators."""

    def __init__(self, message: str, *, error_code: ErrorCode | str, failed_stage: str, recoverable: bool, recommended_action: str, details: object = None, retracted: bool = False) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.failed_stage = failed_stage
        self.recoverable = recoverable
        self.recommended_action = recommended_action
        self.details = details
        self.retracted = retracted


def failure_result(
    code: ErrorCode | str,
    message: str,
    *,
    failed_stage: str,
    recoverable: bool,
    recommended_action: str = "Inspect the structured error and runtime context.",
    status: str = "failed",
    details: object = None,
) -> Dict[str, Any]:
    value = code.value if isinstance(code, ErrorCode) else str(code)
    result = {
        "success": False,
        "status": status,
        "selection": None,
        "press_result": None,
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
