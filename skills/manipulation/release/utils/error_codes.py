"""Structured Release error codes and failure results."""

from __future__ import annotations

from enum import Enum
from typing import Any, Dict


class ErrorCode(str, Enum):
    INVALID_REQUEST = "INVALID_REQUEST"
    UNKNOWN_STRATEGY = "UNKNOWN_STRATEGY"
    INVALID_GRIPPER_PARAMETERS = "INVALID_GRIPPER_PARAMETERS"
    GRIPPER_NOT_READY = "GRIPPER_NOT_READY"
    OBJECT_NOT_HELD = "OBJECT_NOT_HELD"
    RELEASE_VERIFICATION_FAILED = "RELEASE_VERIFICATION_FAILED"
    RELEASE_TIMEOUT = "RELEASE_TIMEOUT"
    GRIPPER_EXECUTION_ERROR = "GRIPPER_EXECUTION_ERROR"
    INTERNAL_ERROR = "INTERNAL_ERROR"


def failure_result(code: ErrorCode | str, message: str, *, failed_stage: str, recoverable: bool, recommended_action: str = "Inspect the release request and gripper state.", status: str = "failed") -> Dict[str, Any]:
    value = code.value if isinstance(code, ErrorCode) else str(code)
    return {
        "success": False, "status": status, "selection": None, "release_result": None,
        "error": {"error_code": value, "error_message": message, "failed_stage": failed_stage, "recoverable": recoverable, "recommended_action": recommended_action},
    }
