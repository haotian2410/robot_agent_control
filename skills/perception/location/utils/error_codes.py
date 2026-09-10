"""Shared structured error codes for Locate Skill."""

from __future__ import annotations

from enum import Enum
from typing import Any, Dict


class ErrorCode(str, Enum):
    INVALID_REQUEST = "INVALID_REQUEST"
    UNKNOWN_STRATEGY = "UNKNOWN_STRATEGY"
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"
    TARGET_NOT_FOUND = "TARGET_NOT_FOUND"
    TARGET_DESCRIPTION_AMBIGUOUS = "TARGET_DESCRIPTION_AMBIGUOUS"
    MODEL_NOT_FOUND = "MODEL_NOT_FOUND"
    INSTANCE_AMBIGUOUS = "INSTANCE_AMBIGUOUS"
    SENSOR_DATA_UNAVAILABLE = "SENSOR_DATA_UNAVAILABLE"
    FRAME_TRANSFORM_FAILED = "FRAME_TRANSFORM_FAILED"
    REFERENCE_INVALID = "REFERENCE_INVALID"
    DIRECTION_UNRESOLVED = "DIRECTION_UNRESOLVED"
    SEMANTIC_DIRECTION_AMBIGUOUS = "SEMANTIC_DIRECTION_AMBIGUOUS"
    INVALID_OFFSET = "INVALID_OFFSET"
    REGISTRATION_FAILED = "REGISTRATION_FAILED"
    SEGMENTATION_FAILED = "SEGMENTATION_FAILED"
    DEPTH_UNAVAILABLE = "DEPTH_UNAVAILABLE"
    INSUFFICIENT_POINTS = "INSUFFICIENT_POINTS"
    PROJECTION_FAILED = "PROJECTION_FAILED"
    ORIENTATION_ESTIMATION_FAILED = "ORIENTATION_ESTIMATION_FAILED"
    POSE_ESTIMATION_FAILED = "POSE_ESTIMATION_FAILED"
    GRASP_POSE_PREDICTION_FAILED = "GRASP_POSE_PREDICTION_FAILED"
    LOW_CONFIDENCE = "LOW_CONFIDENCE"
    TARGET_UNREACHABLE = "TARGET_UNREACHABLE"
    COLLISION_DETECTED = "COLLISION_DETECTED"
    CLEARANCE_INSUFFICIENT = "CLEARANCE_INSUFFICIENT"
    NO_VALID_CANDIDATE = "NO_VALID_CANDIDATE"
    POSE_VALIDATION_FAILED = "POSE_VALIDATION_FAILED"
    LOCALIZATION_TIMEOUT = "LOCALIZATION_TIMEOUT"
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
    """Build a result that follows the public Locate output schema."""
    value = code.value if isinstance(code, ErrorCode) else str(code)
    return {
        "success": False,
        "status": status,
        "selection": None,
        "localization_result": None,
        "error": {
            "error_code": value,
            "error_message": message,
            "failed_stage": failed_stage,
            "recoverable": recoverable,
            "recommended_action": recommended_action,
        },
    }
