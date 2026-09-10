"""Shared structured error codes for Search Skill."""

from __future__ import annotations

from enum import Enum
from typing import Any, Dict


class ErrorCode(str, Enum):
    INVALID_REQUEST = "INVALID_REQUEST"
    UNKNOWN_STRATEGY = "UNKNOWN_STRATEGY"
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"
    SEARCH_NOT_REQUIRED = "SEARCH_NOT_REQUIRED"
    OBSERVATION_INSUFFICIENT = "OBSERVATION_INSUFFICIENT"
    SENSOR_DATA_UNAVAILABLE = "SENSOR_DATA_UNAVAILABLE"
    CAMERA_MODEL_UNAVAILABLE = "CAMERA_MODEL_UNAVAILABLE"
    CAMERA_NOT_MOVABLE = "CAMERA_NOT_MOVABLE"
    TARGET_PRIOR_INVALID = "TARGET_PRIOR_INVALID"
    OCCLUSION_EVIDENCE_INSUFFICIENT = "OCCLUSION_EVIDENCE_INSUFFICIENT"
    SEARCH_REGION_INVALID = "SEARCH_REGION_INVALID"
    FRAME_TRANSFORM_FAILED = "FRAME_TRANSFORM_FAILED"
    VIEWPOINT_GENERATION_FAILED = "VIEWPOINT_GENERATION_FAILED"
    LOOK_AT_GENERATION_FAILED = "LOOK_AT_GENERATION_FAILED"
    FIELD_OF_VIEW_INVALID = "FIELD_OF_VIEW_INVALID"
    NO_VISIBILITY_GAIN = "NO_VISIBILITY_GAIN"
    NO_QUALITY_GAIN = "NO_QUALITY_GAIN"
    DUPLICATE_VIEWPOINT = "DUPLICATE_VIEWPOINT"
    TARGET_UNREACHABLE = "TARGET_UNREACHABLE"
    COLLISION_DETECTED = "COLLISION_DETECTED"
    CLEARANCE_INSUFFICIENT = "CLEARANCE_INSUFFICIENT"
    NO_VALID_VIEWPOINT = "NO_VALID_VIEWPOINT"
    SEARCH_EXHAUSTED = "SEARCH_EXHAUSTED"
    SEARCH_TIMEOUT = "SEARCH_TIMEOUT"
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
    """Build a result that follows the public Search output schema."""
    value = code.value if isinstance(code, ErrorCode) else str(code)
    return {
        "success": False,
        "status": status,
        "selection": None,
        "search_result": None,
        "error": {
            "error_code": value,
            "error_message": message,
            "failed_stage": failed_stage,
            "recoverable": recoverable,
            "recommended_action": recommended_action,
        },
    }
