"""Release precondition and result validation."""

from __future__ import annotations

import math
from typing import Any, Mapping

from skills.manipulation.release.utils.error_codes import ErrorCode


class ReleaseExecutionValidationError(RuntimeError):
    def __init__(self, message: str, *, error_code: ErrorCode | str, failed_stage: str, recoverable: bool, recommended_action: str = "Inspect the gripper and payload state.") -> None:
        super().__init__(message)
        self.error_code = error_code
        self.failed_stage = failed_stage
        self.recoverable = recoverable
        self.recommended_action = recommended_action


class ReleaseValidator:
    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def validate_preconditions(self, request: Mapping[str, Any], context: Mapping[str, Any], selection: Mapping[str, Any]) -> None:
        controller_state = context.get("gripper_controller_state", {})
        safety = context.get("safety_state", {})
        state = context.get("current_gripper_state", {})
        if not controller_state.get("ready") or controller_state.get("fault"):
            self._fail("Gripper controller is not ready.", ErrorCode.GRIPPER_NOT_READY, "preconditions", True)
        if not state.get("known") or not state.get("initialized"):
            self._fail("Gripper state is unknown or uninitialized.", ErrorCode.GRIPPER_NOT_READY, "preconditions", True)
        if not safety.get("safe") or safety.get("emergency_stop"):
            self._fail("Robot safety state prevents release execution.", ErrorCode.GRIPPER_NOT_READY, "preconditions", False)
        if request["release"]["operation"] == "release" and not state.get("holding"):
            self._fail(
                f"Target '{request['target']['object_id']}' is not held by the gripper.",
                ErrorCode.OBJECT_NOT_HELD, "preconditions", True,
                "Grasp the target first or correct target.object_id, then retry.",
            )
        defaults = self.config.get("strategies", {}).get("default_release", {}).get("defaults", {})
        params = {**defaults, **request.get("strategy_params", {})}
        width = float(params.get("open_width", 0.08))
        limits = context["gripper_limits"]
        if not float(limits["minimum_width"]) <= width <= float(limits["maximum_width"]):
            self._fail("open_width exceeds the gripper limits.", ErrorCode.INVALID_GRIPPER_PARAMETERS, "preconditions", True)

    def validate_result(self, raw_result: Mapping[str, Any], request: Mapping[str, Any], context: Mapping[str, Any]) -> Mapping[str, Any]:
        width = float(raw_result.get("final_width", float("nan")))
        execution_time = float(raw_result.get("execution_time", float("nan")))
        if not math.isfinite(width) or not math.isfinite(execution_time):
            self._fail("Release result contains non-finite values.", ErrorCode.GRIPPER_EXECUTION_ERROR, "verification", False)
        holding = bool(raw_result.get("holding", True))
        present = bool(raw_result.get("object_present", True))
        released = not holding
        open_only = request["release"]["operation"] == "open"
        # A placed object may remain in passive contact with an open finger pad.
        # Kinematic detachment plus an open gripper is sufficient release
        # evidence; ``object_present`` alone must not make placement flaky.
        evidence_verified = released
        verify = bool(request["constraints"]["verify_release"]) and request["release"]["verification_mode"] != "disabled"
        if verify and not open_only and not evidence_verified:
            self._fail("The target still appears to be held or present between the fingers after opening.", ErrorCode.RELEASE_VERIFICATION_FAILED, "verification", True)
        params = raw_result.get("parameters", {})
        target_width = float(params.get("open_width", width))
        tolerance = float(params.get("position_tolerance", 0.002))
        width_reached = abs(width - target_width) <= tolerance
        if verify and not width_reached:
            self._fail("The gripper did not reach the requested release width.", ErrorCode.RELEASE_VERIFICATION_FAILED, "verification", True)
        return {
            "released": released,
            "final_width": width,
            "holding": holding,
            "object_present": present,
            "verified": width_reached if open_only else evidence_verified and width_reached,
            "verification_method": "gripper_position" if open_only else "holding_presence_and_position" if verify else "disabled",
            "attempts": int(raw_result.get("attempts", 1)),
            "evidence": dict(raw_result.get("evidence", {})),
            "execution_time": execution_time,
        }

    @staticmethod
    def _fail(message: str, code: ErrorCode, stage: str, recoverable: bool, recommended_action: str = "Inspect the gripper and payload state.") -> None:
        raise ReleaseExecutionValidationError(message, error_code=code, failed_stage=stage, recoverable=recoverable, recommended_action=recommended_action)
