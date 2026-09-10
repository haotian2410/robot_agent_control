"""Unified Grasp precondition and result validation skeleton."""

from __future__ import annotations

import math
from typing import Any, Mapping

from skills.manipulation.grasp.utils.error_codes import ErrorCode


class GraspExecutionValidationError(RuntimeError):
    """Raised when grasp preconditions or result evidence are invalid."""

    def __init__(self, message: str, *, error_code: ErrorCode | str, failed_stage: str, recoverable: bool, recommended_action: str = "Inspect the gripper state and request.") -> None:
        super().__init__(message)
        self.error_code = error_code
        self.failed_stage = failed_stage
        self.recoverable = recoverable
        self.recommended_action = recommended_action


class GraspValidator:
    """Validate gripper readiness, parameter limits and grasp evidence."""

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def validate_preconditions(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
        selection: Mapping[str, Any],
    ) -> None:
        """Validate readiness, limits and simulated evidence availability."""
        controller_state = context.get("gripper_controller_state", {})
        safety = context.get("safety_state", {})
        state = context.get("current_gripper_state", {})
        if not controller_state.get("ready") or controller_state.get("fault"):
            self._fail("Gripper controller is not ready.", ErrorCode.GRIPPER_NOT_READY, "preconditions", True)
        if not state.get("known") or not state.get("initialized"):
            self._fail("Gripper state is unknown or uninitialized.", ErrorCode.GRIPPER_NOT_READY, "preconditions", True)
        if not safety.get("safe") or safety.get("emergency_stop"):
            self._fail("Robot safety state prevents grasp execution.", ErrorCode.GRIPPER_NOT_READY, "preconditions", False)

        defaults = self.config.get("strategies", {}).get("default_grasp", {}).get("defaults", {})
        params = {**defaults, **request.get("strategy_params", {})}
        limits = context["gripper_limits"]
        open_width = float(params.get("open_width", 0.08))
        close_width = float(params.get("close_width", 0.0))
        maximum_width = min(float(request["constraints"].get("maximum_width", limits["maximum_width"])), float(limits["maximum_width"]))
        if not 0.0 <= close_width <= open_width <= maximum_width:
            self._fail("Gripper widths must satisfy 0 <= close_width <= open_width <= maximum_width.", ErrorCode.INVALID_GRIPPER_PARAMETERS, "preconditions", True)
        for key in ("grasp_force", "hold_force"):
            force = float(params.get(key, 0.0))
            maximum_force = min(float(request["constraints"]["maximum_force"]), float(limits["maximum_force"]))
            if not 0.0 <= force <= maximum_force:
                self._fail(f"{key} exceeds the permitted force range.", ErrorCode.FORCE_LIMIT_EXCEEDED, "preconditions", True)
        expected_width = request["target"].get("expected_width")
        if expected_width is not None and float(expected_width) > maximum_width:
            self._fail("Target width exceeds gripper capacity.", ErrorCode.TARGET_NOT_GRASPABLE, "preconditions", True)
        mode = request["grasp"]["verification_mode"]
        if mode in {"force", "tactile", "visual"}:
            self._fail(f"Verification mode '{mode}' is unavailable in the current MuJoCo adapter.", ErrorCode.SENSOR_DATA_UNAVAILABLE, "preconditions", True)

    def validate_result(
        self,
        raw_result: Mapping[str, Any],
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """Verify and normalize contact, object-presence and holding evidence."""
        contact = bool(raw_result.get("contact_detected", False))
        present = bool(raw_result.get("object_present", False))
        slip = bool(raw_result.get("slip_detected", False))
        force = float(raw_result.get("final_force", 0.0))
        if force > float(request["constraints"]["maximum_force"]) + 1e-9:
            self._fail("Measured grasp force exceeds the request limit.", ErrorCode.FORCE_LIMIT_EXCEEDED, "verification", False)
        if slip and request["constraints"]["slip_check"]:
            self._fail("Object slip was detected.", ErrorCode.OBJECT_SLIPPED, "verification", True)
        verify = bool(request["constraints"]["verify_grasp"])
        if verify and request["constraints"]["contact_required"] and not contact:
            self._fail("No target contact was detected.", ErrorCode.CONTACT_NOT_DETECTED, "verification", True)
        if verify and request["target"]["type"] == "object" and not present:
            self._fail("Target presence could not be confirmed between the fingers.", ErrorCode.GRASP_VERIFICATION_FAILED, "verification", True)
        expected_width = request["target"].get("expected_width")
        width_ok = True
        if expected_width is not None and contact:
            tolerance = max(float(raw_result.get("parameters", {}).get("position_tolerance", 0.002)), 0.005)
            width_ok = abs(float(raw_result["final_width"]) - float(expected_width)) <= tolerance
        verified = (not verify) or (contact and present and not slip and width_ok)
        if verify and not verified:
            self._fail("Grasp evidence did not satisfy width/contact verification.", ErrorCode.GRASP_VERIFICATION_FAILED, "verification", True)
        if not all(math.isfinite(float(raw_result.get(k, 0.0))) for k in ("final_width", "final_force", "execution_time")):
            self._fail("Grasp result contains non-finite values.", ErrorCode.GRIPPER_EXECUTION_ERROR, "verification", False)
        return {
            "grasped": bool(contact and present),
            "contact_detected": contact,
            "final_width": float(raw_result.get("final_width", 0.0)),
            "final_force": force,
            "holding": bool(raw_result.get("holding", False)),
            "verified": verified,
            "verification_method": "contact_position" if verify else "disabled",
            "slip_detected": slip,
            "attempts": int(raw_result.get("attempts", 1)),
            "evidence": dict(raw_result.get("evidence", {})),
            "execution_time": float(raw_result.get("execution_time", 0.0)),
        }

    @staticmethod
    def _fail(message: str, code: ErrorCode, stage: str, recoverable: bool) -> None:
        raise GraspExecutionValidationError(message, error_code=code, failed_stage=stage, recoverable=recoverable)
