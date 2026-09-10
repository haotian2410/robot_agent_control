"""Press precondition and result validation."""

from __future__ import annotations

import math
from typing import Any, Mapping

import numpy as np

from skills.manipulation.press.utils.error_codes import ErrorCode, PressExecutionError


class PressExecutionValidationError(PressExecutionError):
    """Typed validation failure for Press execution."""


class PressValidator:
    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def validate_preconditions(self, request: Mapping[str, Any], context: Mapping[str, Any], selection: Mapping[str, Any]) -> None:
        controller = context.get("contact_controller_state", {})
        safety = context.get("safety_state", {})
        if not controller.get("ready") or controller.get("fault"):
            self._fail("Press controller is not ready.", ErrorCode.PRESS_CONTROLLER_NOT_READY, "preconditions", True, "Enable and initialize the local contact controller, then retry.")
        if not safety.get("safe") or safety.get("emergency_stop"):
            self._fail("Robot safety state prevents press execution.", ErrorCode.PRESS_CONTROLLER_NOT_READY, "preconditions", False, "Clear the safety stop only after inspecting the robot and contact area.")
        direction = np.asarray(request.get("resolved_press_direction"), dtype=float)
        if direction.shape != (3,) or not np.all(np.isfinite(direction)) or abs(np.linalg.norm(direction) - 1.0) > 1e-6:
            self._fail("Press direction is invalid or not normalized.", ErrorCode.INVALID_PRESS_DIRECTION, "preconditions", True, "Provide a valid world/base-frame press direction from target geometry.")
        local_collision = context.get("local_collision_state", {})
        if local_collision.get("in_collision"):
            self._fail("The pre-press state is already in local collision.", ErrorCode.LOCAL_COLLISION_DETECTED, "preconditions", True, "Use Move Skill to reach a collision-free pre-press pose.", details=local_collision)
        if selection["press_strategy"] == "force_controlled_press":
            force = context.get("force_torque_state", {})
            tactile = context.get("tactile_state", {})
            if not (force.get("available") and force.get("calibrated") or tactile.get("available")):
                self._fail("Calibrated force or tactile feedback is unavailable.", ErrorCode.SENSOR_DATA_UNAVAILABLE, "preconditions", True, "Calibrate or enable force/tactile feedback; do not silently use displacement control.")
        mode = request["press"]["verification_mode"]
        if mode == "digital_io" and not context.get("digital_io_state", {}).get("available"):
            self._fail("Digital-I/O press verification is unavailable.", ErrorCode.SENSOR_DATA_UNAVAILABLE, "preconditions", True, "Enable the target digital input or choose an available verification mode.")
        if mode == "visual" and not context.get("camera_observation", {}).get("available"):
            self._fail("Visual press verification is unavailable.", ErrorCode.SENSOR_DATA_UNAVAILABLE, "preconditions", True, "Provide a synchronized camera observation or choose another verification mode.")
        if request["target"].get("expected_state_change") is not None and request["constraints"]["verify_press"]:
            target_sensor = context.get("target_state_sensor", {})
            if not target_sensor.get("available"):
                self._fail("Expected target state change was requested but no target-state sensor is available.", ErrorCode.SENSOR_DATA_UNAVAILABLE, "preconditions", True, "Provide digital-I/O, visual, or another target-state signal for verification.")

    def validate_result(self, raw_result: Mapping[str, Any], request: Mapping[str, Any], context: Mapping[str, Any]) -> Mapping[str, Any]:
        numeric = {key: float(raw_result.get(key, 0.0)) for key in ("peak_force", "final_force", "travel_distance", "hold_time", "execution_time")}
        if not all(math.isfinite(value) for value in numeric.values()):
            self._fail("Press result contains non-finite values.", ErrorCode.PRESS_EXECUTION_ERROR, "verification", False, "Inspect controller and sensor output before retrying.")
        if numeric["peak_force"] > float(request["constraints"]["maximum_force"]) + 1e-9:
            self._fail("Maximum press force was exceeded.", ErrorCode.FORCE_LIMIT_EXCEEDED, "verification", False, "Inspect the target and press direction before changing force limits.", details={"peak_force": numeric["peak_force"]})
        if numeric["travel_distance"] > float(request["constraints"]["maximum_travel"]) + 1e-9:
            self._fail("Maximum press travel was exceeded.", ErrorCode.TRAVEL_LIMIT_EXCEEDED, "verification", False, "Recheck the pre-press pose, target location, and allowed travel.", details={"travel_distance": numeric["travel_distance"]})
        contact = bool(raw_result.get("contact_detected", False))
        if request["constraints"]["require_contact"] and not contact:
            self._fail("No target contact was detected within the allowed travel.", ErrorCode.CONTACT_NOT_DETECTED, "verification", True, "Re-localize the target and move to a valid pre-press pose before retrying.")
        strategy = context.get("selected_press_strategy") or ""
        target_force_reached = bool(raw_result.get("target_force_reached", False))
        target_depth_reached = bool(raw_result.get("target_depth_reached", False))
        if strategy == "force_controlled_press" and request["press"]["operation"] != "verify_only" and not target_force_reached:
            self._fail("Target press force was not reached.", ErrorCode.TARGET_FORCE_NOT_REACHED, "verification", True, "Check contact, maximum travel, and force feedback before retrying.")
        if strategy == "displacement_press" and request["press"]["operation"] != "verify_only" and not target_depth_reached:
            self._fail("Target press depth was not reached.", ErrorCode.TARGET_DEPTH_NOT_REACHED, "verification", True, "Check the target location, requested depth, and travel limit.")
        retracted = bool(raw_result.get("retracted", False))
        if request["press"]["retract_after_press"] and not retracted:
            self._fail("Press completed but local retract failed.", ErrorCode.RETRACT_FAILED, "retract", False, "Stop further motion and inspect whether the tool is trapped or the controller faulted.")
        target_actuated = self._target_actuated(raw_result, request, context)
        verify = bool(request["constraints"]["verify_press"])
        verified = (not verify) or target_actuated
        if verify and not verified:
            self._fail("Press evidence did not confirm target actuation.", ErrorCode.TARGET_NOT_ACTUATED, "verification", True, "Check the press point, depth/force, hold time, and selected verification signal.")
        return {
            "contact_detected": contact,
            "target_force_reached": target_force_reached,
            "target_depth_reached": target_depth_reached,
            "peak_force": numeric["peak_force"], "final_force": numeric["final_force"],
            "travel_distance": numeric["travel_distance"], "hold_time": numeric["hold_time"],
            "target_actuated": target_actuated, "verified": verified,
            "verification_method": self._verification_method(request, context),
            "retracted": retracted, "final_pose": raw_result.get("final_pose"),
            "attempts": int(raw_result.get("attempts", 1)),
            "evidence": dict(raw_result.get("evidence", {})),
            "execution_time": numeric["execution_time"],
        }

    @staticmethod
    def _target_actuated(raw_result: Mapping[str, Any], request: Mapping[str, Any], context: Mapping[str, Any]) -> bool:
        mode = request["press"]["verification_mode"]
        if mode == "digital_io":
            return bool(context.get("digital_io_state", {}).get("target_actuated"))
        if mode == "visual":
            return bool(context.get("camera_observation", {}).get("target_actuated"))
        if mode == "combined":
            external = bool(context.get("digital_io_state", {}).get("target_actuated") or context.get("camera_observation", {}).get("target_actuated"))
            return external and bool(raw_result.get("target_actuated"))
        target_sensor = context.get("target_state_sensor", {})
        if target_sensor.get("available"):
            return bool(target_sensor.get("target_actuated"))
        return bool(raw_result.get("target_actuated"))

    @staticmethod
    def _verification_method(request: Mapping[str, Any], context: Mapping[str, Any]) -> str:
        mode = request["press"]["verification_mode"]
        if mode != "auto":
            return mode
        return "force_profile" if context.get("selected_press_strategy") == "force_controlled_press" else "position_contact"

    @staticmethod
    def _fail(message: str, code: ErrorCode, stage: str, recoverable: bool, recommended_action: str, details: object = None) -> None:
        raise PressExecutionValidationError(message, error_code=code, failed_stage=stage, recoverable=recoverable, recommended_action=recommended_action, details=details)
