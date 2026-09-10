"""Executable Press Skill orchestration."""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

from collision import MujocoCollisionChecker
from control import MujocoPressController
from skills.manipulation.press.selector import StrategySelectionError, StrategySelector
from skills.manipulation.press.strategies import DisplacementPress, ForceControlledPress
from skills.manipulation.press.utils.error_codes import ErrorCode, PressExecutionError, failure_result
from skills.manipulation.press.utils.validation import PressValidationError, validate_press_request
from skills.manipulation.press.validators import PressExecutionValidationError, PressValidator


class PressSkill:
    """Execute and verify one bounded fixed-axis local press."""

    def __init__(self, config: Optional[Mapping[str, Any]] = None, *, robot_runtime: Any = None) -> None:
        self.config: Mapping[str, Any] = config or {}
        self.robot_runtime = robot_runtime
        self.selector = StrategySelector(self.config)
        self.strategies = {
            "displacement_press": DisplacementPress(self.config),
            "force_controlled_press": ForceControlledPress(self.config),
        }
        self.press_validator = PressValidator(self.config)

    def execute(self, request: Mapping[str, Any]) -> Dict[str, Any]:
        selection: Dict[str, Any] | None = None
        try:
            normalized = validate_press_request(request, self.config)
            context = dict(self._get_runtime_context(normalized))
            selection = self.selector.select(normalized, context)
            context["selected_press_strategy"] = selection["press_strategy"]
            self.press_validator.validate_preconditions(normalized, context, selection)
            raw_result = self._execute_with_retry(normalized, context, selection)
            press_result = self.press_validator.validate_result(raw_result, normalized, context)
            return {"success": True, "status": "completed", "selection": selection, "press_result": press_result, "error": None}
        except PressValidationError as exc:
            result = failure_result(ErrorCode.INVALID_REQUEST, str(exc), failed_stage="validation", recoverable=True, recommended_action="Correct the request using schema.yaml and the selected strategy schema.")
        except StrategySelectionError as exc:
            code = ErrorCode.SENSOR_DATA_UNAVAILABLE if "feedback" in str(exc).lower() else ErrorCode.UNKNOWN_STRATEGY
            result = failure_result(code, str(exc), failed_stage="strategy_selection", recoverable=True, recommended_action="Provide a valid press depth or target force with compatible sensor context.")
        except (PressExecutionValidationError, PressExecutionError) as exc:
            code_value = exc.error_code.value if isinstance(exc.error_code, ErrorCode) else str(exc.error_code)
            status = "timeout" if code_value == ErrorCode.PRESS_TIMEOUT.value else "failed"
            result = failure_result(exc.error_code, str(exc), failed_stage=exc.failed_stage, recoverable=exc.recoverable, recommended_action=exc.recommended_action, status=status, details=exc.details)
        except TimeoutError as exc:
            result = failure_result(ErrorCode.PRESS_TIMEOUT, str(exc), failed_stage="execution", recoverable=True, recommended_action="Inspect contact progress and timeout, then retry only from a safe pre-press pose.", status="timeout")
        except Exception as exc:
            result = failure_result(ErrorCode.INTERNAL_ERROR, str(exc), failed_stage="unknown", recoverable=False)
        if selection is not None:
            result["selection"] = selection
        return result

    def _get_runtime_context(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        if self.robot_runtime is None:
            raise PressExecutionError("A robot runtime is required.", error_code=ErrorCode.PRESS_CONTROLLER_NOT_READY, failed_stage="runtime_context", recoverable=True, recommended_action="Provide an initialized robot runtime at the pre-press pose.")
        controller = getattr(self.robot_runtime, "press_controller", None)
        if controller is None:
            controller = MujocoPressController(self.robot_runtime)
            self.robot_runtime.press_controller = controller
        state = controller.get_state(request["target"].get("object_id"))
        runtime_context = self.robot_runtime.get_context(request)
        collision = MujocoCollisionChecker(self.robot_runtime).check(self.robot_runtime.get_joint_positions())
        target_state = self._target_state(request)
        return {
            **runtime_context,
            "press_controller": controller,
            "current_end_effector_pose": self.robot_runtime.get_end_effector_pose(),
            "current_joint_state": self.robot_runtime.get_joint_positions(),
            "force_torque_state": {"available": bool(state["force_feedback_available"]), "calibrated": bool(state["force_sensor_calibrated"]), "normal_force": float(state["normal_force"])},
            "tactile_state": {"available": False},
            "contact_controller_state": {"ready": bool(state["ready"]), "enabled": bool(state["enabled"]), "fault": bool(state["fault"])},
            "local_collision_state": {"in_collision": collision.in_collision, "contacts": list(collision.contacts)},
            "tool_geometry": {"end_effector_site": self.robot_runtime.end_effector_site},
            "tool_compliance": {"model": "mujoco_contact"},
            "target_state_sensor": target_state,
            "digital_io_state": target_state.get("digital_io", {"available": False}),
            "camera_observation": target_state.get("visual", {"available": False}),
            "safety_state": {"safe": True, "emergency_stop": False},
        }

    def _execute_with_retry(self, request: Mapping[str, Any], context: Mapping[str, Any], selection: Dict[str, Any]) -> Mapping[str, Any]:
        strategy = self.strategies[selection["press_strategy"]]
        attempts = int(request["constraints"]["retry_count"]) + 1
        last_error: PressExecutionError | None = None
        for attempt in range(1, attempts + 1):
            raw: dict[str, Any] = {}
            try:
                raw = dict(strategy.execute(request, context))
                raw["attempts"] = attempt
                self.press_validator.validate_result(raw, request, context)
                selection["retry_used"] = attempt > 1
                return raw
            except PressExecutionError as exc:
                exc.retracted = exc.retracted or bool(raw.get("retracted", False))
                last_error = exc
                can_retry = attempt < attempts and exc.recoverable and exc.retracted and context.get("safety_state", {}).get("safe")
                if not can_retry:
                    selection["retry_used"] = attempt > 1
                    raise
        raise last_error or PressExecutionError("Press failed without a result.", error_code=ErrorCode.PRESS_EXECUTION_ERROR, failed_stage="execution", recoverable=False, recommended_action="Inspect the press controller state.")

    def _target_state(self, request: Mapping[str, Any]) -> dict[str, Any]:
        getter = getattr(self.robot_runtime, "get_press_target_state", None)
        if callable(getter):
            value = getter(request["target"].get("object_id"))
            return dict(value or {})
        return {"available": False, "digital_io": {"available": False}, "visual": {"available": False}}
