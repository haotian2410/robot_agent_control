"""Executable Release Skill orchestration."""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

from control import MujocoGripperController
from skills.manipulation.release.selector import StrategySelectionError, StrategySelector
from skills.manipulation.release.strategies import DefaultRelease
from skills.manipulation.release.utils.error_codes import ErrorCode, failure_result
from skills.manipulation.release.utils.validation import ReleaseValidationError, validate_release_request
from skills.manipulation.release.validators import ReleaseExecutionValidationError, ReleaseValidator


class ReleaseSkill:
    """Validate, select, execute, and verify one same-pose release action."""

    def __init__(self, config: Optional[Mapping[str, Any]] = None, *, robot_runtime: Any = None) -> None:
        self.config: Mapping[str, Any] = config or {}
        self.robot_runtime = robot_runtime
        self.selector = StrategySelector(self.config)
        self.strategies = {"default_release": DefaultRelease(self.config)}
        self.release_validator = ReleaseValidator(self.config)

    def execute(self, request: Mapping[str, Any]) -> Dict[str, Any]:
        try:
            normalized = validate_release_request(request, self.config)
            context = self._get_runtime_context(normalized)
            selection = self.selector.select(normalized, context)
            self.release_validator.validate_preconditions(normalized, context, selection)
            raw_result = self._execute_with_retry(normalized, context, selection)
            release_result = self.release_validator.validate_result(raw_result, normalized, context)
            return {"success": True, "status": "completed", "selection": selection, "release_result": release_result, "error": None}
        except ReleaseValidationError as exc:
            return failure_result(ErrorCode.INVALID_REQUEST, str(exc), failed_stage="validation", recoverable=True, recommended_action="Correct the request using schema.yaml.")
        except StrategySelectionError as exc:
            return failure_result(ErrorCode.UNKNOWN_STRATEGY, str(exc), failed_stage="strategy_selection", recoverable=True, recommended_action="Select a supported strategy for an open-close gripper.")
        except ReleaseExecutionValidationError as exc:
            return failure_result(exc.error_code, str(exc), failed_stage=exc.failed_stage, recoverable=exc.recoverable, recommended_action=exc.recommended_action)
        except TimeoutError as exc:
            return failure_result(ErrorCode.RELEASE_TIMEOUT, str(exc), failed_stage="execution", recoverable=True, status="timeout")
        except Exception as exc:
            return failure_result(ErrorCode.INTERNAL_ERROR, str(exc), failed_stage="unknown", recoverable=False)

    def _get_runtime_context(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        if self.robot_runtime is None:
            raise ReleaseExecutionValidationError("A robot runtime is required.", error_code=ErrorCode.GRIPPER_NOT_READY, failed_stage="runtime_context", recoverable=True)
        controller = getattr(self.robot_runtime, "gripper_controller", None)
        if controller is None:
            controller = MujocoGripperController(self.robot_runtime)
            self.robot_runtime.gripper_controller = controller
        target_id = request["target"]["object_id"]
        state = controller.get_state(target_id)
        return {
            "runtime": self.robot_runtime,
            "gripper_controller": controller,
            "current_gripper_state": state,
            "current_end_effector_pose": self.robot_runtime.get_end_effector_pose(),
            "gripper_model": {"type": "parallel", "name": "robotiq_2f85"},
            "gripper_limits": {"minimum_width": 0.0, "maximum_width": controller.maximum_width},
            "gripper_controller_state": {"ready": True, "enabled": True, "fault": False},
            "object_presence_state": {"available": True, "present": state["object_present"]},
            "payload_state": {"holding": state["holding"], "object_id": target_id},
            "safety_state": {"safe": True, "emergency_stop": False},
        }

    def _execute_with_retry(self, request: Mapping[str, Any], context: Mapping[str, Any], selection: Dict[str, Any]) -> Mapping[str, Any]:
        strategy = self.strategies[selection["release_strategy"]]
        attempts = int(request["constraints"]["retry_count"]) + 1
        last_result: Mapping[str, Any] | None = None
        for attempt in range(1, attempts + 1):
            last_result = dict(strategy.execute(request, context))
            last_result["attempts"] = attempt
            released = not bool(last_result.get("holding", True))
            absent = not bool(last_result.get("object_present", True))
            if request["release"]["operation"] == "verify_only" or released and absent:
                selection["retry_used"] = attempt > 1
                return last_result
        selection["retry_used"] = attempts > 1
        return last_result or {}
