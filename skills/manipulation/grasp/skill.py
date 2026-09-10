"""Executable Grasp Skill orchestration."""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

from control import MujocoGripperController
from skills.manipulation.grasp.selector import StrategySelectionError, StrategySelector
from skills.manipulation.grasp.strategies import DefaultGrasp
from skills.manipulation.grasp.utils.error_codes import ErrorCode, failure_result
from skills.manipulation.grasp.utils.validation import GraspValidationError, validate_grasp_request
from skills.manipulation.grasp.validators import GraspExecutionValidationError, GraspValidator


class GraspSkill:
    """Coordinate request validation, strategy execution and grasp verification."""

    def __init__(
        self,
        config: Optional[Mapping[str, Any]] = None,
        *,
        robot_runtime: Any = None,
    ) -> None:
        self.config: Mapping[str, Any] = config or {}
        self.robot_runtime = robot_runtime

        self.selector = StrategySelector(self.config)
        self.strategies = {
            "default_grasp": DefaultGrasp(self.config),
        }
        self.grasp_validator = GraspValidator(self.config)

    def execute(self, request: Mapping[str, Any]) -> Dict[str, Any]:
        """Execute one Grasp request.

        Intended pipeline:
        1. Validate and normalize the external request.
        2. Read synchronized gripper, controller, sensor and robot context.
        3. Select one Grasp Strategy using deterministic rules.
        4. Validate execution preconditions and hardware limits.
        5. Execute the selected strategy.
        6. Verify contact, width, force, object presence and slip evidence.
        7. Retry only when configured, recoverable and safe.
        8. Return a structured result without moving the robot arm.
        """
        try:
            normalized = validate_grasp_request(request, self.config)
            context = self._get_runtime_context(normalized)
            selection = self.selector.select(normalized, context)
            self.grasp_validator.validate_preconditions(
                normalized, context, selection
            )

            raw_result = self._execute_with_retry(
                normalized, context, selection
            )
            grasp_result = self.grasp_validator.validate_result(
                raw_result, normalized, context
            )

            return {
                "success": True,
                "status": "completed",
                "selection": selection,
                "grasp_result": grasp_result,
                "error": None,
            }
        except GraspValidationError as exc:
            return failure_result(
                ErrorCode.INVALID_REQUEST,
                str(exc),
                failed_stage="validation",
                recoverable=True,
                recommended_action="Correct the request using schema.yaml.",
            )
        except StrategySelectionError as exc:
            return failure_result(
                ErrorCode.INVALID_REQUEST,
                str(exc),
                failed_stage="strategy_selection",
                recoverable=True,
                recommended_action=(
                    "Select a supported strategy and provide a compatible gripper state."
                ),
            )
        except GraspExecutionValidationError as exc:
            return failure_result(
                exc.error_code,
                str(exc),
                failed_stage=exc.failed_stage,
                recoverable=exc.recoverable,
                recommended_action=exc.recommended_action,
            )
        except TimeoutError as exc:
            return failure_result(
                ErrorCode.GRASP_TIMEOUT,
                str(exc),
                failed_stage="execution",
                recoverable=True,
                status="timeout",
            )
        except NotImplementedError as exc:
            return failure_result(
                ErrorCode.NOT_IMPLEMENTED,
                str(exc),
                failed_stage="implementation",
                recoverable=False,
            )
        except Exception as exc:  # Replace with typed runtime exceptions.
            return failure_result(
                ErrorCode.INTERNAL_ERROR,
                str(exc),
                failed_stage="unknown",
                recoverable=False,
            )

    def _get_runtime_context(
        self, request: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        """Read synchronized simulated gripper, controller and robot context."""
        if self.robot_runtime is None:
            raise GraspExecutionValidationError(
                "A robot runtime is required.",
                error_code=ErrorCode.GRIPPER_NOT_READY,
                failed_stage="runtime_context",
                recoverable=True,
            )
        controller = getattr(self.robot_runtime, "gripper_controller", None)
        if controller is None:
            controller = MujocoGripperController(self.robot_runtime)
            self.robot_runtime.gripper_controller = controller
        target_id = request["target"].get("object_id")
        state = controller.get_state(target_id)
        return {
            "runtime": self.robot_runtime,
            "gripper_controller": controller,
            "current_gripper_state": state,
            "current_end_effector_pose": self.robot_runtime.get_end_effector_pose(),
            "gripper_model": {"type": "parallel", "name": "robotiq_2f85"},
            "gripper_limits": {"minimum_width": 0.0, "maximum_width": controller.maximum_width, "maximum_force": controller.maximum_force},
            "gripper_controller_state": {"ready": True, "enabled": True, "fault": False},
            "finger_state": state,
            "object_presence_state": {"available": True, "present": state["object_present"]},
            "slip_state": {"available": True, "slip_detected": state["slip_detected"]},
            "safety_state": {"safe": True, "emergency_stop": False},
        }

    def _execute_with_retry(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
        selection: Dict[str, Any],
    ) -> Mapping[str, Any]:
        """Execute the strategy and apply only legal same-pose retries.

        Required implementation behavior:
        - Never change the robot pose during retry.
        - Retry only recoverable failures and only while the state is safe.
        - Reopen the gripper before retry when configured.
        - Record attempts and set selection['retry_used'] accurately.
        - Preserve typed failure evidence from every attempt.
        """
        strategy = self.strategies[selection["grasp_strategy"]]
        attempts = int(request["constraints"]["retry_count"]) + 1
        last_result: Mapping[str, Any] | None = None
        for attempt in range(1, attempts + 1):
            last_result = strategy.execute(request, context)
            last_result = dict(last_result)
            last_result["attempts"] = attempt
            if request["grasp"]["operation"] == "verify_only" or last_result.get("contact_detected"):
                selection["retry_used"] = attempt > 1
                return last_result
            if attempt < attempts:
                context["gripper_controller"].open(
                    last_result["parameters"]["open_width"],
                    speed=last_result["parameters"]["close_speed"],
                    timeout=request["constraints"]["timeout"],
                )
        selection["retry_used"] = attempts > 1
        return last_result or {}
