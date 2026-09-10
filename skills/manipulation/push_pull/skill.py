"""Executable Push/Pull Skill orchestration."""
from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

from control import MujocoPushPullController
from skills.manipulation.push_pull.selector import StrategySelector
from skills.manipulation.push_pull.strategies import CircularPushPull, FreeObjectPush, LinearPushPull
from skills.manipulation.push_pull.utils import PushPullValidationError, validate_push_pull_request


class PushPullSkill:
    def __init__(self, config: Optional[Mapping[str, Any]] = None, *, robot_runtime: Any = None) -> None:
        self.config = config or {}
        self.robot_runtime = robot_runtime
        self.selector = StrategySelector()
        self.strategies = {
            "linear_push_pull": LinearPushPull(),
            "circular_push_pull": CircularPushPull(),
            "free_object_push": FreeObjectPush(),
        }

    def execute(self, request: Mapping[str, Any]) -> Dict[str, Any]:
        selection = None
        try:
            normalized = validate_push_pull_request(request, self.config)
            context = self._runtime_context(normalized)
            selection = self.selector.select(normalized, context)
            raw = dict(self.strategies[selection["push_pull_strategy"]].execute(normalized, context))
            result = self._verify(raw, normalized)
            return {"success": True, "status": "completed", "selection": selection, "push_pull_result": result, "error": None}
        except PushPullValidationError as exc:
            return self._failure("INVALID_REQUEST", str(exc), "validation", True, selection)
        except TimeoutError as exc:
            return self._failure("PUSH_PULL_TIMEOUT", str(exc), "execution", True, selection, "timeout")
        except Exception as exc:
            return self._failure(getattr(exc, "error_code", "PUSH_PULL_EXECUTION_ERROR"), str(exc), getattr(exc, "failed_stage", "execution"), bool(getattr(exc, "recoverable", False)), selection)

    def _runtime_context(self, request: Mapping[str, Any]) -> Dict[str, Any]:
        if self.robot_runtime is None:
            raise RuntimeError("An initialized robot runtime at the pre-contact pose is required.")
        controller = getattr(self.robot_runtime, "push_pull_controller", None)
        if controller is None:
            controller = MujocoPushPullController(self.robot_runtime)
            self.robot_runtime.push_pull_controller = controller
        state = dict(controller.get_state(
            request["target"]["object_id"],
            interaction_mode=request["resolved_interaction_mode"],
        ))
        if not state.get("ready") or state.get("fault"):
            raise RuntimeError("Push/Pull controller is not ready.")
        if not state.get("safe", True):
            raise RuntimeError("Robot safety state prevents Push/Pull execution.")
        return {"push_pull_controller": controller, "controller_state": state}

    @staticmethod
    def _verify(raw: Mapping[str, Any], request: Mapping[str, Any]) -> Dict[str, Any]:
        peak_force = float(raw.get("peak_force", 0.0))
        if not math_isfinite(peak_force) or peak_force > request["constraints"]["maximum_force"] + 1e-9:
            raise RuntimeError("Maximum Push/Pull force was exceeded.")
        if request["manipulation"]["maintain_contact"] and not raw.get("contact_maintained", False):
            raise RuntimeError("Target contact was lost during Push/Pull execution.")
        moved = bool(raw.get("target_moved", False))
        verify_only = request["manipulation"]["operation"] == "verify_only"
        if request["constraints"]["verify_motion"] and not verify_only and not moved:
            raise RuntimeError("Target motion could not be verified.")
        return {"contact_established": bool(raw.get("contact_established", False)), "contact_maintained": bool(raw.get("contact_maintained", False)), "target_moved": moved, "verified": verify_only or moved or not request["constraints"]["verify_motion"], "travel_distance": float(raw.get("travel_distance", 0.0)), "rotation_angle": float(raw.get("rotation_angle", 0.0)), "peak_force": peak_force, "final_pose": raw.get("final_pose"), "object_final_state": raw.get("object_final_state"), "evidence": dict(raw.get("evidence", {})), "execution_time": float(raw.get("execution_time", 0.0))}

    @staticmethod
    def _failure(code: Any, message: str, stage: str, recoverable: bool, selection: Any, status: str = "failed") -> Dict[str, Any]:
        return {"success": False, "status": status, "selection": selection, "push_pull_result": None, "error": {"error_code": str(getattr(code, "value", code)), "error_message": message, "failed_stage": stage, "recoverable": recoverable}}


def math_isfinite(value: float) -> bool:
    return value == value and value not in (float("inf"), float("-inf"))
