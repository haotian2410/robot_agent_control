"""Executable Move Skill for a MuJoCo UR5e simulation."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Optional

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[3]

from collision import MujocoCollisionChecker
from kinematics import IKError, MujocoKinematics
from skills.motion.move.planners import CollisionFreePlanner, CollisionPlanningError, DirectPlanner, PlanningError
from skills.motion.move.selector import StrategySelectionError, StrategySelector
from skills.motion.move.strategies import CircularMove, JointMove, LinearMove
from skills.motion.move.utils.error_codes import ErrorCode, failure_result
from skills.motion.move.utils.validation import MoveValidationError, validate_move_request
from utils import SceneRobotRuntime


DEFAULT_SCENE = PROJECT_ROOT / "world_model" / "robotsim" / "scene_000.xml"


class MoveSkill:
    """Validate, select, plan, collision-check, and execute one motion request."""

    def __init__(self, config: Optional[Mapping[str, Any]] = None, *, robot_runtime: SceneRobotRuntime | None = None, scene_path: str | Path = DEFAULT_SCENE, trajectory_callback: Callable[[Mapping[str, Any], Mapping[str, Any], Mapping[str, Any]], None] | None = None) -> None:
        self.config: Mapping[str, Any] = config or {}
        self.robot_runtime = robot_runtime or SceneRobotRuntime(scene_path)
        self.kinematics = MujocoKinematics(self.robot_runtime)
        self.collision_checker = MujocoCollisionChecker(self.robot_runtime)
        self.selector = StrategySelector(self.config)
        self.trajectory_callback = trajectory_callback
        self.strategies = {
            "joint_move": JointMove(self.config, kinematics=self.kinematics),
            "linear_move": LinearMove(self.config, kinematics=self.kinematics, collision_checker=self.collision_checker),
            "circular_move": CircularMove(self.config, kinematics=self.kinematics),
        }
        planner_kwargs = {
            "collision_checker": self.collision_checker,
            "joint_limits": self.robot_runtime.joint_limits,
        }
        self.direct_planner = DirectPlanner(self.config, **planner_kwargs)
        self.collision_free_planner = CollisionFreePlanner(self.config, kinematics=self.kinematics, **planner_kwargs)

    def execute(self, request: Mapping[str, Any]) -> Dict[str, Any]:
        """Run the complete Move pipeline and always return a structured result."""
        selection: Dict[str, Any] | None = None
        try:
            normalized = validate_move_request(request, self.config)
            target_body_name = normalized.get("target_body_name")
            if target_body_name:
                body_id = mujoco.mj_name2id(self.robot_runtime.model, mujoco.mjtObj.mjOBJ_BODY, str(target_body_name))
                if body_id >= 0:
                    self.collision_checker.allowed_target_geom_ids = {
                        geom_id for geom_id in range(self.robot_runtime.model.ngeom)
                        if int(self.robot_runtime.model.geom_bodyid[geom_id]) == body_id
                    }
            context = self._get_runtime_context(normalized)
            selection = self.selector.select(normalized, context)
            strategy = self.strategies[selection["path_strategy"]]
            path_request = strategy.build_path_request(normalized, context)
            trajectory = self._plan_with_fallback(normalized, context, selection, path_request)
            if self.trajectory_callback is not None:
                self.trajectory_callback(trajectory, normalized, selection)
            execution_result = self._execute_trajectory(trajectory, normalized, context)
            execution_result["planning_time"] = trajectory.get("planning_time", 0.0)
            execution_result["planned_duration"] = trajectory.get("duration", 0.0)
            return {"success": True, "status": "completed", "selection": selection, "execution_result": execution_result, "trajectory": trajectory, "error": None}
        except MoveValidationError as exc:
            result = failure_result(ErrorCode.INVALID_REQUEST, str(exc), failed_stage="validation", recoverable=True, recommended_action="Correct the request fields and retry.")
        except StrategySelectionError as exc:
            result = failure_result(ErrorCode.UNKNOWN_STRATEGY, str(exc), failed_stage="selection", recoverable=True)
        except IKError as exc:
            result = failure_result(ErrorCode.IK_FAILED, str(exc), failed_stage="kinematics", recoverable=True, recommended_action="Try TRAC-IK, another seed, or a reachable target pose.")
        except CollisionPlanningError as exc:
            result = failure_result(ErrorCode.COLLISION_DETECTED, str(exc), failed_stage="planning", recoverable=True, recommended_action="Allow collision-free planning or change the target/path constraint.", details={"contacts": exc.details or []})
        except PlanningError as exc:
            result = failure_result(
                exc.code, str(exc), failed_stage="planning", recoverable=exc.recoverable,
                recommended_action=self._planning_recommendation(exc.code), details=exc.details,
            )
        except MoveExecutionError as exc:
            result = failure_result(
                exc.code, str(exc), failed_stage="execution", recoverable=exc.recoverable,
                recommended_action=exc.recommended_action,
            )
        except TimeoutError as exc:
            result = failure_result(ErrorCode.EXECUTION_TIMEOUT, str(exc), failed_stage="execution", recoverable=True, status="timeout")
        except ValueError as exc:
            result = failure_result(ErrorCode.PLANNING_FAILED, str(exc), failed_stage="path_generation", recoverable=True)
        except RuntimeError as exc:
            result = failure_result(ErrorCode.EXECUTION_FAILED, str(exc), failed_stage="execution", recoverable=True)
        except Exception as exc:
            result = failure_result(ErrorCode.INTERNAL_ERROR, str(exc), failed_stage="unknown", recoverable=False)
        if selection is not None:
            result["selection"] = selection
        return result

    def _get_runtime_context(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        """Delegate scene and robot-state acquisition to the root utils module."""
        return self.robot_runtime.get_context(request)

    def _plan_with_fallback(self, request: Mapping[str, Any], context: Mapping[str, Any], selection: Dict[str, Any], path_request: Mapping[str, Any]) -> Mapping[str, Any]:
        """Always test the direct path before applying a permitted fallback."""
        try:
            trajectory = self.direct_planner.plan(path_request, request, context)
            selection["planning_mode"] = "direct"
            return trajectory
        except CollisionPlanningError as collision_error:
            mode = request["planning"]["mode"]
            hard_constraint = request["motion"]["path_constraint"] == "hard"
            fallback_allowed = mode == "collision_free" or (mode == "auto" and request["planning"]["allow_replan"])
            if hard_constraint and path_request["path_type"] != "joint":
                raise PlanningError(
                    "Direct path collides and its hard Cartesian constraint cannot be relaxed.",
                    code=ErrorCode.PATH_CONSTRAINT_INFEASIBLE.value, recoverable=True,
                    details={"contacts": collision_error.details or []},
                )
            if not fallback_allowed:
                raise
            trajectory = self.collision_free_planner.plan(path_request, request, context)
            selection["planning_mode"] = "collision_free"
            selection["fallback_used"] = True
            selection["selection_reason"] += "; direct path collision triggered RRT-Connect"
            return trajectory

    def _execute_trajectory(self, trajectory: Mapping[str, Any], request: Mapping[str, Any], context: Mapping[str, Any]) -> Dict[str, Any]:
        if not context["controller_state"].get("ready"):
            raise MoveExecutionError(
                "MuJoCo controller is not ready.", code=ErrorCode.CONTROLLER_NOT_READY.value,
                recoverable=True, recommended_action="Initialize or enable the controller, clear faults, and retry.",
            )
        result = self.robot_runtime.execute_trajectory(trajectory, timeout=float(request["constraints"]["timeout"]))
        if result["joint_error"] > 0.02:
            raise MoveExecutionError(
                f"Final joint tracking error {result['joint_error']:.6f} exceeds tolerance.",
                code=ErrorCode.GOAL_TOLERANCE_EXCEEDED.value, recoverable=True,
                recommended_action="Retry at a lower speed or inspect controller tracking and goal tolerance.",
            )
        return result

    @staticmethod
    def _planning_recommendation(code: str) -> str:
        return {
            ErrorCode.PATH_CONSTRAINT_INFEASIBLE.value: "Allow path changes, change the start/target pose, or relax the path/orientation constraint.",
            ErrorCode.JOINT_LIMIT_VIOLATION.value: "Choose joint values within the reported limits or use another reachable pose.",
            ErrorCode.COLLISION_DETECTED.value: "Change the start/goal state or allow collision-free replanning.",
            ErrorCode.PATH_NOT_FOUND.value: "Increase planning time, relax constraints, or change the start/target pose.",
            ErrorCode.TRAJECTORY_INVALID.value: "Replan after refreshing the scene and collision state.",
            ErrorCode.INVALID_ARC_DEFINITION.value: "Correct the arc center, axis, angle, or via point and retry.",
            ErrorCode.TARGET_UNREACHABLE.value: "Choose a defined named state or another reachable target.",
        }.get(str(code), "Inspect the planning diagnostics and adjust the target or constraints.")


class MoveExecutionError(RuntimeError):
    def __init__(self, message: str, *, code: str, recoverable: bool, recommended_action: str) -> None:
        super().__init__(message)
        self.code = code
        self.recoverable = recoverable
        self.recommended_action = recommended_action


def run(request: Mapping[str, Any], *, scene_path: str | Path = DEFAULT_SCENE) -> Dict[str, Any]:
    """Convenience entrypoint used by agents and simple scripts."""
    return MoveSkill(scene_path=scene_path).execute(request)
