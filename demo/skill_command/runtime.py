"""Persistent MuJoCo runtime used by the structured skill-command demo."""

from __future__ import annotations

import json
from copy import deepcopy
from typing import Any, Mapping

import mujoco
import numpy as np

from demo.common.registry import SceneRegistry
from demo.skill_command.approach_collision import MujocoApproachCollisionChecker
from skills.manipulation.grasp.skill import GraspSkill
from skills.manipulation.press.skill import PressSkill
from skills.manipulation.push_pull.skill import PushPullSkill
from skills.manipulation.release.skill import ReleaseSkill
from skills.motion.move.skill import MoveSkill
from utils import SceneRobotRuntime


class SkillRuntime:
    """Own one persistent robot state and all skills operating on it."""

    def __init__(self, registry: SceneRegistry, runtime_config: Mapping[str, Any] | None = None) -> None:
        config = runtime_config or {}
        self.registry = registry
        self.runtime = SceneRobotRuntime(
            registry.scene_path,
            end_effector_site=config.get("end_effector_site", "robotiq_2f85_pinch"),
            execution_mode=config.get("execution_mode", "kinematic"),
            realtime=bool(config.get("realtime", True)),
            playback_fps=config.get("playback_fps", 60.0),
            playback_speed=config.get("playback_speed", 1.0),
            minimum_playback_duration=config.get("minimum_playback_duration", 4.0),
        )
        self.move_skill = MoveSkill(robot_runtime=self.runtime)
        self.grasp_skill = GraspSkill(robot_runtime=self.runtime)
        self.press_skill = PressSkill(robot_runtime=self.runtime)
        self.push_pull_skill = PushPullSkill(robot_runtime=self.runtime)
        self.release_skill = ReleaseSkill(robot_runtime=self.runtime)
        self.approach_checker = MujocoApproachCollisionChecker(
            registry, config, runtime=self.runtime
        )

    @property
    def model(self) -> Any:
        return self.runtime.model

    @property
    def data(self) -> Any:
        return self.runtime.data

    def pose_provider(self, object_key: str, spatial: Mapping[str, Any]) -> Mapping[str, Any]:
        return self.approach_checker.pose_provider(object_key, spatial)

    def update(self) -> None:
        """Synchronize derived MuJoCo positions after every action."""
        mujoco.mj_forward(self.runtime.model, self.runtime.data)
        # Approach caches include live pose, robot, gripper, and movable-world
        # state in their keys.  Keeping them across updates allows unchanged
        # states to be reused while changed states naturally miss the cache.

    def refresh_move_skill(self) -> None:
        self.move_skill = MoveSkill(robot_runtime=self.runtime)

    def execute_step(
        self, step: Mapping[str, Any], defaults: Mapping[str, Any]
    ) -> dict[str, Any]:
        step_type = step.get("type", "move")
        if step_type == "move":
            request = _build_request(defaults, step)
            result = self.move_skill.execute(request)
        elif step_type == "grasp":
            result = self.grasp_skill.execute(step["request"])
        elif step_type == "release":
            result = self.release_skill.execute(step["request"])
            if result.get("success"):
                self._verify_release(step, result)
        elif step_type == "press":
            result = self.press_skill.execute(step["request"])
        elif step_type == "push_pull":
            result = self.push_pull_skill.execute(step["request"])
        else:
            return {"success": False, "error": {"error_code": "INVALID_TEST_ACTION", "error_message": f"Unsupported action type: {step_type}"}}

        self.update()
        if result.get("success") and step_type in {"grasp", "release"}:
            self.refresh_move_skill()
        return result

    def _verify_release(self, step: Mapping[str, Any], result: dict[str, Any]) -> None:
        region = step.get("verify_region")
        if not region:
            return
        body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            str(step.get("object_body", "red_ball_body")),
        )
        if body_id < 0:
            result["placement_verification"] = {"inside": False, "error": "body not found"}
            return
        position = self.data.xpos[body_id]
        inside = all(region["min"][i] <= position[i] <= region["max"][i] for i in range(3))
        result["placement_verification"] = {"inside": inside, "position": position.tolist()}
        if not inside:
            result["success"] = False
            result["error"] = {
                "error_code": "PLACE_VERIFICATION_FAILED",
                "error_message": f"Released object is outside the expected region: {position.tolist()}",
                "failed_stage": "release_verification",
                "recoverable": True,
            }


def _build_request(defaults: Mapping[str, Any], step: Mapping[str, Any]) -> dict[str, Any]:
    request = _deep_merge(defaults, step["request"])
    allow_change = bool(step.get("allow_path_change", False))
    request.setdefault("motion", {})["path_constraint"] = "soft" if allow_change else "hard"
    if not allow_change:
        request.setdefault("planning", {})["mode"] = "direct"
        request["planning"]["allow_replan"] = False
    return request


def _deep_merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> dict[str, Any]:
    result = deepcopy(dict(base))
    for key, value in override.items():
        if isinstance(result.get(key), Mapping) and isinstance(value, Mapping):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result
