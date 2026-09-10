"""Joint-space path strategy."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from ...planners.errors import PlanningError


class JointMove:
    """Translate a normalized Move request into a joint-path specification."""

    strategy_id = "joint_move"

    def __init__(self, config: Mapping[str, Any] | None = None, *, kinematics: Any = None) -> None:
        self.config = config or {}
        self.kinematics = kinematics

    def build_path_request(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        target = request["target"]
        start = np.asarray(context["current_joint_state"], dtype=float)
        diagnostics = {}
        if target["type"] == "joint":
            goal = np.asarray(target["joint_positions"], dtype=float)
        elif target["type"] == "named_state":
            presets = self.config.get("named_states", {})
            defaults = {
                "home": [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
                "ready": [0.0, -1.2, 1.4, -1.75, -1.5708, 0.0],
                "park": [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0],
            }
            name = target["state_name"]
            if name not in presets and name not in defaults:
                raise PlanningError(f"Unknown named robot state: {name}", code="TARGET_UNREACHABLE", recoverable=True, details={"state_name": name})
            goal = np.asarray(presets.get(name, defaults[name]), dtype=float)
        else:
            seed_value = request["strategy_params"].get("ik_seed")
            ik_seed = np.asarray(seed_value, dtype=float) if seed_value is not None else start
            result = self.kinematics.solve(
                target,
                seed=ik_seed,
                method=request["strategy_params"].get("ik_method", "auto"),
                timeout=min(float(request["planning"]["planning_time"]), 2.0),
            )
            goal = np.asarray(result["solution"], dtype=float)
            diagnostics["ik"] = {key: value for key, value in result.items() if key not in {"solution", "solutions"}}
        limits = self.kinematics.limits
        if np.any(goal < limits[:, 0]) or np.any(goal > limits[:, 1]):
            raise PlanningError(
                "Target joint state violates UR5e joint limits.",
                code="JOINT_LIMIT_VIOLATION", recoverable=True,
                details={"target": goal.tolist(), "limits": limits.tolist()},
            )
        return {"path_type": "joint", "joint_waypoints": np.vstack((start, goal)), "start": start, "goal": goal, "diagnostics": diagnostics}
