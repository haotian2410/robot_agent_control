"""Direct trajectory generation and validation."""

from __future__ import annotations

import time
from typing import Any, Mapping

import numpy as np

from .errors import CollisionPlanningError, PlanningError


class DirectPlanner:
    """Generate and validate the direct trajectory implied by a Path Strategy."""

    planner_id = "direct"

    def __init__(self, config: Mapping[str, Any] | None = None, *, collision_checker: Any = None, joint_limits: Any = None) -> None:
        self.config = config or {}
        self.collision_checker = collision_checker
        self.joint_limits = np.asarray(joint_limits, dtype=float)

    def plan(
        self,
        path_request: Mapping[str, Any],
        move_request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Any:
        started = time.perf_counter()
        source = np.asarray(path_request["joint_waypoints"], dtype=float)
        if source.ndim != 2 or source.shape[1] != 6 or len(source) < 2:
            raise PlanningError("Path strategy returned an invalid joint path.")
        if not np.all(np.isfinite(source)):
            raise PlanningError("Path contains non-finite joint values.")
        if np.any(source < self.joint_limits[:, 0]) or np.any(source > self.joint_limits[:, 1]):
            raise PlanningError("Path violates joint limits.", code="JOINT_LIMIT_VIOLATION")
        waypoints = _densify(source, max_step=0.08)
        if move_request["constraints"]["avoid_collision"]:
            safety_distance = float(move_request["constraints"].get("safety_distance", 0.0))
            result = self.collision_checker.check_path(
                waypoints,
                resolution=0.04,
                safety_distance=safety_distance,
                allowed_geom_ids=getattr(self.collision_checker, "allowed_target_geom_ids", None),
            )
            if result.in_collision:
                raise CollisionPlanningError("Direct path collides with the robot or environment.", details=result.contacts)
        velocity_scale = float(move_request["constraints"]["velocity_scale"])
        duration = float(np.sum(np.max(np.abs(np.diff(waypoints, axis=0)), axis=1)) / max(velocity_scale, 0.01))
        return {
            "planner": self.planner_id,
            "path_type": path_request["path_type"],
            "waypoints": waypoints.tolist(),
            "duration": duration,
            "planning_time": time.perf_counter() - started,
            "diagnostics": path_request.get("diagnostics", {}),
        }


def _densify(points: np.ndarray, max_step: float) -> np.ndarray:
    output = [points[0]]
    for start, goal in zip(points[:-1], points[1:]):
        count = max(1, int(np.ceil(np.max(np.abs(goal - start)) / max_step)))
        output.extend(start + alpha * (goal - start) for alpha in np.linspace(0.0, 1.0, count + 1)[1:])
    return np.asarray(output)
