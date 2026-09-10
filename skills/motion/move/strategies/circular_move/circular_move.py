"""Cartesian circular path strategy."""

from __future__ import annotations

import math
from typing import Any, Mapping

import numpy as np

from ..pose_utils import effective_orientation
from ...planners.errors import PlanningError


class CircularMove:
    """Translate a normalized Move request into a Cartesian arc specification."""

    strategy_id = "circular_move"

    def __init__(self, config: Mapping[str, Any] | None = None, *, kinematics: Any = None) -> None:
        self.config = config or {}
        self.kinematics = kinematics

    def build_path_request(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        target = request["target"]
        if target["type"] != "pose":
            raise ValueError("Circular Move requires a pose target.")
        params = request["strategy_params"]
        start_q = np.asarray(context["current_joint_state"], dtype=float)
        start = np.asarray(context["current_pose"]["position"], dtype=float)
        goal = np.array([target["position"][key] for key in ("x", "y", "z")], dtype=float)
        orientation, hold_orientation = effective_orientation(request, context)
        if params.get("via_point") is not None:
            via = _vector(params["via_point"], "via_point")
            center, axis = _circle_from_three_points(start, via, goal)
            total_angle = _angle_through_via(start - center, via - center, goal - center, axis)
        else:
            center = _vector(params["arc_center"], "arc_center")
            axis = _vector(params["arc_axis"], "arc_axis")
            axis /= np.linalg.norm(axis)
            total_angle = float(params["arc_angle"])
            if abs(total_angle) > 2.0 * math.pi + 1e-6:
                total_angle = math.radians(total_angle)
            predicted = center + _rotate(start - center, axis, total_angle)
            if np.linalg.norm(predicted - goal) > 0.03:
                raise PlanningError("arc_center/axis/angle does not reach the target position.", code="INVALID_ARC_DEFINITION", recoverable=True)
        radius = float(np.linalg.norm(start - center))
        if radius < 1e-5 or abs(total_angle) < 1e-5:
            raise PlanningError("Circular arc is degenerate.", code="INVALID_ARC_DEFINITION", recoverable=True)
        step = float(params.get("cartesian_step", 0.01))
        count = max(3, int(np.ceil(radius * abs(total_angle) / step)) + 1)
        joints = [start_q]
        seed = start_q
        for angle in np.linspace(0.0, total_angle, count)[1:]:
            point = center + _rotate(start - center, axis, angle)
            pose = {"position": point, "orientation": orientation}
            result = self.kinematics.solve(pose, seed=seed, method=params.get("ik_method", "trac_ik"), timeout=min(float(request["planning"]["planning_time"]), 1.0))
            seed = np.asarray(result["solution"], dtype=float)
            joints.append(seed)
        points = np.asarray(joints)
        return {
            "path_type": "circular", "joint_waypoints": points, "start": points[0], "goal": points[-1],
            "orientation_constraint": {"enabled": hold_orientation, "orientation": orientation},
            "diagnostics": {"center": center.tolist(), "axis": axis.tolist(), "angle": total_angle},
        }


def _vector(value: Any, field: str) -> np.ndarray:
    if isinstance(value, Mapping):
        vector = np.array([value[key] for key in ("x", "y", "z")], dtype=float)
    else:
        vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise PlanningError(f"{field} must contain three finite values.", code="INVALID_ARC_DEFINITION", recoverable=True)
    return vector


def _rotate(vector: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    return vector * math.cos(angle) + np.cross(axis, vector) * math.sin(angle) + axis * np.dot(axis, vector) * (1.0 - math.cos(angle))


def _circle_from_three_points(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ab, ac = b - a, c - a
    normal = np.cross(ab, ac)
    normal_sq = float(np.dot(normal, normal))
    if normal_sq < 1e-12:
        raise PlanningError("Start, via, and target points are collinear.", code="INVALID_ARC_DEFINITION", recoverable=True)
    center = a + (np.cross(normal, ab) * np.dot(ac, ac) + np.cross(ac, normal) * np.dot(ab, ab)) / (2.0 * normal_sq)
    return center, normal / math.sqrt(normal_sq)


def _signed_angle(a: np.ndarray, b: np.ndarray, axis: np.ndarray) -> float:
    return math.atan2(float(np.dot(axis, np.cross(a, b))), float(np.dot(a, b)))


def _angle_through_via(start: np.ndarray, via: np.ndarray, goal: np.ndarray, axis: np.ndarray) -> float:
    via_angle = _signed_angle(start, via, axis) % (2.0 * math.pi)
    goal_angle = _signed_angle(start, goal, axis) % (2.0 * math.pi)
    return goal_angle if via_angle <= goal_angle else goal_angle - 2.0 * math.pi
