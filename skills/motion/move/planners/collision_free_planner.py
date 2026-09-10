"""Dependency-free bidirectional RRT-Connect planner."""

from __future__ import annotations

import time
from typing import Any, Mapping

import numpy as np

from .errors import PlanningError


class CollisionFreePlanner:
    """Search for a safe alternative trajectory while preserving legal constraints."""

    planner_id = "collision_free"

    def __init__(self, config: Mapping[str, Any] | None = None, *, collision_checker: Any = None, joint_limits: Any = None, kinematics: Any = None) -> None:
        self.config = config or {}
        self.collision_checker = collision_checker
        self.joint_limits = np.asarray(joint_limits, dtype=float)
        self.kinematics = kinematics

    def plan(
        self,
        path_request: Mapping[str, Any],
        move_request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Any:
        if move_request["motion"]["path_constraint"] == "hard" and path_request["path_type"] != "joint":
            raise PlanningError("A hard Cartesian path cannot be relaxed for collision avoidance.", code="PATH_CONSTRAINT_INFEASIBLE", recoverable=True)
        started = time.perf_counter()
        deadline = started + float(move_request["planning"]["planning_time"])
        attempts = int(move_request["planning"]["max_attempts"])
        start = np.asarray(path_request["start"], dtype=float)
        goal = np.asarray(path_request["goal"], dtype=float)
        safety_distance = float(move_request["constraints"].get("safety_distance", 0.0))
        if self.collision_checker.check(start, safety_distance=safety_distance).in_collision:
            raise PlanningError("Start state is in collision.", code="COLLISION_DETECTED", recoverable=True)
        if self.collision_checker.check(goal, safety_distance=safety_distance).in_collision:
            raise PlanningError("Goal state is in collision.", code="COLLISION_DETECTED", recoverable=True)
        orientation_constraint = path_request.get("orientation_constraint", {})
        if orientation_constraint.get("enabled"):
            if self.kinematics is None:
                raise PlanningError("Fixed-orientation collision planning requires a kinematics service.", code="PLANNING_FAILED")
            return self._plan_fixed_orientation(
                start, goal, move_request, context, orientation_constraint["orientation"], deadline, attempts, safety_distance, started,
            )
        rng = np.random.default_rng(17)
        path = None
        for _ in range(attempts):
            path = self._rrt_connect(start, goal, deadline, rng, safety_distance)
            if path is not None:
                break
        if path is None:
            raise PlanningError("RRT-Connect could not find a collision-free path within the planning budget.", code="PATH_NOT_FOUND", recoverable=True)
        path = self._shortcut(path, rng, safety_distance, iterations=120)
        waypoints = _densify(np.asarray(path), 0.06)
        final_check = self.collision_checker.check_path(waypoints, resolution=0.02, safety_distance=safety_distance)
        if final_check.in_collision:
            raise PlanningError("Final RRT path failed collision validation.", code="TRAJECTORY_INVALID", recoverable=True)
        return {
            "planner": self.planner_id,
            "path_type": "joint",
            "original_path_type": path_request["path_type"],
            "waypoints": waypoints.tolist(),
            "planning_time": time.perf_counter() - started,
            "duration": float(np.sum(np.max(np.abs(np.diff(waypoints, axis=0)), axis=1)) / max(float(move_request["constraints"]["velocity_scale"]), 0.01)),
            "diagnostics": {"algorithm": "rrt_connect", "path_relaxed": path_request["path_type"] != "joint"},
        }

    def _plan_fixed_orientation(
        self,
        start_q: np.ndarray,
        goal_q: np.ndarray,
        move_request: Mapping[str, Any],
        context: Mapping[str, Any],
        orientation: Mapping[str, Any],
        deadline: float,
        attempts: int,
        safety_distance: float,
        started: float,
    ) -> dict[str, Any]:
        """Plan a translational detour while keeping one Cartesian orientation.

        The tree lives in end-effector position space.  Every edge is converted
        to a dense, continuous IK chain before being accepted, so unlike the
        joint-space RRT fallback it cannot introduce an unconstrained wrist turn.
        """
        start_position = np.asarray(context["current_pose"]["position"], dtype=float)
        goal_position = np.array([move_request["target"]["position"][key] for key in ("x", "y", "z")], dtype=float)
        cartesian_step = float(move_request["strategy_params"].get("cartesian_step", 0.01))
        rng = np.random.default_rng(17)
        nodes: list[tuple[np.ndarray, np.ndarray, int]] = [(start_position, start_q, -1)]
        goal_index: int | None = None
        # A bounded workspace region centred on the requested move keeps the
        # sampler useful without assuming a robot-specific global workspace.
        margin = max(0.20, min(0.50, np.linalg.norm(goal_position - start_position) + 0.10))
        lower = np.minimum(start_position, goal_position) - margin
        upper = np.maximum(start_position, goal_position) + margin
        for _ in range(max(1, attempts) * 900):
            if time.perf_counter() >= deadline:
                break
            sample = goal_position if rng.random() < 0.20 else rng.uniform(lower, upper)
            parent_index = int(np.argmin([np.linalg.norm(node[0] - sample) for node in nodes]))
            parent_position, parent_q, _ = nodes[parent_index]
            delta = sample - parent_position
            length = float(np.linalg.norm(delta))
            if length < 1e-9:
                continue
            candidate = sample if length <= 0.08 else parent_position + delta / length * 0.08
            edge = self._fixed_orientation_edge(parent_position, parent_q, candidate, orientation, cartesian_step, safety_distance, move_request)
            if edge is None:
                continue
            nodes.append((candidate, edge[-1], parent_index))
            candidate_index = len(nodes) - 1
            if np.linalg.norm(candidate - goal_position) <= 0.08:
                final_edge = self._fixed_orientation_edge(candidate, edge[-1], goal_position, orientation, cartesian_step, safety_distance, move_request)
                if final_edge is not None:
                    nodes.append((goal_position, final_edge[-1], candidate_index))
                    goal_index = len(nodes) - 1
                    break
        if goal_index is None:
            raise PlanningError("No collision-free path exists with the end-effector orientation fixed.", code="PATH_NOT_FOUND", recoverable=True)
        chain: list[tuple[np.ndarray, np.ndarray]] = []
        index = goal_index
        while index >= 0:
            position, joints, parent = nodes[index]
            chain.append((position, joints))
            index = parent
        chain.reverse()
        waypoints = [chain[0][1]]
        for (position_a, joints_a), (position_b, _) in zip(chain[:-1], chain[1:]):
            edge = self._fixed_orientation_edge(position_a, joints_a, position_b, orientation, cartesian_step, safety_distance, move_request)
            if edge is None:  # Defensive: a previously validated edge became invalid.
                raise PlanningError("Fixed-orientation path could not be reconstructed.", code="TRAJECTORY_INVALID", recoverable=True)
            waypoints.extend(edge[1:])
        output = np.asarray(waypoints)
        return {
            "planner": self.planner_id,
            "path_type": "cartesian_fixed_orientation",
            "original_path_type": "joint",
            "waypoints": output.tolist(),
            "planning_time": time.perf_counter() - started,
            "duration": float(np.sum(np.max(np.abs(np.diff(output, axis=0)), axis=1)) / max(float(move_request["constraints"]["velocity_scale"]), 0.01)),
            "diagnostics": {"algorithm": "cartesian_rrt_fixed_orientation", "orientation_fixed": True},
        }

    def _fixed_orientation_edge(
        self,
        start_position: np.ndarray,
        seed: np.ndarray,
        goal_position: np.ndarray,
        orientation: Mapping[str, Any],
        step: float,
        safety_distance: float,
        move_request: Mapping[str, Any],
    ) -> list[np.ndarray] | None:
        count = max(1, int(np.ceil(np.linalg.norm(goal_position - start_position) / step)))
        result = [np.asarray(seed, dtype=float)]
        current = result[0]
        for alpha in np.linspace(0.0, 1.0, count + 1)[1:]:
            point = start_position + alpha * (goal_position - start_position)
            try:
                solved = self.kinematics.solve(
                    {"position": point, "orientation": orientation}, seed=current,
                    method=move_request["strategy_params"].get("ik_method", "trac_ik"), timeout=0.25,
                )
            except Exception:
                return None
            candidate = np.asarray(solved["solution"], dtype=float)
            if self.collision_checker.check_path([current, candidate], resolution=0.02, safety_distance=safety_distance).in_collision:
                return None
            current = candidate
            result.append(current)
        return result

    def _rrt_connect(self, start: np.ndarray, goal: np.ndarray, deadline: float, rng: np.random.Generator, safety_distance: float) -> list[np.ndarray] | None:
        tree_a = _Tree(start)
        tree_b = _Tree(goal)
        swapped = False
        while time.perf_counter() < deadline:
            sample = goal if rng.random() < 0.15 else rng.uniform(self.joint_limits[:, 0], self.joint_limits[:, 1])
            new_index = self._extend_once(tree_a, sample, safety_distance)
            if new_index is not None:
                target = tree_a.nodes[new_index]
                connected_index = self._connect(tree_b, target, safety_distance)
                if connected_index is not None and np.linalg.norm(tree_b.nodes[connected_index] - target) < 1e-6:
                    path_a = tree_a.path(new_index)
                    path_b = tree_b.path(connected_index)
                    return path_a + list(reversed(path_b[:-1])) if not swapped else path_b + list(reversed(path_a[:-1]))
            tree_a, tree_b = tree_b, tree_a
            swapped = not swapped
        return None

    def _extend_once(self, tree: "_Tree", target: np.ndarray, safety_distance: float) -> int | None:
        distances = [np.linalg.norm(node - target) for node in tree.nodes]
        parent = int(np.argmin(distances))
        source = tree.nodes[parent]
        delta = target - source
        distance = float(np.linalg.norm(delta))
        if distance < 1e-9:
            return parent
        candidate = target.copy() if distance <= 0.22 else source + delta / distance * 0.22
        if self.collision_checker.check_path([source, candidate], resolution=0.02, safety_distance=safety_distance).in_collision:
            return None
        return tree.add(candidate, parent)

    def _connect(self, tree: "_Tree", target: np.ndarray, safety_distance: float) -> int | None:
        last = None
        for _ in range(80):
            last = self._extend_once(tree, target, safety_distance)
            if last is None:
                return None
            if np.linalg.norm(tree.nodes[last] - target) < 1e-6:
                return last
        return None

    def _shortcut(self, path: list[np.ndarray], rng: np.random.Generator, safety_distance: float, *, iterations: int) -> list[np.ndarray]:
        result = list(path)
        for _ in range(iterations):
            if len(result) < 3:
                break
            first, second = sorted(rng.choice(len(result), size=2, replace=False).tolist())
            if second - first < 2:
                continue
            if not self.collision_checker.check_path([result[first], result[second]], resolution=0.02, safety_distance=safety_distance).in_collision:
                result = result[:first + 1] + result[second:]
        return result


class _Tree:
    def __init__(self, root: np.ndarray) -> None:
        self.nodes = [root.copy()]
        self.parents = [-1]

    def add(self, node: np.ndarray, parent: int) -> int:
        self.nodes.append(node.copy())
        self.parents.append(parent)
        return len(self.nodes) - 1

    def path(self, index: int) -> list[np.ndarray]:
        result = []
        while index >= 0:
            result.append(self.nodes[index])
            index = self.parents[index]
        return list(reversed(result))


def _densify(points: np.ndarray, max_step: float) -> np.ndarray:
    output = [points[0]]
    for start, goal in zip(points[:-1], points[1:]):
        count = max(1, int(np.ceil(np.max(np.abs(goal - start)) / max_step)))
        output.extend(start + alpha * (goal - start) for alpha in np.linspace(0.0, 1.0, count + 1)[1:])
    return np.asarray(output)
