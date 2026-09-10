"""Cartesian linear path strategy with sequential numerical IK."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from ..pose_utils import effective_orientation
from ...planners.errors import PlanningError


class LinearMove:
    """Translate a normalized Move request into a Cartesian line specification."""

    strategy_id = "linear_move"

    def __init__(self, config: Mapping[str, Any] | None = None, *, kinematics: Any = None, collision_checker: Any = None) -> None:
        self.config = config or {}
        self.kinematics = kinematics
        self.collision_checker = collision_checker

    def build_path_request(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        target = request["target"]
        if target["type"] != "pose":
            raise ValueError("Linear Move requires a pose target.")
        start_q = np.asarray(context["current_joint_state"], dtype=float)
        start_position = np.asarray(context["current_pose"]["position"], dtype=float)
        goal_position = np.array([target["position"][key] for key in ("x", "y", "z")], dtype=float)
        orientation, hold_orientation = effective_orientation(request, context)
        step = float(request["strategy_params"].get("cartesian_step", 0.01))
        count = max(2, int(np.ceil(np.linalg.norm(goal_position - start_position) / step)) + 1)
        params = request["strategy_params"]
        seed_value = params.get("ik_seed")
        seed = np.asarray(start_q if seed_value is None else seed_value, dtype=float)
        if seed.shape != (6,) or not np.all(np.isfinite(seed)):
            raise ValueError("strategy_params.ik_seed must contain six finite joint values.")
        method = params.get("ik_method", "trac_ik")
        poses = [
            {"position": start_position + alpha * (goal_position - start_position), "orientation": orientation}
            for alpha in np.linspace(0.0, 1.0, count)[1:]
        ]

        # An explicit seed remains an authoritative branch hint.  Otherwise,
        # enumerate target-pose branches and validate each complete Cartesian
        # path before selecting the smoothest collision-free one.
        if seed_value is not None:
            branch_seeds = [seed]
        else:
            physical_branches = self.kinematics.solve_candidates(
                poses[-1], seed=start_q, method=method,
                timeout=min(float(request["planning"]["planning_time"]), 3.0), max_solutions=8,
            )
            branch_seeds = []
            for physical_branch in physical_branches:
                branch_seeds.extend(self.kinematics.equivalent_configurations(physical_branch, start_q, max_variants=8))
            branch_seeds.sort(key=lambda q: float(np.linalg.norm(q - start_q)))
            branch_seeds = branch_seeds[:32]
            if not branch_seeds:
                branch_seeds = [seed]

        candidates = []
        colliding_candidates = []
        rejected = []
        for branch_index, branch_seed in enumerate(branch_seeds):
            joints = [start_q]
            current_seed = np.asarray(branch_seed, dtype=float)
            ik_diagnostics = []
            first_collision = None
            try:
                for pose in poses:
                    result = self.kinematics.solve(
                        pose, seed=current_seed, method=method,
                        timeout=min(float(request["planning"]["planning_time"]), 1.0),
                    )
                    next_q = np.asarray(result["solution"], dtype=float)
                    previous_q = joints[-1]
                    joints.append(next_q)
                    current_seed = next_q
                    ik_diagnostics.append({key: value for key, value in result.items() if key not in {"solution", "solutions"}})
                    if first_collision is None and request["constraints"]["avoid_collision"] and self.collision_checker is not None:
                        collision = self.collision_checker.check_path(
                            [previous_q, next_q], resolution=0.04,
                            safety_distance=float(request["constraints"].get("safety_distance", 0.0)),
                        )
                        if collision.in_collision:
                            first_collision = collision
            except Exception as exc:
                rejected.append({"branch": branch_index, "reason": f"ik_failed: {exc}"})
                continue
            if len(joints) != len(poses) + 1:
                continue
            points = np.asarray(joints)
            deltas = np.diff(points, axis=0)
            score = float(np.sum(np.linalg.norm(deltas, axis=1)) + 2.0 * np.max(np.abs(deltas)))
            candidate = (score, points, ik_diagnostics, branch_index)
            if first_collision is None:
                candidates.append(candidate)
            else:
                rejected.append({"branch": branch_index, "reason": "collision", "contacts": list(first_collision.contacts)})
                colliding_candidates.append(candidate)

        if not candidates:
            if colliding_candidates:
                # A soft Cartesian request may legally relax its path.  Pass
                # the best reachable goal branch to DirectPlanner so its
                # collision result can trigger the configured RRT fallback.
                _, points, ik_diagnostics, selected_branch = min(colliding_candidates, key=lambda item: item[0])
            else:
                raise PlanningError(
                    f"No continuous IK branch reaches the target ({len(rejected)} branches rejected).",
                    code="PATH_CONSTRAINT_INFEASIBLE", recoverable=True,
                    details={
                        "candidate_count": len(branch_seeds),
                        "rejected_count": len(rejected),
                        "reason_counts": {
                            reason: sum(1 for item in rejected if str(item.get("reason", "")).split(":", 1)[0] == reason)
                            for reason in {str(item.get("reason", "")).split(":", 1)[0] for item in rejected}
                        },
                        "samples": rejected[:3],
                    },
                )
        else:
            _, points, ik_diagnostics, selected_branch = min(candidates, key=lambda item: item[0])
        return {
            "path_type": "linear", "joint_waypoints": points, "start": points[0], "goal": points[-1],
            "orientation_constraint": {"enabled": hold_orientation, "orientation": orientation},
            "diagnostics": {"ik": ik_diagnostics, "ik_branch_selection": {
                "candidate_count": len(branch_seeds), "rejected": rejected, "selected_branch": selected_branch,
                "explicit_seed": seed_value is not None,
            }},
        }
