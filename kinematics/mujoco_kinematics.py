"""UR5e kinematics backed by the active MuJoCo model."""

from __future__ import annotations

import importlib
import itertools
import math
import time
from typing import Any, Mapping, Sequence

import mujoco
import numpy as np


class IKError(RuntimeError):
    """Raised when no requested IK backend can solve a target."""


def _rotation_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    relative = target @ current.T
    angle = math.acos(float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([relative[2, 1] - relative[1, 2], relative[0, 2] - relative[2, 0], relative[1, 0] - relative[0, 1]])
    axis /= max(2.0 * math.sin(angle), 1e-9)
    return axis * angle


class MujocoKinematics:
    """FK/Jacobian service plus IKFast and TRAC-IK-compatible solving."""

    def __init__(self, runtime: Any) -> None:
        self.runtime = runtime
        self.model = runtime.model
        self.site_id = runtime.end_effector_site_id
        self.qpos_indices = runtime.qpos_indices
        self.dof_indices = runtime.dof_indices
        self.limits = runtime.joint_limits
        self._work_data = mujoco.MjData(self.model)

    def forward(self, joints: Sequence[float]) -> dict[str, np.ndarray]:
        self._set_work_joints(joints)
        return {"position": self._work_data.site_xpos[self.site_id].copy(), "rotation_matrix": self._work_data.site_xmat[self.site_id].reshape(3, 3).copy()}

    def jacobian(self, joints: Sequence[float]) -> np.ndarray:
        self._set_work_joints(joints)
        jac_pos = np.zeros((3, self.model.nv))
        jac_rot = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self._work_data, jac_pos, jac_rot, self.site_id)
        return np.vstack((jac_pos[:, self.dof_indices], jac_rot[:, self.dof_indices]))

    def solve(self, target_pose: Mapping[str, Any], *, seed: Sequence[float] | None = None, method: str = "auto", timeout: float = 1.0) -> dict[str, Any]:
        target_position, target_rotation = self._parse_pose(target_pose)
        seed_array = np.asarray(self.runtime.get_joint_positions() if seed is None else seed, dtype=float)
        errors = []
        if method in {"auto", "ikfast"}:
            try:
                solutions = self._solve_ikfast(target_position, target_rotation)
                if solutions:
                    solutions = [self._normalize_to_reference(q, seed_array) for q in solutions]
                    selected = min(solutions, key=lambda q: np.linalg.norm(q - seed_array))
                    return {"solution": selected, "solutions": solutions, "solver": "ikfast"}
            except Exception as exc:
                errors.append(f"IKFast: {exc}")
                if method == "ikfast":
                    raise IKError(errors[-1]) from exc
        if method in {"auto", "trac_ik"}:
            try:
                solution, residual = self._solve_trac_ik_compatible(target_position, target_rotation, seed_array, timeout)
                return {"solution": solution, "solutions": [solution], "solver": "trac_ik", "backend": "mujoco_jacobian_compat", "residual": residual}
            except Exception as exc:
                errors.append(f"TRAC-IK: {exc}")
        raise IKError("; ".join(errors) or f"Unsupported IK method: {method}")

    def solve_candidates(
        self,
        target_pose: Mapping[str, Any],
        *,
        seed: Sequence[float] | None = None,
        method: str = "auto",
        timeout: float = 2.0,
        max_solutions: int = 8,
    ) -> list[np.ndarray]:
        """Return distinct IK branches instead of stopping at the first solution.

        Numerical IK is local, so a deterministic set of starts is explored.  The
        caller remains responsible for collision and path-continuity filtering.
        """
        target_position, target_rotation = self._parse_pose(target_pose)
        reference = np.asarray(self.runtime.get_joint_positions() if seed is None else seed, dtype=float)
        solutions: list[np.ndarray] = []

        if method in {"auto", "ikfast"}:
            try:
                for candidate in self._solve_ikfast(target_position, target_rotation):
                    self._append_distinct(solutions, candidate)
            except Exception:
                if method == "ikfast":
                    raise

        if method in {"auto", "trac_ik"} and len(solutions) < max_solutions:
            deadline = time.perf_counter() + max(timeout, 0.1)
            rng = np.random.default_rng(7)
            starts = [reference]
            starts.extend(rng.uniform(self.limits[:, 0], self.limits[:, 1]) for _ in range(24))
            for start in starts:
                candidate = self._solve_numerical_from_start(target_position, target_rotation, start, deadline)
                if candidate is not None:
                    self._append_distinct(solutions, candidate)
                    if len(solutions) >= max_solutions:
                        break
                if time.perf_counter() >= deadline:
                    break

        solutions.sort(key=lambda q: float(np.linalg.norm(self._normalize_to_reference(q, reference) - reference)))
        return solutions[:max_solutions]

    def _solve_ikfast(self, position: np.ndarray, rotation: np.ndarray) -> list[np.ndarray]:
        module = importlib.import_module("libraries.ikfast.ur5e_backend")
        solutions = []
        for raw in module.solve(position, rotation):
            candidate = np.asarray(raw, dtype=float)
            if candidate.shape == (6,) and self._within_limits(candidate):
                solutions.append(candidate)
        return solutions

    def _solve_trac_ik_compatible(self, target_position: np.ndarray, target_rotation: np.ndarray, seed: np.ndarray, timeout: float) -> tuple[np.ndarray, float]:
        deadline = time.perf_counter() + max(timeout, 0.05)
        rng = np.random.default_rng(7)
        starts = [seed] + [rng.uniform(self.limits[:, 0], self.limits[:, 1]) for _ in range(12)]
        best: tuple[float, np.ndarray] | None = None
        for start in starts:
            q = np.clip(np.asarray(start, dtype=float), self.limits[:, 0], self.limits[:, 1])
            for _ in range(300):
                pose = self.forward(q)
                error = np.concatenate((target_position - pose["position"], _rotation_error(target_rotation, pose["rotation_matrix"])))
                weighted = error.copy()
                weighted[3:] *= 0.35
                residual = float(np.linalg.norm(weighted))
                if best is None or residual < best[0]:
                    best = (residual, q.copy())
                if np.linalg.norm(error[:3]) < 2e-4 and np.linalg.norm(error[3:]) < 2e-3:
                    return self._normalize_to_reference(q, seed), residual
                jac = self.jacobian(q)
                jac[3:] *= 0.35
                damping = 2e-3 + min(residual, 0.2) * 0.03
                delta = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6) * damping * damping, weighted)
                scale = min(1.0, 0.18 / max(float(np.max(np.abs(delta))), 1e-9))
                q = np.clip(q + delta * scale, self.limits[:, 0], self.limits[:, 1])
                if time.perf_counter() >= deadline:
                    break
            if time.perf_counter() >= deadline:
                break
        residual = best[0] if best else float("inf")
        raise IKError(f"No numerical IK solution; best residual={residual:.6f}")

    def _solve_numerical_from_start(
        self,
        target_position: np.ndarray,
        target_rotation: np.ndarray,
        start: Sequence[float],
        deadline: float,
    ) -> np.ndarray | None:
        q = np.clip(np.asarray(start, dtype=float), self.limits[:, 0], self.limits[:, 1])
        for _ in range(300):
            pose = self.forward(q)
            error = np.concatenate((target_position - pose["position"], _rotation_error(target_rotation, pose["rotation_matrix"])))
            if np.linalg.norm(error[:3]) < 2e-4 and np.linalg.norm(error[3:]) < 2e-3:
                return self._normalize_to_reference(q, start)
            weighted = error.copy()
            weighted[3:] *= 0.35
            residual = float(np.linalg.norm(weighted))
            jac = self.jacobian(q)
            jac[3:] *= 0.35
            damping = 2e-3 + min(residual, 0.2) * 0.03
            delta = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6) * damping * damping, weighted)
            scale = min(1.0, 0.18 / max(float(np.max(np.abs(delta))), 1e-9))
            q = np.clip(q + delta * scale, self.limits[:, 0], self.limits[:, 1])
            if time.perf_counter() >= deadline:
                return None
        return None

    @staticmethod
    def _append_distinct(solutions: list[np.ndarray], candidate: Sequence[float], tolerance: float = 1e-3) -> None:
        value = np.asarray(candidate, dtype=float)
        if not any(np.linalg.norm((value - existing + math.pi) % (2.0 * math.pi) - math.pi) < tolerance for existing in solutions):
            solutions.append(value.copy())

    def equivalent_configurations(
        self, joints: Sequence[float], reference: Sequence[float], max_variants: int = 8,
    ) -> list[np.ndarray]:
        """Return limit-valid 2*pi representations ordered by joint travel."""
        value = np.asarray(joints, dtype=float)
        target = np.asarray(reference, dtype=float)
        choices: list[np.ndarray] = []
        for index in range(len(value)):
            equivalents = value[index] + 2.0 * math.pi * np.arange(-2, 3)
            valid = equivalents[
                (equivalents >= self.limits[index, 0] - 1e-9)
                & (equivalents <= self.limits[index, 1] + 1e-9)
            ]
            choices.append(valid)
        variants = [np.asarray(items, dtype=float) for items in itertools.product(*choices)]
        variants.sort(key=lambda q: float(np.linalg.norm(q - target)))
        return variants[:max_variants]

    def _normalize_to_reference(self, joints: Sequence[float], reference: Sequence[float]) -> np.ndarray:
        """Choose the limit-valid 2*pi representation nearest to reference.

        MuJoCo's revolute joints use bounded numeric coordinates even when
        multiple coordinates describe the same physical angle.  Numerical IK
        may therefore return an equivalent value one full turn away.
        """
        value = np.asarray(joints, dtype=float).copy()
        target = np.asarray(reference, dtype=float)
        for index in range(len(value)):
            equivalents = value[index] + 2.0 * math.pi * np.arange(-2, 3)
            valid = equivalents[
                (equivalents >= self.limits[index, 0] - 1e-9)
                & (equivalents <= self.limits[index, 1] + 1e-9)
            ]
            if len(valid):
                value[index] = valid[int(np.argmin(np.abs(valid - target[index])))]
        return value

    def _set_work_joints(self, joints: Sequence[float]) -> None:
        values = np.asarray(joints, dtype=float)
        if values.shape != (6,):
            raise ValueError("UR5e joint vector must contain 6 values.")
        self._work_data.qpos[:] = self.runtime.data.qpos
        self._work_data.qpos[self.qpos_indices] = values
        mujoco.mj_forward(self.model, self._work_data)

    def _within_limits(self, joints: np.ndarray) -> bool:
        return bool(np.all(joints >= self.limits[:, 0]) and np.all(joints <= self.limits[:, 1]))

    def _parse_pose(self, pose: Mapping[str, Any]) -> tuple[np.ndarray, np.ndarray]:
        value = pose.get("position")
        position = np.array([value[k] for k in ("x", "y", "z")], dtype=float) if isinstance(value, Mapping) else np.asarray(value, dtype=float)
        if position.shape != (3,):
            raise ValueError("Pose position must contain x, y, z.")
        orientation = pose.get("orientation")
        if not orientation:
            rotation = self.runtime.get_end_effector_pose()["rotation_matrix"]
        elif orientation.get("representation", "quaternion") == "rpy":
            roll, pitch, yaw = np.radians([orientation[k] for k in ("roll", "pitch", "yaw")])
            cr, cp, cy = np.cos([roll, pitch, yaw]); sr, sp, sy = np.sin([roll, pitch, yaw])
            rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
        else:
            quaternion = np.array([orientation[k] for k in ("w", "x", "y", "z")], dtype=float)
            quaternion /= np.linalg.norm(quaternion)
            flat = np.empty(9); mujoco.mju_quat2Mat(flat, quaternion); rotation = flat.reshape(3, 3)
        return position, rotation
