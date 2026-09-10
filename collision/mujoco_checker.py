"""Collision checking isolated from the live MuJoCo simulation state."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import mujoco
import numpy as np


@dataclass(frozen=True)
class CollisionResult:
    in_collision: bool
    minimum_distance: float
    contacts: tuple[dict[str, object], ...] = ()


class MujocoCollisionChecker:
    """Check contacts involving the robot while ignoring environment-only contacts."""

    def __init__(self, runtime: object) -> None:
        self.runtime = runtime
        self.model = runtime.model
        self._data = mujoco.MjData(self.model)
        first_joint_body = int(self.model.jnt_bodyid[runtime.joint_ids[0]])
        self.robot_root_body = int(self.model.body_parentid[first_joint_body])
        self.robot_body_ids = self._descendants(self.robot_root_body)
        self.robot_geom_ids = {
            geom_id for geom_id in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom_id]) in self.robot_body_ids
        }
        controller = getattr(runtime, "gripper_controller", None)
        held_body_id = getattr(controller, "_held_body_id", None)
        if held_body_id is not None:
            self.robot_geom_ids.update(
                geom_id for geom_id in range(self.model.ngeom)
                if int(self.model.geom_bodyid[geom_id]) == held_body_id
            )
        self.baseline_allowed_pairs = self._initial_robot_contact_pairs()

    def check(self, joints: Sequence[float], *, safety_distance: float = 0.0, allowed_geom_ids: set[int] | None = None) -> CollisionResult:
        if allowed_geom_ids is None:
            allowed_geom_ids = getattr(self, "allowed_target_geom_ids", None)
        values = np.asarray(joints, dtype=float)
        if values.shape != (6,):
            raise ValueError("Collision check requires a 6-value UR5e joint vector.")
        self._data.qpos[:] = self.runtime.data.qpos
        self._data.qpos[self.runtime.qpos_indices] = values
        mujoco.mj_forward(self.model, self._data)
        controller = getattr(self.runtime, "gripper_controller", None)
        held_geom_ids: set[int] = set()
        if controller is not None:
            controller.sync_held_object(self._data)
            held_body_id = getattr(controller, "_held_body_id", None)
            if held_body_id is not None:
                held_geom_ids = {
                    geom_id for geom_id in range(self.model.ngeom)
                    if int(self.model.geom_bodyid[geom_id]) == held_body_id
                }
        active_robot_geom_ids = self.robot_geom_ids | held_geom_ids
        contacts = []
        minimum = float("inf")
        for index in range(self._data.ncon):
            contact = self._data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 not in active_robot_geom_ids and geom2 not in active_robot_geom_ids:
                continue
            if allowed_geom_ids and (geom1 in allowed_geom_ids or geom2 in allowed_geom_ids):
                continue
            if self._is_held_object_pad_contact(geom1, geom2, held_geom_ids):
                continue
            if frozenset((geom1, geom2)) in self.baseline_allowed_pairs:
                continue
            distance = float(contact.dist)
            minimum = min(minimum, distance)
            if distance <= safety_distance:
                contacts.append({
                    "geom1": self._geom_name(geom1),
                    "geom2": self._geom_name(geom2),
                    "distance": distance,
                    "self_collision": geom1 in active_robot_geom_ids and geom2 in active_robot_geom_ids,
                })
        return CollisionResult(bool(contacts), minimum, tuple(contacts))

    def check_path(self, waypoints: Sequence[Sequence[float]], *, resolution: float = 0.05, safety_distance: float = 0.0, allowed_geom_ids: set[int] | None = None) -> CollisionResult:
        if allowed_geom_ids is None:
            allowed_geom_ids = getattr(self, "allowed_target_geom_ids", None)
        points = np.asarray(waypoints, dtype=float)
        all_contacts = []
        minimum = float("inf")
        for start, goal in zip(points[:-1], points[1:]):
            count = max(2, int(np.ceil(np.max(np.abs(goal - start)) / resolution)) + 1)
            for alpha in np.linspace(0.0, 1.0, count):
                result = self.check(start + alpha * (goal - start), safety_distance=safety_distance, allowed_geom_ids=allowed_geom_ids)
                minimum = min(minimum, result.minimum_distance)
                if result.in_collision:
                    all_contacts.extend(result.contacts)
                    return CollisionResult(True, minimum, tuple(all_contacts))
        return CollisionResult(False, minimum, ())

    def _descendants(self, root: int) -> set[int]:
        result = {root}
        changed = True
        while changed:
            changed = False
            for body_id in range(1, self.model.nbody):
                if body_id not in result and int(self.model.body_parentid[body_id]) in result:
                    result.add(body_id)
                    changed = True
        return result

    def _initial_robot_contact_pairs(self) -> set[frozenset[int]]:
        self._data.qpos[:] = self.runtime.data.qpos
        mujoco.mj_forward(self.model, self._data)
        pairs = set()
        for index in range(self._data.ncon):
            contact = self._data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 in self.robot_geom_ids or geom2 in self.robot_geom_ids:
                pairs.add(frozenset((geom1, geom2)))
        return pairs

    def _geom_name(self, geom_id: int) -> str:
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"

    def _is_held_object_pad_contact(self, geom1: int, geom2: int, held_geom_ids: set[int]) -> bool:
        """Allow payload contact with its gripper, but not with the arm or scene."""
        if not held_geom_ids or (geom1 not in held_geom_ids and geom2 not in held_geom_ids):
            return False
        other = geom2 if geom1 in held_geom_ids else geom1
        body_id = int(self.model.geom_bodyid[other])
        body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        return body_name.startswith("robotiq_2f85")
