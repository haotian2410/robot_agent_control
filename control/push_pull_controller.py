"""Bounded passive-mechanism controller used by the MuJoCo Push/Pull Skill."""
from __future__ import annotations

import time
from typing import Any, Sequence

import mujoco
import numpy as np

from kinematics import IKError, MujocoKinematics


class MujocoPushPullController:
    """Animate a target hinge or slide joint while preserving hard path limits.

    This simulation adapter models a successful maintained tool contact. Real
    hardware adapters must replace it with measured contact and force feedback.
    """

    def __init__(self, runtime: Any) -> None:
        self.runtime = runtime
        self.model = runtime.model
        self.data = runtime.data
        self.kinematics = MujocoKinematics(runtime)
        first_joint_body = int(self.model.jnt_bodyid[runtime.joint_ids[0]])
        robot_root_body = int(self.model.body_parentid[first_joint_body])
        self.robot_body_ids = self._descendants(robot_root_body)
        self.robot_geom_ids = {geom_id for geom_id in range(self.model.ngeom)
                               if int(self.model.geom_bodyid[geom_id]) in self.robot_body_ids}

    def get_state(self, object_id: str, *, interaction_mode: str = "mechanism") -> dict[str, Any]:
        joint_id = self._joint_for_target(object_id)
        gripper = getattr(self.runtime, "gripper_controller", None)
        gripper_holding = bool(gripper and gripper.get_state(object_id).get("holding"))
        gripper_holding_any = bool(gripper and getattr(gripper, "_holding", False)
                                   and getattr(gripper, "_held_body_id", None) is not None)
        joint_type = None if joint_id is None else int(self.model.jnt_type[joint_id])
        if interaction_mode == "free_object":
            ready = joint_type == int(mujoco.mjtJoint.mjJNT_FREE) and not gripper_holding_any
        else:
            ready = joint_type in {int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)} and gripper_holding
        return {
            "ready": ready,
            "fault": False,
            "safe": True,
            "interaction_mode": interaction_mode,
            "gripper_holding": gripper_holding,
            "gripper_holding_any": gripper_holding_any,
            "joint_id": joint_id,
            "joint_type": joint_type,
            "joint_position": None if joint_id is None else self._joint_position(joint_id),
        }

    def execute_circular(self, *, operation: str, object_id: str, contact_normal: Sequence[float], arc_center: Sequence[float],
                         arc_axis: Sequence[float], arc_angle: float, frame: str,
                         maximum_force: float, contact_threshold: float, angular_speed: float,
                         maintain_contact: bool, stop_on_contact_loss: bool, timeout: float,
                         emergency_retract: bool, normal_alignment_tolerance: float,
                         avoid_collision: bool) -> dict[str, Any]:
        del operation, arc_center, frame, maximum_force, contact_threshold
        del maintain_contact, stop_on_contact_loss, emergency_retract
        joint_id = self._require_joint(object_id, mujoco.mjtJoint.mjJNT_HINGE)
        start = self._joint_position(joint_id)
        target = self._bounded_target(joint_id, start + float(arc_angle))
        started = time.perf_counter()
        alignment_error, collision_checks = self._animate_joint_with_robot(
            joint_id, object_id, start, target, abs(float(angular_speed)), float(timeout), started,
            contact_normal=np.asarray(contact_normal, dtype=float), rotation_axis=np.asarray(arc_axis, dtype=float),
            normal_alignment_tolerance_deg=float(normal_alignment_tolerance),
            avoid_collision=bool(avoid_collision),
            carried_objects=(),
        )
        actual = self._joint_position(joint_id) - start
        return self._result(object_id, actual_angle=actual, actual_distance=0.0, started=started, normal_alignment_error_deg=alignment_error, collision_checks=collision_checks)

    def execute_linear(self, *, operation: str, object_id: str, contact_normal: Sequence[float], direction: Sequence[float],
                       distance: float, frame: str, maximum_force: float,
                       contact_threshold: float, speed: float, maintain_contact: bool,
                       stop_on_contact_loss: bool, timeout: float,
                       emergency_retract: bool, normal_alignment_tolerance: float,
                       avoid_collision: bool, carried_objects: Sequence[str]) -> dict[str, Any]:
        del operation, frame, maximum_force, contact_threshold
        del maintain_contact, stop_on_contact_loss, emergency_retract
        joint_id = self._require_joint(object_id, mujoco.mjtJoint.mjJNT_SLIDE)
        start = self._joint_position(joint_id)
        requested_direction = np.asarray(direction, dtype=float)
        requested_direction /= np.linalg.norm(requested_direction)
        joint_axis = self._joint_axis_world(joint_id)
        alignment = float(np.dot(requested_direction, joint_axis))
        if abs(alignment) < 0.95:
            raise RuntimeError("Linear Push/Pull direction is not collinear with the target slide axis.")
        signed_distance = float(distance) if alignment > 0.0 else -float(distance)
        target = self._bounded_target(joint_id, start + signed_distance)
        started = time.perf_counter()
        alignment_error, collision_checks = self._animate_joint_with_robot(
            joint_id, object_id, start, target, abs(float(speed)), float(timeout), started,
            contact_normal=np.asarray(contact_normal, dtype=float), rotation_axis=None,
            normal_alignment_tolerance_deg=float(normal_alignment_tolerance),
            avoid_collision=bool(avoid_collision),
            carried_objects=carried_objects,
        )
        actual = self._joint_position(joint_id) - start
        return self._result(object_id, actual_angle=0.0, actual_distance=actual, started=started, normal_alignment_error_deg=alignment_error, collision_checks=collision_checks)

    def execute_free_object_push(self, *, object_id: str, contact_normal: Sequence[float],
                                 direction: Sequence[float], distance: float, frame: str,
                                 maximum_force: float, contact_threshold: float, speed: float,
                                 maintain_contact: bool, stop_on_contact_loss: bool, timeout: float,
                                 emergency_retract: bool, normal_alignment_tolerance: float,
                                 avoid_collision: bool) -> dict[str, Any]:
        """Translate a non-grasped free body while preserving an existing pad contact."""
        del frame, maximum_force, contact_threshold, emergency_retract
        joint_id = self._require_joint(object_id, mujoco.mjtJoint.mjJNT_FREE)
        body_id = self._target_body_id(object_id)
        direction = np.asarray(direction, dtype=float)
        direction /= np.linalg.norm(direction)
        contact_normal = np.asarray(contact_normal, dtype=float)
        contact_normal /= np.linalg.norm(contact_normal)
        if float(np.dot(direction, contact_normal)) > -np.cos(np.radians(normal_alignment_tolerance)):
            raise RuntimeError("Free-object push direction is not opposite the outward contact normal.")

        start_pose = self.runtime.get_end_effector_pose()
        start_position = np.asarray(start_pose["position"], dtype=float)
        start_rotation = np.asarray(start_pose["rotation_matrix"], dtype=float)
        maximum_alignment_error = self._normal_alignment_error_deg(start_rotation, contact_normal)
        if maximum_alignment_error > normal_alignment_tolerance + 1e-9:
            raise RuntimeError(
                f"End-effector tool axis is not collinear with the contact normal: {maximum_alignment_error:.6f} deg."
            )

        allowed_tool_pairs, allowed_support_pairs = self._free_push_contact_baseline(
            body_id, direction, contact_normal, normal_alignment_tolerance
        )
        if not allowed_tool_pairs:
            raise RuntimeError("Free-object push requires an existing finger-pad contact at the pre-contact pose.")

        duration = float(distance) / max(float(speed), 1e-6)
        if duration > timeout:
            raise TimeoutError("Free-object push exceeds the configured timeout.")
        playback_duration = duration / self.runtime.playback_speed if self.runtime.realtime else duration
        frames = max(2, int(np.ceil(playback_duration * self.runtime.playback_fps)) + 1)
        qpos_address = int(self.model.jnt_qposadr[joint_id])
        dof_address = int(self.model.jnt_dofadr[joint_id])
        start_object_qpos = self.data.qpos[qpos_address:qpos_address + 7].copy()
        seed = self.runtime.get_joint_positions()
        baseline_self_pairs = self._current_robot_self_contact_pairs()
        collision_checks = 0
        contact_maintained = True
        started = time.perf_counter()
        if avoid_collision:
            forbidden = self._forbidden_free_push_contacts(
                body_id, allowed_tool_pairs, allowed_support_pairs, baseline_self_pairs
            )
            collision_checks += 1
            if forbidden:
                raise RuntimeError(f"Free-object push starts in forbidden collision: {forbidden}")

        next_frame = time.perf_counter()
        for travel in np.linspace(0.0, float(distance), frames)[1:]:
            if time.perf_counter() - started > timeout:
                raise TimeoutError("Free-object push execution timed out.")
            previous_arm = seed.copy()
            previous_object = self.data.qpos[qpos_address:qpos_address + 7].copy()
            desired_position = start_position + direction * float(travel)
            try:
                solved = self.kinematics.solve(
                    {"position": desired_position, "orientation": self._rotation_mapping(start_rotation)},
                    seed=seed, method="trac_ik", timeout=min(0.25, timeout),
                )
            except IKError as exc:
                self._restore_free_push_state(qpos_address, dof_address, previous_object, previous_arm)
                raise RuntimeError(f"Arm cannot follow the free-object push line: {exc}") from exc
            seed = np.asarray(solved["solution"], dtype=float)
            self.data.qpos[self.runtime.qpos_indices] = seed
            self.data.qvel[self.runtime.dof_indices] = 0.0
            self.data.qpos[qpos_address:qpos_address + 3] = start_object_qpos[:3] + direction * float(travel)
            self.data.qpos[qpos_address + 3:qpos_address + 7] = start_object_qpos[3:]
            self.data.qvel[dof_address:dof_address + 6] = 0.0
            mujoco.mj_forward(self.model, self.data)

            actual_rotation = self.runtime.get_end_effector_pose()["rotation_matrix"]
            alignment_error = self._normal_alignment_error_deg(actual_rotation, contact_normal)
            maximum_alignment_error = max(maximum_alignment_error, alignment_error)
            if alignment_error > normal_alignment_tolerance + 1e-9:
                self._restore_free_push_state(qpos_address, dof_address, previous_object, previous_arm)
                raise RuntimeError(f"End-effector/contact-normal alignment was lost: {alignment_error:.6f} deg.")

            has_contact = self._contact_pair_present(allowed_tool_pairs)
            contact_maintained = contact_maintained and has_contact
            if stop_on_contact_loss and not has_contact:
                self._restore_free_push_state(qpos_address, dof_address, previous_object, previous_arm)
                raise RuntimeError("Finger-pad contact with the free object was lost.")
            if allowed_support_pairs and not self._contact_pair_present(set(allowed_support_pairs)):
                self._restore_free_push_state(qpos_address, dof_address, previous_object, previous_arm)
                raise RuntimeError("The free object lost its initial support surface during the push.")
            if avoid_collision:
                forbidden = self._forbidden_free_push_contacts(
                    body_id, allowed_tool_pairs, allowed_support_pairs, baseline_self_pairs
                )
                collision_checks += 1
                if forbidden:
                    self._restore_free_push_state(qpos_address, dof_address, previous_object, previous_arm)
                    raise RuntimeError(f"Forbidden collision during free-object push: {forbidden}")
            if self.runtime.viewer is not None:
                self.runtime.viewer.sync()
            if self.runtime.realtime:
                next_frame += 1.0 / self.runtime.playback_fps
                delay = next_frame - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)

        final_position = self.data.xpos[body_id].copy()
        actual_distance = float(np.linalg.norm(final_position - start_object_qpos[:3]))
        return {
            "contact_established": True,
            "contact_maintained": bool(contact_maintained or not maintain_contact),
            "target_moved": actual_distance > 1e-5,
            "travel_distance": actual_distance,
            "rotation_angle": 0.0,
            "peak_force": 0.0,
            "object_final_state": {
                "object_id": object_id,
                "position": final_position.tolist(),
                "quaternion_wxyz": self.data.xquat[body_id].copy().tolist(),
            },
            "evidence": {
                "simulation_adapter": "free_body_contact_coupled_motion",
                "interaction_mode": "free_object",
                "gripper_holding": False,
                "maximum_normal_alignment_error_deg": maximum_alignment_error,
                "collision_checks": collision_checks,
                "forbidden_collision_count": 0,
                "allowed_tool_contact_pairs": len(allowed_tool_pairs),
                "allowed_support_contact_pairs": len(allowed_support_pairs),
            },
            "execution_time": time.perf_counter() - started,
        }

    def verification_snapshot(self, object_id: str, *, interaction_mode: str = "mechanism") -> dict[str, Any]:
        joint_id = self._require_joint(object_id)
        if interaction_mode == "free_object":
            body_id = self._target_body_id(object_id)
            return {
                "contact_established": self._has_free_object_pad_contact(body_id),
                "contact_maintained": self._has_free_object_pad_contact(body_id),
                "target_moved": False,
                "travel_distance": 0.0,
                "rotation_angle": 0.0,
                "peak_force": 0.0,
                "object_final_state": {
                    "object_id": object_id,
                    "position": self.data.xpos[body_id].copy().tolist(),
                    "quaternion_wxyz": self.data.xquat[body_id].copy().tolist(),
                },
                "evidence": {"interaction_mode": "free_object", "collision_checks": 0},
                "execution_time": 0.0,
            }
        return {
            **self._result(object_id, actual_angle=0.0, actual_distance=0.0, started=time.perf_counter(), normal_alignment_error_deg=0.0, collision_checks=0),
            "joint_position": self._joint_position(joint_id),
        }

    def _animate_joint_with_robot(self, joint_id: int, object_id: str, start: float,
                                  target: float, speed: float, timeout: float,
                                  started: float, *, contact_normal: np.ndarray,
                                  rotation_axis: np.ndarray | None,
                                  normal_alignment_tolerance_deg: float,
                                  avoid_collision: bool,
                                  carried_objects: Sequence[str]) -> tuple[float, int]:
        """Move the arm TCP with the grasped handle while advancing the hinge."""
        handle_site_id = self._handle_site_id(object_id)
        start_pose = self.runtime.get_end_effector_pose()
        tcp_position = np.asarray(start_pose["position"], dtype=float)
        handle_position = self.data.site_xpos[handle_site_id].copy()
        tcp_handle_offset = tcp_position - handle_position
        start_rotation = np.asarray(start_pose["rotation_matrix"], dtype=float)
        contact_normal = contact_normal / np.linalg.norm(contact_normal)
        initial_error = self._normal_alignment_error_deg(start_rotation, contact_normal)
        if initial_error > normal_alignment_tolerance_deg + 1e-9:
            raise RuntimeError(
                f"End-effector tool axis is not collinear with the contact normal: {initial_error:.6f} deg."
            )
        if rotation_axis is not None:
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        seed = self.runtime.get_joint_positions()
        target_body_id = self._target_body_id(object_id)
        carried = self._carried_free_bodies(carried_objects, carrier_body_id=target_body_id)
        moving_body_ids = {target_body_id, *(item["body_id"] for item in carried)}
        maximum_alignment_error = initial_error
        baseline_self_pairs = self._current_robot_self_contact_pairs()
        collision_checks = 0
        if avoid_collision:
            forbidden = self._forbidden_contacts(object_id, baseline_self_pairs, moving_body_ids)
            collision_checks += 1
            if forbidden:
                raise RuntimeError(f"Push/Pull starts in forbidden collision: {forbidden}")
        duration = abs(target - start) / max(speed, 1e-6)
        if duration > timeout:
            raise TimeoutError("Push/Pull mechanism motion exceeds the configured timeout.")
        playback_duration = duration / self.runtime.playback_speed if self.runtime.realtime else duration
        frames = max(2, int(np.ceil(playback_duration * self.runtime.playback_fps)) + 1)
        joint_qpos = int(self.model.jnt_qposadr[joint_id])
        joint_dof = int(self.model.jnt_dofadr[joint_id])
        next_frame = time.perf_counter()
        for value in np.linspace(start, target, frames)[1:]:
            if time.perf_counter() - started > timeout:
                raise TimeoutError("Push/Pull mechanism execution timed out.")
            previous = float(self.data.qpos[joint_qpos])
            previous_arm = seed.copy()
            previous_target_position = self.data.xpos[target_body_id].copy()
            previous_carried = {item["qpos_address"]: self.data.qpos[item["qpos_address"]:item["qpos_address"] + 7].copy() for item in carried}
            self.data.qpos[joint_qpos] = value
            self.data.qvel[joint_dof] = 0.0
            mujoco.mj_forward(self.model, self.data)
            target_delta = self.data.xpos[target_body_id] - previous_target_position
            for item in carried:
                address = item["qpos_address"]
                self.data.qpos[address:address + 3] += target_delta
                self.data.qvel[item["dof_address"]:item["dof_address"] + 6] = 0.0
            if carried:
                mujoco.mj_forward(self.model, self.data)
            desired_position = self.data.site_xpos[handle_site_id].copy() + tcp_handle_offset
            if rotation_axis is None:
                desired_rotation = start_rotation
                desired_normal = contact_normal
            else:
                rotation_delta = self._axis_angle_matrix(rotation_axis, float(value - start))
                desired_rotation = rotation_delta @ start_rotation
                desired_normal = rotation_delta @ contact_normal
            try:
                solved = self.kinematics.solve(
                    {"position": desired_position, "orientation": self._rotation_mapping(desired_rotation)},
                    seed=seed, method="trac_ik", timeout=min(0.25, timeout),
                )
            except IKError as exc:
                self._restore_safe_state(joint_qpos, joint_dof, previous, previous_arm, previous_carried)
                raise RuntimeError(f"Arm cannot follow the cabinet-handle arc: {exc}") from exc
            seed = np.asarray(solved["solution"], dtype=float)
            self.data.qpos[self.runtime.qpos_indices] = seed
            self.data.qvel[self.runtime.dof_indices] = 0.0
            mujoco.mj_forward(self.model, self.data)
            actual_rotation = self.runtime.get_end_effector_pose()["rotation_matrix"]
            alignment_error = self._normal_alignment_error_deg(actual_rotation, desired_normal)
            maximum_alignment_error = max(maximum_alignment_error, alignment_error)
            if alignment_error > normal_alignment_tolerance_deg + 1e-9:
                self._restore_safe_state(joint_qpos, joint_dof, previous, previous_arm, previous_carried)
                raise RuntimeError(
                    f"End-effector/contact-normal alignment was lost: {alignment_error:.6f} deg."
                )
            if avoid_collision:
                forbidden = self._forbidden_contacts(object_id, baseline_self_pairs, moving_body_ids)
                collision_checks += 1
                if forbidden:
                    self._restore_safe_state(joint_qpos, joint_dof, previous, previous_arm, previous_carried)
                    raise RuntimeError(f"Forbidden collision during Push/Pull: {forbidden}")
            if self.runtime.viewer is not None:
                self.runtime.viewer.sync()
            if self.runtime.realtime:
                next_frame += 1.0 / self.runtime.playback_fps
                delay = next_frame - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
        return maximum_alignment_error, collision_checks

    def _handle_site_id(self, object_id: str) -> int:
        candidates = (f"{object_id}_handle_site", "blue_cabinet_handle_site")
        for name in candidates:
            site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
            if site_id >= 0:
                return int(site_id)
        raise RuntimeError(f"Push/Pull target has no handle site: {object_id}")

    def _joint_for_target(self, object_id: str) -> int | None:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, object_id)
        if body_id < 0:
            geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, object_id)
            body_id = int(self.model.geom_bodyid[geom_id]) if geom_id >= 0 else -1
        if body_id < 0 or int(self.model.body_jntnum[body_id]) != 1:
            return None
        return int(self.model.body_jntadr[body_id])

    def _require_joint(self, object_id: str, expected_type: Any = None) -> int:
        joint_id = self._joint_for_target(object_id)
        if joint_id is None:
            raise RuntimeError(f"Push/Pull target has no single movable joint: {object_id}")
        if expected_type is not None and self.model.jnt_type[joint_id] != expected_type:
            raise RuntimeError(f"Push/Pull target joint type is incompatible: {object_id}")
        return joint_id

    def _joint_position(self, joint_id: int) -> float:
        return float(self.data.qpos[int(self.model.jnt_qposadr[joint_id])])

    def _joint_axis_world(self, joint_id: int) -> np.ndarray:
        body_id = int(self.model.jnt_bodyid[joint_id])
        axis = self.data.xmat[body_id].reshape(3, 3) @ self.model.jnt_axis[joint_id]
        return axis / np.linalg.norm(axis)

    def _bounded_target(self, joint_id: int, value: float) -> float:
        if self.model.jnt_limited[joint_id]:
            low, high = self.model.jnt_range[joint_id]
            return float(np.clip(value, low, high))
        return float(value)

    def _result(self, object_id: str, *, actual_angle: float, actual_distance: float,
                started: float, normal_alignment_error_deg: float, collision_checks: int) -> dict[str, Any]:
        moved = abs(actual_angle) > 1e-5 or abs(actual_distance) > 1e-5
        gripper = getattr(self.runtime, "gripper_controller", None)
        holding = bool(gripper and gripper.get_state(object_id).get("holding"))
        return {
            "contact_established": True,
            "contact_maintained": holding,
            "target_moved": moved,
            "travel_distance": abs(actual_distance),
            "rotation_angle": actual_angle,
            "peak_force": 0.0,
            "object_final_state": {"object_id": object_id},
            "evidence": {"simulation_adapter": "arm_handle_coupled_motion", "gripper_holding": holding,
                         "maximum_normal_alignment_error_deg": normal_alignment_error_deg,
                         "collision_checks": collision_checks, "forbidden_collision_count": 0},
            "execution_time": time.perf_counter() - started,
        }

    @staticmethod
    def _quaternion_mapping(value: Sequence[float]) -> dict[str, float]:
        q = np.asarray(value, dtype=float)
        return {"representation": "quaternion", "w": float(q[0]), "x": float(q[1]), "y": float(q[2]), "z": float(q[3])}

    @staticmethod
    def _rotation_mapping(rotation: np.ndarray) -> dict[str, float]:
        quaternion = np.empty(4, dtype=float)
        mujoco.mju_mat2Quat(quaternion, np.asarray(rotation, dtype=float).reshape(-1))
        return {"representation": "quaternion", "w": float(quaternion[0]), "x": float(quaternion[1]), "y": float(quaternion[2]), "z": float(quaternion[3])}

    @staticmethod
    def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
        x, y, z = axis
        skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
        return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)

    @staticmethod
    def _normal_alignment_error_deg(rotation: np.ndarray, normal: np.ndarray) -> float:
        tool_axis = np.asarray(rotation, dtype=float)[:, 2]
        tool_axis /= np.linalg.norm(tool_axis)
        normal = np.asarray(normal, dtype=float) / np.linalg.norm(normal)
        return float(np.degrees(np.arccos(np.clip(abs(float(np.dot(tool_axis, normal))), -1.0, 1.0))))

    def _forbidden_contacts(self, object_id: str, baseline_self_pairs: set[frozenset[int]], moving_body_ids: set[int] | None = None) -> list[dict[str, Any]]:
        target_body = self._target_body_id(object_id)
        moving_body_ids = set(moving_body_ids or {target_body})
        target_geoms = {geom_id for geom_id in range(self.model.ngeom)
                        if int(self.model.geom_bodyid[geom_id]) in moving_body_ids}
        handle_geom = self._handle_geom_id(object_id)
        forbidden: list[dict[str, Any]] = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 not in self.robot_geom_ids and geom2 not in self.robot_geom_ids and geom1 not in target_geoms and geom2 not in target_geoms:
                continue
            pair = frozenset((geom1, geom2))
            if geom1 in self.robot_geom_ids and geom2 in self.robot_geom_ids and pair in baseline_self_pairs:
                continue
            if self._is_allowed_handle_contact(geom1, geom2, handle_geom):
                continue
            body1 = int(self.model.geom_bodyid[geom1]); body2 = int(self.model.geom_bodyid[geom2])
            carried_body_ids = moving_body_ids - {target_body}
            if ({body1, body2} & carried_body_ids) and target_body in {body1, body2} and float(contact.dist) >= -0.001:
                # A free payload may rest on the drawer tray while both move
                # together. Small solver penetration is an intended support
                # contact; deeper interpenetration remains forbidden.
                continue
            if float(contact.dist) <= 0.0:
                forbidden.append({"geom1": self._geom_name(geom1), "geom2": self._geom_name(geom2), "distance": float(contact.dist)})
        return forbidden

    def _is_allowed_handle_contact(self, geom1: int, geom2: int, handle_geom: int) -> bool:
        if handle_geom < 0 or handle_geom not in {geom1, geom2}:
            return False
        other = geom2 if geom1 == handle_geom else geom1
        body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, int(self.model.geom_bodyid[other])) or ""
        return body_name.startswith("robotiq_2f85") and "pad" in body_name

    def _free_push_contact_baseline(self, target_body_id: int, direction: np.ndarray,
                                    contact_normal: np.ndarray, tolerance_deg: float) -> tuple[set[frozenset[int]], dict[frozenset[int], float]]:
        """Capture only the intended pad contact and orthogonal support contacts."""
        target_geoms = {geom_id for geom_id in range(self.model.ngeom)
                        if int(self.model.geom_bodyid[geom_id]) == target_body_id}
        tool_pairs: set[frozenset[int]] = set()
        support_pairs: dict[frozenset[int], float] = {}
        # Mesh/sphere contact normals can differ slightly from the nominal face
        # normal even when the TCP axis is exact. This wider gate identifies
        # the intended contact pair; TCP alignment remains governed by the
        # request's stricter normal_alignment_tolerance.
        normal_cosine = np.cos(np.radians(max(float(tolerance_deg), 5.0)))
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 not in target_geoms and geom2 not in target_geoms:
                continue
            other = geom2 if geom1 in target_geoms else geom1
            pair = frozenset((geom1, geom2))
            physical_normal = np.asarray(contact.frame[:3], dtype=float)
            physical_normal /= max(np.linalg.norm(physical_normal), 1e-12)
            if other in self.robot_geom_ids:
                if self._is_gripper_pad_geom(other) and abs(float(np.dot(physical_normal, contact_normal))) >= normal_cosine:
                    tool_pairs.add(pair)
                continue
            if abs(float(np.dot(physical_normal, direction))) <= 0.25 and float(contact.dist) >= -0.001:
                support_pairs[pair] = float(contact.dist)
        return tool_pairs, support_pairs

    def _forbidden_free_push_contacts(self, target_body_id: int,
                                      allowed_tool_pairs: set[frozenset[int]],
                                      allowed_support_pairs: dict[frozenset[int], float],
                                      baseline_self_pairs: set[frozenset[int]]) -> list[dict[str, Any]]:
        target_geoms = {geom_id for geom_id in range(self.model.ngeom)
                        if int(self.model.geom_bodyid[geom_id]) == target_body_id}
        forbidden: list[dict[str, Any]] = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            involves_robot = geom1 in self.robot_geom_ids or geom2 in self.robot_geom_ids
            involves_target = geom1 in target_geoms or geom2 in target_geoms
            if not involves_robot and not involves_target:
                continue
            pair = frozenset((geom1, geom2))
            distance = float(contact.dist)
            if geom1 in self.robot_geom_ids and geom2 in self.robot_geom_ids and pair in baseline_self_pairs:
                continue
            if pair in allowed_tool_pairs:
                if distance >= -0.001:
                    continue
            elif pair in allowed_support_pairs:
                minimum_allowed = min(float(allowed_support_pairs[pair]) - 0.0005, -0.001)
                if distance >= minimum_allowed:
                    continue
            if distance <= 0.0:
                forbidden.append({
                    "geom1": self._geom_name(geom1),
                    "geom2": self._geom_name(geom2),
                    "distance": distance,
                })
        return forbidden

    def _contact_pair_present(self, expected_pairs: set[frozenset[int]]) -> bool:
        return any(
            frozenset((int(self.data.contact[index].geom1), int(self.data.contact[index].geom2))) in expected_pairs
            and float(self.data.contact[index].dist) <= 0.001
            for index in range(self.data.ncon)
        )

    def _has_free_object_pad_contact(self, target_body_id: int) -> bool:
        target_geoms = {geom_id for geom_id in range(self.model.ngeom)
                        if int(self.model.geom_bodyid[geom_id]) == target_body_id}
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 in target_geoms and self._is_gripper_pad_geom(geom2):
                return True
            if geom2 in target_geoms and self._is_gripper_pad_geom(geom1):
                return True
        return False

    def _is_gripper_pad_geom(self, geom_id: int) -> bool:
        body_id = int(self.model.geom_bodyid[geom_id])
        body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        return body_name.startswith("robotiq_2f85") and "pad" in body_name

    def _current_robot_self_contact_pairs(self) -> set[frozenset[int]]:
        pairs: set[frozenset[int]] = set()
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 in self.robot_geom_ids and geom2 in self.robot_geom_ids:
                pairs.add(frozenset((geom1, geom2)))
        return pairs

    def _restore_safe_state(self, joint_qpos: int, joint_dof: int, joint_value: float, arm_joints: np.ndarray,
                            carried_states: dict[int, np.ndarray] | None = None) -> None:
        self.data.qpos[joint_qpos] = joint_value
        self.data.qvel[joint_dof] = 0.0
        self.data.qpos[self.runtime.qpos_indices] = arm_joints
        self.data.qvel[self.runtime.dof_indices] = 0.0
        for address, state in (carried_states or {}).items():
            self.data.qpos[address:address + 7] = state
        mujoco.mj_forward(self.model, self.data)

    def _restore_free_push_state(self, qpos_address: int, dof_address: int,
                                 object_qpos: np.ndarray, arm_joints: np.ndarray) -> None:
        self.data.qpos[qpos_address:qpos_address + 7] = object_qpos
        self.data.qvel[dof_address:dof_address + 6] = 0.0
        self.data.qpos[self.runtime.qpos_indices] = arm_joints
        self.data.qvel[self.runtime.dof_indices] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _target_body_id(self, object_id: str) -> int:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, object_id)
        if body_id < 0:
            raise RuntimeError(f"Push/Pull target body not found: {object_id}")
        return int(body_id)

    def _handle_geom_id(self, object_id: str) -> int:
        for name in (f"{object_id}_handle", "blue_cabinet_handle"):
            geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if geom_id >= 0:
                return int(geom_id)
        return -1

    def _carried_free_bodies(self, names: Sequence[str], *, carrier_body_id: int) -> list[dict[str, int]]:
        result: list[dict[str, int]] = []
        for name in names:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id < 0 or int(self.model.body_jntnum[body_id]) != 1:
                raise RuntimeError(f"Carried drawer object is not a single free body: {name}")
            joint_id = int(self.model.body_jntadr[body_id])
            if self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
                raise RuntimeError(f"Carried drawer object is not free: {name}")
            if np.linalg.norm(self.data.xpos[body_id] - self.data.xpos[carrier_body_id]) > 0.30:
                # carried_objects declares possible drawer contents. An object
                # that has since been removed must not follow later drawer
                # motion or participate in that mechanism's collision set.
                continue
            result.append({"body_id": int(body_id), "qpos_address": int(self.model.jnt_qposadr[joint_id]),
                           "dof_address": int(self.model.jnt_dofadr[joint_id])})
        return result

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

    def _geom_name(self, geom_id: int) -> str:
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"
