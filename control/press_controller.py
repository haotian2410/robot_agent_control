"""Bounded local Cartesian press controller for the MuJoCo runtime."""

from __future__ import annotations

import time
from typing import Any, Mapping, Sequence

import mujoco
import numpy as np

from kinematics import IKError, MujocoKinematics
from skills.manipulation.press.utils.error_codes import ErrorCode, PressExecutionError


class MujocoPressController:
    """Execute short fixed-axis presses without invoking global planning."""

    def __init__(self, runtime: Any) -> None:
        self.runtime = runtime
        self.model = runtime.model
        self.data = runtime.data
        self.kinematics = MujocoKinematics(runtime)
        self.ready = True
        self.force_feedback_available = True
        self.force_sensor_calibrated = True
        self._baseline_self_pairs = self._current_self_contact_pairs()

    def get_state(self, target_object_id: str | None = None) -> dict[str, Any]:
        force, contact, contacts = self._target_contact(target_object_id)
        return {
            "ready": self.ready,
            "enabled": True,
            "fault": False,
            "force_feedback_available": self.force_feedback_available,
            "force_sensor_calibrated": self.force_sensor_calibrated,
            "normal_force": force,
            "contact_detected": contact,
            "contacts": contacts,
        }

    def execute_displacement(self, *, direction: Sequence[float], target_object_id: str | None, press_depth: float | None, travel_distance: float | None, stop_on_contact: bool, post_contact_depth: float, maximum_force: float, maximum_travel: float, contact_threshold: float, speed: float, hold_time: float, timeout: float, retract: bool, retract_speed: float, emergency_retract: bool) -> dict[str, Any]:
        desired_total = float(travel_distance) if travel_distance is not None else float(maximum_travel)
        desired_post_contact = float(press_depth) if press_depth is not None else float(post_contact_depth)
        return self._execute_profile(
            direction=np.asarray(direction, dtype=float), target_object_id=target_object_id,
            maximum_force=maximum_force, maximum_travel=maximum_travel,
            contact_threshold=contact_threshold, speed=speed, hold_time=hold_time,
            timeout=timeout, retract=retract, retract_speed=retract_speed,
            emergency_retract=emergency_retract, target_force=None,
            force_tolerance=0.0, desired_total=desired_total,
            desired_post_contact=desired_post_contact, stop_on_contact=stop_on_contact,
            travel_distance_mode=travel_distance is not None,
        )

    def execute_force(self, *, direction: Sequence[float], target_object_id: str | None, target_force: float, maximum_force: float, maximum_travel: float, contact_threshold: float, force_tolerance: float, speed: float, hold_time: float, timeout: float, retract: bool, retract_speed: float, emergency_retract: bool) -> dict[str, Any]:
        if not self.force_feedback_available or not self.force_sensor_calibrated:
            raise PressExecutionError(
                "Calibrated force feedback is unavailable.", error_code=ErrorCode.SENSOR_DATA_UNAVAILABLE,
                failed_stage="preconditions", recoverable=True,
                recommended_action="Calibrate or enable force feedback before using force control.",
            )
        return self._execute_profile(
            direction=np.asarray(direction, dtype=float), target_object_id=target_object_id,
            maximum_force=maximum_force, maximum_travel=maximum_travel,
            contact_threshold=contact_threshold, speed=speed, hold_time=hold_time,
            timeout=timeout, retract=retract, retract_speed=retract_speed,
            emergency_retract=emergency_retract, target_force=target_force,
            force_tolerance=force_tolerance, desired_total=maximum_travel,
            desired_post_contact=0.0, stop_on_contact=False, travel_distance_mode=False,
        )

    def verification_snapshot(self, target_object_id: str | None, *, strategy: str) -> dict[str, Any]:
        force, contact, contacts = self._target_contact(target_object_id)
        pose = self._serializable_pose(self.runtime.get_end_effector_pose())
        return {
            "contact_detected": contact,
            "target_force_reached": strategy == "force" and contact,
            "target_depth_reached": strategy == "displacement" and contact,
            "peak_force": force, "final_force": force, "travel_distance": 0.0,
            "hold_time": 0.0, "target_actuated": contact, "retracted": True,
            "final_pose": pose, "force_profile": [], "execution_time": 0.0,
            "evidence": {"verification_snapshot": True, "contacts": contacts},
        }

    def _execute_profile(self, **options: Any) -> dict[str, Any]:
        direction = options["direction"] / np.linalg.norm(options["direction"])
        start_q = self.runtime.get_joint_positions()
        start_pose = self.runtime.get_end_effector_pose()
        start_position = np.asarray(start_pose["position"], dtype=float)
        orientation = self._quaternion_mapping(start_pose["quaternion_wxyz"])
        step = min(0.001, max(float(options["speed"]) * 0.05, 0.0002))
        maximum_travel = float(options["maximum_travel"])
        started = time.perf_counter()
        samples: list[dict[str, Any]] = []
        contact_travel: float | None = None
        peak_force = 0.0
        final_q = start_q.copy()
        movable_target = self._movable_target_joint(options["target_object_id"], direction)
        reached = False
        failure: PressExecutionError | None = None
        travel = 0.0
        try:
            while travel < maximum_travel - 1e-9:
                if time.perf_counter() - started > float(options["timeout"]):
                    raise PressExecutionError("Press execution timed out.", error_code=ErrorCode.PRESS_TIMEOUT, failed_stage="execution", recoverable=True, recommended_action="Check the pre-press pose and increase timeout only if safe.")
                travel = min(travel + step, maximum_travel)
                target_position = start_position + direction * travel
                try:
                    solved = self.kinematics.solve({"position": target_position, "orientation": orientation}, seed=final_q, method="trac_ik", timeout=0.25)
                except IKError as exc:
                    raise PressExecutionError(f"Local press IK failed at travel {travel:.6f} m: {exc}", error_code=ErrorCode.PRESS_EXECUTION_ERROR, failed_stage="local_motion", recoverable=True, recommended_action="Move to a better pre-press pose without changing the requested press axis.") from exc
                next_q = np.asarray(solved["solution"], dtype=float)
                self._set_joints(next_q)
                final_q = next_q
                force, raw_contact, contacts = self._target_contact(options["target_object_id"])
                contact = raw_contact and force >= float(options["contact_threshold"])
                peak_force = max(peak_force, force)
                if contact and contact_travel is None:
                    contact_travel = travel
                if contact_travel is not None and movable_target is not None:
                    self._set_target_joint_travel(movable_target, travel - contact_travel)
                    force, raw_contact, contacts = self._target_contact(options["target_object_id"])
                    contact = raw_contact and force >= float(options["contact_threshold"])
                    peak_force = max(peak_force, force)
                foreign = self._foreign_contacts(options["target_object_id"])
                if foreign:
                    raise PressExecutionError("Unexpected local collision during press.", error_code=ErrorCode.LOCAL_COLLISION_DETECTED, failed_stage="execution", recoverable=True, recommended_action="Correct the pre-press pose or local scene; Press Skill will not change the press direction.", details={"contacts": foreign})
                samples.append({"travel": travel, "force": force, "contact": contact, "contacts": contacts})
                if force > float(options["maximum_force"]) + 1e-9:
                    raise PressExecutionError("Maximum press force exceeded.", error_code=ErrorCode.FORCE_LIMIT_EXCEEDED, failed_stage="execution", recoverable=False, recommended_action="Inspect the target, press direction, and force limit before retrying.", details={"peak_force": force, "maximum_force": options["maximum_force"]})
                if options["target_force"] is not None:
                    if force >= float(options["target_force"]) - float(options["force_tolerance"]):
                        reached = True
                        break
                elif options["travel_distance_mode"]:
                    if travel >= float(options["desired_total"]) - 1e-9:
                        reached = True
                        break
                elif contact_travel is not None:
                    post_contact = travel - contact_travel
                    if options["stop_on_contact"] or post_contact >= float(options["desired_post_contact"]) - 1e-9:
                        reached = True
                        break
            if float(options["hold_time"]) > 0 and self.runtime.realtime:
                time.sleep(float(options["hold_time"]))
        except PressExecutionError as exc:
            failure = exc
        retracted = False
        if options["retract"] or (failure is not None and options["emergency_retract"]):
            retracted = self._retract(start_q, float(options["retract_speed"]), started, float(options["timeout"]))
            if movable_target is not None:
                self._release_target_joint(movable_target)
        if failure is not None:
            failure.retracted = retracted
            raise failure
        final_force = samples[-1]["force"] if samples else 0.0
        contact_detected = contact_travel is not None
        return {
            "contact_detected": contact_detected,
            "target_force_reached": bool(options["target_force"] is not None and reached),
            "target_depth_reached": bool(options["target_force"] is None and reached),
            "peak_force": peak_force,
            "final_force": final_force,
            "travel_distance": travel,
            "contact_travel": contact_travel,
            "hold_time": float(options["hold_time"]),
            "target_actuated": reached,
            "retracted": retracted,
            "final_pose": self._serializable_pose(self.runtime.get_end_effector_pose()),
            "force_profile": samples,
            "execution_time": time.perf_counter() - started,
            "evidence": {
                "sample_count": len(samples),
                "samples": self._decimate(samples, 25),
                "target_object_id": options["target_object_id"],
            },
        }

    def _set_joints(self, joints: np.ndarray) -> None:
        self.data.qpos[self.runtime.qpos_indices] = joints
        self.data.qvel[self.runtime.dof_indices] = 0.0
        mujoco.mj_forward(self.model, self.data)
        gripper = getattr(self.runtime, "gripper_controller", None)
        if gripper is not None:
            gripper.sync_held_object()
        if self.runtime.viewer is not None:
            self.runtime.viewer.sync()

    def _movable_target_joint(self, target_object_id: str | None, direction: np.ndarray) -> dict[str, Any] | None:
        """Return a compatible passive slide joint for a simulated press target."""
        body_id = self._target_body_id(target_object_id)
        if body_id is None or int(self.model.body_jntnum[body_id]) != 1:
            return None
        joint_id = int(self.model.body_jntadr[body_id])
        if self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_SLIDE:
            return None
        axis = self.data.xmat[body_id].reshape(3, 3) @ self.model.jnt_axis[joint_id]
        alignment = float(np.dot(direction, axis))
        if abs(alignment) < 0.95:
            return None
        qpos_address = int(self.model.jnt_qposadr[joint_id])
        return {
            "joint_id": joint_id,
            "qpos_address": qpos_address,
            "initial_qpos": float(self.data.qpos[qpos_address]),
            "travel_sign": 1.0 if alignment > 0.0 else -1.0,
        }

    def _set_target_joint_travel(self, target: Mapping[str, Any], travel: float) -> None:
        joint_id = int(target["joint_id"])
        value = float(target["initial_qpos"]) + float(target["travel_sign"]) * max(float(travel), 0.0)
        if self.model.jnt_limited[joint_id]:
            value = float(np.clip(value, *self.model.jnt_range[joint_id]))
        self.data.qpos[int(target["qpos_address"])] = value
        mujoco.mj_forward(self.model, self.data)
        if self.runtime.viewer is not None:
            self.runtime.viewer.sync()

    def _release_target_joint(self, target: Mapping[str, Any]) -> None:
        address = int(target["qpos_address"])
        initial = float(target["initial_qpos"])
        current = float(self.data.qpos[address])
        frame_count = 20 if self.runtime.realtime else 2
        for value in np.linspace(current, initial, frame_count)[1:]:
            self.data.qpos[address] = value
            mujoco.mj_forward(self.model, self.data)
            if self.runtime.viewer is not None:
                self.runtime.viewer.sync()
            if self.runtime.realtime:
                time.sleep(1.0 / self.runtime.playback_fps)

    def _retract(self, start_q: np.ndarray, speed: float, started: float, timeout: float) -> bool:
        current = self.runtime.get_joint_positions()
        count = max(2, int(np.ceil(np.max(np.abs(current - start_q)) / max(speed * 0.05, 0.01))) + 1)
        for alpha in np.linspace(0.0, 1.0, count)[1:]:
            if time.perf_counter() - started > timeout:
                return False
            self._set_joints(current + alpha * (start_q - current))
        return True

    def _target_contact(self, target_object_id: str | None) -> tuple[float, bool, list[dict[str, Any]]]:
        target_body = self._target_body_id(target_object_id)
        if target_object_id and target_body is None:
            return 0.0, False, []
        contacts = []
        peak = 0.0
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            body1 = int(self.model.geom_bodyid[contact.geom1]); body2 = int(self.model.geom_bodyid[contact.geom2])
            if target_body is not None and target_body not in {body1, body2}:
                continue
            if target_body is not None:
                other = body2 if body1 == target_body else body1
                if not self._robot_body(other):
                    continue
            if target_body is None and not (self._robot_body(body1) or self._robot_body(body2)):
                continue
            penetration_force = max(0.0, -float(contact.dist)) * 5000.0
            wrench = np.zeros(6); mujoco.mj_contactForce(self.model, self.data, index, wrench)
            force = max(abs(float(wrench[0])), penetration_force)
            peak = max(peak, force)
            contacts.append({"geom1": self._geom_name(int(contact.geom1)), "geom2": self._geom_name(int(contact.geom2)), "distance": float(contact.dist), "normal_force": force})
        return peak, bool(contacts), contacts

    def _foreign_contacts(self, target_object_id: str | None) -> list[dict[str, Any]]:
        target_body = self._target_body_id(target_object_id)
        result = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            body1 = int(self.model.geom_bodyid[contact.geom1]); body2 = int(self.model.geom_bodyid[contact.geom2])
            if not (self._robot_body(body1) or self._robot_body(body2)):
                continue
            if target_body is not None and target_body in {body1, body2}:
                continue
            if self._robot_body(body1) and self._robot_body(body2):
                pair = frozenset((int(contact.geom1), int(contact.geom2)))
                if pair in self._baseline_self_pairs:
                    continue
            if float(contact.dist) <= 0.0:
                result.append({"geom1": self._geom_name(int(contact.geom1)), "geom2": self._geom_name(int(contact.geom2)), "distance": float(contact.dist)})
        return result

    def _current_self_contact_pairs(self) -> set[frozenset[int]]:
        result: set[frozenset[int]] = set()
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            body1 = int(self.model.geom_bodyid[contact.geom1]); body2 = int(self.model.geom_bodyid[contact.geom2])
            if self._robot_body(body1) and self._robot_body(body2):
                result.add(frozenset((int(contact.geom1), int(contact.geom2))))
        return result

    def _robot_body(self, body_id: int) -> bool:
        name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        return name.startswith(("shoulder", "upper_arm", "forearm", "wrist", "robotiq_2f85", "ur5"))

    def _target_body_id(self, name: str | None) -> int | None:
        if not name:
            return None
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body >= 0:
            return int(body)
        geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        return int(self.model.geom_bodyid[geom]) if geom >= 0 else None

    def _geom_name(self, geom_id: int) -> str:
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"

    @staticmethod
    def _quaternion_mapping(value: Sequence[float]) -> dict[str, float]:
        q = np.asarray(value, dtype=float)
        return {"representation": "quaternion", "w": float(q[0]), "x": float(q[1]), "y": float(q[2]), "z": float(q[3])}

    @staticmethod
    def _serializable_pose(pose: Mapping[str, Any]) -> dict[str, Any]:
        return {"position": np.asarray(pose["position"]).tolist(), "quaternion_wxyz": np.asarray(pose["quaternion_wxyz"]).tolist()}

    @staticmethod
    def _decimate(samples: list[dict[str, Any]], maximum: int) -> list[dict[str, Any]]:
        if len(samples) <= maximum:
            return samples
        indices = np.unique(np.linspace(0, len(samples) - 1, maximum, dtype=int))
        return [samples[int(index)] for index in indices]
