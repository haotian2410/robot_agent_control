"""MuJoCo adapter for the Robotiq 2F-85 gripper."""

from __future__ import annotations

import time
from typing import Any

import mujoco
import numpy as np


class MujocoGripperController:
    """Control and inspect the scene's Robotiq open/close actuator."""

    actuator_name = "robotiq_2f85_fingers_actuator"
    driver_joint_names = (
        "robotiq_2f85_left_driver_joint",
        "robotiq_2f85_right_driver_joint",
    )
    maximum_width = 0.085
    maximum_force = 40.0

    def __init__(self, runtime: Any) -> None:
        self.runtime = runtime
        self.model = runtime.model
        self.data = runtime.data
        self.actuator_id = self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, self.actuator_name)
        self.driver_joint_ids = np.asarray(
            [self._id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.driver_joint_names],
            dtype=int,
        )
        self.driver_qpos = self.model.jnt_qposadr[self.driver_joint_ids].astype(int)
        self._holding = False
        self._commanded_force = 0.0
        self._held_body_id: int | None = None
        self._held_qpos_address: int | None = None
        self._held_joint_type: int | None = None
        self._held_offset = np.zeros(3)
        self._held_relative_rotation = np.eye(3)
        self._hold_arm_pose()

    def _id(self, object_type: mujoco.mjtObj, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"MuJoCo gripper object not found: {name}")
        return object_id

    def open(self, width: float, *, speed: float = 0.5, timeout: float = 5.0) -> dict[str, Any]:
        self._holding = False
        self._commanded_force = 0.0
        released_joint_id = None if self._held_body_id is None else int(self.model.body_jntadr[self._held_body_id])
        released_joint_type = self._held_joint_type
        result = self._move(width, speed=speed, timeout=timeout, target_object_id=None, stop_on_contact=False)
        self._detach_object()
        if released_joint_id is not None and released_joint_type == mujoco.mjtJoint.mjJNT_FREE:
            dof_address = int(self.model.jnt_dofadr[released_joint_id])
            self.data.qvel[dof_address : dof_address + 6] = 0.0
            self._settle(250)
        return result

    def close(
        self,
        width: float,
        *,
        speed: float = 0.5,
        force: float = 20.0,
        timeout: float = 5.0,
        target_object_id: str | None = None,
    ) -> dict[str, Any]:
        self._commanded_force = float(force)
        result = self._move(
            width,
            speed=speed,
            timeout=timeout,
            target_object_id=target_object_id,
            stop_on_contact=True,
        )
        if result["contact_detected"]:
            self._attach_object(target_object_id)
        result["force"] = self._commanded_force if result["contact_detected"] else 0.0
        return result

    def hold(self, force: float) -> None:
        self._holding = True
        self._commanded_force = float(force)

    def release(self, *, width: float | None = None, timeout: float = 5.0) -> dict[str, Any]:
        return self.open(self.maximum_width if width is None else width, timeout=timeout)

    def get_state(self, target_object_id: str | None = None) -> dict[str, Any]:
        contact = self._target_contact(target_object_id)
        held = self._target_is_held(target_object_id)
        return {
            "known": True,
            "initialized": True,
            "width": self.width,
            "force": self._commanded_force if contact else 0.0,
            "contact_detected": contact or held,
            "object_present": contact or held,
            "holding": self._holding and held,
            "slip_detected": False,
        }

    @property
    def width(self) -> float:
        angle = float(np.mean(self.data.qpos[self.driver_qpos]))
        return float(np.clip(self.maximum_width * (1.0 - angle / 0.8), 0.0, self.maximum_width))

    def _move(
        self,
        target_width: float,
        *,
        speed: float,
        timeout: float,
        target_object_id: str | None,
        stop_on_contact: bool,
    ) -> dict[str, Any]:
        target_width = float(np.clip(target_width, 0.0, self.maximum_width))
        final_ctrl = 255.0 * (1.0 - target_width / self.maximum_width)
        start_ctrl = float(self.data.ctrl[self.actuator_id])
        locked_arm_qpos = self.data.qpos[self.runtime.qpos_indices].copy()
        locked_target = self._constrained_target_lock(target_object_id)
        duration = max(abs(final_ctrl - start_ctrl) / 255.0 / max(float(speed), 0.05), 0.05)
        start = time.perf_counter()
        contact = False
        while True:
            elapsed = time.perf_counter() - start
            alpha = min(elapsed / duration, 1.0)
            self.data.ctrl[self.actuator_id] = start_ctrl + (final_ctrl - start_ctrl) * alpha
            self._hold_arm_pose()
            for _ in range(5):
                mujoco.mj_step(self.model, self.data)
                self.data.qpos[self.runtime.qpos_indices] = locked_arm_qpos
                self.data.qvel[self.runtime.dof_indices] = 0.0
                if locked_target is not None:
                    self.data.qpos[locked_target[0]] = locked_target[2]
                    self.data.qvel[locked_target[1]] = 0.0
                mujoco.mj_forward(self.model, self.data)
                if self._held_body_id is not None:
                    # A grasp is represented by a temporary kinematic attachment.
                    # During opening, release it as soon as neither finger pad
                    # touches the payload; waiting for the fully-open command
                    # makes small objects visibly hover in the gripper.
                    if self._held_object_has_gripper_contact():
                        self.sync_held_object()
                    else:
                        self._detach_object()
            contact = self._target_contact(target_object_id)
            if self.runtime.viewer is not None:
                self.runtime.viewer.sync()
            if stop_on_contact and contact:
                break
            if alpha >= 1.0:
                break
            if elapsed >= timeout:
                raise TimeoutError("Gripper motion timed out.")
            if self.runtime.realtime:
                time.sleep(self.model.opt.timestep * 5)
        return {
            "target_width": target_width,
            "final_width": self.width,
            "contact_detected": contact,
            "object_present": contact,
        }

    def _constrained_target_lock(self, target_object_id: str | None) -> tuple[int, int, float] | None:
        """Keep a hinged/sliding handle fixed while the fingers establish grasp."""
        body_id = self._target_body_id(target_object_id)
        if body_id is None:
            body_id = self._held_body_id
        if body_id is None or int(self.model.body_jntnum[body_id]) != 1:
            return None
        joint_id = int(self.model.body_jntadr[body_id])
        if self.model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE:
            return None
        qpos_address = int(self.model.jnt_qposadr[joint_id])
        dof_address = int(self.model.jnt_dofadr[joint_id])
        return qpos_address, dof_address, float(self.data.qpos[qpos_address])

    def _hold_arm_pose(self) -> None:
        actuator_ids = getattr(self.runtime, "actuator_ids", None)
        qpos_indices = getattr(self.runtime, "qpos_indices", None)
        if actuator_ids is not None and qpos_indices is not None:
            self.data.ctrl[actuator_ids] = self.data.qpos[qpos_indices]

    def _settle(self, steps: int) -> None:
        locked_arm_qpos = self.data.qpos[self.runtime.qpos_indices].copy()
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
            self.data.qpos[self.runtime.qpos_indices] = locked_arm_qpos
            self.data.qvel[self.runtime.dof_indices] = 0.0
            mujoco.mj_forward(self.model, self.data)
            if self.runtime.viewer is not None:
                self.runtime.viewer.sync()

    def sync_held_object(self, data: Any | None = None) -> None:
        """Kinematically carry an attached free body with the pinch site."""
        if self._held_body_id is None or self._held_qpos_address is None:
            return
        data = self.data if data is None else data
        site_id = self.runtime.end_effector_site_id
        rotation = data.site_xmat[site_id].reshape(3, 3)
        position = data.site_xpos[site_id] + rotation @ self._held_offset
        body_rotation = rotation @ self._held_relative_rotation
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(quaternion, body_rotation.reshape(-1))
        address = self._held_qpos_address
        data.qpos[address : address + 3] = position
        data.qpos[address + 3 : address + 7] = quaternion
        joint_id = int(self.model.body_jntadr[self._held_body_id])
        dof_address = int(self.model.jnt_dofadr[joint_id])
        data.qvel[dof_address : dof_address + 6] = 0.0
        mujoco.mj_forward(self.model, data)

    def _attach_object(self, target_object_id: str | None) -> None:
        body_id = self._target_body_id(target_object_id)
        if body_id is None:
            return
        joint_id = int(self.model.body_jntadr[body_id])
        if joint_id < 0:
            return
        self._held_body_id = body_id
        self._held_joint_type = int(self.model.jnt_type[joint_id])
        if self._held_joint_type != mujoco.mjtJoint.mjJNT_FREE:
            # Hinged/sliding mechanisms are constrained by their own joint.
            # Mark them held so a contact skill may move the robot and mechanism
            # together, but never overwrite their qpos as a free payload.
            self._held_qpos_address = None
            return
        self._held_qpos_address = int(self.model.jnt_qposadr[joint_id])
        site_id = self.runtime.end_effector_site_id
        rotation = self.data.site_xmat[site_id].reshape(3, 3)
        self._held_offset = rotation.T @ (self.data.xpos[body_id] - self.data.site_xpos[site_id])
        body_rotation = self.data.xmat[body_id].reshape(3, 3)
        self._held_relative_rotation = rotation.T @ body_rotation

    def _detach_object(self) -> None:
        self._held_body_id = None
        self._held_qpos_address = None
        self._held_joint_type = None
        self._held_offset = np.zeros(3)
        self._held_relative_rotation = np.eye(3)

    def _target_is_held(self, target_object_id: str | None) -> bool:
        target_body = self._target_body_id(target_object_id)
        return target_body is not None and target_body == self._held_body_id

    def _target_body_id(self, target_object_id: str | None) -> int | None:
        if not target_object_id:
            return None
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, target_object_id)
        if body_id >= 0:
            return int(body_id)
        geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, target_object_id)
        if geom_id >= 0:
            return int(self.model.geom_bodyid[geom_id])
        return None

    def _target_contact(self, target_object_id: str | None) -> bool:
        if not target_object_id:
            return False
        target_body = self._target_body_id(target_object_id)
        if target_body is None:
            return False
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            body1 = int(self.model.geom_bodyid[contact.geom1])
            body2 = int(self.model.geom_bodyid[contact.geom2])
            name1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body1) or ""
            name2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body2) or ""
            if body1 == target_body and name2.startswith("robotiq_2f85") and "pad" in name2:
                return True
            if body2 == target_body and name1.startswith("robotiq_2f85") and "pad" in name1:
                return True
        return False

    def _held_object_has_gripper_contact(self) -> bool:
        """Return whether the currently attached payload still touches a finger pad."""
        if self._held_body_id is None:
            return False
        body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, self._held_body_id)
        return self._target_contact(body_name)
