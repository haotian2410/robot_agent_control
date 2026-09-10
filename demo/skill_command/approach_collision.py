"""Parallel MuJoCo validation and caching for generated approach poses."""

from __future__ import annotations

from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
import threading
from typing import Any, Mapping, Sequence

import mujoco
import numpy as np

from control import MujocoGripperController
from demo.common.registry import SceneRegistry
from skills.motion.move.skill import MoveSkill
from skills.motion.move.utils.validation import validate_move_request
from utils import SceneRobotRuntime


_CACHE_MISS = object()


class MujocoApproachCollisionChecker:
    """Check pre-poses using bounded parallel IK and collision validation.

    Two independent LRU caches are maintained: the IK cache is keyed by the
    approach pose and solver seed, while the collision/path cache additionally
    includes a live world-state signature. Worker threads never touch the live
    runtime; each owns an isolated MuJoCo model/data pair synchronized from one
    immutable snapshot at the start of a batch.
    """

    def __init__(
        self,
        registry: SceneRegistry,
        runtime_config: Mapping[str, Any] | None = None,
        *,
        runtime: SceneRobotRuntime | None = None,
    ) -> None:
        config = dict(runtime_config or {})
        search = config.get("approach_search", {})
        if not isinstance(search, Mapping):
            search = {}
        self.registry = registry
        self.runtime_config = config
        self.runtime = runtime or SceneRobotRuntime(
            registry.scene_path,
            end_effector_site=config.get("end_effector_site", "robotiq_2f85_pinch"),
            execution_mode=config.get("execution_mode", "kinematic"),
        )
        # Kept for compatibility with callers that inspect this attribute.
        # Parallel work uses isolated thread-local MoveSkill instances.
        self.skill = MoveSkill(robot_runtime=self.runtime)
        self.defaults = registry.data.get("move_defaults", {})
        self.max_workers = max(1, min(int(search.get("parallel_workers", 3)), 8))
        self.initial_ik_candidates = max(
            1, int(search.get("initial_ik_candidates", 2))
        )
        self.max_ik_candidates = max(
            self.initial_ik_candidates,
            int(search.get("max_ik_candidates", 4)),
        )
        self.ik_timeout = max(0.05, float(search.get("ik_timeout", 0.5)))
        self.cache_size = max(16, int(search.get("cache_size", 512)))

        self._executor = ThreadPoolExecutor(
            max_workers=self.max_workers,
            thread_name_prefix="approach-check",
        )
        self._worker_local = threading.local()
        self._cache_lock = threading.RLock()
        self._ik_cache: OrderedDict[tuple[Any, ...], tuple[float, ...] | None] = (
            OrderedDict()
        )
        self._collision_cache: OrderedDict[tuple[Any, ...], bool] = OrderedDict()

    def pose_provider(self, object_key: str, spatial: Mapping[str, Any]) -> Mapping[str, Any]:
        source = spatial.get("source", {})
        source_type = source.get("type")
        object_types = {
            "body": mujoco.mjtObj.mjOBJ_BODY,
            "geom": mujoco.mjtObj.mjOBJ_GEOM,
            "site": mujoco.mjtObj.mjOBJ_SITE,
        }
        if source_type not in object_types:
            raise ValueError(f"unsupported source type: {source_type}")
        source_id = mujoco.mj_name2id(
            self.runtime.model,
            object_types[source_type],
            str(source.get("name")),
        )
        if source_id < 0:
            raise ValueError(f"source not found: {source_type} {source.get('name')}")
        positions = {
            "body": self.runtime.data.xpos,
            "geom": self.runtime.data.geom_xpos,
            "site": self.runtime.data.site_xpos,
        }
        rotations = {
            "body": self.runtime.data.xmat,
            "geom": self.runtime.data.geom_xmat,
            "site": self.runtime.data.site_xmat,
        }
        rotation = np.asarray(rotations[source_type][source_id]).reshape(3, 3).copy()
        quaternion = np.empty(4, dtype=float)
        mujoco.mju_mat2Quat(quaternion, rotation.reshape(-1))
        return {
            "position": np.asarray(positions[source_type][source_id]).copy(),
            "quaternion_wxyz": quaternion,
            "rotation_matrix": rotation,
        }

    def __call__(
        self,
        approach_pose: Mapping[str, Any],
        target_pose: Mapping[str, Any],
        object_key: str,
    ) -> bool:
        return self.check_many([approach_pose], target_pose, object_key)[0]

    def check_many(
        self,
        approach_poses: Sequence[Mapping[str, Any]],
        target_pose: Mapping[str, Any],
        object_key: str,
    ) -> list[bool]:
        """Validate an ordered distance batch with bounded concurrency.

        Two IK seeds are tried first. If a lower-priority distance succeeds,
        only preceding higher-priority distances are expanded. If none succeeds,
        every distance expands up to ``max_ik_candidates``. This preserves the
        converter's farthest-first selection rule.
        """
        poses = [deepcopy(dict(pose)) for pose in approach_poses]
        if not poses:
            return []
        snapshot = self._runtime_snapshot()
        seeds = self._ik_seeds(snapshot)
        initial_count = min(self.initial_ik_candidates, len(seeds))
        indices = list(range(len(poses)))
        results = self._evaluate_stage(
            poses,
            indices,
            target_pose,
            object_key,
            snapshot,
            seeds[:initial_count],
        )

        first_valid = next((index for index, valid in enumerate(results) if valid), None)
        if first_valid is None:
            expand_indices = indices
        else:
            # Do not select a nearer point until every farther point has
            # exhausted its remaining IK seeds.
            expand_indices = [index for index in range(first_valid) if not results[index]]

        if expand_indices and initial_count < len(seeds):
            expanded = self._evaluate_stage(
                poses,
                expand_indices,
                target_pose,
                object_key,
                snapshot,
                seeds[initial_count:],
            )
            for local_index, pose_index in enumerate(expand_indices):
                results[pose_index] = results[pose_index] or expanded[local_index]
        return results

    def clear(self) -> None:
        """Clear both cache levels after a model/topology change."""
        with self._cache_lock:
            self._ik_cache.clear()
            self._collision_cache.clear()

    def cache_info(self) -> dict[str, int]:
        with self._cache_lock:
            return {
                "ik_entries": len(self._ik_cache),
                "collision_entries": len(self._collision_cache),
            }

    def close(self) -> None:
        self._executor.shutdown(wait=True, cancel_futures=True)

    def _evaluate_stage(
        self,
        poses: Sequence[Mapping[str, Any]],
        indices: Sequence[int],
        target_pose: Mapping[str, Any],
        object_key: str,
        snapshot: Mapping[str, Any],
        seeds: Sequence[np.ndarray],
    ) -> list[bool]:
        if not indices or not seeds:
            return [False] * len(indices)

        solutions: dict[int, list[np.ndarray]] = {index: [] for index in indices}
        solve_futures = {}
        for index in indices:
            for seed in seeds:
                future = self._executor.submit(
                    self._solve_one_cached,
                    poses[index],
                    seed,
                    snapshot,
                )
                solve_futures[future] = index
        for future in as_completed(solve_futures):
            index = solve_futures[future]
            candidate = future.result()
            if candidate is not None and not any(
                _same_joint_solution(candidate, known) for known in solutions[index]
            ):
                solutions[index].append(candidate)

        stage_results = {index: False for index in indices}
        validation_futures = {}
        for index in indices:
            for joints in solutions[index]:
                future = self._executor.submit(
                    self._validate_candidate_cached,
                    joints,
                    target_pose,
                    object_key,
                    snapshot,
                )
                validation_futures[future] = index
        for future in as_completed(validation_futures):
            index = validation_futures[future]
            if future.result():
                stage_results[index] = True
        return [stage_results[index] for index in indices]

    def _solve_one_cached(
        self,
        approach_pose: Mapping[str, Any],
        seed: np.ndarray,
        snapshot: Mapping[str, Any],
    ) -> np.ndarray | None:
        method = str(self.defaults.get("strategy_params", {}).get("ik_method", "trac_ik"))
        key = (
            _pose_key(approach_pose),
            _joint_key(seed),
            method,
            round(self.ik_timeout, 6),
        )
        cached = self._cache_get(self._ik_cache, key)
        if cached is not _CACHE_MISS:
            return None if cached is None else np.asarray(cached, dtype=float)

        worker = self._get_worker()
        worker.sync(snapshot)
        try:
            solved = worker.skill.kinematics.solve(
                approach_pose,
                seed=seed,
                method=method,
                timeout=self.ik_timeout,
            )
            candidate = np.asarray(solved["solution"], dtype=float)
            stored: tuple[float, ...] | None = tuple(float(value) for value in candidate)
        except Exception:
            candidate = None
            stored = None
        self._cache_put(self._ik_cache, key, stored)
        return candidate

    def _validate_candidate_cached(
        self,
        joints: np.ndarray,
        target_pose: Mapping[str, Any],
        object_key: str,
        snapshot: Mapping[str, Any],
    ) -> bool:
        constraints = self.defaults.get("constraints", {})
        safety_distance = float(constraints.get("safety_distance", 0.0))
        key = (
            object_key,
            _joint_key(joints),
            _pose_key(target_pose),
            snapshot["world_signature"],
            round(safety_distance, 8),
        )
        cached = self._cache_get(self._collision_cache, key)
        if cached is not _CACHE_MISS:
            return bool(cached)

        worker = self._get_worker()
        worker.sync(snapshot)
        allowed_geom_ids = self._target_geom_ids(object_key)
        worker.skill.collision_checker.allowed_target_geom_ids = allowed_geom_ids
        collision = worker.skill.collision_checker.check(
            joints,
            safety_distance=safety_distance,
            allowed_geom_ids=allowed_geom_ids,
        )
        valid = not collision.in_collision and self._linear_path_is_clear(
            worker,
            joints,
            target_pose,
            object_key,
        )
        self._cache_put(self._collision_cache, key, valid)
        return valid

    def _linear_path_is_clear(
        self,
        worker: "_ApproachWorker",
        start_joints: np.ndarray,
        target_pose: Mapping[str, Any],
        object_key: str,
    ) -> bool:
        request = _deep_merge(
            self.defaults,
            {
                "target": deepcopy(dict(target_pose)),
                "motion": {
                    "path_type": "linear",
                    "path_constraint": "hard",
                    "phase": "contact",
                },
                "planning": {"mode": "direct", "allow_replan": False},
                "constraints": {
                    "avoid_collision": True,
                    "keep_end_effector_orientation": True,
                },
            },
        )
        body_name = self.registry.objects[object_key].get("body_name")
        if body_name:
            request["target_body_name"] = body_name
        try:
            normalized = validate_move_request(request, worker.skill.config)
            pose = worker.skill.kinematics.forward(start_joints)
            quaternion = np.empty(4, dtype=float)
            mujoco.mju_mat2Quat(quaternion, pose["rotation_matrix"].reshape(-1))
            context = dict(worker.runtime.get_context(normalized))
            context["current_joint_state"] = np.asarray(start_joints, dtype=float)
            context["current_pose"] = {
                "position": pose["position"],
                "quaternion_wxyz": quaternion,
            }
            path_request = worker.skill.strategies["linear_move"].build_path_request(
                normalized, context
            )
            worker.skill.direct_planner.plan(path_request, normalized, context)
            return True
        except Exception:
            return False

    def _runtime_snapshot(self) -> dict[str, Any]:
        controller = getattr(self.runtime, "gripper_controller", None)
        held_state = {
            "holding": bool(getattr(controller, "_holding", False)),
            "commanded_force": float(getattr(controller, "_commanded_force", 0.0)),
            "held_body_id": getattr(controller, "_held_body_id", None),
            "held_qpos_address": getattr(controller, "_held_qpos_address", None),
            "held_joint_type": getattr(controller, "_held_joint_type", None),
            "held_offset": np.asarray(
                getattr(controller, "_held_offset", np.zeros(3)), dtype=float
            ).copy(),
            "held_relative_rotation": np.asarray(
                getattr(controller, "_held_relative_rotation", np.eye(3)), dtype=float
            ).copy(),
        }
        qpos = np.asarray(self.runtime.data.qpos, dtype=float).copy()
        held_id = held_state["held_body_id"]
        return {
            "qpos": qpos,
            "qvel": np.asarray(self.runtime.data.qvel, dtype=float).copy(),
            "ctrl": np.asarray(self.runtime.data.ctrl, dtype=float).copy(),
            "time": float(self.runtime.data.time),
            "held_state": held_state,
            # Full qpos is deliberately conservative: robot, gripper, door,
            # and free-object changes all invalidate collision/path results.
            "world_signature": (
                tuple(round(float(value), 7) for value in qpos),
                None if held_id is None else int(held_id),
            ),
        }

    def _ik_seeds(self, snapshot: Mapping[str, Any]) -> list[np.ndarray]:
        current = np.asarray(snapshot["qpos"], dtype=float)[self.runtime.qpos_indices]
        seeds = [current.copy()]
        rng = np.random.default_rng(7)
        limits = np.asarray(self.runtime.joint_limits, dtype=float)
        while len(seeds) < self.max_ik_candidates:
            seeds.append(rng.uniform(limits[:, 0], limits[:, 1]))
        return seeds

    def _get_worker(self) -> "_ApproachWorker":
        worker = getattr(self._worker_local, "worker", None)
        if worker is None:
            worker = _ApproachWorker(self.registry, self.runtime_config)
            self._worker_local.worker = worker
        return worker

    def _target_geom_ids(self, object_key: str) -> set[int]:
        obj = self.registry.objects[object_key]
        source = obj.get("spatial", {}).get("source", {})
        body_name = obj.get("body_name")
        body_id = -1
        if body_name:
            body_id = mujoco.mj_name2id(
                self.runtime.model, mujoco.mjtObj.mjOBJ_BODY, body_name
            )
        elif source.get("type") == "body":
            body_id = mujoco.mj_name2id(
                self.runtime.model,
                mujoco.mjtObj.mjOBJ_BODY,
                str(source.get("name")),
            )
        elif source.get("type") == "site":
            site_id = mujoco.mj_name2id(
                self.runtime.model,
                mujoco.mjtObj.mjOBJ_SITE,
                str(source.get("name")),
            )
            if site_id >= 0:
                body_id = int(self.runtime.model.site_bodyid[site_id])
        elif source.get("type") == "geom":
            geom_id = mujoco.mj_name2id(
                self.runtime.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                str(source.get("name")),
            )
            if geom_id >= 0:
                body_id = int(self.runtime.model.geom_bodyid[geom_id])
        if body_id < 0:
            return set()
        return {
            geom_id
            for geom_id in range(self.runtime.model.ngeom)
            if int(self.runtime.model.geom_bodyid[geom_id]) == body_id
        }

    def _cache_get(
        self,
        cache: OrderedDict[tuple[Any, ...], Any],
        key: tuple[Any, ...],
    ) -> Any:
        with self._cache_lock:
            if key not in cache:
                return _CACHE_MISS
            value = cache.pop(key)
            cache[key] = value
            return value

    def _cache_put(
        self,
        cache: OrderedDict[tuple[Any, ...], Any],
        key: tuple[Any, ...],
        value: Any,
    ) -> None:
        with self._cache_lock:
            cache.pop(key, None)
            cache[key] = value
            while len(cache) > self.cache_size:
                cache.popitem(last=False)


class _ApproachWorker:
    """One thread's isolated model/data, IK solver, and collision checker."""

    def __init__(self, registry: SceneRegistry, config: Mapping[str, Any]) -> None:
        self.runtime = SceneRobotRuntime(
            registry.scene_path,
            end_effector_site=config.get("end_effector_site", "robotiq_2f85_pinch"),
            execution_mode="kinematic",
            realtime=False,
        )
        self.controller = MujocoGripperController(self.runtime)
        self.runtime.gripper_controller = self.controller
        self.skill = MoveSkill(robot_runtime=self.runtime)

    def sync(self, snapshot: Mapping[str, Any]) -> None:
        self.runtime.data.qpos[:] = snapshot["qpos"]
        self.runtime.data.qvel[:] = snapshot["qvel"]
        self.runtime.data.ctrl[:] = snapshot["ctrl"]
        self.runtime.data.time = float(snapshot["time"])
        held = snapshot["held_state"]
        self.controller._holding = bool(held["holding"])
        self.controller._commanded_force = float(held["commanded_force"])
        self.controller._held_body_id = held["held_body_id"]
        self.controller._held_qpos_address = held["held_qpos_address"]
        self.controller._held_joint_type = held["held_joint_type"]
        self.controller._held_offset = np.asarray(held["held_offset"], dtype=float).copy()
        self.controller._held_relative_rotation = np.asarray(
            held["held_relative_rotation"], dtype=float
        ).copy()
        mujoco.mj_forward(self.runtime.model, self.runtime.data)


def _pose_key(pose: Mapping[str, Any]) -> tuple[Any, ...]:
    position = pose["position"]
    orientation = pose["orientation"]
    orientation_keys = (
        ("roll", "pitch", "yaw")
        if orientation.get("representation") == "rpy"
        else ("x", "y", "z", "w")
    )
    return tuple(round(float(position[key]), 10) for key in ("x", "y", "z")) + tuple(
        round(float(orientation[key]), 10) for key in orientation_keys
    )


def _joint_key(joints: Sequence[float]) -> tuple[float, ...]:
    return tuple(round(float(value), 8) for value in joints)


def _same_joint_solution(first: Sequence[float], second: Sequence[float]) -> bool:
    a = np.asarray(first, dtype=float)
    b = np.asarray(second, dtype=float)
    wrapped = (a - b + np.pi) % (2.0 * np.pi) - np.pi
    return bool(np.linalg.norm(wrapped) < 1e-3)


def _deep_merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> dict[str, Any]:
    result = deepcopy(dict(base))
    for key, value in override.items():
        if isinstance(result.get(key), Mapping) and isinstance(value, Mapping):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result
