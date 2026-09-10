"""Convert compact skill commands to the visual-test configuration format."""

from __future__ import annotations

import argparse
import json
import math
from copy import deepcopy
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence


DEFAULT_RUNTIME = {
    "end_effector_site": "robotiq_2f85_pinch",
    "execution_mode": "kinematic",
    "playback_fps": 60.0,
    "playback_speed": 2.0,
    "minimum_playback_duration": 1.0,
}

DEFAULT_VIEWER = {
    "azimuth": 135.0,
    "elevation": -25.0,
    "distance_scale": 1.4,
    "initial_pause": 1.0,
    "wait_for_key_between_steps": False,
    "continue_keys": ["SPACE", "ENTER"],
    "keep_open_after_last_test": True,
}

END_EFFECTOR_DIRECTIONS = {
    "at": [0.0, 0.0, 0.0],
    "front": [1.0, 0.0, 0.0],
    "behind": [-1.0, 0.0, 0.0],
    "left": [0.0, 1.0, 0.0],
    "right": [0.0, -1.0, 0.0],
    "above": [0.0, 0.0, 1.0],
    "below": [0.0, 0.0, -1.0],
}

PoseProvider = Callable[
    [str, Mapping[str, Any]], Mapping[str, Any] | tuple[Sequence[float], Sequence[float]]
]

class CommandConversionError(ValueError):
    """A command or interaction registry cannot be converted safely."""


class SkillCommandConverter:
    """Convert a ``commands`` document using one scene interaction registry.

    Object poses default to ``spatial.reference_pose``.  A ``pose_provider``
    can be supplied for sequential execution so each target is resolved from
    the current scene state instead.
    """

    def __init__(
        self,
        registry: Mapping[str, Any],
        *,
        collision_checker: Callable[[Mapping[str, Any], Mapping[str, Any], str], bool] | None = None,
        pose_provider: PoseProvider | None = None,
    ) -> None:
        self.registry = deepcopy(dict(registry))
        self.objects = self.registry.get("objects")
        if not isinstance(self.objects, dict):
            raise CommandConversionError("interaction registry must contain an objects mapping")
        if not isinstance(self.registry.get("scene"), str):
            raise CommandConversionError("interaction registry must contain a scene path")
        self._target_index = self._build_target_index()
        self._last_pose: dict[str, Any] | None = None
        self.collision_checker = collision_checker
        self.pose_provider = pose_provider

    @classmethod
    def from_file(
        cls,
        registry_path: str | Path,
        *,
        collision_checker: Callable[[Mapping[str, Any], Mapping[str, Any], str], bool] | None = None,
        pose_provider: PoseProvider | None = None,
    ) -> "SkillCommandConverter":
        path = Path(registry_path)
        with path.open("r", encoding="utf-8") as stream:
            registry = json.load(stream)
        return cls(registry, collision_checker=collision_checker, pose_provider=pose_provider)

    def convert(self, source: Mapping[str, Any]) -> dict[str, Any]:
        commands = source.get("commands")
        if not isinstance(commands, list):
            raise CommandConversionError("command document must contain a commands list")

        self._last_pose = None
        steps = []
        for index, command in enumerate(commands, start=1):
            if not isinstance(command, Mapping):
                raise CommandConversionError(f"command {index} must be an object")
            step = self.convert_command(command, index)
            if step is None:
                continue
            if isinstance(step, list):
                steps.extend(step)
            else:
                steps.append(step)

        config = {
            "scene": self.registry["scene"],
            "runtime": _deep_merge(DEFAULT_RUNTIME, source.get("runtime", {})),
            "viewer": _deep_merge(DEFAULT_VIEWER, source.get("viewer", {})),
            "request_defaults": _deep_merge(
                self.registry.get("move_defaults", {}), source.get("request_defaults", {})
            ),
            "steps": steps,
        }
        return config

    def reset(self) -> None:
        """Reset state used by relative end-effector commands."""
        self._last_pose = None

    def convert_command(
        self, command: Mapping[str, Any], index: int = 1
    ) -> dict[str, Any] | list[dict[str, Any]] | None:
        """Convert one command using the converter's current runtime state."""
        if not isinstance(command, Mapping):
            raise CommandConversionError(f"command {index} must be an object")
        return self._convert_command(command, index)

    def _convert_command(
        self, command: Mapping[str, Any], index: int
    ) -> dict[str, Any] | list[dict[str, Any]] | None:
        skill_name = command.get("skill_name")
        parameters = command.get("parameters", {})
        if not isinstance(skill_name, str) or not skill_name:
            raise CommandConversionError(f"command {index} has no valid skill_name")
        if not isinstance(parameters, Mapping):
            raise CommandConversionError(f"command {index} parameters must be an object")

        # Locate only queries the runtime and has no corresponding executable
        # branch in tests/test.py, so it intentionally produces no test step.
        if skill_name == "locate":
            return None
        if skill_name == "move":
            return self._convert_move(parameters, index)
        if skill_name in {"grasp", "release", "press", "pull", "push"}:
            return self._convert_action(skill_name, parameters, index)
        raise CommandConversionError(f"command {index} uses unsupported skill {skill_name!r}")

    def _convert_move(
        self, parameters: Mapping[str, Any], index: int
    ) -> dict[str, Any] | list[dict[str, Any]]:
        target_id = parameters.get("target")
        if not isinstance(target_id, str) or not target_id:
            raise CommandConversionError(f"move command {index} has no valid target")

        named_targets = self.registry.get("named_targets", {})
        if target_id in named_targets:
            request = deepcopy(named_targets[target_id].get("request", {}))
            target = request.get("target", {})
            default_path_type = "joint" if target.get("type") == "joint" else "joint"
        elif target_id in {"end_effector", "__end_effector__"}:
            request = {"target": self._relative_end_effector_pose(parameters, index)}
            default_path_type = "linear"
        else:
            object_key, _, _ = self._resolve_target(target_id, index)
            request = {"target": self._object_pose(target_id, parameters, index)}
            default_path_type = "joint"

        path_type = parameters.get("path_type", parameters.get("planning_method", default_path_type))
        if path_type == "auto":
            path_type = default_path_type
        if path_type not in {"joint", "linear", "circular"}:
            raise CommandConversionError(
                f"move command {index} has unsupported path type {path_type!r}"
            )
        request.setdefault("motion", {})["path_type"] = path_type
        if "phase" in parameters:
            request["motion"]["phase"] = parameters["phase"]
        if "keep_end_effector_orientation" in parameters:
            request.setdefault("constraints", {})["keep_end_effector_orientation"] = bool(
                parameters["keep_end_effector_orientation"]
            )
        if "avoid_collision" in parameters:
            request.setdefault("constraints", {})["avoid_collision"] = bool(
                parameters["avoid_collision"]
            )
        request = _deep_merge(request, parameters.get("request", {}))

        if request.get("target", {}).get("type") == "pose":
            self._last_pose = deepcopy(request["target"])

        # Interactable objects get a straight-line approach step only when a
        # collision checker is available and confirms both the pre-point and
        # the connecting segment. Offline conversion otherwise remains a safe
        # one-step conversion.
        if (
            target_id not in named_targets
            and target_id not in {"end_effector", "__end_effector__"}
            and self.objects[object_key].get("interactable") is True
            and request.get("target", {}).get("type") == "pose"
        ):
            approach = self._find_approach_pose(request["target"], object_key)
            if approach is not None:
                approach_request = deepcopy(request)
                approach_request["target"] = approach
                approach_request.setdefault("motion", {})["path_type"] = path_type
                final_request = deepcopy(request)
                final_request.setdefault("motion", {})["path_type"] = "linear"
                final_request.setdefault("motion", {})["phase"] = "contact"
                return [
                    {
                        "name": str(parameters.get("name", f"move near {target_id}")),
                        "type": "move",
                        "allow_path_change": bool(parameters.get("allow_path_change", True)),
                        "request": approach_request,
                    },
                    {
                        "name": str(parameters.get("contact_name", f"move linearly to {target_id}")),
                        "type": "move",
                        "allow_path_change": False,
                        "request": final_request,
                    },
                ]

        return {
            "name": str(parameters.get("name", f"move to {target_id}")),
            "type": "move",
            "allow_path_change": bool(parameters.get("allow_path_change", True)),
            "request": request,
        }

    def _find_approach_pose(
        self, target_pose: Mapping[str, Any], object_key: str
    ) -> dict[str, Any] | None:
        """Find a collinear pre-point 10–20 cm behind the tool z-axis."""
        if self.collision_checker is None:
            return None
        target_position = target_pose.get("position", {})
        orientation = target_pose.get("orientation", {})
        if not all(axis in target_position for axis in ("x", "y", "z")):
            return None
        z_axis = _tool_z_axis(orientation)

        def build_candidate(distance_cm: int) -> dict[str, Any]:
            distance = distance_cm / 100.0
            candidate = deepcopy(dict(target_pose))
            candidate["position"] = {
                axis: float(target_position[axis]) - z_axis[i] * distance
                for i, axis in enumerate(("x", "y", "z"))
            }
            return candidate

        # Check the representative far/middle/near points together.  Only if
        # that coarse pass fails do we inspect the remaining centimetre-spaced
        # positions, in small descending batches.  A batch-capable MuJoCo
        # checker evaluates each batch concurrently; simple injected checkers
        # retain the original deterministic sequential behaviour.
        # The farthest point gets a fast-path of its own.  When it succeeds we
        # avoid waiting for speculative nearer checks; after it fails the
        # remaining representative distances are evaluated concurrently.
        coarse_batches = ((20,), (15, 10))
        fine = (19, 18, 17, 16, 14, 13, 12, 11)
        distance_batches = list(coarse_batches)
        distance_batches.extend(
            tuple(fine[start : start + 3]) for start in range(0, len(fine), 3)
        )
        batch_checker = getattr(self.collision_checker, "check_many", None)
        for distances in distance_batches:
            candidates = [build_candidate(distance_cm) for distance_cm in distances]
            if callable(batch_checker):
                valid = list(batch_checker(candidates, target_pose, object_key))
                if len(valid) != len(candidates):
                    raise CommandConversionError(
                        "approach collision checker returned an invalid batch result"
                    )
                for candidate, is_valid in zip(candidates, valid):
                    if is_valid:
                        return candidate
                continue
            for candidate in candidates:
                if self.collision_checker(candidate, target_pose, object_key):
                    return candidate
        return None

    def _convert_action(
        self, skill_name: str, parameters: Mapping[str, Any], index: int
    ) -> dict[str, Any]:
        target_id = parameters.get("target")
        if not isinstance(target_id, str) or not target_id:
            raise CommandConversionError(f"{skill_name} command {index} has no valid target")
        object_key, _, _ = self._resolve_target(target_id, index)
        obj = self.objects[object_key]
        action_requests = obj.get("action_requests", {})
        if skill_name not in action_requests:
            raise CommandConversionError(
                f"target {target_id!r} does not define action_requests.{skill_name}"
            )
        request = _deep_merge(action_requests[skill_name], parameters.get("request", {}))
        return {
            "name": str(parameters.get("name", f"{skill_name} {target_id}")),
            "type": "push_pull" if skill_name in {"push", "pull"} else skill_name,
            "request": request,
        }

    def _object_pose(
        self, target_id: str, parameters: Mapping[str, Any], index: int
    ) -> dict[str, Any]:
        object_key, indexed_anchor, indexed_relation = self._resolve_target(target_id, index)
        spatial = self.objects[object_key].get("spatial")
        if not isinstance(spatial, Mapping):
            raise CommandConversionError(f"target {target_id!r} has no spatial configuration")

        reference = spatial.get("reference_pose", {})
        position = reference.get("position")
        quaternion = reference.get("quaternion_wxyz", [1.0, 0.0, 0.0, 0.0])
        source_rotation = _quaternion_matrix(quaternion)
        if self.pose_provider is not None:
            live_pose = self.pose_provider(object_key, spatial)
            if isinstance(live_pose, Mapping):
                position = live_pose.get("position")
                quaternion = live_pose.get("quaternion_wxyz", quaternion)
                source_rotation = live_pose.get("rotation_matrix", _quaternion_matrix(quaternion))
            else:
                position, quaternion = live_pose
                source_rotation = _quaternion_matrix(quaternion)
        if not _is_vector(position, 3) or not _is_vector(quaternion, 4):
            raise CommandConversionError(f"target {target_id!r} has an invalid reference_pose")

        # Runtime providers commonly return NumPy arrays or other iterable
        # array-like values. Normalize validated vectors so the pose math can
        # safely use indexing and float conversion for every provider type.
        position = list(position)
        quaternion = list(quaternion)

        anchor_key = str(parameters.get("anchor", indexed_anchor or spatial.get("default_anchor", "target")))
        anchors = spatial.get("anchors", {})
        if anchor_key not in anchors:
            raise CommandConversionError(f"target {target_id!r} has no anchor {anchor_key!r}")
        anchor = anchors[anchor_key]
        local_anchor = anchor.get("local_position", [0.0, 0.0, 0.0])
        relation = str(parameters.get("relation", indexed_relation or "at"))
        direction = spatial.get("directions_local", {}).get(relation)
        if not _is_vector(local_anchor, 3) or not _is_vector(direction, 3):
            raise CommandConversionError(
                f"target {target_id!r} has no valid local direction for relation {relation!r}"
            )
        default_distance = spatial.get("default_distances", {}).get(relation, 0.0)
        distance = float(parameters.get("distance_m", default_distance))
        unit_direction = _normalize(direction)
        local_target = [float(local_anchor[i]) + unit_direction[i] * distance for i in range(3)]
        world_offset = _rotate_by_quaternion(local_target, quaternion)
        world_position = [float(position[i]) + world_offset[i] for i in range(3)]

        orientation = anchor.get("tool_orientation")
        if orientation is None:
            orientation = spatial.get("relation_tool_orientations", {}).get(relation)
        if orientation is None:
            orientation = spatial.get("tool_orientation")
        if not isinstance(orientation, Mapping):
            raise CommandConversionError(f"target {target_id!r} has no tool orientation")
        moved_orientation = spatial.get("moved_tool_orientation")
        if moved_orientation:
            reference_rotation = _quaternion_matrix(
                reference.get("quaternion_wxyz", [1.0, 0.0, 0.0, 0.0])
            )
            threshold = float(spatial.get("moved_orientation_threshold", 0.1))
            if _matrix_distance(source_rotation, reference_rotation) > threshold:
                orientation = moved_orientation
        return {
            "type": "pose",
            "frame": "world",
            "position": dict(zip(("x", "y", "z"), world_position)),
            "orientation": deepcopy(dict(orientation)),
        }

    def _relative_end_effector_pose(
        self, parameters: Mapping[str, Any], index: int
    ) -> dict[str, Any]:
        if self._last_pose is None:
            raise CommandConversionError(
                f"move command {index} targets the end effector before any pose target is known"
            )
        relation = str(parameters.get("relation", "at"))
        direction = END_EFFECTOR_DIRECTIONS.get(relation)
        if direction is None:
            raise CommandConversionError(f"move command {index} has unknown relation {relation!r}")
        distance = float(parameters.get("distance_m", 0.0))
        pose = deepcopy(self._last_pose)
        for axis, delta in zip(("x", "y", "z"), direction):
            pose["position"][axis] = float(pose["position"][axis]) + delta * distance
        return pose

    def _resolve_target(self, target_id: str, index: int) -> tuple[str, str | None, str | None]:
        matches = self._target_index.get(target_id, [])
        if not matches:
            raise CommandConversionError(
                f"command {index} target {target_id!r} is not defined in the interaction registry"
            )
        unique = list(dict.fromkeys(matches))
        if len(unique) != 1:
            raise CommandConversionError(
                f"command {index} target {target_id!r} is ambiguous in the interaction registry"
            )
        return unique[0]

    def _build_target_index(self) -> dict[str, list[tuple[str, str | None, str | None]]]:
        index: dict[str, list[tuple[str, str | None, str | None]]] = {}
        for object_key, obj in self.objects.items():
            base = (object_key, None, None)
            for identifier in {object_key, obj.get("object_id")}:
                if isinstance(identifier, str):
                    index.setdefault(identifier, []).append(base)
            spatial = obj.get("spatial", {})
            for anchor_key, anchor in spatial.get("anchors", {}).items():
                identifier = anchor.get("target_id")
                if isinstance(identifier, str):
                    index.setdefault(identifier, []).append((object_key, anchor_key, "at"))
            for relation, identifier in spatial.get("relation_target_ids", {}).items():
                if isinstance(identifier, str):
                    index.setdefault(identifier, []).append((object_key, None, relation))
        return index


def convert_file(
    command_path: str | Path,
    output_path: str | Path | None = None,
    *,
    collision_checker: Callable[[Mapping[str, Any], Mapping[str, Any], str], bool] | None = None,
    check_approach_collisions: bool = True,
    pose_provider: PoseProvider | None = None,
) -> dict[str, Any]:
    """Convert one command JSON document and optionally write the result."""
    command_path = Path(command_path).resolve()
    with command_path.open("r", encoding="utf-8") as stream:
        source = json.load(stream)
    registry_value = source.get("registry")
    if not isinstance(registry_value, str):
        raise CommandConversionError("command document must contain a registry path")
    registry_path = Path(registry_value)
    if not registry_path.is_absolute():
        registry_path = command_path.parent / registry_path
    if collision_checker is None and check_approach_collisions:
        # Import lazily so the pure conversion class remains usable without
        # initializing MuJoCo when callers explicitly disable approach checks.
        from demo.common.registry import SceneRegistry
        from demo.skill_command.approach_collision import MujocoApproachCollisionChecker

        collision_checker = MujocoApproachCollisionChecker(
            SceneRegistry(registry_path), source.get("runtime", {})
        )
    result = SkillCommandConverter.from_file(
        registry_path,
        collision_checker=collision_checker,
        pose_provider=pose_provider,
    ).convert(source)
    if output_path is not None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
    return result


def _deep_merge(base: Mapping[str, Any], override: Any) -> dict[str, Any]:
    result = deepcopy(dict(base))
    if override is None:
        return result
    if not isinstance(override, Mapping):
        raise CommandConversionError("configuration overrides must be objects")
    for key, value in override.items():
        if isinstance(result.get(key), Mapping) and isinstance(value, Mapping):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def _is_vector(value: Any, length: int) -> bool:
    if isinstance(value, (str, bytes)):
        return False
    try:
        values = list(value)
    except (TypeError, ValueError):
        return False
    return len(values) == length and all(
        isinstance(item, (int, float)) and math.isfinite(float(item)) for item in values
    )


def _normalize(vector: Sequence[float]) -> list[float]:
    length = math.sqrt(sum(float(value) ** 2 for value in vector))
    if length <= 1e-12:
        return [0.0, 0.0, 0.0]
    return [float(value) / length for value in vector]


def _rotate_by_quaternion(vector: Sequence[float], quaternion_wxyz: Sequence[float]) -> list[float]:
    w, x, y, z = _normalize_quaternion(quaternion_wxyz)
    rotation = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    return [sum(row[i] * float(vector[i]) for i in range(3)) for row in rotation]


def _quaternion_matrix(quaternion: Sequence[float]) -> tuple[tuple[float, float, float], ...]:
    w, x, y, z = _normalize_quaternion(quaternion)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _matrix_distance(first: Any, second: Any) -> float:
    return math.sqrt(
        sum(
            (float(first[row][column]) - float(second[row][column])) ** 2
            for row in range(3)
            for column in range(3)
        )
    )


def _normalize_quaternion(quaternion: Sequence[float]) -> tuple[float, float, float, float]:
    length = math.sqrt(sum(float(value) ** 2 for value in quaternion))
    if length <= 1e-12:
        raise CommandConversionError("reference_pose quaternion must not be zero")
    return tuple(float(value) / length for value in quaternion)  # type: ignore[return-value]


def _tool_z_axis(orientation: Mapping[str, Any]) -> list[float]:
    """Return the tool's local +Z axis expressed in world coordinates."""
    if orientation.get("representation", "quaternion") == "rpy":
        roll, pitch, yaw = (
            math.radians(float(orientation[key])) for key in ("roll", "pitch", "yaw")
        )
        cr, cp, cy = math.cos(roll), math.cos(pitch), math.cos(yaw)
        sr, sp, sy = math.sin(roll), math.sin(pitch), math.sin(yaw)
        return [cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr]
    quaternion = _normalize_quaternion(
        [orientation.get(key, 0.0) for key in ("w", "x", "y", "z")]
    )
    return _rotate_by_quaternion([0.0, 0.0, 1.0], quaternion)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command_file", type=Path, help="input *.commands.json file")
    parser.add_argument("-o", "--output", type=Path, help="output test JSON file; stdout if omitted")
    args = parser.parse_args()
    result = convert_file(args.command_file, args.output)
    if args.output is None:
        print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
