from __future__ import annotations

import json
import unittest
from pathlib import Path

from demo.skill_command.converter import CommandConversionError, SkillCommandConverter, convert_file


PROJECT_ROOT = Path(__file__).resolve().parents[1]
COMMAND_FILE = PROJECT_ROOT / "demo" / "skill_command" / "config_001.commands.json"


class SkillCommandConverterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = convert_file(COMMAND_FILE, check_approach_collisions=False)

    def test_builds_test_configuration_shape(self) -> None:
        self.assertEqual(self.config["scene"], "world_model/robotsim/scene_001.xml")
        self.assertIn("runtime", self.config)
        self.assertIn("viewer", self.config)
        self.assertIn("request_defaults", self.config)
        source = json.loads(COMMAND_FILE.read_text(encoding="utf-8"))
        expected = sum(command["skill_name"] != "locate" for command in source["commands"])
        self.assertEqual(len(self.config["steps"]), expected)

    def test_scene_registry_uses_explicit_interactable_flags(self) -> None:
        registry = json.loads(
            (PROJECT_ROOT / "demo" / "common" / "scenes" / "scene_001.interactions.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertNotIn("adaptive_approach", registry)
        self.assertTrue(registry["objects"]["red_ball"]["interactable"])
        self.assertTrue(registry["objects"]["blue_cabinet_door"]["interactable"])
        self.assertFalse(registry["objects"]["work_table"]["interactable"])

    def test_resolves_object_pose_and_defaults_allow_path_change(self) -> None:
        first = self.config["steps"][0]
        self.assertEqual(first["type"], "move")
        self.assertTrue(first["allow_path_change"])
        self.assertEqual(first["request"]["motion"]["path_type"], "joint")
        self.assertEqual(
            first["request"]["target"]["position"],
            {"x": 0.4304673206, "y": -0.2246414123, "z": 0.81},
        )

    def test_reuses_action_request(self) -> None:
        pull = next(step for step in self.config["steps"] if step["type"] == "push_pull")
        self.assertEqual(pull["type"], "push_pull")
        self.assertEqual(pull["request"]["manipulation"]["operation"], "pull")
        self.assertEqual(pull["request"]["target"]["object_id"], "blue_cabinet_door")

    def test_resolves_relative_end_effector_pose(self) -> None:
        lift = next(
            step for step in self.config["steps"]
            if step["type"] == "move" and step["request"]["target"].get("position", {}).get("z") == 0.795
        )
        self.assertEqual(lift["request"]["target"]["position"]["z"], 0.795)

    def test_named_joint_target(self) -> None:
        home = self.config["steps"][-1]
        self.assertEqual(home["request"]["target"]["type"], "joint")
        self.assertEqual(home["request"]["motion"]["path_type"], "joint")

    def test_unknown_target_has_clear_error(self) -> None:
        temporary = COMMAND_FILE.parent / "_invalid.commands.json"
        source = json.loads(COMMAND_FILE.read_text(encoding="utf-8"))
        source["commands"] = [{"skill_name": "move", "parameters": {"target": "missing"}}]
        try:
            temporary.write_text(json.dumps(source), encoding="utf-8")
            with self.assertRaisesRegex(CommandConversionError, "not defined"):
                convert_file(temporary, check_approach_collisions=False)
        finally:
            temporary.unlink(missing_ok=True)

    def test_interactable_move_can_expand_to_verified_approach(self) -> None:
        registry_path = PROJECT_ROOT / "demo" / "common" / "scenes" / "scene_001.interactions.json"
        registry = json.loads(registry_path.read_text(encoding="utf-8"))
        source = {"commands": [{"skill_name": "move", "parameters": {"target": "red_ball", "planning_method": "linear"}}]}
        converter = SkillCommandConverter(
            registry, collision_checker=lambda candidate, target, object_key: True
        )
        steps = converter.convert(source)["steps"]
        self.assertEqual(len(steps), 2)
        self.assertEqual(steps[0]["request"]["motion"]["path_type"], "linear")
        self.assertEqual(steps[1]["request"]["motion"]["path_type"], "linear")
        self.assertFalse(steps[1]["allow_path_change"])
        self.assertAlmostEqual(steps[0]["request"]["target"]["position"]["z"], 0.87)

    def test_approach_search_uses_coarse_then_fine_batches(self) -> None:
        class BatchChecker:
            def __init__(self) -> None:
                self.calls = []

            def check_many(self, candidates, target, object_key):
                self.calls.append(candidates)
                if len(self.calls) < 3:
                    return [False] * len(candidates)
                return [False, True, False]

        registry_path = PROJECT_ROOT / "demo" / "common" / "scenes" / "scene_001.interactions.json"
        registry = json.loads(registry_path.read_text(encoding="utf-8"))
        checker = BatchChecker()
        converter = SkillCommandConverter(registry, collision_checker=checker)
        converted = converter.convert_command(
            {"skill_name": "move", "parameters": {"target": "red_ball"}}
        )
        self.assertEqual([len(batch) for batch in checker.calls], [1, 2, 3])
        self.assertAlmostEqual(converted[0]["request"]["target"]["position"]["z"], 0.85)

    def test_pose_provider_is_used_for_live_target_conversion(self) -> None:
        registry_path = PROJECT_ROOT / "demo" / "common" / "scenes" / "scene_001.interactions.json"
        registry = json.loads(registry_path.read_text(encoding="utf-8"))
        converter = SkillCommandConverter(
            registry,
            pose_provider=lambda object_key, spatial: {
                "position": [1.0, 2.0, 3.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
        )
        step = converter.convert_command(
            {"skill_name": "move", "parameters": {"target": "blue_cabinet_handle"}}
        )
        self.assertEqual(step["request"]["target"]["position"], {"x": 1.0, "y": 2.0, "z": 3.0})

    def test_pose_provider_accepts_array_like_values(self) -> None:
        class ArrayLike:
            def __init__(self, values):
                self.values = values

            def __iter__(self):
                return iter(self.values)

        registry_path = PROJECT_ROOT / "demo" / "common" / "scenes" / "scene_001.interactions.json"
        registry = json.loads(registry_path.read_text(encoding="utf-8"))
        converter = SkillCommandConverter(
            registry,
            pose_provider=lambda object_key, spatial: {
                "position": ArrayLike([1.0, 2.0, 3.0]),
                "quaternion_wxyz": ArrayLike([1.0, 0.0, 0.0, 0.0]),
            },
        )
        step = converter.convert_command(
            {"skill_name": "move", "parameters": {"target": "blue_cabinet_handle"}}
        )
        self.assertEqual(step["request"]["target"]["position"], {"x": 1.0, "y": 2.0, "z": 3.0})


if __name__ == "__main__":
    unittest.main()
