"""Smooth configurable visual motion sequence."""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

import mujoco.viewer


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from skills.motion.move.skill import MoveSkill
from skills.manipulation.grasp.skill import GraspSkill
from skills.manipulation.press.skill import PressSkill
from skills.manipulation.push_pull.skill import PushPullSkill
from skills.manipulation.release.skill import ReleaseSkill
from utils import SceneRobotRuntime


DEFAULT_CONFIG = Path(__file__).with_name("test_config_001.json")
KEY_CODES = {"SPACE": 32, "ENTER": 257}


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    scene_path = Path(config["scene"])
    if not scene_path.is_absolute():
        scene_path = PROJECT_ROOT / scene_path
    config["scene"] = str(scene_path.resolve())
    steps = config.get("steps", config.get("movements"))
    if not steps:
        raise ValueError("Config must contain at least one step.")
    config["steps"] = steps
    return config


def build_request(defaults: Mapping[str, Any], movement: Mapping[str, Any]) -> dict[str, Any]:
    request = _deep_merge(deepcopy(defaults), movement["request"])
    allow_change = bool(movement.get("allow_path_change", False))
    request.setdefault("motion", {})["path_constraint"] = "soft" if allow_change else "hard"
    if not allow_change:
        request.setdefault("planning", {})["mode"] = "direct"
        request["planning"]["allow_replan"] = False
    return request


def run_visual_test(config_path: Path = DEFAULT_CONFIG) -> list[dict[str, Any]]:
    config = load_config(config_path)
    runtime_config = config.get("runtime", {})
    viewer_config = config.get("viewer", {})
    runtime = SceneRobotRuntime(
        config["scene"],
        end_effector_site=runtime_config.get("end_effector_site", "robotiq_2f85_pinch"),
        execution_mode=runtime_config.get("execution_mode", "kinematic"),
        realtime=True,
        playback_fps=runtime_config.get("playback_fps", 60.0),
        playback_speed=runtime_config.get("playback_speed", 1.0),
        minimum_playback_duration=runtime_config.get("minimum_playback_duration", 4.0),
    )
    skill = MoveSkill(robot_runtime=runtime)
    grasp_skill = GraspSkill(robot_runtime=runtime)
    press_skill = PressSkill(robot_runtime=runtime)
    push_pull_skill = PushPullSkill(robot_runtime=runtime)
    release_skill = ReleaseSkill(robot_runtime=runtime)
    continue_event = threading.Event()
    allowed_keys = {KEY_CODES[name] for name in viewer_config.get("continue_keys", ["SPACE", "ENTER"])}

    def on_key(keycode: int) -> None:
        if keycode in allowed_keys:
            continue_event.set()

    results: list[dict[str, Any]] = []
    with mujoco.viewer.launch_passive(runtime.model, runtime.data, key_callback=on_key) as viewer:
        _configure_camera(viewer, runtime, viewer_config)
        runtime.attach_viewer(viewer)
        viewer.sync()
        _hold_viewer(viewer, float(viewer_config.get("initial_pause", 1.0)), runtime.playback_fps)

        steps = config["steps"]
        for index, step in enumerate(steps):
            if not viewer.is_running():
                break
            step_type = step.get("type", "move")
            print(f"\n[{index + 1}/{len(steps)}] {step['name']}")
            if step_type == "move":
                request = build_request(config.get("request_defaults", {}), step)
                print(f"path_type={request['motion']['path_type']}, allow_path_change={step.get('allow_path_change', False)}")
                print(f"target={request['target']}")
                result = skill.execute(request)
                _print_result(result)
            else:
                result = _execute_action(step, runtime, grasp_skill, release_skill, press_skill, push_pull_skill)
                _print_action_result(step, result)
                if result["success"] and step_type in {"grasp", "release"}:
                    skill = MoveSkill(robot_runtime=runtime)
            results.append(result)
            if not result["success"]:
                print("测试失败，保留当前画面。关闭窗口结束测试。")
                _wait_until_closed(viewer, runtime.playback_fps)
                break
            if index < len(steps) - 1 and viewer_config.get("wait_for_key_between_steps", True):
                continue_event.clear()
                next_name = steps[index + 1]["name"]
                print(f"{step['name']}完成。请在 MuJoCo 窗口按 Space 或 Enter，继续{next_name}。")
                _wait_for_key(viewer, continue_event, runtime.playback_fps)
            elif index < len(steps) - 1:
                print(f"{step['name']}完成，自动继续下一步。")
        else:
            print(f"全部 {len(steps)} 个独立步骤完成。")
            if viewer_config.get("keep_open_after_last_test", True):
                print("关闭 MuJoCo 窗口以结束测试。")
                _wait_until_closed(viewer, runtime.playback_fps)
        runtime.attach_viewer(None)
    return results


def _configure_camera(viewer: Any, runtime: SceneRobotRuntime, config: Mapping[str, Any]) -> None:
    viewer.cam.lookat[:] = runtime.model.stat.center
    viewer.cam.distance = max(float(config.get("distance_scale", 1.4)) * runtime.model.stat.extent, 1.0)
    viewer.cam.azimuth = float(config.get("azimuth", 135.0))
    viewer.cam.elevation = float(config.get("elevation", -25.0))


def _wait_for_key(viewer: Any, event: threading.Event, fps: float) -> None:
    while viewer.is_running() and not event.is_set():
        viewer.sync()
        time.sleep(1.0 / fps)


def _wait_until_closed(viewer: Any, fps: float) -> None:
    while viewer.is_running():
        viewer.sync()
        time.sleep(1.0 / fps)


def _hold_viewer(viewer: Any, seconds: float, fps: float) -> None:
    deadline = time.perf_counter() + seconds
    while viewer.is_running() and time.perf_counter() < deadline:
        viewer.sync()
        time.sleep(1.0 / fps)


def _deep_merge(base: dict[str, Any], override: Mapping[str, Any]) -> dict[str, Any]:
    for key, value in override.items():
        if isinstance(value, Mapping) and isinstance(base.get(key), dict):
            base[key] = _deep_merge(base[key], value)
        else:
            base[key] = deepcopy(value)
    return base


def _print_result(result: Mapping[str, Any]) -> None:
    if result["success"]:
        selection = result["selection"]
        print(f"完成：{selection['path_strategy']} / {selection['planning_mode']}，轨迹点={len(result['trajectory']['waypoints'])}")
    else:
        print(json.dumps(result["error"], ensure_ascii=False, indent=2))


def _execute_action(
    action: Mapping[str, Any],
    runtime: SceneRobotRuntime,
    grasp_skill: GraspSkill,
    release_skill: ReleaseSkill,
    press_skill: PressSkill,
    push_pull_skill: PushPullSkill,
) -> dict[str, Any]:
    action_type = action.get("type")
    if action_type == "grasp":
        return grasp_skill.execute(action["request"])
    if action_type == "release":
        result = release_skill.execute(action["request"])
        if not result["success"]:
            return result
        region = action.get("verify_region")
        if region:
            body_id = mujoco.mj_name2id(runtime.model, mujoco.mjtObj.mjOBJ_BODY, action.get("object_body", "red_ball_body"))
            position = runtime.data.xpos[body_id]
            inside = all(region["min"][i] <= position[i] <= region["max"][i] for i in range(3))
            result["placement_verification"] = {"inside": inside, "position": position.tolist()}
            if not inside:
                return {"success": False, "error": {"error_code": "PLACE_VERIFICATION_FAILED", "error_message": f"Released object is outside the expected cabinet region: {position.tolist()}", "failed_stage": "release_verification", "recoverable": True}}
        return result
    if action_type == "press":
        return press_skill.execute(action["request"])
    if action_type == "push_pull":
        return push_pull_skill.execute(action["request"])
    return {"success": False, "error": {"error_code": "INVALID_TEST_ACTION", "error_message": f"Unsupported post_action type: {action_type}", "failed_stage": "test_action", "recoverable": True}}


def _print_action_result(action: Mapping[str, Any], result: Mapping[str, Any]) -> None:
    labels = {"grasp": "抓取", "release": "释放", "press": "按压", "push_pull": "推拉"}
    label = labels.get(action.get("type"), "动作")
    if result["success"]:
        print(f"{label}完成。")
    else:
        print(f"{label}失败：{json.dumps(result['error'], ensure_ascii=False)}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Visual Move Skill sequence test")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    config_path = args.config.resolve()
    expected_count = len(load_config(config_path)["steps"])
    results = run_visual_test(config_path)
    raise SystemExit(0 if len(results) == expected_count and all(result["success"] for result in results) else 1)


if __name__ == "__main__":
    main()
