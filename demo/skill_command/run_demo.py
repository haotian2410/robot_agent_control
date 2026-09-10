"""Run structured skill commands with live, sequential target conversion."""

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

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from demo.common.registry import SceneRegistry
from demo.skill_command.converter import DEFAULT_RUNTIME, DEFAULT_VIEWER, SkillCommandConverter
from demo.skill_command.runtime import SkillRuntime


DEFAULT_COMMAND_FILE = Path(__file__).with_name("config_001.commands.json")
KEY_CODES = {"SPACE": 32, "ENTER": 257}


def run_demo(command_file: str | Path = DEFAULT_COMMAND_FILE, *, generated_config: str | Path | None = None) -> list[dict[str, Any]]:
    """Convert and execute one command at a time on one persistent runtime."""
    command_path = Path(command_file).resolve()
    with command_path.open("r", encoding="utf-8") as stream:
        source = json.load(stream)
    registry_value = source.get("registry")
    if not isinstance(registry_value, str):
        raise ValueError("command document must contain a registry path")
    registry_path = Path(registry_value)
    if not registry_path.is_absolute():
        registry_path = (command_path.parent / registry_path).resolve()
    registry = SceneRegistry(registry_path)
    runtime_config = _deep_merge(DEFAULT_RUNTIME, source.get("runtime", {}))
    viewer_config = _deep_merge(DEFAULT_VIEWER, source.get("viewer", {}))
    request_defaults = _deep_merge(registry.data.get("move_defaults", {}), source.get("request_defaults", {}))
    commands = source.get("commands")
    if not isinstance(commands, list):
        raise ValueError("command document must contain a commands list")

    session = SkillRuntime(registry, runtime_config)
    converter = SkillCommandConverter(registry.data, collision_checker=session.approach_checker, pose_provider=session.pose_provider)
    converter.reset()
    allowed_keys = {KEY_CODES[name] for name in viewer_config.get("continue_keys", ["SPACE", "ENTER"])}
    continue_event = threading.Event()

    def on_key(keycode: int) -> None:
        if keycode in allowed_keys:
            continue_event.set()

    results: list[dict[str, Any]] = []
    executed_steps: list[dict[str, Any]] = []
    with mujoco.viewer.launch_passive(session.model, session.data, key_callback=on_key) as viewer:
        _configure_camera(viewer, session.runtime, viewer_config)
        session.runtime.attach_viewer(viewer)
        viewer.sync()
        _hold_viewer(viewer, float(viewer_config.get("initial_pause", 1.0)), session.runtime.playback_fps)
        for command_index, command in enumerate(commands, start=1):
            if not viewer.is_running():
                break
            session.update()
            converted = converter.convert_command(command, command_index)
            if converted is None:
                continue
            steps = converted if isinstance(converted, list) else [converted]
            for step_index, step in enumerate(steps, start=1):
                if not viewer.is_running():
                    break
                label = f"[{command_index}/{len(commands)}]"
                if len(steps) > 1:
                    label += f" ({step_index}/{len(steps)})"
                print(f"\n{label} {step['name']}")
                result = session.execute_step(step, request_defaults)
                results.append(result)
                executed_steps.append(deepcopy(step))
                _print_result(step, result)
                if not result.get("success"):
                    print("测试失败，保留当前画面。关闭窗口结束测试。")
                    _wait_until_closed(viewer, session.runtime.playback_fps)
                    break
                if viewer_config.get("wait_for_key_between_steps", True):
                    continue_event.clear()
                    _wait_for_key(viewer, continue_event, session.runtime.playback_fps)
                else:
                    print(f"{step['name']}完成，自动继续下一步。")
            if results and not results[-1].get("success"):
                break
        else:
            print(f"全部 {len(results)} 个步骤完成。")
            if viewer_config.get("keep_open_after_last_test", True):
                print("关闭 MuJoCo 窗口以结束测试。")
                _wait_until_closed(viewer, session.runtime.playback_fps)
        session.runtime.attach_viewer(None)

    if generated_config is not None:
        output = Path(generated_config).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        config = {"scene": registry.data["scene"], "runtime": runtime_config, "viewer": viewer_config, "request_defaults": request_defaults, "steps": executed_steps}
        output.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return results


def _configure_camera(viewer: Any, runtime: Any, config: Mapping[str, Any]) -> None:
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


def _print_result(step: Mapping[str, Any], result: Mapping[str, Any]) -> None:
    if result.get("success"):
        if step.get("type") == "move":
            selection = result.get("selection", {})
            print(f"完成：{selection.get('path_strategy')} / {selection.get('planning_mode')}")
        else:
            print(f"{step.get('type')}完成。")
    else:
        print(json.dumps(result.get("error", result), ensure_ascii=False, indent=2))


def _deep_merge(base: Mapping[str, Any], override: Any) -> dict[str, Any]:
    result = deepcopy(dict(base))
    if not isinstance(override, Mapping):
        return result
    for key, value in override.items():
        if isinstance(result.get(key), Mapping) and isinstance(value, Mapping):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--commands", type=Path, default=DEFAULT_COMMAND_FILE)
    parser.add_argument("-o", "--generated-config", type=Path)
    args = parser.parse_args()
    results = run_demo(args.commands, generated_config=args.generated_config)
    raise SystemExit(0 if results and all(result.get("success") for result in results) else 1)


if __name__ == "__main__":
    main()
