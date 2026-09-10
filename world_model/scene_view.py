"""Load and display a MuJoCo XML scene."""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_SCENE = PROJECT_ROOT / "empty_scene.xml"


def create_scene(scene_path: str | Path = DEFAULT_SCENE) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Load an MJCF/XML file and create its runtime data."""
    path = Path(scene_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Scene file not found: {path}")

    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    # MuJoCo does not automatically apply XML keyframes.  Treat a scene's
    # optional "home" key as its authored initial display state.
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_key)
        for joint_name in (
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                continue
            actuator_ids = [
                actuator_id
                for actuator_id in range(model.nu)
                if model.actuator_trnid[actuator_id, 0] == joint_id
            ]
            if actuator_ids:
                data.ctrl[actuator_ids[0]] = data.qpos[model.jnt_qposadr[joint_id]]
    mujoco.mj_forward(model, data)
    return model, data


def choose_scene() -> Path | None:
    """Open the native file picker and return the selected XML file."""
    from tkinter import Tk, filedialog

    root = Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    selected = filedialog.askopenfilename(
        title="Select a MuJoCo scene",
        initialdir=DEFAULT_SCENE.parent,
        filetypes=(("MuJoCo XML", "*.xml"), ("All files", "*.*")),
    )
    root.destroy()
    return Path(selected) if selected else None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Load and display a MuJoCo scene.")
    parser.add_argument(
        "scene",
        nargs="?",
        type=Path,
        default=DEFAULT_SCENE,
        help=f"scene XML path (default: {DEFAULT_SCENE})",
    )
    parser.add_argument(
        "--select",
        action="store_true",
        help="choose a scene using a file dialog",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    scene_path = choose_scene() if args.select else args.scene
    if scene_path is None:
        print("No scene selected.")
        return

    model, data = create_scene(scene_path)
    print(f"MuJoCo {mujoco.__version__}: loaded——————[ {Path(scene_path).resolve()}]")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = model.stat.center
        viewer.cam.distance = max(1.5 * model.stat.extent, 1.0)
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -35.0

        while viewer.is_running():
            step_start = time.perf_counter()
            mujoco.mj_step(model, data)
            viewer.sync()
            remaining = model.opt.timestep - (time.perf_counter() - step_start)
            if remaining > 0:
                time.sleep(remaining)

if __name__ == "__main__":
    main()
