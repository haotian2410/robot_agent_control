"""Backward-compatible frontend for the formal control executor."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from robot_agent_control import ControlExecutor


DEFAULT_COMMAND_FILE = Path(__file__).with_name("config_001.commands.json")


def run_demo(
    command_file: str | Path = DEFAULT_COMMAND_FILE,
    *,
    generated_config: str | Path | None = None,
) -> list[dict[str, Any]]:
    """Execute a legacy command file while preserving the original API."""
    if generated_config is not None:
        raise ValueError("--generated-config moved to the compile stage")
    report = ControlExecutor().execute(command_file, viewer_mode="auto")
    return [step.result for step in report.steps]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--commands", type=Path, default=DEFAULT_COMMAND_FILE)
    parser.add_argument(
        "--viewer-mode", choices=("auto", "step", "headless"), default="auto"
    )
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    report = ControlExecutor().execute(
        args.commands, viewer_mode=args.viewer_mode, output_dir=args.output_dir
    )
    raise SystemExit(0 if report.success else 1)


if __name__ == "__main__":
    main()
