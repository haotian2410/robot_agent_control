"""Command-line entry point for deterministic control execution."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .contracts import ViewerMode
from .executor import ControlExecutor, ExecutionPreflightError


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--commands", type=Path, required=True)
    parser.add_argument(
        "--viewer-mode", choices=[item.value for item in ViewerMode], default="auto"
    )
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    try:
        report = ControlExecutor().execute(
            args.commands, viewer_mode=args.viewer_mode, output_dir=args.output_dir
        )
    except ExecutionPreflightError as exc:
        print(json.dumps({"success": False, "error_code": exc.code, "error_message": str(exc)}, ensure_ascii=False))
        raise SystemExit(2) from exc
    print(report.model_dump_json(indent=2))
    raise SystemExit(0 if report.success else 1)


if __name__ == "__main__":
    main()

