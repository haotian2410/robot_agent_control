"""Public API for the deterministic robot control package."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def _ensure_control_demo_namespace() -> None:
    """Avoid an unrelated cwd ``demo`` package shadowing control fixtures.

    The legacy skills still import ``demo.common`` as a top-level namespace.
    Load control's bundled package by its explicit installed source location
    when another package with that name is already visible.  This does not
    mutate ``sys.path`` and keeps imports deterministic from any cwd.
    """
    existing = sys.modules.get("demo")
    root = Path(__file__).resolve().parents[2]
    package_init = root / "demo" / "__init__.py"
    if not package_init.is_file():
        return
    existing_file = getattr(existing, "__file__", None)
    if existing is not None and existing_file:
        try:
            if Path(existing_file).resolve().is_relative_to((root / "demo").resolve()):
                return
        except (OSError, ValueError):
            pass
    if existing is None or existing_file:
        spec = importlib.util.spec_from_file_location(
            "demo", package_init, submodule_search_locations=[str(root / "demo")]
        )
        if spec is None or spec.loader is None:
            return
        module = importlib.util.module_from_spec(spec)
        sys.modules["demo"] = module
        spec.loader.exec_module(module)


_ensure_control_demo_namespace()

from .contracts import (
    CommandDocument,
    ExecutionFailure,
    ExecutionOptions,
    ExecutionReport,
    RuntimeStepReport,
    SkillCommand,
    ViewerMode,
    load_command_document,
)
from .executor import ControlExecutor

__all__ = [
    "CommandDocument",
    "ControlExecutor",
    "ExecutionFailure",
    "ExecutionOptions",
    "ExecutionReport",
    "RuntimeStepReport",
    "SkillCommand",
    "ViewerMode",
    "load_command_document",
]
