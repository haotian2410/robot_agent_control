"""Compatibility import for the canonical execution namespace."""

from ..executor import ControlExecutor, ExecutionPreflightError

__all__ = ["ControlExecutor", "ExecutionPreflightError"]
