"""Coordinate-frame transformation interface skeleton."""

from __future__ import annotations

from typing import Any


class FrameTransformError(RuntimeError):
    """Raised when a required transform is missing, stale or invalid."""


class FrameTransformer:
    """Adapter around TF2 or another transform service."""

    def __init__(self, transform_backend: Any = None) -> None:
        self.transform_backend = transform_backend

    def transform_pose(
        self,
        pose: Any,
        *,
        source_frame: str,
        target_frame: str,
        timestamp: Any = None,
    ) -> Any:
        """Transform a pose into target_frame."""
        raise NotImplementedError("Connect transform_pose() to TF/runtime backend.")

    def transform_point(
        self,
        point: Any,
        *,
        source_frame: str,
        target_frame: str,
        timestamp: Any = None,
    ) -> Any:
        """Transform a 3D point into target_frame."""
        raise NotImplementedError("Connect transform_point() to TF/runtime backend.")

    def transform_vector(
        self,
        vector: Any,
        *,
        source_frame: str,
        target_frame: str,
        timestamp: Any = None,
    ) -> Any:
        """Transform a direction vector without applying translation."""
        raise NotImplementedError("Connect transform_vector() to TF/runtime backend.")
