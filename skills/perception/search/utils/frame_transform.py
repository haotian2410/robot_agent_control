"""Camera and control-target frame transformation interfaces."""

from __future__ import annotations

from typing import Any, Mapping


class FrameTransformError(RuntimeError):
    """Raised when a viewpoint cannot be transformed between required frames."""


class ViewpointTransformer:
    """Backend-independent transform and hand-eye conversion interface."""

    def __init__(self, transform_service: Any = None) -> None:
        self.transform_service = transform_service

    def transform_camera_pose(
        self,
        camera_pose: Mapping[str, Any],
        output_frame: str,
        timestamp: Any = None,
    ) -> Mapping[str, Any]:
        """Transform a candidate camera pose to the requested output frame."""
        raise NotImplementedError("Connect to the project's transform service.")

    def camera_pose_to_control_target(
        self,
        camera_pose: Mapping[str, Any],
        camera: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """Convert desired camera pose to end-effector, pan-tilt, or base target."""
        raise NotImplementedError(
            "Implement hand-eye, pan-tilt, or mobile-camera target conversion."
        )
