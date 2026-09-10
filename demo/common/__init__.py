"""Shared building blocks used by the demo command frontends."""

from .registry import SceneRegistry, SceneRegistryError
from .trajectory_interface import TrajectoryReceiver, send_trajectory

__all__ = ["SceneRegistry", "SceneRegistryError", "TrajectoryReceiver", "send_trajectory"]
