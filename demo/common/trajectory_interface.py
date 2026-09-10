"""Trajectory receiver contract shared by demo frontends."""

from __future__ import annotations

from typing import Any, Mapping, Protocol, runtime_checkable


@runtime_checkable
class TrajectoryReceiver(Protocol):
    def receive_trajectory(self, packet: Mapping[str, Any]) -> None:
        """Receive one planned trajectory before execution."""


def send_trajectory(receiver: TrajectoryReceiver, packet: Mapping[str, Any]) -> None:
    if not isinstance(receiver, TrajectoryReceiver):
        raise TypeError("trajectory_receiver must implement receive_trajectory(packet)")
    receiver.receive_trajectory(packet)
