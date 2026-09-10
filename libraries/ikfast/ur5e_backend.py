"""Load a compiled UR5e IKFast extension without hiding its availability."""

from __future__ import annotations

import importlib
from typing import Sequence

import numpy as np


def solve(position: Sequence[float], rotation: Sequence[Sequence[float]]) -> list[list[float]]:
    """Return all analytic solutions from a compiled `ur5e_ikfast` module."""
    try:
        module = importlib.import_module("ur5e_ikfast")
    except ImportError as exc:
        raise RuntimeError(
            "Compiled ur5e_ikfast extension is unavailable on this Windows environment."
        ) from exc
    factory = getattr(module, "PyKinematics", None)
    if not callable(factory):
        raise RuntimeError("ur5e_ikfast does not expose PyKinematics.")
    backend = factory()
    transform = np.eye(4)
    transform[:3, :3] = np.asarray(rotation, dtype=float)
    transform[:3, 3] = np.asarray(position, dtype=float)
    raw = backend.inverse(transform[:3].reshape(-1).tolist())
    dof = int(backend.getDOF())
    if not raw:
        return []
    if dof <= 0 or len(raw) % dof:
        raise RuntimeError("IKFast returned an invalid solution array.")
    return [raw[index:index + dof] for index in range(0, len(raw), dof)]
