"""Small helpers shared by Cartesian Move strategies."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np


def effective_orientation(request: Mapping[str, Any], context: Mapping[str, Any]) -> tuple[Any, bool]:
    """Return the orientation to use for every Cartesian waypoint.

    Holding orientation is deliberately expressed as a path constraint, rather
    than by relying on a caller to repeat the current orientation in its target.
    """
    hold = bool(request["constraints"].get("keep_end_effector_orientation", False))
    if not hold:
        return request["target"].get("orientation"), False

    target_orientation = request["target"].get("orientation")
    if target_orientation is not None:
        # A configured target orientation is authoritative.  Keep it fixed
        # for every Cartesian waypoint instead of silently replacing it with
        # the orientation at the start of the segment.
        return target_orientation, True
    quaternion = np.asarray(context["current_pose"]["quaternion_wxyz"], dtype=float)
    if quaternion.shape != (4,):
        raise ValueError("Current end-effector pose does not contain a quaternion.")
    quaternion /= np.linalg.norm(quaternion)
    return {
        "representation": "quaternion",
        "x": float(quaternion[1]),
        "y": float(quaternion[2]),
        "z": float(quaternion[3]),
        "w": float(quaternion[0]),
    }, True
