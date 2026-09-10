"""Default open-close-hold grasp strategy."""

from __future__ import annotations

import time
from typing import Any, Mapping


class DefaultGrasp:
    """Execute a standard open-close-hold grasp using a compatible gripper."""

    strategy_id = "default_grasp"

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def execute(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """Execute the selected operation and return raw verification evidence."""
        defaults = {
            "open_width": 0.08, "close_width": 0.0, "close_speed": 0.5,
            "grasp_force": 20.0, "hold_force": 10.0,
            "contact_threshold": 2.0, "position_tolerance": 0.002,
            "force_tolerance": 2.0, "settle_time": 0.3,
        }
        configured = self.config.get("strategies", {}).get("default_grasp", {}).get("defaults", {})
        params = {**defaults, **configured, **request.get("strategy_params", {})}
        controller = context["gripper_controller"]
        operation = request["grasp"]["operation"]
        started = time.perf_counter()

        if operation == "verify_only":
            state = controller.get_state(request["target"].get("object_id"))
        else:
            if operation == "grasp":
                controller.open(params["open_width"], speed=params["close_speed"], timeout=request["constraints"]["timeout"])
            close_result = controller.close(
                params["close_width"], speed=params["close_speed"],
                force=params["grasp_force"], timeout=request["constraints"]["timeout"],
                target_object_id=request["target"].get("object_id"),
            )
            if params["settle_time"] > 0 and context["runtime"].realtime:
                time.sleep(params["settle_time"])
            if close_result["contact_detected"] and request["constraints"]["hold_on_success"]:
                controller.hold(params["hold_force"])
            state = controller.get_state(request["target"].get("object_id"))

        return {
            "contact_detected": bool(state["contact_detected"]),
            "final_width": float(state["width"]),
            "final_force": float(state["force"]),
            "holding": bool(state["holding"]),
            "object_present": bool(state["object_present"]),
            "slip_detected": bool(state["slip_detected"]),
            "parameters": params,
            "evidence": {"gripper_state": state},
            "execution_time": time.perf_counter() - started,
        }
