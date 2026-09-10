"""Default open-and-detach Release Strategy."""

from __future__ import annotations

import time
from typing import Any, Mapping


class DefaultRelease:
    strategy_id = "default_release"

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def execute(self, request: Mapping[str, Any], context: Mapping[str, Any]) -> Mapping[str, Any]:
        defaults = {"open_width": 0.08, "open_speed": 0.5, "position_tolerance": 0.002, "settle_time": 0.0}
        configured = self.config.get("strategies", {}).get("default_release", {}).get("defaults", {})
        params = {**defaults, **configured, **request.get("strategy_params", {})}
        controller = context["gripper_controller"]
        target_id = request["target"]["object_id"]
        before = controller.get_state(target_id)
        started = time.perf_counter()
        controller_result = None
        if request["release"]["operation"] != "verify_only":
            controller_result = controller.open(params["open_width"], speed=params["open_speed"], timeout=request["constraints"]["timeout"])
            if params["settle_time"] > 0 and context["runtime"].realtime:
                time.sleep(params["settle_time"])
        after = controller.get_state(target_id)
        return {
            "final_width": float(after["width"]),
            "holding": bool(after["holding"]),
            "object_present": bool(after["object_present"]),
            "parameters": params,
            "evidence": {"before": before, "after": after, "controller_result": controller_result},
            "execution_time": time.perf_counter() - started,
        }
