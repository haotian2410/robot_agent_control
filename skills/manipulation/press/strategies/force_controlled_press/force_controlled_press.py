"""Bounded force-controlled Press Strategy."""

from __future__ import annotations

from typing import Any, Mapping


class ForceControlledPress:
    strategy_id = "force_controlled_press"

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def execute(self, request: Mapping[str, Any], context: Mapping[str, Any]) -> Mapping[str, Any]:
        defaults = {"target_force": 10.0, "contact_force": 2.0, "force_ramp_rate": 20.0, "force_settle_time": 0.2, "allow_force_overshoot": False, "overshoot_limit": 1.0}
        configured = self.config.get("strategies", {}).get("force_controlled_press", {}).get("defaults", {})
        params = {**defaults, **configured, **request.get("strategy_params", {})}
        controller = context["press_controller"]
        if request["press"]["operation"] == "verify_only":
            return controller.verification_snapshot(request["target"].get("object_id"), strategy="force")
        constraints = request["constraints"]
        return controller.execute_force(
            direction=request["resolved_press_direction"], target_object_id=request["target"].get("object_id"),
            target_force=float(params["target_force"]), maximum_force=float(constraints["maximum_force"]),
            maximum_travel=float(constraints["maximum_travel"]), contact_threshold=float(constraints["contact_threshold"]),
            force_tolerance=float(constraints["force_tolerance"]), speed=float(constraints["press_speed"]),
            hold_time=float(constraints["hold_time"]), timeout=float(constraints["timeout"]),
            retract=bool(request["press"]["retract_after_press"]), retract_speed=float(constraints["retract_speed"]),
            emergency_retract=bool(constraints["emergency_retract"]),
        )
