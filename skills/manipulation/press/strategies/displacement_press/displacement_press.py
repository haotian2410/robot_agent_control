"""Bounded displacement-controlled Press Strategy."""

from __future__ import annotations

from typing import Any, Mapping


class DisplacementPress:
    strategy_id = "displacement_press"

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def execute(self, request: Mapping[str, Any], context: Mapping[str, Any]) -> Mapping[str, Any]:
        defaults = {"press_depth": None, "travel_distance": None, "stop_on_contact": False, "post_contact_depth": 0.003, "motion_profile": "constant_velocity"}
        configured = self.config.get("strategies", {}).get("displacement_press", {}).get("defaults", {})
        params = {**defaults, **configured, **request.get("strategy_params", {})}
        controller = context["press_controller"]
        if request["press"]["operation"] == "verify_only":
            return controller.verification_snapshot(request["target"].get("object_id"), strategy="displacement")
        constraints = request["constraints"]
        return controller.execute_displacement(
            direction=request["resolved_press_direction"], target_object_id=request["target"].get("object_id"),
            press_depth=params.get("press_depth"), travel_distance=params.get("travel_distance"),
            stop_on_contact=bool(params.get("stop_on_contact", False)), post_contact_depth=float(params.get("post_contact_depth", 0.003)),
            maximum_force=float(constraints["maximum_force"]), maximum_travel=float(constraints["maximum_travel"]),
            contact_threshold=float(constraints["contact_threshold"]), speed=float(constraints["press_speed"]),
            hold_time=float(constraints["hold_time"]), timeout=float(constraints["timeout"]),
            retract=bool(request["press"]["retract_after_press"]), retract_speed=float(constraints["retract_speed"]),
            emergency_retract=bool(constraints["emergency_retract"]),
        )
