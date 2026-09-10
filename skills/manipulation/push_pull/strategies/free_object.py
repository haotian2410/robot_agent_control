"""Straight-line pushing of a non-grasped free body."""

from typing import Any, Mapping


class FreeObjectPush:
    def execute(self, request: Mapping[str, Any], context: Mapping[str, Any]) -> Mapping[str, Any]:
        controller = context["push_pull_controller"]
        if request["manipulation"]["operation"] == "verify_only":
            return controller.verification_snapshot(
                request["target"]["object_id"], interaction_mode="free_object"
            )
        params, constraints = request["strategy_params"], request["constraints"]
        return controller.execute_free_object_push(
            object_id=request["target"]["object_id"],
            contact_normal=request["resolved_contact_normal"],
            direction=params["direction"],
            distance=params["distance"],
            frame=params.get("frame", "world"),
            maximum_force=constraints["maximum_force"],
            contact_threshold=constraints["contact_threshold"],
            speed=constraints["speed"],
            maintain_contact=request["manipulation"]["maintain_contact"],
            stop_on_contact_loss=constraints["stop_on_contact_loss"],
            timeout=constraints["timeout"],
            emergency_retract=constraints["emergency_retract"],
            normal_alignment_tolerance=constraints["normal_alignment_tolerance"],
            avoid_collision=constraints["avoid_collision"],
        )
