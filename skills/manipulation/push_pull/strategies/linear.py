from typing import Any, Mapping


class LinearPushPull:
    def execute(self, request: Mapping[str, Any], context: Mapping[str, Any]) -> Mapping[str, Any]:
        if request["manipulation"]["operation"] == "verify_only":
            return context["push_pull_controller"].verification_snapshot(request["target"]["object_id"])
        p, c = request["strategy_params"], request["constraints"]
        return context["push_pull_controller"].execute_linear(
            operation=request["manipulation"]["operation"], object_id=request["target"]["object_id"],
            contact_normal=request["resolved_contact_normal"],
            direction=p["direction"], distance=p["distance"], frame=p.get("frame", "world"),
            carried_objects=p.get("carried_objects", []),
            maximum_force=c["maximum_force"], contact_threshold=c["contact_threshold"], speed=c["speed"],
            maintain_contact=request["manipulation"]["maintain_contact"], stop_on_contact_loss=c["stop_on_contact_loss"],
            timeout=c["timeout"], emergency_retract=c["emergency_retract"],
            normal_alignment_tolerance=c["normal_alignment_tolerance"], avoid_collision=c["avoid_collision"])
