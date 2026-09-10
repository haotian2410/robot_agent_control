from typing import Any, Mapping


class CircularPushPull:
    def execute(self, request: Mapping[str, Any], context: Mapping[str, Any]) -> Mapping[str, Any]:
        if request["manipulation"]["operation"] == "verify_only":
            return context["push_pull_controller"].verification_snapshot(request["target"]["object_id"])
        p, c = request["strategy_params"], request["constraints"]
        return context["push_pull_controller"].execute_circular(
            operation=request["manipulation"]["operation"], object_id=request["target"]["object_id"],
            contact_normal=request["resolved_contact_normal"],
            arc_center=p["arc_center"], arc_axis=p["arc_axis"], arc_angle=p["arc_angle"], frame=p.get("frame", "world"),
            maximum_force=c["maximum_force"], contact_threshold=c["contact_threshold"], angular_speed=c["angular_speed"],
            maintain_contact=request["manipulation"]["maintain_contact"], stop_on_contact_loss=c["stop_on_contact_loss"],
            timeout=c["timeout"], emergency_retract=c["emergency_retract"],
            normal_alignment_tolerance=c["normal_alignment_tolerance"], avoid_collision=c["avoid_collision"])
