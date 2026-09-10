"""GraspPosePrediction Strategy skeleton."""

from __future__ import annotations

from typing import Any, Mapping


class GraspPosePrediction:
    """Generate target-conditioned 6-DoF grasp-pose candidates."""

    strategy_id = "grasp_pose_prediction"

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.config = config or {}

    def locate(
        self,
        request: Mapping[str, Any],
        context: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """Return backend-independent grasp-pose candidates.

        Required backend behavior:
        1. Validate grasp_pose_prediction.schema.yaml.
        2. Resolve the requested target instance or scene-level permission.
        3. Acquire synchronized RGB-D / point-cloud observations.
        4. Invoke the configured grasp-pose predictor.
        5. Convert predictor poses into the project's gripper/TCP convention.
        6. Reject candidates assigned to the wrong target instance.
        7. Filter by score, gripper width, NMS and approach clearance.
        8. Return candidates with pose, score, gripper_width,
           approach_direction, target_instance_id and evidence.
        9. Leave final frame, reachability, collision and clearance validation
           to the shared PoseValidator.

        Expected raw result shape:
        {
            "target_pose": <best candidate pose or None>,
            "confidence": <best grasp score>,
            "source": "grasp_pose_prediction",
            "candidates": [
                {
                    "pose": ...,
                    "score": ...,
                    "gripper_width": ...,
                    "approach_direction": ...,
                    "target_instance_id": ...,
                }
            ],
            "grasp_metadata": {
                "gripper_width": ...,
                "approach_direction": ...,
                "grasp_score": ...,
                "predictor": ...,
                "target_instance_id": ...,
            },
            "evidence": {...},
        }
        """
        raise NotImplementedError("Implement GraspPosePrediction.locate().")
