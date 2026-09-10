---
name: release
description: Open a compatible robot gripper at the current pose, detach or verify a held object, or recover a closed empty gripper, and return structured release evidence. Use after placement motion or failed-grasp recovery; do not use it to move the arm or validate a placement region.
metadata:
  display_name: Release
  entrypoint: skill.py
  input_schema: schema.yaml
---

# Release Skill

Release an object at the robot's current end-effector pose.

## Responsibilities

- Validate the target, opening width, timeout, and controller readiness.
- Command the gripper to open through `control/gripper_controller.py`.
- Detach the simulated payload only after the fingers have opened.
- Verify that the object is no longer held by the gripper.
- Select a compatible Release Strategy and return structured selection, result, evidence, and typed failure data.
- Retry only the same-pose open action when explicitly configured and safe.

## Boundaries

This skill does not move the arm, select a placement pose, plan a path, or verify a task-level placement region. A Place Skill should compose Move and Release and perform region-level placement verification.

The current MuJoCo implementation supports a Robotiq 2F-85-compatible open/close controller. Real hardware adapters should preserve the same request and result semantics.

## Execution contract

Use the same lifecycle as other manipulation skills: normalize the request, acquire synchronized runtime context, select a strategy, validate preconditions, execute, verify evidence, and return a structured result. `verify_only` may inspect a target that is already released; the normal `release` operation requires the target to be held. `open` is a recovery operation that only opens the gripper and does not require a held target.
