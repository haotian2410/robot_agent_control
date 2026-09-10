"""Versioned command and execution-report contracts."""

from __future__ import annotations

import hashlib
import json
from datetime import datetime
from enum import StrEnum
from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class ViewerMode(StrEnum):
    AUTO = "auto"
    STEP = "step"
    HEADLESS = "headless"


class ExecutionOptions(BaseModel):
    model_config = ConfigDict(extra="forbid")

    end_effector_site: str = "robotiq_2f85_pinch"
    execution_mode: Literal["kinematic", "actuator"] = "kinematic"
    playback_fps: float = Field(default=60.0, gt=0)
    playback_speed: float = Field(default=2.0, gt=0)
    minimum_playback_duration: float = Field(default=1.0, ge=0)


class SkillCommand(BaseModel):
    model_config = ConfigDict(extra="forbid")

    command_id: str = Field(pattern=r"^[\w-]+$")
    source_skill_step_id: str | None = Field(default=None, pattern=r"^[\w-]+$")
    skill_name: Literal["locate", "move", "grasp", "release", "press", "pull", "push"]
    parameters: dict[str, Any]

    @model_validator(mode="after")
    def target_is_present(self):
        target = self.parameters.get("target")
        if not isinstance(target, str) or not target:
            raise ValueError("parameters.target must be a non-empty string")
        return self


class CommandDocument(BaseModel):
    model_config = ConfigDict(extra="forbid")

    schema_version: Literal["1.0"] = "1.0"
    robot: Literal["ur5e"] = "ur5e"
    scene: str
    registry: str
    scene_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    runtime: ExecutionOptions = Field(default_factory=ExecutionOptions)
    request_defaults: dict[str, Any] = Field(default_factory=dict)
    commands: list[SkillCommand] = Field(min_length=1)

    @field_validator("scene", "registry")
    @classmethod
    def path_is_absolute(cls, value: str) -> str:
        if not Path(value).is_absolute():
            raise ValueError("execution bundle paths must be absolute")
        return value


class RuntimeStepReport(BaseModel):
    model_config = ConfigDict(extra="forbid")

    runtime_step_id: str
    command_id: str
    source_skill_step_id: str | None = None
    skill: str
    target: str
    success: bool
    started_at: datetime
    finished_at: datetime
    duration_seconds: float = Field(ge=0)
    result: dict[str, Any] = Field(default_factory=dict)


class ExecutionFailure(BaseModel):
    model_config = ConfigDict(extra="forbid")

    command_id: str | None = None
    skill_plan_step_id: str | None = None
    runtime_step_id: str | None = None
    error_code: str
    error_message: str
    recoverable: bool = False


class ExecutionReport(BaseModel):
    model_config = ConfigDict(extra="forbid")

    schema_version: Literal["1.0"] = "1.0"
    success: bool
    robot: str
    scene: str
    started_at: datetime
    finished_at: datetime
    commands_total: int = Field(ge=0)
    commands_started: int = Field(ge=0)
    commands_completed: int = Field(ge=0)
    steps: list[RuntimeStepReport] = Field(default_factory=list)
    failure: ExecutionFailure | None = None


def scene_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_command_document(path: str | Path) -> CommandDocument:
    """Load the versioned contract or enrich the legacy demo document."""
    command_path = Path(path).expanduser().resolve()
    raw = json.loads(command_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("command document must be a JSON object")

    registry_value = raw.get("registry")
    if not isinstance(registry_value, str):
        raise ValueError("command document must contain a registry path")
    registry_path = Path(registry_value)
    if not registry_path.is_absolute():
        registry_path = (command_path.parent / registry_path).resolve()
    registry_raw = json.loads(registry_path.read_text(encoding="utf-8"))

    scene_value = raw.get("scene", registry_raw.get("scene"))
    if not isinstance(scene_value, str):
        raise ValueError("command document or registry must contain a scene path")
    scene_path = Path(scene_value)
    if not scene_path.is_absolute():
        # A command-owned scene is relative to the command document; a legacy
        # registry-owned scene is relative to the registry itself.
        base = command_path.parent if "scene" in raw else registry_path.parent
        scene_path = (base / scene_path).resolve()

    commands = []
    for index, item in enumerate(raw.get("commands", []), 1):
        if not isinstance(item, dict):
            raise ValueError(f"command {index} must be an object")
        enriched = dict(item)
        enriched.setdefault("command_id", f"command-{index:03d}")
        enriched.setdefault("source_skill_step_id", None)
        commands.append(enriched)

    runtime_raw = dict(raw.get("runtime", {}))
    allowed_runtime = set(ExecutionOptions.model_fields)
    runtime_raw = {key: value for key, value in runtime_raw.items() if key in allowed_runtime}
    value = {
        "schema_version": raw.get("schema_version", "1.0"),
        "robot": raw.get("robot", "ur5e"),
        "scene": str(scene_path),
        "registry": str(registry_path),
        "scene_fingerprint": raw.get("scene_fingerprint", registry_raw.get("scene_fingerprint", scene_sha256(scene_path))),
        "runtime": runtime_raw,
        "request_defaults": raw.get("request_defaults", registry_raw.get("move_defaults", {})),
        "commands": commands,
    }
    return CommandDocument.model_validate(value)

