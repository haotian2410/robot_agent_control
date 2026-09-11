from __future__ import annotations

import json
from pathlib import Path

import pytest
from pydantic import ValidationError

from robot_agent_control import CommandDocument, ControlExecutor, load_command_document
from robot_agent_control.contracts import scene_sha256
from robot_agent_control.executor import ExecutionPreflightError


ROOT = Path(__file__).resolve().parents[1]
COMMANDS = ROOT / "demo" / "skill_command" / "config_001.commands.json"


def test_legacy_document_is_enriched_to_versioned_absolute_contract():
    document = load_command_document(COMMANDS)
    assert document.schema_version == "1.0"
    assert document.robot == "ur5e"
    assert Path(document.scene).is_absolute()
    assert Path(document.registry).is_absolute()
    assert document.scene_fingerprint == scene_sha256(document.scene)
    assert document.commands[0].command_id == "command-001"
    assert len(document.commands) == 18


def test_contract_rejects_unknown_fields():
    value = load_command_document(COMMANDS).model_dump()
    value["unexpected"] = True
    with pytest.raises(ValidationError, match="unexpected"):
        CommandDocument.model_validate(value)


def test_scene_fingerprint_is_checked_before_model_load():
    document = load_command_document(COMMANDS).model_copy(
        update={"scene_fingerprint": "0" * 64}
    )
    with pytest.raises(ExecutionPreflightError, match="fingerprint mismatch") as captured:
        ControlExecutor().execute(document, viewer_mode="headless")
    assert captured.value.code == "SCENE_FINGERPRINT_MISMATCH"


def test_headless_executor_writes_report_and_trace(tmp_path):
    original = load_command_document(COMMANDS)
    home = original.commands[-1].model_copy(
        update={"command_id": "home-command", "source_skill_step_id": "step-1"}
    )
    document = original.model_copy(update={"commands": [home]})
    report = ControlExecutor().execute(document, viewer_mode="headless", output_dir=tmp_path)
    assert report.success
    assert report.commands_total == report.commands_completed == 1
    assert len(report.steps) == 1
    assert report.steps[0].source_skill_step_id == "step-1"
    saved = json.loads((tmp_path / "execution_report.json").read_text(encoding="utf-8"))
    assert saved["success"] is True
    trace = (tmp_path / "skill_trace.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(trace) == 1
    event = json.loads(trace[0])
    assert event["runtime_step_id"] == "runtime-step-001"
    assert event["started"] is event["completed"] is event["success"] is True
    assert event["started_at"] <= event["finished_at"]


def test_registry_source_names_are_checked_before_execution(tmp_path):
    document = load_command_document(COMMANDS)
    registry = json.loads(Path(document.registry).read_text(encoding="utf-8"))
    registry["objects"]["red_ball"]["spatial"]["source"] = {
        "type": "body",
        "name": "body_that_does_not_exist",
    }
    registry_path = tmp_path / "interactions.json"
    registry_path.write_text(json.dumps(registry), encoding="utf-8")
    document = document.model_copy(update={"registry": str(registry_path)})
    with pytest.raises(ExecutionPreflightError, match="body_that_does_not_exist") as captured:
        ControlExecutor().execute(document, viewer_mode="headless")
    assert captured.value.code == "REGISTRY_INVALID"
