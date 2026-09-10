"""Scene interaction registry shared by all demo command frontends."""

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping


class SceneRegistryError(ValueError):
    """The scene interaction registry is missing or inconsistent."""


class SceneRegistry:
    """Read-only access to scene objects, targets and action requests."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path).resolve()
        with self.path.open("r", encoding="utf-8") as stream:
            self.data: dict[str, Any] = json.load(stream)
        self._validate()

    @property
    def scene_path(self) -> Path:
        path = Path(self.data["scene"])
        if not path.is_absolute():
            path = self.path.parents[3] / path
        return path.resolve()

    @property
    def objects(self) -> Mapping[str, Any]:
        return self.data["objects"]

    @property
    def named_targets(self) -> Mapping[str, Any]:
        return self.data.get("named_targets", {})

    def object(self, object_key: str) -> dict[str, Any]:
        try:
            return deepcopy(self.objects[object_key])
        except KeyError as exc:
            raise SceneRegistryError(f"场景中没有对象 {object_key!r}") from exc

    def action_request(self, object_key: str, action: str) -> dict[str, Any]:
        try:
            return deepcopy(self.objects[object_key]["action_requests"][action])
        except KeyError as exc:
            raise SceneRegistryError(f"对象 {object_key!r} 不支持动作 {action!r}") from exc

    def _validate(self) -> None:
        if not isinstance(self.data.get("scene"), str):
            raise SceneRegistryError("注册表必须包含字符串字段 scene")
        if not isinstance(self.data.get("objects"), dict):
            raise SceneRegistryError("注册表必须包含对象映射 objects")
        for object_key, obj in self.data["objects"].items():
            if "interactable" in obj and not isinstance(obj["interactable"], bool):
                raise SceneRegistryError(
                    f"对象 {object_key!r} 的 interactable 必须是布尔值"
                )
            if not obj.get("aliases"):
                raise SceneRegistryError(f"对象 {object_key!r} 至少需要一个 alias")
            spatial = obj.get("spatial")
            if not isinstance(spatial, dict):
                raise SceneRegistryError(f"对象 {object_key!r} 缺少 spatial 配置")
            if not spatial.get("source") or not spatial.get("reference_pose"):
                raise SceneRegistryError(f"对象 {object_key!r} 缺少实时位姿源或离线参考位姿")
            if not spatial.get("anchors") or spatial.get("default_anchor") not in spatial["anchors"]:
                raise SceneRegistryError(f"对象 {object_key!r} 的默认局部锚点无效")
            if not spatial.get("tool_orientation"):
                raise SceneRegistryError(f"对象 {object_key!r} 缺少末端执行器姿态")
        for owner_key, obj in self.data["objects"].items():
            for action, specification in obj.get("affordances", {}).items():
                if not isinstance(specification, dict):
                    raise SceneRegistryError(f"对象 {owner_key!r} 的 affordance {action!r} 无效")
                acting_key = str(specification.get("acting_target", owner_key))
                contact_key = specification.get("contact_target")
                if acting_key not in self.data["objects"] or contact_key not in self.data["objects"]:
                    raise SceneRegistryError(f"对象 {owner_key!r} 的 affordance {action!r} 引用了未知对象")
                if action not in self.data["objects"][acting_key].get("action_requests", {}):
                    raise SceneRegistryError(f"作用目标 {acting_key!r} 未配置动作 {action!r}")
