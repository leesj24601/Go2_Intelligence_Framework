from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional

import yaml


class SemanticMapError(ValueError):
    pass


class SemanticMapValidationError(SemanticMapError):
    pass


@dataclass(frozen=True)
class SemanticMapManifest:
    map_id: str
    source_fingerprint: str
    frame_id: str
    source_rtabmap_db: str = ""
    source_rosbag: str = ""
    generated_at: str = ""
    builder_version: str = ""
    detector_name: str = ""
    detector_model: str = ""


@dataclass(frozen=True)
class SemanticObject:
    object_id: str
    label: str
    aliases: tuple[str, ...]
    frame_id: str
    x: float
    y: float
    z: float = 0.0
    yaw_deg: Optional[float] = None
    place_id: Optional[str] = None
    radius_m: Optional[float] = None
    confidence: float = 0.0
    observation_count: int = 0
    updated_at: str = ""
    source: str = ""
    approach_yaw_deg: Optional[float] = None
    observer_x: Optional[float] = None
    observer_y: Optional[float] = None


class SemanticObjectRegistry:
    def __init__(self, yaml_path: Path | str | None):
        self._yaml_path = Path(yaml_path) if yaml_path else None
        self.manifest: SemanticMapManifest | None = None
        self._objects: Dict[str, SemanticObject] = {}
        self._aliases: Dict[str, list[str]] = {}
        self.reload()

    def reload(self) -> None:
        self.manifest = None
        self._objects.clear()
        self._aliases.clear()

        if self._yaml_path is None or not self._yaml_path.exists():
            return

        raw = yaml.safe_load(self._yaml_path.read_text()) or {}
        manifest_raw = raw.get("manifest") or {}
        self.manifest = self._parse_manifest(manifest_raw)

        for object_id, entry in (raw.get("objects") or {}).items():
            semantic_object = self._parse_object(str(object_id), entry or {})
            self._objects[semantic_object.object_id] = semantic_object
            self._index_alias(semantic_object.label, semantic_object.object_id)
            for alias in semantic_object.aliases:
                self._index_alias(alias, semantic_object.object_id)

    def validate_manifest(
        self,
        *,
        active_map_id: str,
        active_source_fingerprint: str,
        policy: str = "strict",
        override: bool = False,
    ) -> None:
        if override:
            return
        if policy not in ("strict", "warn"):
            raise SemanticMapValidationError(f"unknown semantic manifest policy: {policy}")
        if self.manifest is None:
            raise SemanticMapValidationError("semantic_manifest_missing")

        mismatches: list[str] = []
        if not active_map_id or self.manifest.map_id != active_map_id:
            mismatches.append("map_id")
        if not active_source_fingerprint or self.manifest.source_fingerprint != active_source_fingerprint:
            mismatches.append("source_fingerprint")
        if self.manifest.frame_id != "map":
            mismatches.append("frame_id")

        if mismatches and policy == "strict":
            raise SemanticMapValidationError(f"semantic_manifest_mismatch:{','.join(mismatches)}")

    def find_all(self, label_or_alias: str) -> list[SemanticObject]:
        object_ids = self._aliases.get(label_or_alias.lower(), [])
        return [self._objects[object_id] for object_id in object_ids if object_id in self._objects]

    def best_match(
        self,
        label_or_alias: str,
        *,
        current_pose: tuple[float, float] | None = None,
        place_id: str | None = None,
    ) -> SemanticObject | None:
        candidates = self.find_all(label_or_alias)
        if place_id:
            candidates = [obj for obj in candidates if obj.place_id == place_id]
        if not candidates:
            return None

        def sort_key(obj: SemanticObject) -> tuple[float, float, str]:
            distance = 0.0
            if current_pose is not None:
                dx = obj.x - current_pose[0]
                dy = obj.y - current_pose[1]
                distance = (dx * dx + dy * dy) ** 0.5
            return (-obj.confidence, distance, _reverse_string(obj.updated_at))

        return sorted(candidates, key=sort_key)[0]

    def list_objects(self) -> list[SemanticObject]:
        return [self._objects[object_id] for object_id in sorted(self._objects.keys())]

    def _parse_manifest(self, raw: dict) -> SemanticMapManifest:
        required = ("map_id", "source_fingerprint", "frame_id")
        missing = [field for field in required if not str(raw.get(field, "")).strip()]
        if missing:
            raise SemanticMapValidationError(f"semantic_manifest_missing_fields:{','.join(missing)}")

        detector = raw.get("detector") or {}
        return SemanticMapManifest(
            map_id=str(raw.get("map_id")),
            source_fingerprint=str(raw.get("source_fingerprint")),
            frame_id=str(raw.get("frame_id")),
            source_rtabmap_db=str(raw.get("source_rtabmap_db", "")),
            source_rosbag=str(raw.get("source_rosbag", "")),
            generated_at=str(raw.get("generated_at", "")),
            builder_version=str(raw.get("builder_version", "")),
            detector_name=str(detector.get("name", "")),
            detector_model=str(detector.get("model", "")),
        )

    def _parse_object(self, object_id: str, raw: dict) -> SemanticObject:
        label = str(raw.get("label", "")).strip()
        if not object_id.strip() or not label:
            raise SemanticMapError("semantic_object_missing_id_or_label")

        return SemanticObject(
            object_id=object_id,
            label=label,
            aliases=tuple(str(alias).lower() for alias in raw.get("aliases", [])),
            frame_id=str(raw.get("frame_id", "map")),
            x=float(raw.get("x", 0.0)),
            y=float(raw.get("y", 0.0)),
            z=float(raw.get("z", 0.0)),
            yaw_deg=_optional_float(raw.get("yaw_deg")),
            place_id=str(raw.get("place_id", "")).strip() or None,
            radius_m=_optional_float(raw.get("radius_m")),
            confidence=float(raw.get("confidence", 0.0)),
            observation_count=int(raw.get("observation_count", 0)),
            updated_at=str(raw.get("updated_at", "")),
            source=str(raw.get("source", "")),
            approach_yaw_deg=_optional_float(raw.get("approach_yaw_deg")),
            observer_x=_optional_float(raw.get("observer_x")),
            observer_y=_optional_float(raw.get("observer_y")),
        )

    def _index_alias(self, alias: str, object_id: str) -> None:
        key = alias.lower().strip()
        if not key:
            return
        self._aliases.setdefault(key, []).append(object_id)


def _optional_float(value) -> Optional[float]:
    if value is None or value == "":
        return None
    return float(value)


def _reverse_string(value: str) -> str:
    return "".join(chr(0x10FFFF - ord(char)) for char in value)
