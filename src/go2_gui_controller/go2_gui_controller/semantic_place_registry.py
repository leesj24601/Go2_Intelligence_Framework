from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict

import re

import yaml

from .semantic_place_geometry import Point, point_in_polygon, polygon_area, polygon_centroid


class SemanticPlaceError(ValueError):
    pass


@dataclass(frozen=True)
class SemanticPlace:
    place_id: str
    label: str
    aliases: tuple[str, ...]
    frame_id: str
    polygon: tuple[Point, ...]
    centroid_x: float
    centroid_y: float
    area_m2: float
    confidence: float = 0.0
    source: str = ""
    enabled: bool = True


class SemanticPlaceRegistry:
    def __init__(self, yaml_path: Path | str | None):
        self._yaml_path = Path(yaml_path) if yaml_path else None
        self._places: Dict[str, SemanticPlace] = {}
        self._aliases: Dict[str, list[str]] = {}
        self.reload()

    def reload(self) -> None:
        self._places.clear()
        self._aliases.clear()

        if self._yaml_path is None or not self._yaml_path.exists():
            return

        raw = yaml.safe_load(self._yaml_path.read_text(encoding="utf-8")) or {}
        for place_id, entry in (raw.get("places") or {}).items():
            place = self._parse_place(str(place_id), entry or {})
            self._places[place.place_id] = place
            if place.enabled:
                self._index_alias(place.place_id, place.place_id)
                self._index_alias(place.label, place.place_id)
                for alias in place.aliases:
                    self._index_alias(alias, place.place_id)

    def best_match(self, label_or_alias: str) -> SemanticPlace | None:
        for key in _expanded_place_keys(label_or_alias):
            place = self._places.get(key)
            if place is not None and place.enabled:
                return place
            place_ids = self._aliases.get(key, [])
            for place_id in place_ids:
                place = self._places.get(place_id)
                if place is not None and place.enabled:
                    return place
        return None

    def place_for_point(self, x: float, y: float) -> SemanticPlace | None:
        matches = [
            place
            for place in self._places.values()
            if place.enabled and point_in_polygon(x, y, place.polygon)
        ]
        if not matches:
            return None
        return sorted(matches, key=lambda place: (place.area_m2, place.place_id))[0]

    def list_places(self) -> list[SemanticPlace]:
        return [self._places[place_id] for place_id in sorted(self._places.keys())]

    def _parse_place(self, place_id: str, raw: dict) -> SemanticPlace:
        label = str(raw.get("label", "")).strip()
        if not place_id.strip() or not label:
            raise SemanticPlaceError("semantic_place_missing_id_or_label")

        polygon = tuple((float(point[0]), float(point[1])) for point in raw.get("polygon", []))
        if len(polygon) < 3:
            raise SemanticPlaceError(f"semantic_place_invalid_polygon:{place_id}")

        centroid = polygon_centroid(polygon)
        area = polygon_area(polygon)
        return SemanticPlace(
            place_id=place_id,
            label=label,
            aliases=tuple(str(alias).lower() for alias in raw.get("aliases", [])),
            frame_id=str(raw.get("frame_id", "map")),
            polygon=polygon,
            centroid_x=float(raw.get("centroid_x", centroid[0])),
            centroid_y=float(raw.get("centroid_y", centroid[1])),
            area_m2=float(raw.get("area_m2", area)),
            confidence=float(raw.get("confidence", 0.0)),
            source=str(raw.get("source", "")),
            enabled=bool(raw.get("enabled", True)),
        )

    def _index_alias(self, alias: str, place_id: str) -> None:
        for key in _expanded_place_keys(alias):
            place_ids = self._aliases.setdefault(key, [])
            if place_id not in place_ids:
                place_ids.append(place_id)


def _normalize_place_key(value: str) -> str:
    key = str(value or "").lower().strip()
    key = re.sub(r"[\s-]+", "_", key)
    key = re.sub(r"_+", "_", key)
    return key.strip("_")


def _expanded_place_keys(value: str) -> tuple[str, ...]:
    base = _normalize_place_key(value)
    if not base:
        return ()

    keys = {base, base.replace("_", "")}
    candidate_match = re.fullmatch(r"(room|corridor)_?candidate_?(\d+)", base)
    if candidate_match:
        kind, number = candidate_match.groups()
        keys.add(f"{kind}_{number}")
        keys.add(f"{kind}{number}")

    numbered_match = re.fullmatch(r"(room|corridor)_?(\d+)", base)
    if numbered_match:
        kind, number = numbered_match.groups()
        keys.add(f"{kind}_candidate_{number}")
        keys.add(f"{kind}candidate{number}")

    korean_room_match = re.fullmatch(r"(방|룸)_?(\d+)", base)
    if korean_room_match:
        number = korean_room_match.group(2)
        keys.add(f"room_candidate_{number}")
        keys.add(f"room_{number}")
        keys.add(f"room{number}")

    korean_corridor_match = re.fullmatch(r"복도_?(\d+)", base)
    if korean_corridor_match:
        number = korean_corridor_match.group(1)
        keys.add(f"corridor_candidate_{number}")
        keys.add(f"corridor_{number}")
        keys.add(f"corridor{number}")

    if base in ("mainhall", "main_hall", "메인홀", "메인_홀", "중앙홀", "중앙_홀", "홀"):
        keys.add("main_hall")
        keys.add("mainhall")
        keys.add("hall")

    return tuple(keys)
