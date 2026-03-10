from __future__ import annotations

from dataclasses import dataclass
from math import radians, sin, cos
from pathlib import Path
from typing import Dict, List, Optional

import yaml
from geometry_msgs.msg import PoseStamped


@dataclass
class Waypoint:
    name: str
    frame_id: str
    x: float
    y: float
    yaw_deg: float
    aliases: List[str]

    def to_pose_stamped(self, yaw_deg_override: Optional[float] = None) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        yaw_value = self.yaw_deg if yaw_deg_override is None else yaw_deg_override
        yaw = radians(yaw_value)
        pose.pose.orientation.z = sin(yaw / 2.0)
        pose.pose.orientation.w = cos(yaw / 2.0)
        return pose


class WaypointRegistry:
    def __init__(self, yaml_path: Path):
        self._yaml_path = Path(yaml_path)
        self._waypoints: Dict[str, Waypoint] = {}
        self._aliases: Dict[str, str] = {}
        self.reload()

    def reload(self) -> None:
        raw = yaml.safe_load(self._yaml_path.read_text()) or {}
        self._waypoints.clear()
        self._aliases.clear()
        for name, entry in raw.get("waypoints", {}).items():
            waypoint = Waypoint(
                name=name,
                frame_id=str(entry.get("frame_id", "map")),
                x=float(entry.get("x", 0.0)),
                y=float(entry.get("y", 0.0)),
                yaw_deg=float(entry.get("yaw_deg", 0.0)),
                aliases=[str(alias).lower() for alias in entry.get("aliases", [])],
            )
            self._waypoints[name] = waypoint
            self._aliases[name.lower()] = name
            for alias in waypoint.aliases:
                self._aliases[alias] = name

    def get(self, name_or_alias: str) -> Optional[Waypoint]:
        key = self._aliases.get(name_or_alias.lower())
        if key is None:
            return None
        return self._waypoints.get(key)

    def list_waypoints(self) -> List[Waypoint]:
        return [self._waypoints[name] for name in sorted(self._waypoints.keys())]

    def save_waypoint(self, name: str, frame_id: str, x: float, y: float, yaw_deg: float) -> Waypoint:
        canonical_name = name.strip()
        if not canonical_name:
            raise ValueError("Waypoint name cannot be empty")

        raw = yaml.safe_load(self._yaml_path.read_text()) or {}
        waypoints = raw.setdefault("waypoints", {})
        existing = waypoints.get(canonical_name, {})
        aliases = existing.get("aliases", [canonical_name])
        if canonical_name not in aliases:
            aliases = [canonical_name, *aliases]

        waypoints[canonical_name] = {
            "frame_id": frame_id,
            "x": float(x),
            "y": float(y),
            "yaw_deg": float(yaw_deg),
            "aliases": aliases,
        }
        self._yaml_path.write_text(yaml.safe_dump(raw, sort_keys=False, allow_unicode=True))
        self.reload()
        waypoint = self.get(canonical_name)
        if waypoint is None:
            raise RuntimeError("Failed to save waypoint")
        return waypoint

    def delete_waypoint(self, name: str) -> bool:
        raw = yaml.safe_load(self._yaml_path.read_text()) or {}
        waypoints = raw.get("waypoints", {})
        if name not in waypoints:
            return False
        del waypoints[name]
        self._yaml_path.write_text(yaml.safe_dump(raw, sort_keys=False, allow_unicode=True))
        self.reload()
        return True

    def rename_waypoint(self, old_name: str, new_name: str) -> Waypoint:
        canonical_new_name = new_name.strip()
        if not canonical_new_name:
            raise ValueError("New waypoint name cannot be empty")

        raw = yaml.safe_load(self._yaml_path.read_text()) or {}
        waypoints = raw.get("waypoints", {})
        if old_name not in waypoints:
            raise ValueError(f"Unknown waypoint: {old_name}")

        entry = waypoints.pop(old_name)
        aliases = [str(alias) for alias in entry.get("aliases", [])]
        aliases = [alias for alias in aliases if alias != old_name]
        if canonical_new_name not in aliases:
            aliases.insert(0, canonical_new_name)

        waypoints[canonical_new_name] = {
            "frame_id": str(entry.get("frame_id", "map")),
            "x": float(entry.get("x", 0.0)),
            "y": float(entry.get("y", 0.0)),
            "yaw_deg": float(entry.get("yaw_deg", 0.0)),
            "aliases": aliases,
        }
        raw["waypoints"] = waypoints
        self._yaml_path.write_text(yaml.safe_dump(raw, sort_keys=False, allow_unicode=True))
        self.reload()
        waypoint = self.get(canonical_new_name)
        if waypoint is None:
            raise RuntimeError("Failed to rename waypoint")
        return waypoint
