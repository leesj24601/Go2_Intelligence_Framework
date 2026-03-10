from __future__ import annotations

import re
from typing import Optional

from .commands import CommandType, ParsedCommand
from .waypoint_registry import WaypointRegistry


class TextCommandParser:
    _number_unit_pattern = re.compile(
        r"(?P<value>\d+(?:\.\d+)?)\s*(?P<unit>m|meter|meters|cm|deg|degree|degrees|미터|m터|도)?",
        re.IGNORECASE,
    )

    def __init__(self, waypoint_registry: WaypointRegistry):
        self._waypoint_registry = waypoint_registry

    def parse(self, text: str) -> Optional[ParsedCommand]:
        normalized = " ".join(text.strip().lower().split())
        if not normalized:
            return None
        simplified = self._simplify_korean(normalized)

        if any(token in simplified for token in ("stop", "halt", "멈춰", "정지", "서", "스톱", "멈춰줘", "세워")):
            return ParsedCommand(CommandType.STOP, source_text=text)
        if any(token in simplified for token in ("cancel", "abort", "취소", "중지", "그만")):
            return ParsedCommand(CommandType.CANCEL_NAVIGATION, source_text=text)

        if any(token in simplified for token in ("turn", "rotate", "돌아", "회전")):
            degrees = self._extract_scalar(simplified, default=90.0)
            if "반바퀴" in simplified:
                degrees = 180.0
            if any(token in simplified for token in ("right", "오른쪽", "우측")):
                degrees *= -1.0
            return ParsedCommand(
                CommandType.ROTATE_RELATIVE,
                yaw_deg=degrees,
                source_text=text,
            )

        if any(token in simplified for token in ("forward", "backward", "left", "right", "앞", "뒤", "왼쪽", "오른쪽", "좌측", "우측")):
            distance_m = self._extract_distance_m(simplified, default=1.0)
            x_m = 0.0
            y_m = 0.0
            if any(token in simplified for token in ("forward", "앞")):
                x_m = distance_m
            elif any(token in simplified for token in ("backward", "뒤")):
                x_m = -distance_m
            elif any(token in simplified for token in ("left", "왼쪽", "좌측")):
                y_m = distance_m
            elif any(token in simplified for token in ("right", "오른쪽", "우측")):
                y_m = -distance_m
            return ParsedCommand(
                CommandType.MOVE_RELATIVE,
                x_m=x_m,
                y_m=y_m,
                source_text=text,
            )

        if any(token in simplified for token in ("go to", "navigate to", "이동", "가", "가줘")):
            waypoint_name = self._extract_waypoint_candidate(simplified)
            waypoint = self._waypoint_registry.get(waypoint_name)
            if waypoint:
                return ParsedCommand(
                    CommandType.NAVIGATE_TO_WAYPOINT,
                    waypoint_name=waypoint.name,
                    source_text=text,
                )

        waypoint = self._waypoint_registry.get(simplified)
        if waypoint:
            return ParsedCommand(
                CommandType.NAVIGATE_TO_WAYPOINT,
                waypoint_name=waypoint.name,
                source_text=text,
            )

        return None

    def _extract_distance_m(self, text: str, default: float) -> float:
        if "조금" in text or "살짝" in text:
            return 0.3
        match = self._number_unit_pattern.search(text)
        if not match:
            return default
        value = float(match.group("value"))
        unit = (match.group("unit") or "m").lower()
        if unit == "cm":
            return value / 100.0
        return value

    def _extract_scalar(self, text: str, default: float) -> float:
        if "반바퀴" in text:
            return 180.0
        match = self._number_unit_pattern.search(text)
        if not match:
            return default
        return float(match.group("value"))

    def _extract_waypoint_candidate(self, text: str) -> str:
        candidate = text
        for prefix in ("go to", "navigate to"):
            if prefix in candidate:
                candidate = candidate.split(prefix, 1)[1].strip()
        for suffix in ("으로 이동", "로 이동", "으로 가", "로 가", "가", "이동"):
            if candidate.endswith(suffix):
                candidate = candidate[: -len(suffix)].strip()
        return candidate

    def _simplify_korean(self, text: str) -> str:
        simplified = text
        replacements = {
            "앞으로": "앞",
            "뒤로": "뒤",
            "왼쪽으로": "왼쪽",
            "오른쪽으로": "오른쪽",
            "좌측으로": "좌측",
            "우측으로": "우측",
            "돌아줘": "돌아",
            "회전해": "회전",
            "이동해": "이동",
            "가줘": "가",
            "가봐": "가",
            "조금만": "조금",
            "살짝만": "살짝",
        }
        for old, new in replacements.items():
            simplified = simplified.replace(old, new)
        return simplified
