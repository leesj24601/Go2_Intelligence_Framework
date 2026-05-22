from dataclasses import dataclass
from enum import Enum
from typing import Optional


class CommandType(str, Enum):
    STOP = "stop"
    MOVE_RELATIVE = "move_relative"
    ROTATE_RELATIVE = "rotate_relative"
    NAVIGATE_TO_WAYPOINT = "navigate_to_waypoint"
    NAVIGATE_TO_OBJECT = "navigate_to_object"
    CANCEL_NAVIGATION = "cancel_navigation"


@dataclass
class ParsedCommand:
    command_type: CommandType
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_deg: float = 0.0
    waypoint_name: Optional[str] = None
    object_label: Optional[str] = None
    object_relation: str = "near"
    place_label: Optional[str] = None
    source_text: str = ""
