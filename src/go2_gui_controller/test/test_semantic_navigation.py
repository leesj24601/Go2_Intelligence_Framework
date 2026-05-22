from __future__ import annotations

import sys
import tempfile
import types
import unittest
from pathlib import Path


class _PoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id="")
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


geometry_msgs = types.ModuleType("geometry_msgs")
geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
geometry_msgs_msg.PoseStamped = _PoseStamped
geometry_msgs.msg = geometry_msgs_msg
sys.modules.setdefault("geometry_msgs", geometry_msgs)
sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "go2_gui_controller"))

from go2_gui_controller.commands import CommandType
from go2_gui_controller.semantic_goal_resolver import SemanticGoalError, SemanticGoalResolver
from go2_gui_controller.semantic_object_registry import SemanticMapValidationError, SemanticObjectRegistry
from go2_gui_controller.text_command_parser import TextCommandParser


class _FakeWaypointRegistry:
    def __init__(self, names=()):
        self._names = {name.lower(): types.SimpleNamespace(name=name) for name in names}

    def get(self, name_or_alias: str):
        return self._names.get(name_or_alias.lower())


def _write_registry(text: str) -> Path:
    temp_dir = tempfile.mkdtemp()
    path = Path(temp_dir) / "semantic_objects.yaml"
    path.write_text(text)
    return path


FIXTURE_YAML = """
manifest:
  map_id: fixture_map
  source_fingerprint: fixture:semantic-object-navigation
  frame_id: map
objects:
  sofa_old:
    label: sofa
    aliases: [소파, couch]
    frame_id: map
    x: 0.0
    y: 0.0
    place_id: room_candidate_2
    yaw_deg: 0.0
    radius_m: 0.4
    confidence: 0.8
    updated_at: "2026-04-26T00:00:00Z"
  sofa_1:
    label: sofa
    aliases: [소파, couch]
    frame_id: map
    x: 2.0
    y: 3.0
    place_id: room_candidate_1
    radius_m: 0.4
    confidence: 0.9
    updated_at: "2026-04-27T00:00:00Z"
    observer_x: 1.0
    observer_y: 3.0
"""


class SemanticRegistryTests(unittest.TestCase):
    def test_lookup_alias_and_validate_manifest(self):
        registry = SemanticObjectRegistry(_write_registry(FIXTURE_YAML))

        registry.validate_manifest(
            active_map_id="fixture_map",
            active_source_fingerprint="fixture:semantic-object-navigation",
        )
        self.assertEqual(registry.best_match("소파").object_id, "sofa_1")
        self.assertEqual(registry.best_match("couch").object_id, "sofa_1")

    def test_manifest_mismatch_fails_closed(self):
        registry = SemanticObjectRegistry(_write_registry(FIXTURE_YAML))

        with self.assertRaises(SemanticMapValidationError):
            registry.validate_manifest(
                active_map_id="other_map",
                active_source_fingerprint="fixture:semantic-object-navigation",
            )

    def test_best_match_can_filter_by_place(self):
        registry = SemanticObjectRegistry(_write_registry(FIXTURE_YAML))

        self.assertEqual(registry.best_match("소파", place_id="room_candidate_1").object_id, "sofa_1")
        self.assertEqual(registry.best_match("소파", place_id="room_candidate_2").object_id, "sofa_old")
        self.assertIsNone(registry.best_match("소파", place_id="missing_room"))


class SemanticGoalResolverTests(unittest.TestCase):
    def setUp(self):
        self.obj = SemanticObjectRegistry(_write_registry(FIXTURE_YAML)).best_match("sofa")
        self.resolver = SemanticGoalResolver()

    def test_relation_fixture_coordinates(self):
        front = self.resolver.resolve(self.obj, "front")
        back = self.resolver.resolve(self.obj, "back")
        left = self.resolver.resolve(self.obj, "left")
        right = self.resolver.resolve(self.obj, "right")

        self.assertEqual((round(front.x, 2), round(front.y, 2)), (1.0, 3.0))
        self.assertEqual((round(back.x, 2), round(back.y, 2)), (3.0, 3.0))
        self.assertEqual((round(left.x, 2), round(left.y, 2)), (2.0, 2.0))
        self.assertEqual((round(right.x, 2), round(right.y, 2)), (2.0, 4.0))

    def test_invalid_approach_distance_is_blocked(self):
        resolver = SemanticGoalResolver(approach_distance_m=0.2)

        with self.assertRaises(SemanticGoalError):
            resolver.resolve(self.obj, "front")


class TextCommandParserTests(unittest.TestCase):
    def test_object_commands_parse_before_relative_motion(self):
        parser = TextCommandParser(_FakeWaypointRegistry())

        plain = parser.parse("소파로 가")
        front = parser.parse("소파 앞으로 가")
        relative = parser.parse("앞으로 1미터 가")

        self.assertEqual(plain.command_type, CommandType.NAVIGATE_TO_OBJECT)
        self.assertEqual(plain.object_label, "소파")
        self.assertEqual(plain.object_relation, "near")
        self.assertEqual(front.command_type, CommandType.NAVIGATE_TO_OBJECT)
        self.assertEqual(front.object_label, "소파")
        self.assertEqual(front.object_relation, "front")
        self.assertEqual(relative.command_type, CommandType.MOVE_RELATIVE)

    def test_waypoint_still_wins_for_matching_name(self):
        parser = TextCommandParser(_FakeWaypointRegistry(["home"]))

        command = parser.parse("home으로 가")

        self.assertEqual(command.command_type, CommandType.NAVIGATE_TO_WAYPOINT)
        self.assertEqual(command.waypoint_name, "home")


if __name__ == "__main__":
    unittest.main()
