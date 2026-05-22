from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "go2_gui_controller"))

from go2_gui_controller.semantic_place_registry import SemanticPlaceRegistry


FIXTURE_YAML = """
manifest:
  map_id: fixture_map
  source_fingerprint: fixture
  frame_id: map
places:
  room_candidate_1:
    label: room
    aliases: [회의실, 방1]
    frame_id: map
    polygon:
      - [0.0, 0.0]
      - [4.0, 0.0]
      - [4.0, 3.0]
      - [0.0, 3.0]
    enabled: true
  corridor_candidate_1:
    label: corridor
    aliases: [복도]
    frame_id: map
    polygon:
      - [4.0, 0.0]
      - [8.0, 0.0]
      - [8.0, 1.0]
      - [4.0, 1.0]
    enabled: true
"""


def _write_yaml(text: str) -> Path:
    temp_dir = tempfile.mkdtemp()
    path = Path(temp_dir) / "semantic_objects.yaml"
    path.write_text(text, encoding="utf-8")
    return path


class SemanticPlaceRegistryTests(unittest.TestCase):
    def test_lookup_and_point_membership(self):
        registry = SemanticPlaceRegistry(_write_yaml(FIXTURE_YAML))

        self.assertEqual(registry.best_match("회의실").place_id, "room_candidate_1")
        self.assertEqual(registry.best_match("방 1").place_id, "room_candidate_1")
        self.assertEqual(registry.best_match("room1").place_id, "room_candidate_1")
        self.assertEqual(registry.best_match("복도").place_id, "corridor_candidate_1")
        self.assertEqual(registry.place_for_point(2.0, 1.5).place_id, "room_candidate_1")
        self.assertEqual(registry.place_for_point(6.0, 0.5).place_id, "corridor_candidate_1")
        self.assertIsNone(registry.place_for_point(9.0, 9.0))


if __name__ == "__main__":
    unittest.main()
