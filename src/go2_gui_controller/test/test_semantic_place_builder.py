from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml
from PIL import Image


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "build_semantic_places_from_occupancy.py"
sys.path.insert(0, str(ROOT / "scripts"))

from build_semantic_places_from_occupancy import assign_object_places


class SemanticPlaceBuilderTests(unittest.TestCase):
    def test_builds_places_and_assigns_objects(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            map_image = root / "map.pgm"
            map_yaml = root / "map.yaml"
            semantic_input = root / "semantic_objects.yaml"
            output = root / "semantic_with_places.yaml"

            pixels = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 254, 254, 254, 0, 254, 254, 0],
                [0, 254, 254, 254, 0, 254, 254, 0],
                [0, 254, 254, 254, 254, 254, 254, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
            ]
            image = Image.new("L", (8, 5))
            image.putdata([value for row in pixels for value in row])
            image.save(map_image)
            map_yaml.write_text(
                "image: map.pgm\n"
                "resolution: 1.0\n"
                "origin: [0.0, 0.0, 0.0]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.196\n",
                encoding="utf-8",
            )
            semantic_input.write_text(
                """
manifest:
  map_id: fixture_map
  source_fingerprint: fixture
  frame_id: map
objects:
  chair_1:
    label: chair
    aliases: [의자]
    frame_id: map
    x: 2.0
    y: 2.0
""",
                encoding="utf-8",
            )

            subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--map-yaml",
                    str(map_yaml),
                    "--semantic-input",
                    str(semantic_input),
                    "--output",
                    str(output),
                    "--min-region-area-m2",
                    "2.0",
                    "--inflate-cells",
                    "0",
                ],
                check=True,
                cwd=ROOT,
            )

            data = yaml.safe_load(output.read_text(encoding="utf-8"))
            self.assertIn("places", data)
            self.assertGreaterEqual(len(data["places"]), 1)
            self.assertEqual(data["objects"]["chair_1"]["place_id"], next(iter(data["places"].keys())))
            self.assertEqual(next(iter(data["places"].values()))["source"], "auto_occupancy_watershed")

    def test_trinary_unknown_pixels_are_not_treated_as_free_space(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            map_image = root / "map.pgm"
            map_yaml = root / "map.yaml"
            semantic_input = root / "semantic_objects.yaml"
            output = root / "semantic_with_places.yaml"

            pixels = [
                [0, 0, 0, 0, 0],
                [0, 205, 205, 205, 0],
                [0, 205, 205, 205, 0],
                [0, 205, 205, 205, 0],
                [0, 0, 0, 0, 0],
            ]
            image = Image.new("L", (5, 5))
            image.putdata([value for row in pixels for value in row])
            image.save(map_image)
            map_yaml.write_text(
                "image: map.pgm\n"
                "mode: trinary\n"
                "resolution: 1.0\n"
                "origin: [0.0, 0.0, 0.0]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.25\n",
                encoding="utf-8",
            )
            semantic_input.write_text(
                """
manifest:
  map_id: fixture_map
  source_fingerprint: fixture
  frame_id: map
objects:
  chair_1:
    label: chair
    aliases: [의자]
    frame_id: map
    x: 2.0
    y: 2.0
""",
                encoding="utf-8",
            )

            subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--map-yaml",
                    str(map_yaml),
                    "--semantic-input",
                    str(semantic_input),
                    "--output",
                    str(output),
                    "--min-region-area-m2",
                    "2.0",
                    "--inflate-cells",
                    "0",
                ],
                check=True,
                cwd=ROOT,
            )

            data = yaml.safe_load(output.read_text(encoding="utf-8"))
            self.assertEqual(data["places"], {})
            self.assertNotIn("place_id", data["objects"]["chair_1"])

    def test_splits_connected_rooms_through_narrow_passage(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            map_image = root / "map.pgm"
            map_yaml = root / "map.yaml"
            semantic_input = root / "semantic_objects.yaml"
            output = root / "semantic_with_places.yaml"

            pixels = [
                [0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 254, 254, 254, 0, 254, 254, 254, 0],
                [0, 254, 254, 254, 0, 254, 254, 254, 0],
                [0, 254, 254, 254, 254, 254, 254, 254, 0],
                [0, 254, 254, 254, 0, 254, 254, 254, 0],
                [0, 254, 254, 254, 0, 254, 254, 254, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0],
            ]
            image = Image.new("L", (9, 7))
            image.putdata([value for row in pixels for value in row])
            image.save(map_image)
            map_yaml.write_text(
                "image: map.pgm\n"
                "resolution: 1.0\n"
                "origin: [0.0, 0.0, 0.0]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.196\n",
                encoding="utf-8",
            )
            semantic_input.write_text(
                """
manifest:
  map_id: fixture_map
  source_fingerprint: fixture
  frame_id: map
objects: {}
""",
                encoding="utf-8",
            )

            subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--map-yaml",
                    str(map_yaml),
                    "--semantic-input",
                    str(semantic_input),
                    "--output",
                    str(output),
                    "--min-region-area-m2",
                    "2.0",
                    "--inflate-cells",
                    "0",
                ],
                check=True,
                cwd=ROOT,
            )

            data = yaml.safe_load(output.read_text(encoding="utf-8"))
            enabled_places = [place for place in data["places"].values() if place.get("enabled", True)]
            self.assertGreaterEqual(len(enabled_places), 2)
            self.assertTrue(
                all(place["source"] == "auto_occupancy_watershed" for place in enabled_places)
            )

    def test_assigns_place_from_observer_pose_when_object_center_is_outside_free_space(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            map_image = root / "map.pgm"
            map_yaml = root / "map.yaml"
            semantic_input = root / "semantic_objects.yaml"
            output = root / "semantic_with_places.yaml"

            pixels = [
                [0, 0, 0, 0, 0],
                [0, 254, 254, 254, 0],
                [0, 254, 254, 254, 0],
                [0, 254, 254, 254, 0],
                [0, 0, 0, 0, 0],
            ]
            image = Image.new("L", (5, 5))
            image.putdata([value for row in pixels for value in row])
            image.save(map_image)
            map_yaml.write_text(
                "image: map.pgm\n"
                "resolution: 1.0\n"
                "origin: [0.0, 0.0, 0.0]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.196\n",
                encoding="utf-8",
            )
            semantic_input.write_text(
                """
manifest:
  map_id: fixture_map
  source_fingerprint: fixture
  frame_id: map
objects:
  wall_tv_1:
    label: tv
    aliases: [티비]
    frame_id: map
    x: 0.5
    y: 2.0
    observer_x: 2.0
    observer_y: 2.0
""",
                encoding="utf-8",
            )

            subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--map-yaml",
                    str(map_yaml),
                    "--semantic-input",
                    str(semantic_input),
                    "--output",
                    str(output),
                    "--min-region-area-m2",
                    "2.0",
                    "--inflate-cells",
                    "0",
                ],
                check=True,
                cwd=ROOT,
            )

            data = yaml.safe_load(output.read_text(encoding="utf-8"))
            self.assertEqual(data["objects"]["wall_tv_1"]["place_id"], next(iter(data["places"].keys())))

    def test_assigns_nearest_place_when_object_is_near_eroded_polygon(self):
        document = {
            "places": {
                "room_candidate_1": {
                    "label": "room",
                    "enabled": True,
                    "area_m2": 4.0,
                    "polygon": [
                        [0.0, 0.0],
                        [2.0, 0.0],
                        [2.0, 2.0],
                        [0.0, 2.0],
                    ],
                }
            },
            "objects": {
                "chair_1": {
                    "label": "chair",
                    "x": 2.4,
                    "y": 1.0,
                    "observer_x": 2.5,
                    "observer_y": 1.0,
                }
            },
        }

        assign_object_places(document)

        self.assertEqual(document["objects"]["chair_1"]["place_id"], "room_candidate_1")


if __name__ == "__main__":
    unittest.main()
