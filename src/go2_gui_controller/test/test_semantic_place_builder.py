from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "build_semantic_places_from_occupancy.py"
sys.path.insert(0, str(ROOT / "scripts"))

from build_semantic_places_from_occupancy import (
    assign_object_places,
    build_gateway_watershed_places,
    build_hall_core_places,
    build_place_free_mask,
    build_region_masks,
)


class SemanticPlaceBuilderTests(unittest.TestCase):
    def test_gateway_watershed_splits_large_rooms_at_narrow_neck(self):
        free_mask = np.zeros((60, 100), dtype=np.uint8)
        free_mask[10:50, 5:40] = 1
        free_mask[10:50, 60:95] = 1
        free_mask[27:33, 40:60] = 1

        places = build_gateway_watershed_places(
            free_mask=free_mask,
            resolution=0.1,
            origin=(0.0, 0.0),
            min_region_area_m2=5.0,
            simplify_tolerance_m=0.1,
            corridor_aspect_ratio=2.0,
            gateway_width_m=1.0,
        )

        enabled_places = [place for place in places.values() if place.get("enabled", True)]
        self.assertGreaterEqual(len(enabled_places), 2)
        self.assertTrue(all(place["source"] == "auto_occupancy_gateway_watershed" for place in enabled_places))
        self.assertLess(max(place["area_m2"] for place in enabled_places), 25.0)

    def test_auto_mode_records_selected_segmentation_mode(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            map_image = root / "map.pgm"
            map_yaml = root / "map.yaml"
            semantic_input = root / "semantic_objects.yaml"
            output = root / "semantic_with_places.yaml"

            image = Image.new("L", (100, 60), 0)
            pixels = np.zeros((60, 100), dtype=np.uint8)
            pixels[10:50, 5:40] = 254
            pixels[10:50, 60:95] = 254
            pixels[27:33, 40:60] = 254
            image.putdata(pixels.reshape(-1).tolist())
            image.save(map_image)
            map_yaml.write_text(
                "image: map.pgm\n"
                "mode: trinary\n"
                "resolution: 0.1\n"
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
                    "5.0",
                    "--inflate-cells",
                    "0",
                    "--segmentation-mode",
                    "auto",
                    "--gateway-width-m",
                    "1.0",
                    "--corridor-aspect-ratio",
                    "2.0",
                ],
                check=True,
                cwd=ROOT,
            )

            data = yaml.safe_load(output.read_text(encoding="utf-8"))
            self.assertEqual(data["manifest"]["place_builder_segmentation_mode"], "auto")
            self.assertIn(data["manifest"]["place_builder_selected_segmentation_mode"], {"gateway_watershed", "generic"})
            self.assertGreaterEqual(len(data["places"]), 2)

    def test_hall_core_segmentation_keeps_wide_hall_as_largest_region(self):
        free_mask = np.zeros((80, 100), dtype=np.uint8)
        free_mask[25:75, 5:70] = 1
        free_mask[2:26, 45:55] = 1
        free_mask[28:38, 70:95] = 1
        free_mask[60:75, 72:95] = 1

        places = build_hall_core_places(
            free_mask=free_mask,
            resolution=0.1,
            origin=(0.0, 0.0),
            min_region_area_m2=1.0,
            simplify_tolerance_m=0.1,
            corridor_aspect_ratio=2.0,
            hall_core_distance_m=1.2,
            hall_growth_m=1.0,
        )

        labels = [place["label"] for place in places.values()]
        self.assertIn("hall", labels)
        self.assertIn("corridor", labels)
        self.assertIn("room", labels)
        hall_area = next(place["area_m2"] for place in places.values() if place["label"] == "hall")
        self.assertGreater(
            hall_area,
            max(place["area_m2"] for place in places.values() if place["label"] != "hall"),
        )

    def test_place_mask_bridges_small_unknown_gaps_without_crossing_occupied_walls(self):
        spec = {
            "mode": "trinary",
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        }
        pixels = [
            [0, 0, 0, 0, 0, 0, 0],
            [0, 254, 254, 205, 254, 254, 0],
            [0, 254, 254, 205, 254, 254, 0],
            [0, 254, 254, 205, 254, 254, 0],
            [0, 0, 0, 0, 0, 0, 0],
        ]
        image = Image.new("L", (7, 5))
        image.putdata([value for row in pixels for value in row])

        mask = build_place_free_mask(image=np.asarray(image, dtype=np.uint8), spec=spec, unknown_bridge_cells=1)

        self.assertEqual(int(mask[2, 3]), 1)

        occupied_wall_pixels = [
            [0, 0, 0, 0, 0, 0, 0],
            [0, 254, 254, 0, 254, 254, 0],
            [0, 254, 254, 0, 254, 254, 0],
            [0, 254, 254, 0, 254, 254, 0],
            [0, 0, 0, 0, 0, 0, 0],
        ]
        occupied_wall = Image.new("L", (7, 5))
        occupied_wall.putdata([value for row in occupied_wall_pixels for value in row])

        wall_mask = build_place_free_mask(
            image=np.asarray(occupied_wall, dtype=np.uint8),
            spec=spec,
            unknown_bridge_cells=1,
        )

        self.assertEqual(int(wall_mask[2, 3]), 0)

    def test_place_mask_treats_internal_furniture_obstacles_as_traversable_for_regions(self):
        spec = {
            "mode": "trinary",
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        }
        image = np.full((12, 14), 205, dtype=np.uint8)
        image[1:11, 1:13] = 254
        image[0, :] = 0
        image[-1, :] = 0
        image[:, 0] = 0
        image[:, -1] = 0
        image[5:7, 4:10] = 0

        mask = build_place_free_mask(image=image, spec=spec, unknown_bridge_cells=1)

        self.assertEqual(int(mask[6, 6]), 1)

    def test_eroded_seed_growth_keeps_connected_hall_and_rooms_coarse(self):
        free_mask = np.zeros((32, 38), dtype=np.uint8)
        free_mask[13:28, 3:18] = 1
        free_mask[2:14, 8:13] = 1
        free_mask[15:24, 21:34] = 1
        free_mask[18:21, 18:21] = 1

        regions = build_region_masks(
            free_mask=free_mask,
            resolution=1.0,
            min_region_area_m2=20.0,
            seed_inflate_cells=2,
        )

        self.assertGreaterEqual(len(regions), 2)
        self.assertLessEqual(len(regions), 3)

    def test_peak_seed_growth_respects_max_region_seed_count(self):
        free_mask = np.zeros((32, 38), dtype=np.uint8)
        free_mask[13:28, 3:18] = 1
        free_mask[2:14, 8:13] = 1
        free_mask[15:24, 21:34] = 1
        free_mask[18:21, 18:21] = 1

        regions = build_region_masks(
            free_mask=free_mask,
            resolution=1.0,
            min_region_area_m2=20.0,
            max_region_seeds=3,
            seed_spacing_m=3.0,
        )

        self.assertGreaterEqual(len(regions), 2)
        self.assertLessEqual(len(regions), 3)

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
