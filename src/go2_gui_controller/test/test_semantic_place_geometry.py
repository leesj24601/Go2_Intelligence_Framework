from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "go2_gui_controller"))

from go2_gui_controller.semantic_place_geometry import (
    polygon_area,
    polygon_centroid,
    point_in_polygon,
)


class SemanticPlaceGeometryTests(unittest.TestCase):
    def test_point_membership_area_and_centroid(self):
        polygon = ((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0))

        self.assertTrue(point_in_polygon(2.0, 1.5, polygon))
        self.assertFalse(point_in_polygon(5.0, 1.5, polygon))
        self.assertAlmostEqual(polygon_area(polygon), 12.0)
        self.assertEqual(polygon_centroid(polygon), (2.0, 1.5))


if __name__ == "__main__":
    unittest.main()
