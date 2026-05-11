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
SCRIPT = ROOT / "scripts" / "build_semantic_map_from_yolo.py"


class SemanticMapBuilderTests(unittest.TestCase):
    def test_builds_yaml_from_yolo_depth_and_camera_pose(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            export_dir = root / "export"
            labels_dir = root / "labels"
            output = root / "semantic_objects.yaml"
            source_db = root / "office.db"
            source_db.write_bytes(b"fixture-db")
            _write_export_fixture(export_dir)
            labels_dir.mkdir()
            (labels_dir / "1.txt").write_text("56 0.5 0.5 0.5 0.5 0.9\n")

            subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--export-dir",
                    str(export_dir),
                    "--labels-dir",
                    str(labels_dir),
                    "--source-db",
                    str(source_db),
                    "--output",
                    str(output),
                    "--min-observations",
                    "1",
                ],
                check=True,
                cwd=ROOT,
            )

            data = yaml.safe_load(output.read_text())
            self.assertEqual(data["manifest"]["map_id"], "office")
            self.assertEqual(data["manifest"]["frame_id"], "map")
            self.assertEqual(data["manifest"]["detector"]["model"], "models/yolo11n.pt")
            self.assertIn("chair_1", data["objects"])

            chair = data["objects"]["chair_1"]
            self.assertEqual(chair["label"], "chair")
            self.assertEqual(chair["aliases"], ["의자"])
            self.assertEqual(chair["observation_count"], 1)
            self.assertAlmostEqual(chair["x"], 1.02, places=2)
            self.assertAlmostEqual(chair["y"], 2.02, places=2)
            self.assertAlmostEqual(chair["z"], 5.0, places=2)


def _write_export_fixture(export_dir: Path) -> None:
    rgb_dir = export_dir / "fixture_rgb"
    depth_dir = export_dir / "fixture_depth"
    calib_dir = export_dir / "fixture_calib"
    rgb_dir.mkdir(parents=True)
    depth_dir.mkdir()
    calib_dir.mkdir()

    Image.fromarray(np.zeros((4, 4, 3), dtype=np.uint8)).save(rgb_dir / "1.jpg")
    Image.fromarray(np.full((4, 4), 2000, dtype=np.uint16)).save(depth_dir / "1.png")
    (calib_dir / "1.yaml").write_text(
        """%YAML:1.0
---
camera_name: "1"
image_width: 4
image_height: 4
camera_matrix:
   rows: 3
   cols: 3
   data: [100., 0., 1., 0., 100., 1., 0., 0., 1.]
"""
    )
    (export_dir / "fixture_camera_poses.txt").write_text(
        "#timestamp x y z qx qy qz qw id\n"
        "1.0 1.0 2.0 3.0 0.0 0.0 0.0 1.0 1\n"
    )


if __name__ == "__main__":
    unittest.main()
