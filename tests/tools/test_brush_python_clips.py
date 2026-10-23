"""Check the triangle adapter at boundaries that affect reference collision."""

import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import brush_python_clips as pipeline


class ReferenceInputTests(unittest.TestCase):
    def test_terrain_selection_keeps_only_upward_exterior(self):
        triangles = np.array([[[0., 0., 0.], [16., 0., 4.], [0., 16., 8.]],
                              [[0., 0., -8.], [0., 16., -8.], [16., 0., -8.]],
                              [[0., 0., 0.], [0., 0., -8.], [16., 0., 4.]]])
        normals = np.array([[0., 0., 1.], [0., 0., -1.], [0., -1., 0.]])
        terrain, report = pipeline.select_triangles(triangles, normals, "terrain")
        shell, _ = pipeline.select_triangles(triangles, normals, "shell")
        np.testing.assert_array_equal(terrain, triangles[:1])
        np.testing.assert_array_equal(shell, triangles)
        self.assertEqual(report["selected_before_snap"], 1)

    def test_reference_snap_collapses_sliver_without_changing_winding(self):
        triangles = np.array([[[.02, .02, .02], [8.02, .02, 2.02], [.02, 8.02, 4.02]],
                              [[0., 0., 0.], [.01, 0., 0.], [0., .01, 0.]]])
        result, report = pipeline.select_triangles(triangles, np.tile([0., 0., 1.], (2, 1)), "terrain")
        np.testing.assert_array_equal(result, [[[0., 0., 0.], [8., 0., 2.], [0., 8., 4.]]])
        self.assertEqual(report["snap_degenerate_triangles"], 1)
        self.assertGreater(np.cross(result[0, 1] - result[0, 0], result[0, 2] - result[0, 0])[2], 0)

    def test_empty_surface_rejected(self):
        with self.assertRaisesRegex(ValueError, "No triangles"):
            pipeline.select_triangles(np.zeros((1, 3, 3)), np.array([[0., 0., 1.]]), "terrain")

    def test_mismatched_source_rejected_before_output_creation(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "input.map").write_text("changed input")
            (root / "manifest.json").write_text(json.dumps({"source_sha256": "incorrect"}))
            args = SimpleNamespace(source=root / "input.map", geometry=root / "geometry.json",
                                   manifest=root / "manifest.json", output=root / "out",
                                   scripts=root, compiler=root / "compiler", wad=root / "wad")
            with self.assertRaisesRegex(ValueError, "hashes differ"):
                pipeline.prepare(args)
            self.assertFalse((root / "out").exists())


if __name__ == "__main__":
    unittest.main()
