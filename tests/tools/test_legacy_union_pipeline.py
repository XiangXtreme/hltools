"""Regression checks that legacy targets never receive player-box expansion."""

from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import numpy as np
import legacy_union_pipeline as pipeline
from brush_surface_pipeline import Tree, region_plane_union


class LegacyUnionTests(unittest.TestCase):
    def test_local_plane_union_handles_thin_slope_and_overlaps(self):
        normals = np.r_[np.eye(3), -np.eye(3)]
        distances = np.array([2., 2., 2., 2., 2., 2.])
        cube = np.c_[normals, distances]
        slope = np.array([[0., -.6, .8, .0145126]])
        volumes = [{"planes": np.r_[cube, slope], "bounds": np.array([[-2.] * 3, [2.] * 3])},
                   {"planes": np.c_[normals, [.25, .25, 3., .25, .25, 3.]],
                    "bounds": np.array([[-.25, -.25, -3.], [.25, .25, 3.]])}]
        region = SimpleNamespace(verts=np.array([[x, y, z] for x in (-3., 3.) for y in (-3., 3.) for z in (-4., 4.)]))
        tree = Tree(np.empty((0, 5)), [], -1)
        tree.root = region_plane_union(tree, region, volumes)
        tree.compact()
        rng = np.random.default_rng(82725)
        points = rng.uniform([-3., -3., -4.], [3., 3., 4.], (20000, 3))
        clearance = pipeline.margin(points, volumes)
        reliable = abs(clearance) > 1e-5
        np.testing.assert_array_equal(tree.contents(points)[reliable] == -2, clearance[reliable] < 0)
        pipeline.structure(tree.view())

    def test_compile_uses_exported_plane_without_reexpansion(self):
        # For this slope, legacy gives .6^2*16 + .8^2*36 = 28.8;
        # player-box support gives .6*16 + .8*36 = 38.4.
        plane = np.array([0., -.6, .8, 28.8])
        calls = []
        exact = SimpleNamespace(WAD="test.wad", plane_face=lambda n, d: calls.append((n.copy(), d)) or "SIDE")
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "legacy.map"
            with patch.object(pipeline, "require_command", return_value=.1) as command:
                with patch.object(pipeline.Tree, "draw", return_value="tree"):
                    tree, seconds = pipeline.compile_volumes(path, [{"planes": [plane]}], exact, "compiler")
            self.assertIn("SIDE", path.read_text())
        self.assertEqual(tree, "tree")
        self.assertEqual(len(calls), 1)
        np.testing.assert_array_equal(calls[0][0], plane[:3])
        self.assertAlmostEqual(calls[0][1], 28.8)
        for call in command.call_args_list:
            self.assertIn("-noclip", call.args[0])

    def test_selects_entity_and_matches_audit_geometry(self):
        planes = np.array([[1., 0., 0., 1.], [-1., 0., 0., 1.],
                           [0., 1., 0., 1.], [0., -1., 0., 1.],
                           [0., 0., 1., 1.], [0., 0., -1., 1.]])
        verts = [[-1., -1., -1.], [1., -1., -1.], [-1., 1., -1.], [-1., -1., 1.]]
        legacy = {"planes": planes, "bounds": np.array([[-1.] * 3, [1.] * 3])}
        sky = {"planes": planes * [1., 1., 1., 10.], "bounds": np.array([[-10.] * 3, [10.] * 3])}
        box = {"planes": planes * [1., 1., 1., 2.], "bounds": np.array([[-2.] * 3, [2.] * 3])}
        exported = {"planes": planes.tolist(), "vertices": verts}
        geometry = {"brushes": [{"entity": 3, "contents": -2, "legacy_hulls": [exported] * 3},
                                {"entity": 0, "contents": -6, "legacy_hulls": []}]}
        groups = {h: ([sky, legacy], [sky, box]) for h in (1, 2, 3)}
        selected = pipeline.selected_volumes(geometry, groups, 3)
        for hull in (1, 2, 3):
            self.assertEqual(len(selected[hull][0]), 1)
            np.testing.assert_array_equal(selected[hull][0][0]["planes"], planes)
            np.testing.assert_array_equal(selected[hull][1][0]["planes"], box["planes"])
        geometry["brushes"][0]["legacy_hulls"][0] = {"planes": (planes * 2).tolist(), "vertices": verts}
        with self.assertRaises(ValueError):
            pipeline.selected_volumes(geometry, groups, 3)

    def test_map_override_preserves_brush_geometry(self):
        source = ('{\n"classname" "worldspawn"\n}\n{\n"classname" "func_detail"\n'
                  '"zhlt_cliptype" "simple"\n{\n( 0 0 0 ) ( 1 0 0 ) ( 0 1 0 ) NULL 0 0 0 1 1\n}\n}\n')
        result = pipeline.override_cliptype(source, "legacy")
        self.assertEqual(result, source.replace('"simple"', '"legacy"'))


if __name__ == "__main__":
    unittest.main()
