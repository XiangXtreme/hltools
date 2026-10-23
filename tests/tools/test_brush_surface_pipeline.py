"""Geometry and graph checks for the isolated brush/surface experiment."""

import itertools
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import numpy as np
import brush_surface_pipeline as pipeline
from source_clip_pipeline import HullFile


class BrushSurfaceTests(unittest.TestCase):
    def test_bounded_tree_is_empty_outside_repair_domain(self):
        tree = pipeline.Tree(np.empty((0, 5)), [], -2)
        tree.bound(np.array([-1., -2., -3.]), np.array([1., 2., 3.]))
        tree.compact()
        points = np.array([[0., 0., 0.], [2., 0., 0.], [-2., 0., 0.],
                           [0., 3., 0.], [0., -3., 0.], [0., 0., 4.], [0., 0., -4.]])
        np.testing.assert_array_equal(tree.contents(points), [-2, -1, -1, -1, -1, -1, -1])
        pipeline.structure(tree.view())

    def test_top_contact_matches_but_underneath_can_differ(self):
        triangle = np.array([[0., 0., 0.], [64., 0., 16.], [0., 64., 32.]])
        brush = np.r_[triangle, triangle - [0., 0., 48.]]
        corners = np.array(list(itertools.product((-1., 1.), repeat=3)))
        half = np.array([4., 4., 8.])
        closed = pipeline.expanded([brush], half, corners)
        surface = pipeline.expanded([triangle], half, corners)
        origins = np.array([[10., 10., 100.], [20., 20., 100.], [-1., 10., 100.]])
        directions = np.tile([0., 0., -1.], (len(origins), 1))
        a = pipeline.convex_hit(origins, directions, 200., closed)
        b = pipeline.convex_hit(origins, directions, 200., surface)
        np.testing.assert_allclose(a, b, atol=1e-9)
        point = np.array([[10., 10., -20.]])
        self.assertLess(pipeline.margin(point, closed)[0], 0)
        self.assertGreater(pipeline.margin(point, surface)[0], 0)

    def test_compaction_preserves_shared_graph_and_root_lower_bound(self):
        planes = [[1., 0., 0., 0., 0], [0., 1., 0., 0., 1]]
        nodes = [(0, 1, 2), (1, -1, -2), (1, -1, -2), (0, -2, -2)]
        tree = pipeline.Tree(planes, nodes, 0)
        points = np.array(list(itertools.product((-1., 1.), repeat=3)))
        before = tree.contents(points)
        tree.compact()
        np.testing.assert_array_equal(tree.contents(points), before)
        self.assertEqual(len(tree.nodes), 1)
        self.assertEqual(tree.root, 0)
        self.assertEqual(pipeline.structure(tree.view())["hulls"]["1"]["reachable"], 1)
        with self.assertRaises(ValueError):
            pipeline.compact([(0, 0, -1)], 0)

    def test_splice_preserves_background_and_independent_hulls(self):
        lumps = [b""] * 15
        lumps[0] = b'{\n"classname" "worldspawn"\n}\n\0'
        lumps[1] = struct.pack("<4fi", 1., 0., 0., 10., 0)
        lumps[4] = b"visibility-sentinel"
        lumps[8] = b"lighting-sentinel"
        lumps[9] = struct.pack("<ihh", 0, -2, -1)
        lumps[14] = struct.pack("<9f7i", *([-20.] * 3 + [20.] * 3 + [0.] * 3), 0, 0, 0, 0, 0, 0, 0)
        data = bytearray(124)
        struct.pack_into("<i", data, 0, 30)
        for i, lump in enumerate(lumps):
            data.extend(b"\0" * (-len(data) % 4))
            struct.pack_into("<ii", data, 4 + i * 8, len(data), len(lump))
            data.extend(lump)
        terrain = {h: pipeline.Tree([[0., 1., 0., float(h), 1]], [(0, -2, -1)], 0)
                   for h in (1, 2, 3)}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "background.bsp"
            path.write_bytes(data)
            encoded, checked = pipeline.splice(path, terrain)
        bsp = HullFile(data=encoded)
        self.assertEqual(bsp.lumps[8], lumps[8])
        self.assertEqual(bsp.lumps[4], lumps[4])
        self.assertEqual(len(set(bsp.models[0]["head"][1:])), 3)
        points = np.array([[11., -10., 0.], [0., 1.5, 0.], [0., 2.5, 0.], [0., -10., 0.]])
        np.testing.assert_array_equal(bsp.contents(points, 1), [-2, -2, -2, -1])
        np.testing.assert_array_equal(bsp.contents(points, 2), [-2, -1, -2, -1])
        np.testing.assert_array_equal(bsp.contents(points, 3), [-2, -1, -1, -1])
        self.assertEqual(checked["stored_nodes"], 6)


if __name__ == "__main__":
    unittest.main()
