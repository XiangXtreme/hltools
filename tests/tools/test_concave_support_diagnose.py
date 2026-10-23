"""Shared support cuts must preserve a nonconvex source and legacy offsets."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import numpy as np

from concave_support_diagnose import intersection, shift_legacy, support_planes
from legacy_clip_diagnose import convex_hit, margin


def brush(planes):
    volume = intersection(np.array(planes, dtype=float))
    faces = []
    for p in volume["planes"]:
        points = volume["vertices"][abs(volume["vertices"] @ p[:3] - p[3]) < 1e-6]
        faces.append({"normal": p[:3], "dist": p[3], "vertices": points})
    return {"faces": faces}, volume


class SharedSupportTests(unittest.TestCase):
    def setUp(self):
        normal = 1 / np.sqrt(2)
        self.plateau, self.a = brush([[1, 0, 0, 2], [-1, 0, 0, 0], [0, 1, 0, 1],
                                     [0, -1, 0, 0], [0, 0, 1, 1], [0, 0, -1, 0]])
        self.ramp, self.b = brush([[1, 0, 0, 1], [-1, 0, 0, 0], [0, -1, 0, -1],
                                  [0, 0, -1, 0], [0, normal, normal, 2 * normal]])
        self.support = support_planes([self.plateau, self.ramp])

    def test_rejects_faces_crossing_other_source_geometry(self):
        for plane in ([1, 0, 0, 1], [0, 1, 0, 1], [0, -1, 0, -1]):
            self.assertFalse(any(np.max(abs(p - plane)) < 1e-8 for p in self.support))
        self.assertTrue(any(np.max(abs(p - [0, 0, 1, 1])) < 1e-8 for p in self.support))

    def test_shift_is_legacy_plane_offset(self):
        planes = np.array([[0., -.6, .8, 7.]])
        result = shift_legacy(planes, np.array([16., 16., 36.]))
        self.assertAlmostEqual(result[0, 3] - planes[0, 3], 28.8)
        self.assertNotAlmostEqual(result[0, 3] - planes[0, 3], 38.4)
        self.assertEqual(planes[0, 3], 7.)

    def test_valley_roofs_are_not_global_support_planes(self):
        normal = 1 / np.sqrt(2)
        left, _ = brush([[-1, 0, 0, 2], [1, 0, 0, 0], [0, 1, 0, 1],
                         [0, -1, 0, 1], [0, 0, -1, 0], [normal, 0, normal, normal]])
        right, _ = brush([[-1, 0, 0, 0], [1, 0, 0, 2], [0, 1, 0, 1],
                          [0, -1, 0, 1], [0, 0, -1, 0], [-normal, 0, normal, normal]])
        supports = support_planes([left, right])
        self.assertFalse(np.any(supports[:, 2] > 1e-7))

    def test_trims_ledge_without_filling_concavity_or_removing_source(self):
        half = np.array([.2, .2, .4])
        old = [intersection(shift_legacy(v["planes"], half)) for v in (self.a, self.b)]
        cap = shift_legacy(self.support, half)
        new = [intersection(np.r_[v["planes"], cap]) for v in old]
        o, d = np.array([[1.1, 1.19, 3.]]), np.array([[0., 0., -1.]])
        self.assertGreater(convex_hit(o, d, 5., new)[0] - convex_hit(o, d, 5., old)[0], .1)
        notch = np.array([[1.5, 1.5, .2]])
        self.assertGreater(margin(notch, new)[0], .1)
        for source, candidate in zip((self.a, self.b), new):
            self.assertLess(np.max(source["vertices"] @ candidate["planes"][:, :3].T
                                   - candidate["planes"][:, 3]), 1e-7)
        rng = np.random.default_rng(48293)
        points = rng.uniform([-.5, -.5, -.5], [2.5, 2.5, 2.], (20000, 3))
        self.assertFalse(np.any((margin(points, new) < -1e-6) & (margin(points, old) > 1e-6)))


if __name__ == "__main__":
    unittest.main()
