"""Continuous roof offsets must handle convex ridges and concave valleys."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import numpy as np

from terrain_lattice_diagnose import (build_lattice, coefficients, lattice_value,
                                      lattice_volumes, roof_patches, source_cliffs)
from legacy_clip_diagnose import convex_hit


def rectangle(xlo, xhi):
    return np.array([[xlo, -1.], [xhi, -1.], [xhi, 1.], [xlo, 1.]])


class RoofLatticeTests(unittest.TestCase):
    def setUp(self):
        self.planes = np.array([[-1., 0., 1., 0.], [1., 0., 1., 0.]]) / np.sqrt(2)
        self.xy = np.c_[np.linspace(-2., 2., 101), np.zeros(101)]

    def test_convex_ridge_and_concave_valley_preserve_source(self):
        for left, right, sign in ((0, 1, -1), (1, 0, 1)):
            terms, _ = build_lattice([(rectangle(-2., 0.), left), (rectangle(0., 2.), right)], self.planes)
            expected = sign * abs(self.xy[:, 0])
            np.testing.assert_allclose(lattice_value(self.xy, coefficients(self.planes), terms), expected)
            volumes, shifted = lattice_volumes(self.planes, terms, [-2., -1., 2., 1.], -100.,
                                               np.array([16., 16., 36.]))
            raised = lattice_value(self.xy, coefficients(shifted), terms)
            np.testing.assert_allclose(raised - expected, 26. * np.sqrt(2))
            origins = np.c_[self.xy[1:-1], np.full(99, 100.)]
            dirs = np.tile([0., 0., -1.], (99, 1))
            np.testing.assert_allclose(100. - convex_hit(origins, dirs, 200., volumes), raised[1:-1])

    def test_mixed_flat_and_valley_uses_both_min_and_max(self):
        planes = np.r_[self.planes, [[0., 0., 1., 1.]]]
        patches = [(rectangle(-2., -1.), 2), (rectangle(-1., 0.), 1),
                   (rectangle(0., 1.), 0), (rectangle(1., 2.), 2)]
        terms, _ = build_lattice(patches, planes)
        self.assertGreater(len(terms), 1)
        self.assertTrue(all(len(t) > 1 for t in terms))
        np.testing.assert_allclose(lattice_value(self.xy, coefficients(planes), terms),
                                   np.minimum(1., abs(self.xy[:, 0])))

    def test_coplanar_subdivision_does_not_change_roof(self):
        whole = [(rectangle(-2., 0.), 0), (rectangle(0., 2.), 1)]
        divided = [(rectangle(-2., -1.), 0), (rectangle(-1., 0.), 0), (rectangle(0., 2.), 1)]
        a, _ = build_lattice(whole, self.planes)
        b, _ = build_lattice(divided, self.planes)
        self.assertEqual(a, b)

    def test_discontinuous_source_is_rejected(self):
        planes = np.array([[0., 0., 1., 0.], [0., 0., 1., 1.]])
        with self.assertRaises(ValueError):
            build_lattice([(rectangle(-2., 0.), 0), (rectangle(0., 2.), 1)], planes)

    def test_projection_area_cannot_hide_overlaps_and_holes(self):
        tri = np.array([[-2., -1., 0.], [2., -1., 0.], [2., 1., 0.]])
        normals = np.tile([0., 0., 1.], (2, 1))
        # Duplicating half the crop has the right total area but leaves a hole.
        with self.assertRaisesRegex(ValueError, "Overlapping"):
            roof_patches(np.array([tri, tri]), normals, [-2., -1., 2., 1.])

    def test_real_source_cliff_is_measured_independently(self):
        source = []
        for xlo, xhi, top in ((-2., 0., 2.), (0., 2., 1.)):
            source.append({"planes": np.array([[1., 0., 0., xhi], [-1., 0., 0., -xlo],
                            [0., 1., 0., 1.], [0., -1., 0., 1.],
                            [0., 0., 1., top], [0., 0., -1., 1.]])})
        cliffs = source_cliffs([(rectangle(-2., 0.), 0), (rectangle(0., 2.), 1)],
                                source, [-2., -1., 2., 1.], 10., 20.)
        self.assertTrue(cliffs)
        for cliff in cliffs:
            self.assertAlmostEqual(cliff["smallest_spacing_step"], 1.)


if __name__ == "__main__":
    unittest.main()
