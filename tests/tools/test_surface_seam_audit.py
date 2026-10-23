"""Surface continuity must be checked independently of target conformity."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import numpy as np

from surface_seam_audit import EPSILONS, pair_origins, persistent_steps, scan_candidates


class SurfaceSeamTests(unittest.TestCase):
    def test_real_step_persists_while_source_difference_shrinks(self):
        origins = pair_origins([[4., 7.]], 100.)
        source = origins[..., 0] * .3
        collision = source + np.array([20., 5.])
        self.assertTrue(persistent_steps(source, collision)[0])
        np.testing.assert_array_equal(origins[..., 2], 100.)
        np.testing.assert_allclose(origins[:, 0, :, 0] - 4., EPSILONS[:, None] * [-1., 1.])

    def test_continuous_steep_slope_is_not_a_step(self):
        source = pair_origins([[0., 0.]], 100.)[..., 0] * .3
        collision = pair_origins([[0., 0.]], 100.)[..., 0] * 10000.
        self.assertFalse(persistent_steps(source, collision)[0])

    def test_source_cliff_is_not_a_smooth_source_warning(self):
        source = np.tile([[[0., 8.]]], (3, 1, 1))
        self.assertFalse(persistent_steps(source, source + 36.)[0])

    def test_ray_misses_are_not_reported_as_finite_steps(self):
        source = np.zeros((3, 2, 2))
        collision = np.tile([20., 5.], (3, 2, 1))
        collision[0, 0, 0] = -np.inf
        source[-1, 1, 1] = -np.inf
        self.assertFalse(persistent_steps(source, collision).any())

    def test_scan_uses_only_reachable_vertical_planes(self):
        tree = SimpleNamespace(
            models=[{"head": [-1, 1, -1, -1]}],
            nodes=np.array([[0, -1, -2], [1, 2, -1], [2, -1, -2]]),
            planes=np.array([[1., 0., 0., 99.], [1., 0., 0., .123456789],
                             [0., 0., 1., 2.]]))
        self.assertEqual(scan_candidates(tree, 1, [7.], [-100., 100.]),
                         [(.123456789, 7.)])
        self.assertEqual(scan_candidates(tree, 2, [7.], [-100., 100.]), [])


if __name__ == "__main__":
    unittest.main()
