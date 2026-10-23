"""Thin collision intervals must survive integer-valued trace lengths."""

from types import SimpleNamespace
import unittest

import numpy as np

from legacy_clip_diagnose import convex_hit
from source_clip_validate import tree_first_hit


class CollisionIntervals(unittest.TestCase):
    def test_thin_slab_with_integer_and_float_length(self):
        tree = SimpleNamespace(
            models=[{"head": [-1, 0, -1, -1]}],
            nodes=np.array([[0, -1, 1], [1, -2, -1]]),
            planes=np.array([[1., 0, 0, .9], [1., 0, 0, .2]]))
        volumes = [{"planes": np.array([[1., 0, 0, .9], [-1., 0, 0, -.2]])}]
        origins = np.array([[2., 0, 0], [0., 0, 0], [.5, 0, 0]])
        directions = np.array([[-1., 0, 0], [1., 0, 0], [1., 0, 0]])
        for length in (3, 3.):
            with self.subTest(length=length):
                np.testing.assert_allclose(tree_first_hit(tree, origins, directions, length, 1), [1.1, .2, 0])
                np.testing.assert_allclose(convex_hit(origins, directions, length, volumes), [1.1, .2, 0])

    def test_parallel_ray_outside_slab_misses(self):
        volumes = [{"planes": np.array([[1., 0, 0, .9], [-1., 0, 0, -.2]])}]
        hit = convex_hit(np.array([[2., 0, 0]]), np.array([[0., 0, 1.]]), 3, volumes)
        self.assertTrue(np.isinf(hit[0]))


if __name__ == "__main__":
    unittest.main()
