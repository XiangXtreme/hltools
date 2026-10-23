"""Full boundary composition must preserve source walls, not brush partitions."""

from pathlib import Path
import sys
import itertools
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import numpy as np

from boundary_lattice_diagnose import (arrangement, box, box_planes, clip, compose, compose_source, polyhedron,
                                       outside_volume, supporting_planes, term_volumes, verify_source)
from boundary_lattice_audit import persistent_jumps, wall_height
from concave_support_diagnose import shift_legacy
from legacy_clip_diagnose import convex_hit, margin


def source_box(low, high):
    value = box([low, high])
    value["planes"] = box_planes([low, high])
    return value


class BoundaryLatticeTests(unittest.TestCase):
    def test_axis_audit_distinguishes_a_step_from_a_steep_slope(self):
        values = np.array([[[0., 1.], [0., 1.]], [[0., 1.], [0., .1]], [[0., 1.], [0., .01]]])
        np.testing.assert_array_equal(persistent_jumps(values), [True, False])

    def test_source_outline_is_distinguished_from_an_internal_step(self):
        values = np.tile(np.array([[[-np.inf, 1.]]]), (3, 1, 1))
        self.assertFalse(persistent_jumps(values)[0])
        self.assertTrue(persistent_jumps(values, allow_outline=True)[0])

    def test_wall_height_measures_a_local_sliver_and_rejects_its_extension(self):
        triangles = np.array([[[0., 0., 0.], [1., 1., 1.], [1., 1., 1.0001]]])
        self.assertAlmostEqual(wall_height(triangles, 1, .5), .00005)
        self.assertIsNone(wall_height(triangles, 1, 2.))

    def test_nearby_vertex_does_not_remove_a_supporting_face(self):
        vertices = np.array([[0., 0., 1.], [1., 0., 1.], [0., 1., 1.],
                             [1., 1., 1. - 3.3e-7]])
        self.assertEqual(np.linalg.matrix_rank(vertices[1:] - vertices[0], tol=1e-7), 3)
        plane = np.array([[0., 0., 1., 1.]])
        np.testing.assert_array_equal(supporting_planes({"vertices": vertices}, plane), plane)

    def test_reduction_rejects_a_lost_half_space(self):
        planes = box_planes([[-1., -1., -1.], [1., 1., 1.]])
        with patch("boundary_lattice_diagnose.supporting_planes", return_value=np.delete(planes, 2, axis=0)):
            with self.assertRaisesRegex(ValueError, "reduction changes"):
                term_volumes(planes, [list(range(6))], [[-2., -2., -2.], [2., 2., 2.]])

    def test_clustered_source_vertices_recover_a_closed_hull(self):
        points = np.array([
            [248.4504792332268, 434.94728434504464, 124.48242810545739],
            [248.45047923322693, 434.9472843450442, -289.],
            [248.99999999999994, 434.49999999999636, -289.],
            [248.51086975941308, 434.8981292655907, 124.54255573330151],
            [248.510968819296, 434.8980486354534, 124.45880405786343],
            [264.99999977626766, 434.50000000539785, 150.9999996399947],
            [265.4705882366635, 434.0776772246973, -289.],
            [265.00000000137254, 434.4999999999592, -289.],
            [264.99999999999784, 434.4999999999969, 150.99999998519803],
            [265.00000000073976, 434.499999999331, 150.99999930631992],
            [265.4291124619988, 434.11489907375545, -289.],
            [264.99999999999784, 434.4999999999969, 150.99999999832818],
            [264.9999999999979, 434.4999999999969, 150.99999999897932],
            [264.9999999999992, 434.4999999999956, 150.99999999870957],
            [265.42911246187816, 434.1148990737583, -250.22015068872543]])
        value = polyhedron(points)
        edges = {}
        for tri in value["simplices"]:
            for a, b in itertools.combinations(tri, 2):
                key = tuple(sorted((int(a), int(b))))
                edges[key] = edges.get(key, 0) + 1
        self.assertTrue(all(count == 2 for count in edges.values()))
        self.assertEqual(len(value["vertices"]) - len(edges) + len(value["simplices"]), 2)
        self.assertAlmostEqual(value["volume"], 1567.794682, delta=.001)
        p = np.array([0., 0., 1., 0.])
        halves = [clip(value, p), clip(value, -p)]
        self.assertAlmostEqual(sum(h["volume"] for h in halves), value["volume"], delta=.001)

    def test_thin_translated_sliver_keeps_volume_without_joggling(self):
        points = np.array(list(itertools.product((-64., 64.), (-64., 64.), (-5e-8, 5e-8))))
        angle = .7
        rotation = np.array([[np.cos(angle), 0., np.sin(angle)], [0., 1., 0.],
                             [-np.sin(angle), 0., np.cos(angle)]])
        points = points @ rotation.T + [78., -247., 110.]
        value = polyhedron(points)
        self.assertAlmostEqual(value["volume"], 128. ** 2 * 1e-7, delta=1e-8)
        p = np.array([1., 0., 0., 78.])
        a, b = clip(value, p), clip(value, -p)
        self.assertAlmostEqual(a["volume"] + b["volume"], value["volume"], delta=1e-8)

    def test_clipping_conserves_volume_and_source_subtraction(self):
        cube = box([[-1., -1., -1.], [1., 1., 1.]])
        plane = np.array([1., 1., 1., 0.]) / np.sqrt(3)
        a, b = clip(cube, plane), clip(cube, -plane)
        self.assertAlmostEqual(a["volume"] + b["volume"], 8.)
        source = [source_box([-1., -1., -1.], [0., 1., 1.]),
                  source_box([0., -1., -1.], [1., 1., 1.])]
        self.assertAlmostEqual(outside_volume(cube, source), 0.)

    def test_real_cliff_moves_outward_and_keeps_its_height(self):
        source = [source_box([-2., -1., -1.], [0., 1., 2.]),
                  source_box([0., -1., -1.], [2., 1., 1.])]
        planes = np.r_[box_planes([[-2., -1., -1.], [2., 1., 2.]]),
                       [[1., 0., 0., 0.], [0., 0., 1., 1.]]]
        bounds = np.array([[-3., -2., -2.], [3., 2., 3.]])
        signs, solid, _ = arrangement(planes, bounds, source)
        terms = compose(signs, solid)
        original = term_volumes(planes, terms, bounds)
        self.assertLess(verify_source(original, source, signs, solid, terms)["term_excess_volume_sum"], 1e-8)
        expanded = term_volumes(shift_legacy(planes, np.array([.2, .2, .4])), terms, bounds)
        origins = np.array([[.2 - 1e-4, 0., 3.], [.2 + 1e-4, 0., 3.], [-1e-4, 0., 3.], [1e-4, 0., 3.]])
        tops = 3. - convex_hit(origins, np.tile([0., 0., -1.], (4, 1)), 6., expanded)
        np.testing.assert_allclose(tops, [2.4, 1.4, 2.4, 2.4])

    def test_source_notch_remains_empty_after_offset(self):
        source = [source_box([-2., -2., -1.], [0., 2., 1.]),
                  source_box([0., -2., -1.], [2., 0., 1.])]
        planes = np.r_[box_planes([[-2., -2., -1.], [2., 2., 1.]]),
                       [[1., 0., 0., 0.], [0., 1., 0., 0.]]]
        bounds = np.array([[-3., -3., -2.], [3., 3., 2.]])
        signs, solid, _ = arrangement(planes, bounds, source)
        terms = compose(signs, solid)
        expanded = term_volumes(shift_legacy(planes, np.array([.2, .2, .4])), terms, bounds)
        self.assertGreater(margin(np.array([[1., 1., 0.]]), expanded)[0], .5)
        self.assertLess(margin(np.array([[-1., 1., 0.]]), expanded)[0], -.5)

    def test_missing_source_boundary_plane_is_rejected(self):
        source = [source_box([-1., -1., -1.], [1., 1., 1.])]
        with self.assertRaisesRegex(ValueError, "do not separate source occupancy"):
            arrangement(np.array([[1., 0., 0., .5]]), [[-2., -2., -2.], [2., 2., 2.]], source)

    def test_adaptive_composition_ignores_coplanar_brush_split(self):
        whole = [source_box([-1., -1., -1.], [1., 1., 1.])]
        divided = [source_box([-1., -1., -1.], [0., 1., 1.]),
                   source_box([0., -1., -1.], [1., 1., 1.])]
        planes = box_planes([[-1., -1., -1.], [1., 1., 1.]])
        bounds = [[-2., -2., -2.], [2., 2., 2.]]
        a, ar = compose_source(planes, bounds, whole)
        b, br = compose_source(planes, bounds, divided)
        self.assertEqual(a, b)
        self.assertEqual(ar["missing_source_volume_sum"], 0.)
        self.assertEqual(br["missing_source_volume_sum"], 0.)

    def test_adaptive_composition_preserves_nonconvex_source(self):
        source = [source_box([-2., -2., -1.], [0., 2., 1.]),
                  source_box([0., -2., -1.], [2., 0., 1.])]
        planes = np.r_[box_planes([[-2., -2., -1.], [2., 2., 1.]]),
                       [[1., 0., 0., 0.], [0., 1., 0., 0.]]]
        bounds = [[-3., -3., -2.], [3., 3., 2.]]
        terms, proof = compose_source(planes, bounds, source)
        self.assertEqual(proof["term_excess_volume_sum"], 0.)
        self.assertEqual(proof["missing_source_volume_sum"], 0.)
        target = term_volumes(planes, terms, bounds)
        self.assertGreater(margin(np.array([[1., 1., 0.]]), target)[0], .5)

    def test_adaptive_composition_rejects_incomplete_boundary(self):
        source = [source_box([-1., -1., -1.], [1., 1., 1.])]
        with self.assertRaises(ValueError):
            compose_source(np.array([[1., 0., 0., .5]]), [[-2., -2., -2.], [2., 2., 2.]], source)


if __name__ == "__main__":
    unittest.main()
