"""Conservative seam normalization must not inflate footprints or erase cliffs."""

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import numpy as np
from scipy.spatial import ConvexHull

from boundary_lattice_diagnose import box_planes, compose_source, polyhedron, term_volumes, outside_volume
import boundary_lattice_diagnose as lattice
from boundary_normalize import seam_planes, vertical_envelope
from boundary_lattice_build import repair_builder, source_checks
from legacy_clip_diagnose import margin


def cube(low, high):
    from boundary_lattice_diagnose import box
    value = box([low, high])
    value["planes"] = box_planes([low, high])
    return value


class NormalizationTests(unittest.TestCase):
    def test_covered_piece_reuses_the_accepted_larger_closure(self):
        source = [cube([-.5,-.5,-.5], [.5,.5,.5]), cube([-1.,-1.,-1.], [1.,1.,1.])]
        planes = np.r_[box_planes([[-1.,-1.,-1.], [1.,1.,1.]]), [[1.,0.,0.,.75]]]
        with patch.object(lattice, "outside_fragments", wraps=lattice.outside_fragments) as subtract:
            terms, proof = compose_source(planes, [[-2.,-2.,-2.], [2.,2.,2.]], source)
        self.assertEqual(terms, [list(range(6))])
        self.assertEqual(proof["missing_source_volume_sum"], 0.)
        # One closure check and one coverage subtraction per source brush.
        self.assertEqual(subtract.call_count, 3)

    def normalization_record(self):
        return {"normalization": {"mode": "bounded_vertical_normalization",
                                  "maximum_upward_distance": .001, "roof_lift": .00025},
                "patches": [{"source_coverage_checked": True, "term_excess_volume_sum": 0.,
                             "missing_source_volume_sum": 0., "containment": "supplied_envelope",
                             "unshifted_missing_source_points": 0, "unshifted_outside_envelope_points": 0,
                             "source_contact_grid": {"hit_miss_differences": 0},
                             "source_contact_minimum_gain": 0., "source_contact_maximum_gain": .00025}]}

    def test_build_distinguishes_normalization_from_source_equivalence(self):
        record = self.normalization_record()
        checked = source_checks(record)
        self.assertEqual(checked["containment"], "supplied_envelope")
        self.assertNotIn("unshifted_point_mismatches", checked)
        del record["normalization"]
        with self.assertRaises(ValueError):
            source_checks(record)

    def test_normalized_build_uses_compact_fragment_repair(self):
        self.assertEqual(repair_builder(self.normalization_record()), "fragments")
        self.assertEqual(repair_builder({}), "planes")

    def test_build_rejects_failed_or_nonfinite_envelope_checks(self):
        for key, value in (("source_contact_maximum_gain", .01),
                           ("source_contact_minimum_gain", -.01),
                           ("term_excess_volume_sum", float("nan")),
                           ("missing_source_volume_sum", .01),
                           ("unshifted_missing_source_points", 1),
                           ("unshifted_outside_envelope_points", 1)):
            record = self.normalization_record()
            record["patches"][0][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                source_checks(record)

    def test_build_rejects_an_oversized_normalization_bound(self):
        record = self.normalization_record()
        record["normalization"]["maximum_upward_distance"] = .01
        with self.assertRaises(ValueError):
            source_checks(record)

    def test_extrusion_keeps_tetrahedron_projection(self):
        tetra = polyhedron(np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]))
        tetra["planes"] = np.array([[-1.,0.,0.,0.], [0.,-1.,0.,0.], [0.,0.,-1.,0.],
                                    [1.,1.,1.,1.]]) / np.array([[1.], [1.], [1.], [np.sqrt(3)]])
        allowed = vertical_envelope([tetra], .4)
        values = margin(np.array([[.6,.6,.1], [.2,.2,.9], [.2,.2,-.01]]), allowed)
        self.assertGreater(values[0], 0.)
        self.assertLess(values[1], 0.)
        self.assertGreater(values[2], 0.)

    def test_extrusion_matches_vertex_hull_with_sloped_bottom_faces(self):
        points = np.array([[0.,0.,0.], [1.,0.,0.], [0.,1.,0.], [0.,0.,1.]])
        angle = .7
        rotation = np.array([[np.cos(angle), 0., np.sin(angle)], [0., 1., 0.],
                             [-np.sin(angle), 0., np.cos(angle)]])
        points = points @ rotation.T
        shape = polyhedron(points)
        faces = ConvexHull(points).equations
        shape["planes"] = np.c_[faces[:, :3], -faces[:, 3]]
        extruded = ConvexHull(np.r_[points, points + [0.,0.,.4]]).equations
        expected = [{"planes": np.c_[extruded[:, :3], -extruded[:, 3]]}]
        probes = np.random.default_rng(481).uniform([-1.,-1.,-1.], [2.,2.,2.], (10000, 3))
        np.testing.assert_array_equal(margin(probes, vertical_envelope([shape], .4)) < 0,
                                      margin(probes, expected) < 0)

    def test_a_large_wall_on_the_same_plane_prevents_seam_removal(self):
        planes = np.array([[0., 1., 0., 0.]])
        thin = np.array([[0.,0.,0.], [1.,0.,1.], [1.,0.,1.0001]])
        large = np.array([[0.,0.,0.], [1.,0.,0.], [1.,0.,2.]])
        self.assertEqual(len(seam_planes(planes, np.array([thin]), np.array([[0.,1.,0.]]), .00025)), 1)
        self.assertEqual(seam_planes(planes, np.array([thin, large]), np.array([[0.,1.,0.]]*2), .00025), [])

    def test_small_source_step_is_covered_within_an_explicit_envelope(self):
        source = [cube([-1.,-1.,-1.], [0.,1.,1.0001]), cube([0.,-1.,-1.], [1.,1.,1.])]
        envelope = vertical_envelope(source, .001)
        planes = box_planes([[-1.,-1.,-1.], [1.,1.,1.00025]])
        bounds = [[-2.,-2.,-2.], [2.,2.,2.]]
        terms, proof = compose_source(planes, bounds, source, containment_source=envelope)
        self.assertEqual(proof["missing_source_volume_sum"], 0.)
        self.assertEqual(proof["containment"], "supplied_envelope")
        self.assertLess(sum(outside_volume(v, envelope) for v in term_volumes(planes, terms, bounds)), 1e-8)

    def test_a_real_step_cannot_be_filled_by_the_micro_envelope(self):
        source = [cube([-1.,-1.,-1.], [0.,1.,2.]), cube([0.,-1.,-1.], [1.,1.,1.])]
        planes = box_planes([[-1.,-1.,-1.], [1.,1.,2.]])
        with self.assertRaises(ValueError):
            compose_source(planes, [[-2.,-2.,-2.], [2.,2.,3.]], source,
                           containment_source=vertical_envelope(source, .001))

    def test_a_real_step_is_retained_when_its_wall_is_available(self):
        source = [cube([-1.,-1.,-1.], [0.,1.,2.]), cube([0.,-1.,-1.], [1.,1.,1.])]
        planes = np.r_[box_planes([[-1.,-1.,-1.], [1.,1.,2.00025]]),
                       [[0.,0.,1.,1.00025], [1.,0.,0.,0.]]]
        bounds = [[-2.,-2.,-2.], [2.,2.,3.]]
        terms, proof = compose_source(planes, bounds, source, containment_source=vertical_envelope(source, .001))
        normalized = term_volumes(planes, terms, bounds)
        self.assertEqual(proof["missing_source_volume_sum"], 0.)
        values = margin(np.array([[-.5,0.,1.5], [.5,0.,1.5]]), normalized)
        self.assertLess(values[0], 0.)
        self.assertGreater(values[1], 0.)


if __name__ == "__main__":
    unittest.main()
