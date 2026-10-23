"""Boundary extraction must remove internal and duplicate faces before expansion."""

import itertools
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import brush_exterior_pipeline as exterior


def box(low, high):
    vertices = np.array(list(itertools.product(*zip(low, high))))
    faces = []
    for axis in range(3):
        for side, coordinate in ((-1, low[axis]), (1, high[axis])):
            normal = np.eye(3)[axis] * side
            polygon = vertices[vertices[:, axis] == coordinate]
            others = [i for i in range(3) if i != axis]
            center = polygon.mean(0)
            delta = polygon - center
            polygon = polygon[np.argsort(np.arctan2(delta[:, others[1]], delta[:, others[0]]))]
            faces.append({"normal": normal, "dist": coordinate * side, "vertices": polygon})
    return {"entity": 3, "contents": -2, "faces": faces}


class ExteriorTests(unittest.TestCase):
    def check_boxes(self, boxes, expected_area):
        triangles, normals, _, report = exterior.extract_exterior({"brushes": boxes}, 3)
        self.assertAlmostEqual(report["exterior_area"], expected_area, places=6)
        self.assertEqual(report["boundary_probe_failures"], 0)
        self.assertTrue(np.all(np.einsum("ij,ij->i", np.cross(triangles[:, 1] - triangles[:, 0],
                                                            triangles[:, 2] - triangles[:, 0]), normals) > 0))
        return triangles, normals

    def test_touching_cubes_remove_shared_face(self):
        self.check_boxes([box([0, 0, 0], [1, 1, 1]), box([1, 0, 0], [2, 1, 1])], 10)

    def test_identical_cubes_have_one_boundary(self):
        self.check_boxes([box([0, 0, 0], [1, 1, 1]), box([0, 0, 0], [1, 1, 1])], 6)

    def test_overlapping_cubes_clip_partial_faces(self):
        self.check_boxes([box([0, 0, 0], [2, 2, 2]), box([1, 1, 1], [3, 3, 3])], 42)

    def test_partial_contact_and_enclosed_brush(self):
        self.check_boxes([box([0, 0, 0], [2, 2, 2]), box([2, .5, .5], [3, 1.5, 1.5]),
                          box([.2, .2, .2], [.4, .4, .4])], 28)

    def test_narrow_gap_is_not_an_internal_contact(self):
        self.check_boxes([box([0, 0, 0], [1, 1, 1]), box([1.001, 0, 0], [2.001, 1, 1])], 12)

    def test_disjoint_subtraction_does_not_fragment_polygon(self):
        polygon = np.array([[0., 0., 0.], [4., 0., 0.], [0., 4., 0.]])
        # The bounding boxes overlap, but the triangle is outside this brush.
        other = box([2.5, 2.5, -1], [4, 4, 1])
        planes = np.array([[*f["normal"], f["dist"]] for f in other["faces"]])
        values = exterior.subtract_brush(polygon, planes, np.array([0, 0, 1]), 0, 1)
        self.assertEqual(len(values), 1)
        np.testing.assert_array_equal(values[0], polygon)

    def test_sloping_shared_face(self):
        brushes = [box([0, 0, 0], [1, 1, 1]), box([1, 0, 0], [2, 1, 1])]
        transform = np.array([[1., 0., 0.], [0., 1., 0.], [.4, .2, 1.]])
        for brush in brushes:
            for face in brush["faces"]:
                points = face["vertices"] @ transform.T
                normal = np.linalg.solve(transform.T, face["normal"])
                normal /= np.linalg.norm(normal)
                face.update(vertices=points, normal=normal, dist=points[0] @ normal)
        _, normals, _, report = exterior.extract_exterior({"brushes": brushes}, 3)
        self.assertEqual(report["boundary_probe_failures"], 0)
        self.assertEqual(len(normals), 20)


if __name__ == "__main__":
    unittest.main()
