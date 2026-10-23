"""Entity support clipping must retain source volume and unchanged hull planes."""

import itertools
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import numpy as np

from brush_surface_pipeline import HALF
from concave_support_diagnose import intersection, shift_legacy
from legacy_support_pipeline import clipped_volumes, contact_grid


def cube():
    planes = np.c_[np.r_[np.eye(3), -np.eye(3)], np.ones(6)]
    points = np.array(list(itertools.product((-1., 1.), repeat=3)))
    faces = [{"normal": p[:3], "dist": p[3],
              "vertices": points[abs(points @ p[:3] - p[3]) < 1e-8]} for p in planes]
    expanded = []
    for h in (1, 2, 3):
        v = intersection(shift_legacy(planes, HALF[h]))
        expanded.append({"planes": v["planes"].tolist(), "vertices": v["vertices"].tolist()})
    return {"brush": 7, "faces": faces, "legacy_hulls": expanded}, planes, points


class EntitySupportTests(unittest.TestCase):
    def test_unchanged_brush_retains_exported_planes(self):
        source, supports, _ = cube()
        for h in (1, 2, 3):
            target, changed = clipped_volumes([source], supports, h)
            self.assertEqual(changed, [])
            np.testing.assert_array_equal(target[0]["planes"], source["legacy_hulls"][h - 1]["planes"])

    def test_support_clips_expansion_and_preserves_source(self):
        source, _, points = cube()
        supports = np.array([[0., .6, .8, 1.4]])
        for h in (1, 2, 3):
            target, changed = clipped_volumes([source], supports, h)
            self.assertEqual(changed, [7])
            volume = target[0]
            self.assertLess(np.max(points @ volume["planes"][:, :3].T - volume["planes"][:, 3]), 1e-6)
            cap = shift_legacy(supports, HALF[h])[0]
            self.assertLess(np.max(volume["vertices"] @ cap[:3] - cap[3]), 1e-6)
            self.assertTrue(np.any(np.array(source["legacy_hulls"][h - 1]["vertices"]) @ cap[:3] > cap[3] + .1))

    def test_invalid_cut_cannot_remove_source_volume(self):
        source, _, _ = cube()
        with self.assertRaises(ValueError):
            clipped_volumes([source], np.array([[0., 0., 1., -40.]]), 1)

    def test_contact_rays_follow_map_coordinates(self):
        source, _, _ = cube()
        for face in source["faces"]:
            face["vertices"] = face["vertices"] + [800., -500., 1200.]
        _, _, origins, directions, top, length = contact_grid([source])
        self.assertGreater(top, 1201.)
        self.assertTrue(np.all(origins[:, 2] == top))
        self.assertLess(origins[:, 0].min(), 799.)
        self.assertGreater(origins[:, 0].max(), 801.)
        self.assertTrue(np.all(directions == [0., 0., -1.]))
        self.assertGreater(length, 200.)


if __name__ == "__main__":
    unittest.main()
