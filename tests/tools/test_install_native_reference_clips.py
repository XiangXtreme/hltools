"""Native collision replacement must preserve loading metadata and coverage."""

from pathlib import Path
import struct
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from install_native_reference_clips import adapt
from source_clip_pipeline import HullFile


def fixture(native, distance=10.):
    lumps = [b""] * 15
    lumps[0] = b'{"classname" "worldspawn"}\0'
    lumps[1] = struct.pack("<4fi", 1., 0., 0., distance, 0)
    lumps[5] = struct.pack("<ihh6h2H", 0, -1, -1, *([0] * 8))
    lumps[9] = (struct.pack("<ihh", 0, -1, -1) if native else b"")
    lumps[9] += struct.pack("<ihh", 0, -2, -1) if native else struct.pack("<ihh", 0, -1, -2)
    low = [-4000., -5000., 4000.] if native else [-4500., -100., 6000.]
    high = [5000., 5000., 7000.] if native else [100., 100., 8000.]
    root = 1 if native else 0
    lumps[14] = struct.pack("<9f7i", *low, *high, 0., 0., 0., 0, root, root, root, 0, 0, 0)
    data = bytearray(124)
    struct.pack_into("<i", data, 0, 30)
    for index, lump in enumerate(lumps):
        struct.pack_into("<ii", data, 4 + index * 8, len(data), len(lump))
        data.extend(lump)
    return HullFile(data=bytes(data))


class NativeReplacementTests(unittest.TestCase):
    def test_replacement_preserves_prediction_bounds_and_collision(self):
        native, rebuilt = fixture(True), fixture(False)
        encoded, checks = adapt(native, rebuilt)
        result = HullFile(data=encoded)
        np.testing.assert_array_equal(result.models[0]["mins"], [-4500., -5000., 4000.])
        np.testing.assert_array_equal(result.models[0]["maxs"], [5000., 5000., 8000.])
        self.assertEqual(result.nodes[0].tolist(), [0, -1, -1])
        points = np.array([[20., 0., 0.], [0., 0., 0.]])
        for hull in (1, 2, 3):
            np.testing.assert_array_equal(result.contents(points, hull), rebuilt.contents(points, hull))
            self.assertEqual(checks["hulls"][str(hull)]["root"], 1)
        for index in range(15):
            if index not in (1, 9, 14):
                self.assertEqual(result.lumps[index], native.lumps[index])

    def test_changed_draw_plane_rejected(self):
        with self.assertRaisesRegex(ValueError, "Template draw plane"):
            adapt(fixture(True), fixture(False, distance=11.))


if __name__ == "__main__":
    unittest.main()
