"""Distinguish a PL gain mismatch from amplitude compression, including pixel quantization."""
import unittest

import numpy as np

from apsk_centroids import analyze, ideal_points


class CentroidMeasurements(unittest.TestCase):
    def samples(self, mod, fec, transform):
        rng = np.random.default_rng(123)
        ideal = ideal_points(mod, fec)
        indices = rng.integers(0, len(ideal), 50000)
        z = transform(ideal[indices].copy(), indices, ideal)
        z += .02 * (rng.normal(size=len(z)) + 1j * rng.normal(size=len(z)))
        # The same one-pixel truncation as SDRangel's constellation display.
        row, col = np.floor(128 + 75 * z.real), np.floor(128 - 75 * z.imag)
        pixels = ((row - 127.5) + 1j * (127.5 - col)) / 75
        return analyze(pixels, ideal)[0], ideal

    def test_16apsk_outer_ring_pl_changes_gain_but_not_ring_ratio(self):
        measured, ideal = self.samples("16apsk", "2/3", lambda z, idx, p: z / max(abs(p)))
        self.assertAlmostEqual(measured["payload_gain"], 1 / max(abs(ideal)), delta=.002)
        self.assertAlmostEqual(measured["rings"][-1]["measured_radius"], 1, delta=.002)
        self.assertAlmostEqual(measured["rings"][-1]["measured_ratio_to_inner"], 3.15, delta=.02)
        self.assertLess(measured["centroid_error_after_common_gain_percent"], .2)

    def test_32apsk_outer_ring_pl_changes_all_three_rings(self):
        measured, ideal = self.samples("32apsk", "3/4", lambda z, idx, p: z / max(abs(p)))
        self.assertAlmostEqual(measured["payload_gain"], 1 / max(abs(ideal)), delta=.002)
        self.assertAlmostEqual(measured["rings"][1]["measured_ratio_to_inner"], 2.84, delta=.025)
        self.assertAlmostEqual(measured["rings"][2]["measured_ratio_to_inner"], 5.27, delta=.03)

    def test_outer_only_compression_changes_ring_ratio(self):
        def compress(z, indices, ideal):
            z[indices < 12] *= .93
            return z
        measured, _ = self.samples("16apsk", "2/3", compress)
        self.assertAlmostEqual(measured["rings"][-1]["measured_ratio_to_inner"], 3.15 * .93, delta=.02)
        self.assertGreater(measured["centroid_error_after_common_gain_percent"], 1)


if __name__ == "__main__":
    unittest.main()
