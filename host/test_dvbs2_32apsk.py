"""32APSK PL amplitudes, scrambling, streaming alphabet and legacy output."""
import hashlib
import unittest

import numpy as np

import dvbs2


class APSK32Frames(unittest.TestCase):
    def test_unit_pl_leaves_every_data_symbol_unchanged(self):
        ts = dvbs2._golden_ts()
        for frame in ("normal", "short"):
            for fec in dvbs2.rates("32apsk", frame):
                for pilots in (False, True):
                    with self.subTest(frame=frame, fec=fec, pilots=pilots):
                        outer = dvbs2.Encoder(fec, frame, pilots, mod="32apsk")
                        unit = dvbs2.Encoder(fec, frame, pilots, mod="32apsk", apsk_pl="unit")
                        a, b = outer.frames(ts), unit.frames(ts)
                        self.assertEqual(len(a), len(b))
                        va, vb = np.frombuffer(a, np.uint8), np.frombuffer(b, np.uint8)
                        mask = vb >= 32
                        self.assertTrue(mask.any())
                        self.assertLessEqual(int(vb.max()), 35)
                        self.assertTrue(np.array_equal(va[~mask], vb[~mask]))
                        points = np.concatenate((dvbs2.apsk32_points(fec),
                                                 np.exp(1j * np.radians([45, 135, -135, -45]))))
                        self.assertTrue(np.allclose(abs(points[vb[mask]]), 1))
                        self.assertTrue(np.allclose(np.angle(points[va[mask]]), np.angle(points[vb[mask]])))
                        for start in range(0, len(vb), unit.nsym):
                            expected = (unit.hdr_i + 1j * unit.hdr_q) / np.sqrt(2)
                            self.assertTrue(np.allclose(points[vb[start:start + 90]], expected))

    def test_default_outer_profile_keeps_original_hashes(self):
        for (frame, fec, pilots), expected in dvbs2.GOLDEN_32APSK.items():
            encoded = dvbs2.Encoder(fec, frame, pilots, mod="32apsk").frames(dvbs2._golden_ts())
            self.assertEqual(hashlib.sha256(encoded).hexdigest()[:16], expected)

    def test_unit_points_survive_iq_inversion_and_swap(self):
        v = np.arange(36, dtype=np.uint8)
        points = np.concatenate((dvbs2.apsk32_points("3/4"),
                                 np.exp(1j * np.radians([45, 135, -135, -45]))))
        for invert, swap in [(True, False), (False, True), (True, True)]:
            e = dvbs2.Encoder("3/4", mod="32apsk", invert=invert, swap_iq=swap, apsk_pl="unit")
            got = points[np.frombuffer(e.pack32(v), np.uint8)]
            expected = points.conj() if invert else points.copy()
            if swap:
                expected = 1j * expected.conj()
            self.assertTrue(np.allclose(got, expected))


if __name__ == "__main__":
    unittest.main()
