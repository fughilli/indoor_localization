"""cir_peaks on a synthetic record: the 24-bit signed unpacking and the first-path crossing."""

import unittest

import numpy as np

from swarm.tools import cir_peaks as cp


def pack(taps):
    out = bytearray()
    for z in taps:
        for v in (int(round(z.real)), int(round(z.imag))):
            out += (v & 0xFFFFFF).to_bytes(3, "little")
    return out.hex()


def record(first=100, n=48, path_tap=108, amp=40000.0, fp=None):
    rng = np.random.default_rng(0)
    taps = rng.normal(0, 30, n) + 1j * rng.normal(0, 30, n)
    taps[path_tap - first] += amp * (-1 - 0.5j)           # negative I: sign handling
    return {"type": "cir", "from": 7, "seq": 3, "first": first, "n": n,
            "fp": int((path_tap if fp is None else fp) * 64), "hex": pack(taps)}


class CirPeaksTest(unittest.TestCase):
    def test_unpack_signed_24_bit(self):
        z = cp.decode({"hex": pack([-5 - 7j, 131071 + 1j]), "n": 2})
        self.assertEqual(list(z), [-5 - 7j, 131071 + 1j])

    def test_crossing_matches_chip_first_path_when_consistent(self):
        row = cp.analyze([record()])[0]
        self.assertEqual(row["cross_tap"], 108)
        self.assertEqual(row["peak_tap"], 108)
        self.assertAlmostEqual(row["fp_tap"], 108.0)

    def test_disagreement_is_visible(self):
        row = cp.analyze([record(fp=104)])[0]
        self.assertEqual(row["cross_tap"] - row["fp_tap"], 4)


if __name__ == "__main__":
    unittest.main()
