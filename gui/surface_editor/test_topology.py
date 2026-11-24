"""Offline geometry tests for plane creation, refinement and rebuilding."""
from pathlib import Path
import tempfile
import unittest
import numpy as np

from .model import NurbsModel
from .topology import basis, plane, sample, change_grid


class TopologyTests(unittest.TestCase):
    def test_basis_partition_and_endpoints(self):
        for n in (4, 6, 10, 20):
            b = basis(n, np.linspace(0, 1, 101))
            np.testing.assert_allclose(b.sum(axis=1), 1, atol=1e-12)
            self.assertEqual(b[0, 0], 1)
            self.assertEqual(b[-1, -1], 1)

    def test_flat_dimensions_and_uniform_parameterization(self):
        s = plane(2400, 1200, 9, 7)
        sampled = sample(s, 11, 11)
        np.testing.assert_allclose(sampled[..., 1], 0)
        np.testing.assert_allclose(sampled[:, 0, 0], np.linspace(0, 2400, 11), atol=1e-9)
        np.testing.assert_allclose(sampled[0, :, 2], np.linspace(0, 1200, 11), atol=1e-9)

    def test_refinement_preserves_curved_surface(self):
        m = NurbsModel.load(Path(__file__).resolve().parents[2] / "Reference Surface_1.3dm")
        original = sample(m._srf, 71, 73)
        rebuilt, report = change_grid(m._srf, 11, 9, "refine")
        self.assertEqual((rebuilt.Points.CountU, rebuilt.Points.CountV), (11, 9))
        np.testing.assert_allclose(sample(rebuilt, 71, 73), original, atol=1e-8)
        self.assertLess(report["max_sampled_deviation_mm"], 1e-8)

    def test_rebuild_reduces_plane_to_four_by_four(self):
        rebuilt, report = change_grid(plane(nu=12, nv=10), 4, 4, "rebuild", .001)
        self.assertEqual((rebuilt.Points.CountU, rebuilt.Points.CountV), (4, 4))
        self.assertLess(report["max_sampled_deviation_mm"], 1e-8)

    def test_tolerance_rejects_approximation_without_mutating_source(self):
        m = NurbsModel.load(Path(__file__).resolve().parents[2] / "Reference Surface_1.3dm")
        before = m._srf.Encode()
        with self.assertRaisesRegex(ValueError, "deviate"):
            change_grid(m._srf, 4, 4, "rebuild", 0.00001)
        self.assertEqual(m._srf.Encode(), before)
        rebuilt, report = change_grid(m._srf, 4, 4, "rebuild", 1000)
        self.assertGreater(report["max_sampled_deviation_mm"], .01)
        self.assertTrue(rebuilt.IsValid)

    def test_invalid_counts_and_modes(self):
        s = plane()
        for args in ((3, 6, "refine", 5), (21, 6, "refine", 5), (True, 6, "refine", 5),
                     (4, 4, "refine", 5), (6, 6, "rebuild", float("nan")), (6, 6, "other", 5)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                change_grid(s, *args)

    def test_new_model_repeated_save_has_single_surface(self):
        m = NurbsModel.flat()
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "flat.3dm"
            m.save(p)
            m.save(p)
            other = NurbsModel.load(p)
            self.assertEqual(len(other._file.Objects), 1)
            np.testing.assert_allclose(other.control_points(), m.control_points())


if __name__ == "__main__":
    unittest.main()
