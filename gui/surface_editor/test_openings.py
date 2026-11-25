"""Opening geometry, placement, lintel and persistence regression tests."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

from .model import NurbsModel
from .openings import geometry, validate_openings
from .session import clone_model, surface_hash
from physics_check.pc.openings import OpeningWall, fit_interval, run_opening_check
from physics_check.pc.geom2d import Path2
from physics_check.pc.params import resolve_params


def window(**kwargs):
    return dict({"id": "window-1", "center_fraction": .5, "width_mm": 900, "height_mm": 600,
                 "sill_mm": 500, "lintel": True, "bearing_mm": 190, "allow_custom": True,
                 "min_piece_mm": 50}, **kwargs)


def job(opening=None):
    return {"z_base": 0, "params": {}, "sections": [{"points": [[0, 0], [3000, 0]], "closed": False} for _ in range(22)],
            "openings": [window() if opening is None else opening]}


class OpeningTests(unittest.TestCase):
    def test_curved_opening_regression_generates_without_intrusion(self):
        fixture = Path(__file__).with_name("fixtures") / "curved_opening_job.json"
        r = run_opening_check(json.loads(fixture.read_text()), False)
        self.assertIsNone(r["fatal"])
        self.assertTrue(r["l0_pass"])
        self.assertEqual(len(r["bricks"]), 322)
        self.assertGreater(len(r["boundary_adjustments"]), 0)
        self.assertFalse(any("opening_boundary" in b["l0_fail"] for b in r["bricks"]))
        self.assertTrue(all(b["length"] >= 50 for b in r["bricks"] if b["kind"] == "custom"))

    def test_bad_inputs_and_multiple_openings_rejected(self):
        for value in ([window(), window(id="two")], [window(width_mm=float("nan"))], [window(lintel="yes")], [window(center_fraction=2)]):
            with self.subTest(value=value), self.assertRaises(ValueError): validate_openings(value)

    def test_preview_snaps_courses_without_editing_surface(self):
        m = NurbsModel.flat()
        before = m.control_points().copy()
        o, corners, perimeter = geometry(m, window(), {})
        self.assertEqual(o["effective_sill_mm"], 469)
        self.assertEqual(o["effective_head_mm"], 1139)
        self.assertEqual(corners.shape, (4, 3))
        self.assertGreater(len(perimeter), 20)
        np.testing.assert_array_equal(before, m.control_points())

    def test_tall_opening_fails_until_wall_tall_enough(self):
        with self.assertRaisesRegex(ValueError, "at least"):
            geometry(NurbsModel.flat(), window(height_mm=1200, sill_mm=800), {})
        geometry(NurbsModel.flat(height=2500), window(height_mm=1200, sill_mm=800), {})

    def test_full_and_half_preferred_over_custom(self):
        p = resolve_params({})
        pieces, gap = fit_interval(0, 600, 0, p)
        self.assertEqual([ln for _, ln in pieces], [190]*3)
        self.assertEqual(gap, 15)

    def test_custom_only_at_interval_ends_and_minimum_enforced(self):
        p = resolve_params({})
        pieces, gap = fit_interval(0, 280, 0, p, True, 50)
        self.assertAlmostEqual(pieces[0][0]-pieces[0][1]/2, 0)
        self.assertAlmostEqual(pieces[-1][0]+pieces[-1][1]/2, 280)
        self.assertTrue(all(50 <= ln <= 190 for _, ln in pieces))
        with self.assertRaises(ValueError): fit_interval(0, 280, 0, p, False, 50)

    def test_no_brick_in_opening_and_lintel_is_not_grounded(self):
        p = resolve_params({})
        w = OpeningWall([Path2([[0, 0], [3000, 0]], False) for _ in range(22)], 0, p, [window()])
        for b in w.bricks():
            if 7 <= b.k < 17:
                self.assertTrue(b.s+b.length/2 <= 1050+1e-6 or b.s-b.length/2 >= 1950-1e-6)
            self.assertFalse(w.check(b), (b.k, b.i, w.check(b)))
        lintels = [b for b in w.bricks() if w.kinds[b] == "lintel"]
        self.assertEqual(len(lintels), 1)
        lintel = lintels[0]
        self.assertGreater(lintel.k, 0)
        contacts = w.supports(lintel)
        self.assertTrue(any(c.s < 1050 for c, _, _ in contacts))
        self.assertTrue(any(c.s > 1950 for c, _, _ in contacts))

    def test_no_lintel_reports_unsupported_masonry(self):
        w = OpeningWall([Path2([[0, 0], [3000, 0]], False) for _ in range(22)], 0,
                        resolve_params({}), [window(lintel=False)])
        self.assertTrue(any("contact" in w.check(b) for b in w.bricks()))

    def test_concealed_lintel_preserves_all_head_course_masonry(self):
        with_lintel = run_opening_check(job(), False)
        # Compare placement before repair: unsupported no-lintel masonry may
        # subsequently move during optimization, which is now intentional.
        with patch("physics_check.pc.l0qp.l0_repair_qp", return_value=(False, None)):
            without = run_opening_check(job(window(lintel=False)), False)
        def masonry(r):
            return [(b["course"], b["length"], b["frame"]) for b in r["bricks"] if b["kind"] != "lintel"]
        self.assertEqual(masonry(with_lintel), masonry(without))
        from .wall_view import build_wall_meshes
        visible = dict(with_lintel, bricks=[b for b in with_lintel["bricks"] if not b["hidden"]])
        self.assertEqual(sum(m.n_cells for _, m in build_wall_meshes(with_lintel)),
                         sum(m.n_cells for _, m in build_wall_meshes(visible)))

    def test_concealed_support_transfers_load_to_both_jambs(self):
        j = job(); j["params"]["stage_check"] = False
        r = run_opening_check(j, True)
        self.assertIsNone(r["fatal"])
        self.assertTrue(r["ok"])
        self.assertEqual(r["equilibrium_method"], "rbe_explicit_opening_contacts")
        lintel = next(b for b in r["bricks"] if b["kind"] == "lintel")
        key = [lintel["course"], lintel["index"]]
        bearings = [i for i in r["interfaces"] if i["upper"] == key and i["P"] > 0]
        self.assertTrue(any(i["centroid"][0] < 1050 for i in bearings))
        self.assertTrue(any(i["centroid"][0] > 1950 for i in bearings))
        self.assertTrue(any(i["lower"] == key and i["P"] > 0 for i in r["interfaces"]))

    def test_result_contains_cut_schedule_and_assumptions(self):
        r = run_opening_check(job(), False)
        self.assertTrue(r["l0_pass"], r.get("fatal"))
        self.assertFalse(r["ok"])
        self.assertEqual(r["l1_state"], "skipped")
        self.assertIn("custom_piece_schedule", r)
        self.assertIn("capacity", r["opening_assumptions"].lower())

    def test_persistence_snapshot_and_freshness_hash(self):
        m = NurbsModel.flat()
        before = m.control_points().copy()
        old_hash = surface_hash(m)
        m.openings = [window()]
        self.assertNotEqual(old_hash, surface_hash(m))
        snap = clone_model(m)
        m.openings[0]["width_mm"] = 800
        self.assertEqual(snap.openings[0]["width_mm"], 900)
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/"window.3dm"
            m.save(p)
            loaded = NurbsModel.load(p)
            self.assertEqual(loaded.openings, m.openings)
            np.testing.assert_array_equal(loaded.control_points(), before)


if __name__ == "__main__": unittest.main()
