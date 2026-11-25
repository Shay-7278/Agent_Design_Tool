"""Opening-aware repair and original L1 solver integration regressions."""
import unittest
from unittest.mock import patch
from .test_openings import window, job
from physics_check.pc.openings import OpeningWall, run_opening_check
from physics_check.pc.geom2d import Path2
from physics_check.pc.params import resolve_params
from physics_check.pc.l0qp import l0_repair_qp
from physics_check.pc.l1 import l1_repair


class OpeningOptimizationTests(unittest.TestCase):
    def test_original_solver_choices_with_concealed_lintel(self):
        for solver in ("rbe", "cra_penalty"):
            with self.subTest(solver=solver):
                j = job(window(width_mm=200, sill_mm=0, height_mm=67))
                j["sections"] = [{"points": [[0, 0], [1000, 0]], "closed": False} for _ in range(3)]
                j["params"] = {"solver": solver, "stage_check": False, "cra_timeout": 60}
                r = run_opening_check(j, True)
                self.assertTrue(r["ok"], r["report"])
                self.assertEqual(r["equilibrium_method"], solver+"_explicit_opening_contacts")

    def test_l1_feedback_moves_masonry_and_keeps_qp_target(self):
        wall = OpeningWall([Path2([[0, 0], [2000, 0]], False) for _ in range(5)], 0,
                           resolve_params({}), [window(width_mm=400, sill_mm=67, height_mm=67)])
        b = wall.courses[3][4]; c = wall.supports(b)[0][0]
        x, y = wall.pose(b)[:2]
        interface = {"upper": b, "lower": c, "fails": True, "ratio": 3.,
                     "centroid": [x/1000,y/1000,0], "cop": [x/1000,(y+5)/1000,0],
                     "P": 100., "A": .0171, "sigma_t": 300000.}
        moved = l1_repair(wall, {"state": "solved", "interfaces": [interface]}, [])
        self.assertEqual(moved, 1)
        self.assertGreater(wall.deviation(b), 0)
        self.assertEqual((b.st,b.ut,b.rt), (b.s,b.u,b.r))
        self.assertTrue(wall.hard_constraints_ok())
        ok, fatal = l0_repair_qp(wall, [])
        self.assertTrue(ok)
        self.assertIsNone(fatal)

    def test_jamb_contacts_are_partitioned_and_support_not_grounded(self):
        wall = OpeningWall([Path2([[0, 0], [2000, 0]], False) for _ in range(5)], 0,
                           resolve_params({}), [window(width_mm=400, sill_mm=67, height_mm=67)])
        nodes = list(wall.bricks())
        plate = next(c for c in nodes if wall.kinds[c] == "lintel")
        self.assertFalse(wall.movable(plate))
        for b in wall.courses[2]:
            if b is plate: continue
            self.assertLessEqual(sum(a for _, a, _ in wall.supports(b)), b.length*wall.W+1e-5)
        contacts = wall.cra_contacts(nodes)
        plate_index = nodes.index(plate)
        self.assertTrue(any(up == plate_index for _, up, _ in contacts))
        self.assertTrue(any(lo == plate_index for lo, _, _ in contacts))
        b = wall.courses[1][0]
        self.assertIn("opening_boundary", wall.check(b, s=1000))

    def test_qp_repairs_displaced_masonry_without_closing_window(self):
        p = resolve_params({"qp_max_iter": 12, "max_rotation": 0, "max_drop": 10})
        wall = OpeningWall([Path2([[0, 0], [2000, 0]], False) for _ in range(5)], 0, p,
                           [window(width_mm=400, sill_mm=67, height_mm=67)])
        b = wall.courses[3][4]
        b.u = b.ut = 65
        self.assertTrue(wall.check(b))
        count = len(list(wall.bricks()))
        plate = next(c for c in wall.bricks() if wall.kinds[c] == "lintel")
        plate_pose = (plate.s, plate.u, plate.r)
        log = []
        ok, fatal = l0_repair_qp(wall, log)
        self.assertIsNone(fatal)
        self.assertTrue(ok, log)
        self.assertTrue(wall.hard_constraints_ok())
        self.assertEqual(len(list(wall.bricks())), count)
        self.assertEqual((plate.s, plate.u, plate.r), plate_pose)
        self.assertLess(b.u, 65)
        self.assertTrue(any("QP" in line for line in log))

    def test_l1_feedback_is_rechecked_and_final_iteration_is_not_moved(self):
        j = job(); j["params"].update(stage_check=False, max_iter=2)
        def failing(wall):
            b = wall.courses[-1][2]; c = wall.courses[-2][2]
            return {"state": "solved", "interfaces": [{"upper": b, "lower": c,
                "fails": True, "centroid": [0., 0., 0.], "sigma_t": 1., "P": 1.,
                "ratio": 2., "tensioned": True}]}
        with patch("physics_check.pc.cra.run_cra", side_effect=failing) as solve, \
             patch("physics_check.pc.l1.l1_repair", return_value=1) as repair:
            r = run_opening_check(j, True)
        self.assertEqual(solve.call_count, 2)
        self.assertEqual(repair.call_count, 1)
        self.assertFalse(r["ok"])
        self.assertEqual(r["iterations"], 2)
        self.assertEqual(r["optimization_history"][0]["l1_moves"], 1)
        self.assertEqual(r["optimization_history"][-1]["l1_moves"], 0)


if __name__ == "__main__": unittest.main()
