"""Headless smoke test for surface_editor (no display needed).

Drives the editor's callbacks directly, then verifies the saved .3dm.
Run from gui/:  ../.venv/Scripts/python -m surface_editor.smoke_test
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

from .editor import SurfaceEditor
from .model import NurbsModel
from . import refit, wall_view

ROOT = Path(__file__).resolve().parent.parent.parent  # optionA_test/
SURFACE = ROOT / "Reference Surface_1.3dm"
RESULT = ROOT / "results" / "optionA_v9_end_support_result.json"
GH_JOB = ROOT / "results" / "job_end_support_true.json"


def main() -> int:
    ed = SurfaceEditor(SURFACE, res=40, off_screen=True)
    grid0 = ed.grid.points.copy()
    cps0 = ed.cps.copy()
    failures: list[str] = []

    def check(name, cond):
        print(("PASS " if cond else "FAIL ") + name)
        if not cond:
            failures.append(name)

    # --- point ordering: no-op sync must not permute the mesh -------------
    ed._sync_scene()
    check("sync no-op: mesh unchanged", np.allclose(ed.grid.points, grid0))

    # --- direct drag of CP 0 -------------------------------------------
    w0 = ed.plotter.widgets.sphere_widgets[0]
    target = cps0.reshape(-1, 3)[0] + np.array([80.0, -40.0, 30.0])
    ed.on_drag(target, 0, w0)
    check("drag: CP0 moved", np.allclose(ed.cps.reshape(-1, 3)[0], target))
    check("drag: surface re-evaluated", not np.allclose(ed.grid.points, grid0))
    check("drag: undo snapshot pushed", len(ed.undo_stack) == 1)

    # --- undo ------------------------------------------------------------
    ed.undo()
    check("undo: CPs restored", np.allclose(ed.cps, cps0))
    check("undo: surface restored", np.allclose(ed.grid.points, grid0, atol=1e-6))
    check("undo: handles re-centered", np.allclose(np.array(w0.GetCenter()), cps0.reshape(-1, 3)[0]))

    # --- axis lock ---------------------------------------------------------
    ed.axis_lock = "z"
    orig = ed.cps[0, 0].copy()
    w = ed.plotter.widgets.sphere_widgets[0]
    ed.on_drag(orig + np.array([500.0, 500.0, 60.0]), 0, w)
    moved = ed.cps[0, 0]
    check("axis lock z: x/y frozen", np.isclose(moved[0], orig[0]) and np.isclose(moved[1], orig[1]))
    check("axis lock z: z applied", np.isclose(moved[2], orig[2] + 60.0))
    ed.axis_lock = None
    ed.undo()

    # --- soft selection -----------------------------------------------------
    ed.soft = 0.6
    before = ed.cps.copy()
    w = ed.plotter.widgets.sphere_widgets[7]
    dz = 100.0
    ed.on_drag(before.reshape(-1, 3)[7] + np.array([0, 0, dz]), 7, w)
    after = ed.cps
    i, j = divmod(7, ed.m)  # dragged CP (1, 1)
    d_adj = after[2, 1, 2] - before[2, 1, 2]    # distance 1
    d_far = after[0, 0, 2] - before[0, 0, 2]    # distance sqrt(2)
    d_farther = after[0, 5, 2] - before[0, 5, 2]  # distance sqrt(1+16)
    check("soft: adjacent CP follows (>10% of delta)", d_adj > 0.1 * dz)
    check("soft: falloff decays with distance", d_adj > d_far > d_farther)
    check("soft: far corner follows < 30% of delta", abs(d_far) < 0.3 * dz)
    ed.soft = 0.0
    ed.undo()

    # --- scale ---------------------------------------------------------------
    before = ed.cps.copy()
    ed._mk_scale_cb(0)(1.25)  # slider "scale X" -> 1.25
    after = ed.cps
    grew = (after[..., 0] - ed.pivot[0]) / (before[..., 0] - ed.pivot[0])
    check("scale X: CPs scaled about pivot", np.allclose(grew, 1.25))
    check("scale X: other axes untouched", np.allclose(after[..., 1:], before[..., 1:]))
    check("scale: handles follow", np.allclose(
        np.array(ed.plotter.widgets.sphere_widgets[0].GetCenter()), after.reshape(-1, 3)[0]))

    # --- save round-trip -------------------------------------------------------
    tmp = Path(tempfile.gettempdir()) / "se_smoke.3dm"
    out = ed.model.save(tmp, backup=False)
    m2 = NurbsModel.load(out)
    check("save: file re-readable", m2.shape == ed.cps.shape[:2])
    check("save: degree preserved", m2.degree == ed.model.degree)
    saved_cps = m2.control_points()
    check("save: edited CPs persisted", np.allclose(saved_cps, ed.cps, atol=1e-6))
    dom0, dom1 = m2._srf.Domain(0), m2._srf.Domain(1)
    check("save: domain preserved", np.isclose(dom0.T1, 4503.4) and np.isclose(dom1.T1, 2500.0))
    os.remove(tmp)

    # original file untouched?
    check("original file untouched", NurbsModel.load(SURFACE).control_points().shape == cps0.shape)

    # --- wall overlay -------------------------------------------------------
    result = wall_view.load_result(RESULT)
    meshes = dict(wall_view.build_wall_meshes(result))
    n_cells = sum(m.n_cells for m in meshes.values())
    check("wall: one mesh per status", set(meshes) == {"passed", "moved"})
    check("wall: 214 bricks", n_cells > 0 and sum(m.n_points for m in meshes.values()) > 0
          and len(result["bricks"]) == 214)
    zmid = np.concatenate([m.points for m in meshes.values()])
    check("wall: in mm scale near the surface",
          1200 < zmid[:, 2].max() < 1500 and -1 < zmid[:, 2].min() < 1)
    # half bricks present and shorter
    lens = sorted({b["length"] for b in result["bricks"]})
    check("wall: half bricks in data", lens == [95.0, 190.0])

    # editor integration: overlay + toggle
    ed2 = SurfaceEditor(SURFACE, res=24, off_screen=True, result_path=RESULT)
    check("editor: wall actors added", len(ed2.wall_actors) >= 2)
    check("editor: wall label set", "PASS" in ed2.wall_label)
    ed2.toggle_wall()
    check("editor: wall hidden", ed2.wall_visible is False)
    ed2.toggle_wall()
    ed2.plotter.close()

    # --- refit: section extraction vs GH ground truth ----------------------
    # NOTE: Reference Surface_1.3dm was edited after the GH jobs were cut
    # (mid-surface CP moved ~0.5 m), so only courses near the unedited base
    # can match. Course 0 must match GH to a fraction of a mm.
    m = NurbsModel.load(SURFACE)
    params = wall_view.load_result(RESULT)["params"]
    z_base, mine = refit.extract_sections(m, params)
    gh = json.load(open(GH_JOB))["sections"]  # noqa: SIM115
    check("refit: 23 sections", len(mine) == 23 and z_base == 0.0)
    check("refit: 201 points per section, open",
          len(mine[0]["points"]) == 201 and not mine[0]["closed"])
    a = np.array(mine[0]["points"]); b = np.array(gh[0]["points"])
    d0 = max(np.linalg.norm(b - p, axis=1).min() for p in a)
    check("refit: course 0 matches GH extraction (<0.1 mm)", d0 < 0.1)
    a = np.array(mine[22]["points"]); b = np.array(gh[22]["points"])
    d22 = max(np.linalg.norm(b - p, axis=1).min() for p in a)
    check("refit: course 22 close to GH (<3 mm, top edge barely edited)", d22 < 3.0)
    job = refit.build_job(m, params, use_cra=False)
    check("refit: job dict structure",
          set(job) == {"z_base", "params", "sections", "use_cra"} and not job["use_cra"])

    # regression: an edited surface (z no longer a function of v alone) must
    # still extract — this is the normal edit -> analyze workflow
    cps = m.control_points()
    cps[2, 3] += np.array([120.0, -200.0, 90.0])
    m.set_control_points(cps)
    z_base_e, mine_e = refit.extract_sections(m, params)
    check("refit: edited surface extracts 23 sections",
          len(mine_e) == 23 and z_base_e == 0.0)
    d_mid = np.abs(np.array(mine_e[12]["points"]) - np.array(mine[12]["points"])).max()
    check("refit: edited surface sections moved mid-wall", d_mid > 20.0)

    ed.plotter.close()
    print()
    print("ALL PASS" if not failures else f"FAILURES: {failures}")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
