"""Headless smoke test for surface_editor (no display needed).

Drives the editor's callbacks directly, then verifies the saved .3dm.
Run from gui/:  uv run --project .. python -m surface_editor.smoke_test
Wall/refit checks use the newest full analysis in results/ (skipped if none).
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
from .session import surface_hash
from . import refit, wall_view

ROOT = Path(__file__).resolve().parent.parent.parent  # Agent_Design_Tool/
SURFACE = ROOT / "Reference Surface_1.3dm"
GH_JOB = ROOT / "results" / "job_end_support_true.json"  # optional GH ground truth


def pick_result() -> tuple[Path | None, bool]:
    """Newest full analysis with bricks: (path, analysed on the unedited
    SURFACE?). Prefers a run of SURFACE itself, so course counts compare;
    skips failed fits and fast (geometry-only) checks."""
    reference = surface_hash(NurbsModel.load(SURFACE))
    full = []
    for p in (ROOT / "results").glob("**/*_result.json"):
        try:
            r = wall_view.load_result(p)
        except Exception:  # noqa: BLE001
            continue
        if r.get("bricks") and r.get("use_cra") and r.get("bricks_per_course"):
            same = (r.get("design_context") or {}).get("surface_hash") == reference
            full.append((same, p.stat().st_mtime, p))
    if not full:
        return None, False
    same, _t, path = max(full)
    return path, same


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

    # --- mortar slider -----------------------------------------------------
    # the slider always shows the bed joint of the wall on screen: the
    # auto-loaded result's mortar_t, or the 10 mm default with no wall
    expect = (float(wall_view.load_result(ed._wall_result_path)["params"]["mortar_t"])
              if ed._wall_result_path is not None else 10.0)
    check("mortar: slider starts from the shown wall's bed joint", ed.mortar_t == expect)
    ed._mortar_cb(6.0)
    check("mortar: slider sets mortar_t", ed.mortar_t == 6.0)
    params_a, _source = ed._refit_params()
    check("mortar: 'a' params carry the slider value", params_a.get("mortar_t") == 6.0)
    check("mortar: 'a' params keep the solver pick", params_a.get("solver") == ed.solver)
    ed.set_mortar_t(10.0)
    check("mortar: set_mortar_t moves the slider",
          ed.mortar_slider is not None
          and np.isclose(ed.mortar_slider.GetRepresentation().GetValue(), 10.0))
    n10, _s10 = refit.course_layout(1500.0, 57.0, 10.0)
    n25, _s25 = refit.course_layout(1500.0, 57.0, 25.0)
    check("mortar: thicker bed joint gives fewer courses", n10 == 22 and n25 == 18)

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

    ed.plotter.close()

    # --- wall overlay -------------------------------------------------------
    result_path, same_surface = pick_result()
    if result_path is None:
        print("SKIP wall + refit checks: no full analysis in results/ "
              "(run Full analysis or 'a' once, then re-run)")
        print()
        print("ALL PASS" if not failures else f"FAILURES: {failures}")
        return 0 if not failures else 1
    print(f"using {result_path.relative_to(ROOT)}"
          + ("" if same_surface else " (edited surface: course-count check skipped)"))
    result = wall_view.load_result(result_path)
    meshes = dict(wall_view.build_wall_meshes(result))
    n_cells = sum(m.n_cells for m in meshes.values())
    check("wall: meshes only for known statuses",
          meshes and set(meshes) <= set(wall_view.STATUS_COLORS))
    check("wall: one box (6 faces) per brick", n_cells == 6 * len(result["bricks"]))
    zmid = np.concatenate([m.points for m in meshes.values()])
    check("wall: in mm scale near the surface",
          1200 < zmid[:, 2].max() < 1450 and -1 < zmid[:, 2].min() < 60)
    # half bricks present and shorter
    lens = sorted({b["length"] for b in result["bricks"]})
    check("wall: half bricks in data", lens == [95.0, 190.0])

    # editor integration: overlay + toggle
    ed2 = SurfaceEditor(SURFACE, res=24, off_screen=True, result_path=result_path)
    check("editor: wall actors added", len(ed2.wall_actors) >= 1)
    check("editor: wall label set", "PASS" in ed2.wall_label)
    check("editor: mortar slider synced from result",
          np.isclose(ed2.mortar_t, float(result["params"]["mortar_t"])))
    ed2.toggle_wall()
    check("editor: wall hidden", ed2.wall_visible is False)
    ed2.toggle_wall()
    ed2.plotter.close()

    # --- refit: section extraction -----------------------------------------
    # Against the analysed wall itself (same surface version), and against
    # the GH extraction when that job file is still around.
    m = NurbsModel.load(SURFACE)
    params = result["params"]
    n_courses = len(result["bricks_per_course"])
    z_base, mine = refit.extract_sections(m, params)
    if same_surface:
        check("refit: section count matches the analysed wall",
              len(mine) == n_courses and z_base == 0.0)
    check("refit: 201 points per section, open",
          len(mine[0]["points"]) == 201 and not mine[0]["closed"])
    if GH_JOB.exists():
        # Cut with the GH job's own params (its mortar may differ from the
        # wall above), so both sides slice the surface at the same heights.
        gh_job = json.load(open(GH_JOB))  # noqa: SIM115
        gh = gh_job["sections"]
        _zb, mine_gh = refit.extract_sections(m, gh_job["params"])
        if len(gh) == len(mine_gh):
            for k in (0, len(gh) // 2, len(gh) - 1):
                a = np.array(mine_gh[k]["points"]); b = np.array(gh[k]["points"])
                d = max(np.linalg.norm(b - p, axis=1).min() for p in a)
                check(f"refit: course {k} matches GH extraction (<0.1 mm)", d < 0.1)
        else:
            print("SKIP refit: GH ground truth (course count differs: "
                  f"{GH_JOB.name} was cut from a different surface)")
    else:
        print(f"SKIP refit: GH ground truth ({GH_JOB.name} not in results/)")
    job = refit.build_job(m, params, use_cra=False)
    check("refit: job dict structure",
          set(job) == {"z_base", "params", "sections", "use_cra"} and not job["use_cra"])

    # regression: an edited surface (z no longer a function of v alone) must
    # still extract — this is the normal edit -> analyze workflow
    cps = m.control_points()
    cps[2, 3] += np.array([120.0, -200.0, 90.0])
    m.set_control_points(cps)
    z_base_e, mine_e = refit.extract_sections(m, params)
    check("refit: edited surface extracts the same course count",
          len(mine_e) == len(mine) and z_base_e == 0.0)
    mid = len(mine) // 2
    d_mid = np.abs(np.array(mine_e[mid]["points"]) - np.array(mine[mid]["points"])).max()
    check("refit: edited surface sections moved mid-wall", d_mid > 20.0)

    print()
    print("ALL PASS" if not failures else f"FAILURES: {failures}")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
