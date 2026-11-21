# Handoff

## Objective
- Build a GUI that lets a user edit a NURBS surface by pushing/pulling its control points (like Rhino's `PointsOn` control point editing) and by scaling (stretch/shrink) the surface, working against the reference surface `.3dm` (now: `Reference Surface_1.3dm` in `optionA_test/`).

## Important Details
- Key math: NURBS is linear in its control points → moving one CP only affects `degree+1` surrounding spans (local curvature edit); scaling all CPs about a pivot scales the surface exactly (affine map on CPs = same map on surface).
- Architecture (as built): (1) geometry core via `rhino3dm` — load/save `.3dm` without Rhino, CP grid as `(n, m, 3)` numpy array, tessellate via `PointAt(u,v)` re-evaluated each frame (~20 ms at 60×60); (2) interaction layer via PyVista sphere widgets; (3) save back to `.3dm` so the existing Grasshopper/`physics_check_gh.py` pipeline consumes the edited surface unchanged.
- **rhino3dm 8.35 API quirks discovered and worked around** (verified experimentally):
  - `NurbsSurfacePointList` has no named setter; use `points[(i, j)] = rhino3dm.Point4d(x, y, z, w)`.
  - `BrepFace.ToNurbsSurface()` returns a *detached copy* — CP edits don't propagate to the Brep. On save, rebuild via `Brep.CreateFromSurface(edited_copy)` (degree/knots/domains preserved, verified), then `file.Objects.Delete(old_attributes.Id)` + `file.Objects.AddBrep(new_brep, old_attrs)`.
  - `File3dmObject` has no `.Id`; the GUID lives on `obj.Attributes.Id`.
  - `NurbsSurface.Encode()` returns a base64 blob, NOT friendly JSON — don't bother with encode/decode editing.
- **PyVista 0.49 API notes**: `add_sphere_widget(callback, center=<(k,3) array>)` calls `callback(point, index, widget)` when multiple centers + `pass_widget=True`; use `interaction_event='always'` for live drag (default is 'end' = update only on release); sliders take `interaction_event` (not `event_type`); move handles programmatically via `plotter.widgets.sphere_widgets[i].SetCenter(...)`; the old `plotter.sphere_widgets` attr is deprecated.
- Environment: Windows, Python 3.12.16 in `optionA_test/.venv`, uv-managed. `uv add pyvista rhino3dm` was run (pyvista 0.49.2, rhino3dm 8.35.0, vtk 9.7.2 added to `pyproject.toml`/`uv.lock`).

## Work State
### Completed
- Added `pyvista` + `rhino3dm` deps via uv.
- Built `gui/surface_editor/` package:
  - `model.py` — `NurbsModel`: load / `control_points()` / `set_control_points()` / `evaluate()` / `scale()` / `save()` (rebuilds Brep, replaces object by GUID, writes `.3dm` v7; one-time `<name>.3dm.orig` backup before first overwrite).
  - `editor.py` — `SurfaceEditor` (PyVista): shaded StructuredGrid surface + control polygon + draggable sphere handles (one per CP), scale-X/Y/Z sliders about CP centroid, "soft pull" Gaussian-falloff slider, axis-lock keys (`x`/`y`/`z`, `g` free), `u` undo (64-deep CP snapshots), `b` reset, `s` save, `r` reset view, `q` quit. Handles re-snapshot undo once per drag gesture (time-gap heuristic).
  - `wall_view.py` — fitted brick wall overlay from a physics_check result JSON: one box per brick (frame [origin, xaxis, yaxis] in mm; X = brickW, Y = brick length, Z = brickH via cross product), one mesh per status (passed/moved/flagged/failed), red sphere markers at tensioned/failing interfaces, newest `*_result.json` auto-detected in `<root>/results`. Key `v` toggles the wall. CLI: `python -m surface_editor [file.3dm] [-r result.json]`.
  - `refit.py` — re-fit the wall to the CURRENT surface without Rhino: replicates `physics_check_gh.py` steps 1-2 in Python. Section extraction is a general iso-z contour: exact nurbs bbox → course count / Z-scale (same math as `course_layout`); per course cut height (scaled height / z_scale on the original), per u-column crossings found by bracket scan + bisection, linked into branches, longest branch picked (mimics GH's longest-contour rule) and resampled to 201 points by arc length. Works on any edited surface (z varying along u, mild folds). `build_job()` (GUI thread, ~2 s) + `run_solver()` (blocking subprocess: `[sys.executable] physics_check/physics_check.py run job.json result.json`; exit 1 = checks failed is fine). Key `a` runs it: worker thread + 500 ms timer swaps in the new wall. Writes `results/gui_refit_job.json` / `gui_refit_result.json`.
  - `smoke_test.py` — headless test driving all callbacks: drag, undo, axis lock, soft falloff, scale, save round-trip (degree/domain/CPs preserved), original file untouched, wall overlay, refit extraction vs GH ground truth, edited-surface extraction regression. **35/35 PASS.**
- Verified against `Reference Surface_1.3dm`: single Brep, 1 face, degree 3×3, 6×6 CPs, non-rational, domain U 0–4503.4 / V 0–2500.
- Verified wall data against `results/optionA_v9_end_support_result.json`: 214 bricks (22 courses + base), z 0–1311 mm, statuses {passed, moved}, lengths {95, 190}.
- **Validated refit end-to-end**: my Python section extraction matches the GH-extracted `job_end_support_true.json` to 0.017 mm (course 0, same surface version). Full solve on the current surface with v9 params: 84.9 s, RESULT PASS, 210 bricks (60 moved). No-CRA solve: 17.5 s.
- **Important discovery**: `Reference Surface_1.3dm` was edited after the GH jobs/v9 analysis were made — its CP grid differs from `Reference Surface_1.3dmbak` by up to 542 mm (CP (2,3), mid-surface). Mid-course sections differ from the GH jobs by up to ~160 mm; that is a surface-version difference, not an extraction error. The v9 wall overlay was therefore stale before the first refit; `a` now re-fits against the live surface.
- **Bug found + fixed (user report "refit failed in gui")**: the first extractor required z to be a function of v alone — true only for the untouched surface, so ANY CP z-edit broke `a`. Replaced with general per-column iso-z contouring (bracket + bisection + branch linking). End-to-end verified on an edited surface: drag CP → `a` → 58 s solve → PASS, 212 bricks, wall updates.
- Launched GUI: `python -m surface_editor` from `gui/` (defaults to `../Reference Surface_1.3dm`).

### Active
- GUI relaunched with wall overlay + `a` re-fit; user testing the edit → analyze → wall-update loop.

### Blocked
- The refit's `a` key runs with `use_cra=True` and the params of whatever result the wall was loaded from (default: newest in `results/`). Solver runtime varies with the surface: 58–85 s on edited surfaces (many bricks moved, 44 s in L1) vs ~10-22 s on the unedited v9 one — status line shows progress, GUI stays responsive (worker thread). If too slow for iteration, add a fast-mode key (use_cra=False / `--no-cra` ≈ 17 s, L0 only).
- Section extraction caveats: course count uses the exact nurbs bbox (same as GH); iso-z curve endpoints on heavily-truncated courses can be off by up to one u-column (~11 mm) vs Rhino's exact contour; multiple-folded columns pick the lowest branch per link tolerance. The Rhino/GH pipeline remains the exact path for final checks.
- Surface is single-face only (loader raises otherwise); `.rhl` lock file suggests Rhino may hold the file open — save ('s') writes a `.orig` backup first, but the user should close Rhino if the original is locked.

## Next Move
1. User tries the loop: drag CPs → `a` → watch status line (~1 min) → wall updates to the new fit, colored by status.
2. If 85 s feels slow for iteration, expose a "fast mode" (use_cra=False / solver "rbe" only) on a second key — ~17 s per loop.
3. Optional niceties not yet built: highlight affected spans during drag, numeric CP coordinate readout, multi-CP box select, wall recolor by head-joint width or per-interface stress, keep a history of refit results to diff iterations.

## Relevant Files
- `optionA_test/Reference Surface_1.3dm`: input surface (with `.rhl` lock file + `.3dmbak` alongside).
- `optionA_test/pyproject.toml` / `uv.lock`: now include `pyvista`, `rhino3dm`.
- `optionA_test/gui/surface_editor/model.py`: geometry core.
- `optionA_test/gui/surface_editor/editor.py`: PyVista GUI + `main()` CLI (`python -m surface_editor [file.3dm] [-r result.json]`).
- `optionA_test/gui/surface_editor/wall_view.py`: brick wall overlay from result JSON.
- `optionA_test/gui/surface_editor/smoke_test.py`: headless regression test (28 checks).
- `optionA_test/results/optionA_v9_end_support_result.json`: latest analysis result (214 brick frames + statuses + interfaces) shown as the wall overlay.
- `optionA_test/physics_check/physics_check_gh.py`: downstream GH consumer of the edited `.3dm`; also where section extraction for new analyses currently lives.
