"""Re-fit the brick wall to the live surface from inside the GUI, no Rhino.

Replicates the Rhino side of ``physics_check_gh.py`` (steps 1-2 of the
workflow) in plain Python:

- course count / Z scale from the surface height (``course_layout`` math);
- one section per course at its mid-height. Because this workflow builds
  with horizontal bed joints, the surface's z depends on the v parameter
  only, so a horizontal contour is an exact iso-v curve: find v by
  bisection on the nurbs, sample the iso-curve, resample by arc length.
  (If a future surface breaks that assumption -- z varying along u, or
  non-monotonic in v -- we refuse and point back at the Rhino pipeline
  instead of silently producing wrong sections.)

The solver itself is the existing ``physics_check.py run job.json
result.json`` called as a subprocess with this process's interpreter
(the project venv already has compas_cra). A full run costs what the
solver costs (~20 s); section extraction adds well under a second.
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from .model import NurbsModel

SOLVER_REL = Path("physics_check") / "physics_check.py"


# ------------------------------------------------------------- course layout
def course_layout(h: float, brick_h: float, mortar_t: float) -> tuple[int, float]:
    """(n_courses, z_scale): same math as physics_check_gh.course_layout."""
    pitch = brick_h + mortar_t
    n = int(math.floor((h + mortar_t) / pitch + 1e-9))
    if n < 2:
        raise ValueError(f"surface too short: {h:.1f} mm gives {n} course(s)")
    target = n * pitch - mortar_t
    return n, target / h


# --------------------------------------------------------------- extraction
def _bisect_v(model: NurbsModel, u: float, lo: float, hi: float, flo: float,
              z_target: float, iters: int = 16) -> float:
    """Refine the bracket [lo, hi] (f(lo) = flo) to the root of z(u, v) = z_target."""
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        fm = model._srf.PointAt(u, mid).Z - z_target
        if (fm > 0) == (flo > 0):
            lo, flo = mid, fm
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _column_crossings(model: NurbsModel, u: float, vs: np.ndarray, z_col: np.ndarray,
                      z_target: float, iters: int = 16) -> list[float]:
    """All v where z(u, v) == z_target (bisection-refined), ascending."""
    out: list[float] = []
    s = z_col - z_target
    sign = np.sign(s)
    for k in np.nonzero(sign[:-1] * sign[1:] < 0)[0]:  # strict sign change
        out.append(_bisect_v(model, u, vs[k], vs[k + 1], s[k], z_target, iters))
    for k in np.nonzero(s == 0.0)[0]:  # sample exactly on the cut height
        out.append(float(vs[k]))
    return sorted(out)


def _longest_branch(model: NurbsModel, us: np.ndarray,
                    crossings: list[list[float]], gap_tol: float) -> np.ndarray | None:
    """(n, 2) xy polyline of the longest connected iso-z branch.

    Mimics GH's "pick the longest contour curve": crossings are linked from
    column to column by nearest v; a column without crossings ends every
    open chain; the longest chain by chord length wins.
    """
    done: list[list[tuple[float, float, float]]] = []  # (u, v) chains
    active: list[list[tuple[float, float, float]]] = []
    for i, u in enumerate(us):
        cr = crossings[i]
        if not cr:
            done.extend(active)
            active = []
            continue
        used: set[int] = set()
        still: list[list[tuple[float, float, float]]] = []
        for ch in active:
            j_best, d_best = None, gap_tol
            for j, v in enumerate(cr):
                d = abs(v - ch[-1][1])
                if d < d_best:
                    j_best, d_best = j, d
            if j_best is not None:
                used.add(j_best)
                ch.append((u, cr[j_best]))
                still.append(ch)
        for j, v in enumerate(cr):
            if j not in used:
                still.append([(u, v)])
        active = still
    done.extend(active)

    best: np.ndarray | None = None
    best_len = 0.0
    for ch in done:
        if len(ch) < 2:
            continue
        pts = np.array([[model._srf.PointAt(u, v).X, model._srf.PointAt(u, v).Y]
                        for u, v in ch])
        length = np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()
        if length > best_len:
            best, best_len = pts, length
    return best


def extract_sections(
    model: NurbsModel, params: dict, samples: int = 200, n_cols: int = 400,
    n_bracket: int = 32,
) -> tuple[float, list[dict]]:
    """(z_base, sections) exactly as the GH component writes them.

    Works on any edited surface: for each course cut height, the iso-z curve
    is found per u-column (bracket from a coarse z scan, then bisection),
    linked into branches, and the longest branch is resampled by arc length,
    like GH's CreateContourCurves + longest-curve pick.
    """
    brick_h = float(params.get("brickH", 57.0))
    mortar_t = float(params.get("mortar_t", 10.0))
    bb = model._srf.GetBoundingBox()
    z_min, z_max = bb.Min.Z, bb.Max.Z
    z_base = z_min
    n_courses, z_scale = course_layout(z_max - z_min, brick_h, mortar_t)
    pitch = brick_h + mortar_t

    du, dv = model._srf.Domain(0), model._srf.Domain(1)
    us = np.linspace(du.T0, du.T1, n_cols)
    vs = np.linspace(dv.T0, dv.T1, n_bracket)
    zcols = np.empty((n_cols, n_bracket))
    for i, u in enumerate(us):
        for j, v in enumerate(vs):
            zcols[i, j] = model._srf.PointAt(u, v).Z

    sections = []
    for k in range(n_courses):
        # GH scales the surface in Z, then cuts at k*pitch + brickH/2; cutting
        # the original at (k*pitch + brickH/2)/z_scale is the same curve.
        z_target = z_base + (k * pitch + 0.5 * brick_h) / z_scale
        crossings = [
            _column_crossings(model, u, vs, zcols[i], z_target)
            for i, u in enumerate(us)
        ]
        pts = _longest_branch(model, us, crossings, gap_tol=0.1 * (dv.T1 - dv.T0))
        if pts is None:
            raise ValueError(
                f"course {k}: the surface has no section at z = {z_target:.1f}"
            )
        closed = bool(np.linalg.norm(pts[0] - pts[-1]) < 1e-6)
        poly = _resample_arclength(pts, samples + 1)
        sections.append(
            {
                "points": [[float(x), float(y)] for x, y in poly],
                "closed": closed,
            }
        )
    return z_base, sections


def _resample_arclength(pts: np.ndarray, n: int) -> np.ndarray:
    """n points evenly spaced (by chord length) along the polyline, ends kept."""
    seg = np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 0:
        raise ValueError("degenerate section curve")
    s /= s[-1]
    t = np.linspace(0.0, 1.0, n)
    return np.column_stack([np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])])


# ------------------------------------------------------------------- solver
def build_job(model: NurbsModel, params: dict, use_cra: bool = True) -> dict:
    """Full job dict (section extraction included). Call on the GUI thread."""
    z_base, sections = extract_sections(model, params)
    return {"z_base": z_base, "params": params, "sections": sections, "use_cra": use_cra}


def run_solver(
    job: dict, workdir: str | Path, use_cra: bool = True, timeout: float = 600.0
) -> tuple[Path, float]:
    """Write the job, run physics_check.py on it, return (result_path, seconds).

    Blocking; call from a worker thread. Exit code 1 (checks failed) is fine:
    a result file is still written. Raises only if no result appears.
    """
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    job_path = workdir / "gui_refit_job.json"
    result_path = workdir / "gui_refit_result.json"
    with open(job_path, "w", encoding="utf-8") as fh:
        json.dump(job, fh)

    root = Path(__file__).resolve().parent.parent.parent
    cmd = [sys.executable, str(root / SOLVER_REL), "run", str(job_path), str(result_path)]
    if not use_cra:
        cmd.append("--no-cra")

    t0 = time.perf_counter()
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    elapsed = time.perf_counter() - t0
    if not result_path.exists():
        tail = (proc.stdout or "")[-2000:] + (proc.stderr or "")[-2000:]
        raise RuntimeError(f"solver wrote no result (exit {proc.returncode}):\n{tail}")
    return result_path, elapsed
