"""Overlay the fitted brick wall from a physics_check result JSON.

The result written by ``physics_check.py run`` already contains every brick
as a frame [origin, xaxis, yaxis] in mm (X = across the wall / brickW,
Y = along the course / brickL, Z = up), the brick's own length (half bricks
at open-course ends are brickL / 2) and its status. This module turns that
into pyvista meshes:

- one box per brick, grouped into one mesh per status so the wall can be
  colored by status (passed / moved / flagged / failed);
- one mesh of markers at interfaces that need tension or fail the stress
  check.

The wall is a *snapshot* of the last analysis, in the same coordinate
system as the surface it was fitted to -- it does not follow live CP edits.
That is the point: after you drag the surface, the offset between it and
the wall shows how far the design has drifted from the last checked fit.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pyvista as pv

STATUS_COLORS = {
    "passed": "peru",
    "moved": "orange",
    "flagged": "red",
    "failed": "darkred",
}
MARKER_COLOR = "red"


def load_result(path: str | Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _brick_mesh(result: dict, brick: dict) -> pv.PolyData:
    origin = np.asarray(brick["frame"][0], dtype=float)
    xax = np.asarray(brick["frame"][1], dtype=float)
    yax = np.asarray(brick["frame"][2], dtype=float)
    zax = np.cross(xax, yax)
    w = float(result["brickW"])
    length = float(brick.get("length", result["brickL"]))
    h = float(result["brickH"])
    box = pv.Box((-w / 2, w / 2, -length / 2, length / 2, -h / 2, h / 2))
    t = np.eye(4)
    t[:3, :3] = np.column_stack([xax, yax, zax])
    t[:3, 3] = origin
    box.transform(t, inplace=True)
    return box


def build_wall_meshes(result: dict) -> list[tuple[str, pv.PolyData]]:
    """One (status, mesh) pair per brick status present in the result."""
    groups: dict[str, list[pv.PolyData]] = {}
    for brick in result.get("bricks", []):
        groups.setdefault(brick["status"], []).append(_brick_mesh(result, brick))
    return [(status, pv.merge(parts)) for status, parts in groups.items()]


def interface_markers(result: dict) -> pv.PolyData | None:
    """Points at interfaces that need tension or fail the stress check."""
    ints = [
        i
        for i in result.get("interfaces", [])
        if i.get("tensioned") or i.get("fails")
    ]
    if not ints:
        return None
    pts = np.array([i["centroid"] for i in ints], dtype=float)
    return pv.PolyData(pts)


def result_summary(result: dict, path: str | Path) -> str:
    n = len(result.get("bricks", []))
    if result.get("use_cra") is False:
        state = "GEOMETRY PASS" if result.get("l0_pass") else "GEOMETRY FAIL"
        return f"{Path(path).stem}: {n} bricks, {state} (equilibrium not checked)"
    ok = result.get("ok")
    state = "PASS" if ok else "FAIL"
    l1 = result.get("l1_state", "?")
    return f"{Path(path).stem}: {n} bricks, {state} (L1 {l1})"


def find_latest_result(root: Path) -> Path | None:
    """Newst *_result.json under <root>/results (or root itself)."""
    candidates: list[Path] = []
    for base in (root / "results", root):
        if base.is_dir():
            candidates += [p for p in base.glob("*_result.json") if p.is_file()]
            candidates += [p for p in base.glob("ai_*/*_result.json") if p.is_file()]
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)
