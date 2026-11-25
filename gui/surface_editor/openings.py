"""Surface-attached opening previews and validation shared by UI and AI tools."""
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from physics_check.pc.openings import validate_openings, effective_opening
from physics_check.pc.params import resolve_params
from . import refit


def section(model, height, samples=121):
    """Approximate horizontal section for UI only; solver does its own extraction."""
    surface = model._srf
    du, dv = surface.Domain(0), surface.Domain(1)
    us = np.linspace(du.T0, du.T1, samples)
    vs = np.linspace(dv.T0, dv.T1, 41)
    zcols = np.array([[surface.PointAt(u, v).Z for v in vs] for u in us])
    roots = [refit._column_crossings(model, u, vs, zcols[i], height) for i, u in enumerate(us)]
    if any(len(r) > 1 for r in roots):
        raise ValueError("Opening preview requires single-branch wall sections; folded walls are not supported yet")
    xy = refit._longest_branch(model, us, roots, .15*(dv.T1-dv.T0))
    if xy is None: raise ValueError("Opening boundary does not intersect the surface")
    distances = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    return xy, distances


def geometry(model, opening, params):
    p = resolve_params(params)
    bb = model._srf.GetBoundingBox()
    n, scale = refit.course_layout(bb.Max.Z-bb.Min.Z, p["brickH"], p["mortar_t"])
    o = effective_opening(validate_openings([opening])[0], p, n)
    rows = []
    for z in np.linspace(o["effective_sill_mm"], o["effective_head_mm"], 13):
        height = bb.Min.Z+z/scale
        xy, distances = section(model, height)
        lo = o["center_fraction"]*distances[-1]-o["width_mm"]/2
        hi = lo+o["width_mm"]
        clearance = o["bearing_mm"]+p["min_head"] if o["lintel"] else o["min_piece_mm"]
        if lo < clearance or hi+clearance > distances[-1]:
            raise ValueError("Opening leaves insufficient jamb or lintel bearing width")
        ss = np.linspace(lo, hi, 25)
        rows.append(np.c_[np.interp(ss, distances, xy[:, 0]), np.interp(ss, distances, xy[:, 1]), np.full(25, height)])
    rows = np.asarray(rows)
    corners = np.array([rows[0, 0], rows[0, -1], rows[-1, -1], rows[-1, 0]])
    perimeter = np.concatenate([rows[0], rows[1:, -1], rows[-1, -2::-1], rows[-2::-1, 0]])
    return o, corners, perimeter


def resized(model, opening, corner, point, params):
    """Project a corner handle onto the horizontal wall section, anchor opposite side."""
    p = resolve_params(params)
    bb = model._srf.GetBoundingBox()
    _, scale = refit.course_layout(bb.Max.Z-bb.Min.Z, p["brickH"], p["mortar_t"])
    z = float(np.clip(point[2], bb.Min.Z+.01, bb.Max.Z-.01))
    xy, distances = section(model, z)
    # Closest location on the section polyline.
    a, d = xy[:-1], np.diff(xy, axis=0)
    t = np.clip(np.sum((np.array(point[:2])-a)*d, axis=1)/np.maximum(np.sum(d*d, axis=1), 1e-12), 0, 1)
    near = a+t[:, None]*d
    i = int(np.argmin(np.linalg.norm(near-np.array(point[:2]), axis=1)))
    s = distances[i]+t[i]*(distances[i+1]-distances[i])
    center = opening["center_fraction"]*distances[-1]
    left, right = center-opening["width_mm"]/2, center+opening["width_mm"]/2
    if corner in (0, 3): left = min(s, right-50)
    else: right = max(s, left+50)
    bottom, top = opening["sill_mm"], opening["sill_mm"]+opening["height_mm"]
    h = (z-bb.Min.Z)*scale
    if corner in (0, 1): bottom = min(h, top-50)
    else: top = max(h, bottom+50)
    return {**opening, "center_fraction": (left+right)/2/distances[-1], "width_mm": right-left,
            "sill_mm": max(0, bottom), "height_mm": top-max(0, bottom)}
