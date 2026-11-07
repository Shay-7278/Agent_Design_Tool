"""Step 2: offset, align and stretch each course section in Y.

The Z scaling and the section cuts need Rhino and live in
``physics_check_gh.py``; this module gets the cut sections as plan polylines.
"""

from __future__ import division

import math

from .geom2d import Path2, offset_polyline, poly_signed_area


class FitError(Exception):
    pass


def _stretched_length(pts, closed, sf, yc):
    n = len(pts)
    total = 0.0
    last = n if closed else n - 1
    for i in range(last):
        a, b = pts[i], pts[(i + 1) % n]
        total += math.hypot(b[0] - a[0], sf * (b[1] - a[1]))
    return total


def stretch_factor(pts, closed, target, yc):
    """Bisect the Y stretch factor so the section length equals target."""
    lmin = _stretched_length(pts, closed, 0.0, yc)
    if target < lmin - 1e-6:
        return None, lmin
    hi = 1.0
    while _stretched_length(pts, closed, hi, yc) < target:
        hi *= 2.0
        if hi > 1e6:
            return None, lmin
    lo = 0.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if _stretched_length(pts, closed, mid, yc) < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi), lmin


def _insert_nearest(pts, closed, p):
    """Index of p's nearest point on the polyline, inserting it as a vertex."""
    path = Path2(pts, closed)
    s, _ = path.nearest(p)
    q = path.point(s)
    i, _t = path._locate(s)
    pts = list(path.pts)
    pts.insert(i + 1, q)
    return pts, i + 1


def align_sections(sections):
    """Give every course the same start and direction as course 0.

    Contours of each course can start at different seams or run in opposite
    directions; the running bond needs them aligned.
    """
    pts0, closed0 = sections[0]
    start = pts0[0]
    area0 = poly_signed_area(pts0) if closed0 else None
    out = [(list(pts0), closed0)]
    for pts, closed in sections[1:]:
        pts = list(pts)
        if closed:
            if area0 is not None and (poly_signed_area(pts) > 0) != (area0 > 0):
                pts = pts[::-1]
            pts, j = _insert_nearest(pts, True, start)
            pts = pts[j:] + pts[:j]
        else:
            d0 = math.hypot(pts[0][0] - start[0], pts[0][1] - start[1])
            d1 = math.hypot(pts[-1][0] - start[0], pts[-1][1] - start[1])
            if d1 < d0:
                pts = pts[::-1]
        out.append((pts, closed))
    return out


def fit_sections(sections, p):
    """Offset to the centreline, align, and stretch each course in Y.

    sections: [(pts [(x, y)], closed)] per course, bottom up, already cut from
    the Z-scaled surface.  Returns ([Path2], [stretch factor]).
    """
    secs = [(offset_polyline(pts, closed, p["face_offset"]), closed) for pts, closed in sections]
    secs = align_sections(secs)
    ys = [pt[1] for pts, _c in secs for pt in pts]
    yc = 0.5 * (min(ys) + max(ys))
    target = p["N"] * (p["brickL"] + p["head_joint"])
    paths, factors = [], []
    for k, (pts, closed) in enumerate(secs):
        sf, lmin = stretch_factor(pts, closed, target, yc)
        if sf is None:
            raise FitError(
                "course {}: N*(brickL+head_joint) = {:.1f} mm is below the section's "
                "minimum length {:.1f} mm (its X travel); no stretch fits".format(k, target, lmin))
        fitted = [(x, yc + sf * (y - yc)) for x, y in pts]
        paths.append(Path2(fitted, closed))
        factors.append(sf)
    return paths, factors
