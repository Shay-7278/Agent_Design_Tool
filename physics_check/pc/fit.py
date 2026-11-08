"""Step 2: offset and align each course section, and choose its brick count.

The sections are not stretched: each course keeps the plan of the surface and
gets as many bricks as fit its centreline length, with the head joint closest
to head_joint inside [min_head, max_head].

The Z scaling and the section cuts need Rhino and live in
``physics_check_gh.py``; this module gets the cut sections as plan polylines.
"""

from __future__ import division

import math

from .geom2d import Path2, offset_polyline, poly_signed_area


class FitError(Exception):
    pass


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


def course_count(length, closed, p):
    """(bricks, head joint) for a course centreline of this length [mm].

    A course of n bricks spans n * (brickL + joint), half a joint at each end
    of an open course, so joint = length / n - brickL.  Picks the n whose
    joint is closest to head_joint within [min_head, max_head]; if no n gives
    a joint in that range, the largest n with joint >= min_head.  Returns
    (0, None) if not even one brick fits (three on a closed course).
    """
    L, hj = p["brickL"], p["head_joint"]
    n_max = int(math.floor(length / (L + p["min_head"]) + 1e-9))
    n_min = int(math.ceil(length / (L + p["max_head"]) - 1e-9))
    if n_max < (3 if closed else 1):
        return 0, None
    n_nom = int(round(length / (L + hj)))
    n = n_max if n_min > n_max else max(n_min, min(n_max, n_nom))
    n = max(n, 1)
    return n, length / n - L


def fit_sections(sections, p):
    """Offset to the centreline, align, and choose each course's brick count.

    sections: [(pts [(x, y)], closed)] per course, bottom up, already cut from
    the Z-scaled surface.  Returns ([Path2], [bricks], [head joint]).
    """
    secs = [(offset_polyline(pts, closed, p["face_offset"]), closed) for pts, closed in sections]
    secs = align_sections(secs)
    paths, counts, joints = [], [], []
    for k, (pts, closed) in enumerate(secs):
        path = Path2(pts, closed)
        n, j = course_count(path.length, closed, p)
        if n == 0:
            raise FitError(
                "course {}: centreline is {:.1f} mm, too short for {} brick(s) of "
                "brickL + min_head = {:.1f} mm".format(
                    k, path.length, 3 if closed else 1, p["brickL"] + p["min_head"]))
        paths.append(path)
        counts.append(n)
        joints.append(j)
    return paths, counts, joints
