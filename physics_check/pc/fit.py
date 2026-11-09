"""Step 2: offset and align each course section, and choose its brick count.

Open courses follow the baseline.gh layout (L = brickL, g = head_joint):

* Each course is trimmed at both ends: by L + g/2 on even courses, L + g on
  odd courses.  What is left is the course's middle curve.
* One middle-brick count m is shared by the whole wall:
  m = round(shortest middle curve / (L + g)).  baseline.gh rounds down
  (its N is 2 * m), which keeps every middle joint >= g but can leave them
  nearly a brick too wide; rounding to nearest keeps them closer to g, so
  on the shortest courses they can be somewhat below g.
* Even course: a full brick flush with each end, plus m bricks centred in m
  equal cells of the middle curve.  m + 2 bricks.
* Odd course: a half brick (L/2) flush with each end, plus m + 1 bricks on the
  cell boundaries of the middle curve, so they sit over the joints of the
  courses below.  m + 3 bricks.

The two trims make the end joints about g: on odd courses the joint next to
each half brick is exactly g; on even courses the joint next to each end
brick is g/2 + (cell - L)/2, i.e. about g when the cell is about L + g.
Longer courses keep the same m, so their middle joints are wider (no
max_head cap).

Closed courses keep the per-course count from ``course_count`` and are placed
evenly with the offset_distance running bond.

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


def end_trim(k, p):
    """Arc length cut from each end of open course k to leave its middle curve."""
    L, g = p["brickL"], p["head_joint"]
    return L + g if k % 2 == 1 else L + 0.5 * g


def middle_length(path, k, p):
    """Length of open course k's middle curve [mm]."""
    return path.length - 2.0 * end_trim(k, p)


def middle_count(paths, p):
    """Shared middle-brick count m for the open courses (baseline.gh's N / 2).

    Returns (m, k_short): m = shortest middle curve / (brickL + head_joint)
    rounded to the nearest whole number (baseline.gh rounds down), and the
    index of that shortest course; (None, None) if there are no open courses.
    """
    open_k = [k for k, path in enumerate(paths) if not path.closed]
    if not open_k:
        return None, None
    k_short = min(open_k, key=lambda k: middle_length(paths[k], k, p))
    m = int(math.floor(middle_length(paths[k_short], k_short, p)
                       / (p["brickL"] + p["head_joint"]) + 0.5))
    return m, k_short


def course_count(length, closed, p):
    """(bricks, head joint) for a closed course centreline of this length [mm].

    A course of n bricks spans n * (brickL + joint), so joint = length / n -
    brickL.  Picks the n whose joint is closest to head_joint within
    [min_head, max_head]; if no n gives a joint in that range, the largest n
    with joint >= min_head.  Returns (0, None) if fewer than three fit.
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
    the Z-scaled surface.  Returns ([Path2], [count], [middle head joint]),
    where count is the shared middle-brick count m on open courses (see the
    module docstring) and the total brick count on closed courses.
    """
    secs = [(offset_polyline(pts, closed, p["face_offset"]), closed) for pts, closed in sections]
    secs = align_sections(secs)
    paths = [Path2(pts, closed) for pts, closed in secs]

    m, k_short = middle_count(paths, p)
    if m is not None and m < 1:
        need = 2 * end_trim(k_short, p) + p["brickL"] + p["head_joint"]
        raise FitError(
            "course {}: centreline is {:.1f} mm, too short for the end bricks plus one "
            "middle brick ({:.1f} mm)".format(k_short, paths[k_short].length, need))

    counts, joints = [], []
    for k, path in enumerate(paths):
        if path.closed:
            n, j = course_count(path.length, True, p)
            if n == 0:
                raise FitError(
                    "course {}: centreline is {:.1f} mm, too short for 3 bricks of "
                    "brickL + min_head = {:.1f} mm".format(
                        k, path.length, p["brickL"] + p["min_head"]))
        else:
            n, j = m, middle_length(path, k, p) / m - p["brickL"]
        counts.append(n)
        joints.append(j)
    return paths, counts, joints
