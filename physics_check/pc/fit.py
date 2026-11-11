"""Step 2: offset and align each course section, and choose its brick count.

Open courses follow the baseline.gh layout (L = brickL, g = head_joint):

* Each course is trimmed at both ends: by L + g/2 on even courses, L + g on
  odd courses.  What is left is the course's middle curve.
* Each course gets its own middle-brick count m, so the middle joints
  (middle curve / m - L) stay near g on walls whose courses differ in
  length.  Even courses: the m whose joint is closest to g within
  [min_head, max_head] (``course_count``; if none is in range, the largest
  m with joint >= min_head).  Odd courses keep the m of the course below,
  so they stay bonded over its joints, unless their joint would then be
  under min_head or wider than both max_head and their own choice.
  (baseline.gh shares one m, from the shortest course, across the whole
  wall; on a wall that bulges, the longer courses then get joints wider
  than a brick.)
* Even course: a full brick flush with each end, plus m bricks centred in m
  equal cells of the middle curve.  m + 2 bricks.
* Odd course: a half brick (L/2) flush with each end, plus m + 1 bricks on the
  cell boundaries of the middle curve, so they sit over the joints of the
  course below when it has the same m.  m + 3 bricks.

Where m changes between two courses the upper course's joints no longer
line up with the middle of the bricks below; L0 then checks contact and
overhang there as anywhere else.  The two trims make the end joints about g:
on odd courses the joint next to each half brick is exactly g; on even
courses the joint next to each end brick is g/2 + (cell - L)/2, i.e. about
g when the cell is about L + g.

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


def middle_count(mid, k, below, p):
    """(m, middle head joint) for an open course with a middle curve of
    length mid [mm]; below = the middle-brick count of course k - 1 if that
    course is open, else None.

    Even courses: the count whose joint (mid / m - brickL) is closest to
    head_joint within [min_head, max_head] (course_count; if no count gives
    a joint in that range, the largest with joint >= min_head).  Odd courses
    keep the count of the course below, so their bricks stay on the joints
    below, unless that joint is under min_head or above both max_head and
    the joint of their own choice.  Returns (0, None) if not even one middle
    brick fits.
    """
    own = course_count(mid, False, p)
    if k % 2 == 1 and below:
        j = mid / below - p["brickL"]
        j_max = max(p["max_head"], own[1] if own[0] else p["max_head"])
        if p["min_head"] - 1e-6 <= j <= j_max + 1e-6:
            return below, j
    return own


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


def surface_lean(paths, max_overhang):
    """How far the surface itself leans, from the course centrelines.

    Returns (lean, need): lean[k] = the largest sideways distance from a
    point of course k's centreline to course k - 1's centreline, signed (+ =
    towards the +X side of course k - 1) [mm] (0 for course 0); need[k] = how
    far the bricks lag behind the surface at that worst point if every course
    steps at most max_overhang from the one below: the bricks step towards
    where the surface is, clamped to +/- max_overhang, so they catch up again
    where the surface leans back.  An estimate (worst points of different
    courses need not line up, and the QP spreads the deviation over the
    courses above and below): the bricks must depart from the surface about
    this much to pass L0.
    """
    lean, need = [0.0], [0.0]
    lag = 0.0
    for k in range(1, len(paths)):
        lo = paths[k - 1]
        lats = [lo.nearest(q)[1] for q in paths[k].pts]
        lean.append(max(lats, key=abs))
        want = lag + lean[-1]                      # where the bricks would need to step
        lag = want - max(-max_overhang, min(max_overhang, want))
        need.append(abs(lag))
    return lean, need


CLAMP_MARGIN = 20.0   # mm kept inside max_overhang / min_balance by clamp_paths
CLAMP_STEP = 10.0     # clamp_paths: point spacing of the built course lines [mm]


def _ray_hit(path, o, d):
    """Signed distance t of the nearest crossing of the line o + t d with the
    polyline path, or None."""
    best = None
    for a, b in path.segs:
        sx, sy = b[0] - a[0], b[1] - a[1]
        den = d[0] * sy - d[1] * sx
        if abs(den) < 1e-12:
            continue
        qx, qy = a[0] - o[0], a[1] - o[1]
        t = (qx * sy - qy * sx) / den
        u = (qx * d[1] - qy * d[0]) / den
        if -1e-9 <= u <= 1.0 + 1e-9 and (best is None or abs(t) < abs(best)):
            best = t
    return best


def _end_s(lo, q, end):
    """Arc position on lo (extrapolated past its ends) of the point q."""
    s, _lat = lo.nearest(q)
    f_s = 0.0 if end == 0 else lo.length
    if abs(s - f_s) > 1e-6:
        return s
    f_pt = lo.point(f_s)
    tx, ty = lo.tangent(f_s + (0.5 if end == 0 else -0.5), 0.5)
    return f_s + (q[0] - f_pt[0]) * tx + (q[1] - f_pt[1]) * ty


def clamp_paths(paths, p):
    """Step 2a: build each course line over the course below.

    Where the surface leans or flares further per course than a brick can
    overhang, bricks fitted to it would float.  Bottom up, course k is built
    along the (already built) line of course k - 1: at points every
    CLAMP_STEP mm along it, the surface course k is found along its normal,
    and the new line follows it there, but at most max_overhang -
    CLAMP_MARGIN to either side (smoothed over brickL / 2).  Where the
    surface stays within that, the line is the surface; where it leans
    further, the line is the course below offset by the limit, so the moves
    add up course by course (the surface lag of surface_lean) and give way
    where the surface leans back.  Built this way the line is as fair as the
    one below and cannot fold over itself, however far it is from the
    surface.  An open course spans the surface course's ends, but runs past
    the ends of the course below by no more than an end half brick allows.

    The bricks are then counted and placed on these lines, and the fitted
    position is on them.  Returns (paths, [largest sideways distance from the
    surface per course [mm]], [mm cut from the course's ends]).
    """
    lim = max(0.0, p["max_overhang"] - CLAMP_MARGIN)
    half = 0.5 * p["brickL"]
    allow = max(0.0, 0.25 * p["brickL"] - p["min_balance"] - CLAMP_MARGIN)
    out, moves, trims = [paths[0]], [0.0], [0.0]
    for k in range(1, len(paths)):
        lo, surf = out[k - 1], paths[k]
        closed = surf.closed and lo.closed
        if closed:
            s_a, s_b, cut = 0.0, lo.length, 0.0
        else:
            ea = _end_s(lo, surf.pts[0], 0)
            eb = _end_s(lo, surf.pts[-1], 1)
            s_a, s_b = max(ea, -allow), min(eb, lo.length + allow)
            cut = max(s_a - ea, eb - s_b, 0.0)
        n = max(2, int(math.ceil((s_b - s_a) / CLAMP_STEP)) + (0 if closed else 1))
        ss = [s_a + (s_b - s_a) * i / (n if closed else n - 1) for i in range(n)]
        base, nrm, off = [], [], []
        for s in ss:
            o = lo.point(s)
            tx, ty = lo.tangent(s, half)
            nv = (ty, -tx)
            hit = _ray_hit(surf, o, nv)
            if hit is None:
                hit = -surf.nearest(o)[1]       # no crossing: sideways distance instead
            base.append(o)
            nrm.append(nv)
            off.append(max(-lim, min(lim, hit)))
        win = int(round(0.25 * half / CLAMP_STEP))
        sm = []
        for i in range(n):
            js = [(i + j) % n if closed else min(max(i + j, 0), n - 1) for j in range(-win, win + 1)]
            sm.append(sum(off[j] for j in js) / len(js))
        pts = [(o[0] + e * nv[0], o[1] + e * nv[1]) for o, e, nv in zip(base, sm, nrm)]
        line = Path2(pts, closed)
        out.append(line)
        moves.append(max(abs(surf.nearest(q)[1]) for q in pts[::5]))
        trims.append(cut)
    return out, moves, trims


def fit_sections(sections, p):
    """Offset to the centreline, align, and choose each course's brick count.

    sections: [(pts [(x, y)], closed)] per course, bottom up, already cut from
    the Z-scaled surface.  Returns ([Path2], [count], [middle head joint]),
    where count is the course's middle-brick count m on open courses (see the
    module docstring) and the total brick count on closed courses.
    """
    secs = [(offset_polyline(pts, closed, p["face_offset"]), closed) for pts, closed in sections]
    secs = align_sections(secs)
    surface = [Path2(pts, closed) for pts, closed in secs]
    paths, moves, trims = clamp_paths(surface, p)

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
            below = counts[k - 1] if k > 0 and not paths[k - 1].closed else None
            n, j = middle_count(middle_length(path, k, p), k, below, p)
            if n == 0:
                need = 2 * end_trim(k, p) + p["brickL"] + p["min_head"]
                raise FitError(
                    "course {}: centreline is {:.1f} mm, too short for the end bricks plus one "
                    "middle brick ({:.1f} mm)".format(k, path.length, need))
        counts.append(n)
        joints.append(j)
    return paths, counts, joints, {"surface": surface, "moves": moves, "trims": trims}
