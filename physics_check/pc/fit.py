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


CLAMP_MARGIN = 5.0   # mm kept inside max_overhang / min_balance by clamp_course


def _interp_u(row, s, length, closed):
    """Sideways offset u of course row's bricks at arc position s, linear
    between brick centres (held at the end values on an open course)."""
    bs = sorted(row, key=lambda b: b.s)
    if len(bs) == 1:
        return bs[0].u
    if closed:
        for i, a in enumerate(bs):
            b = bs[(i + 1) % len(bs)]
            d = (b.s - a.s) % length or length
            t = (s - a.s) % length
            if t <= d:
                return a.u + (b.u - a.u) * t / d
        return bs[0].u
    if s <= bs[0].s:
        return bs[0].u
    for a, b in zip(bs, bs[1:]):
        if s <= b.s:
            d = b.s - a.s
            return a.u if d < 1e-9 else a.u + (b.u - a.u) * (s - a.s) / d
    return bs[-1].u


def clamp_course(wall, k):
    """Part of the fit (step 2, called by l0.settle_counts): start course k's
    bricks over the course below.

    The fitted position follows the surface; where the surface leans or
    flares further per course than a brick can overhang, the fitted bricks
    float (no contact with the course below), and the L0 repair has no slope
    to bring them back.  Course k - 1 must already be settled.

    * sideways: a brick whose centre lies more than max_overhang - margin
      from the centre line of the course below (through its brick centres)
      moves sideways until it does not;
    * along the course (end bricks of open courses): a brick whose centre
      lies past the end of the course below, less min_balance + margin, moves
      back along its course.

    Only the current position (s, u) moves, and only by what the limits
    require (the surface lag surface_lean estimates, bottom up).  The fitted
    position (s0, u0) and the QP target (st, ut) stay on the surface, so the
    repair keeps pulling towards it and the reported deviation still measures
    the departure from it.  (Clamping the target too was tried and left more
    bricks failing.)  Returns (bricks moved sideways, end bricks moved along,
    largest move [mm]).
    """
    if k == 0 or not wall.courses[k - 1]:
        return 0, 0, 0.0
    p = wall.p
    lim = max(0.0, p["max_overhang"] - CLAMP_MARGIN)
    lo, row_lo = wall.paths[k - 1], wall.courses[k - 1]
    worst, n_side, n_end = 0.0, 0, 0
    for b in wall.courses[k]:
        moved = 0.0
        for _ in range(2):              # course normals of k and k - 1 differ a little
            cx, cy, _tx, _ty, nx, ny = wall.pose(b)
            s_lo, lat = lo.nearest((cx, cy))
            e = lat - _interp_u(row_lo, s_lo, lo.length, lo.closed)
            excess = e - max(-lim, min(lim, e))
            if abs(excess) < 1e-3:
                break
            ltx, lty = lo.tangent(s_lo, 0.5 * wall.L)
            du = -excess * (lty * nx - ltx * ny)        # along course k - 1's +X normal
            b.u += du
            moved += abs(du)
        if moved > 1e-3:
            n_side += 1
            worst = max(worst, moved)
    if not (wall.paths[k].closed or lo.closed):
        ext_lo = min(c.s - 0.5 * c.length for c in row_lo)
        ext_hi = max(c.s + 0.5 * c.length for c in row_lo)
        need = p["min_balance"] + CLAMP_MARGIN
        bs = sorted(wall.courses[k], key=lambda b: b.s)
        for b in (bs[0], bs[-1]):
            s_lo = lo.nearest(wall.pose(b)[:2])[0]
            if s_lo < ext_lo + need:
                ds = ext_lo + need - s_lo
            elif s_lo > ext_hi - need:
                ds = ext_hi - need - s_lo
            else:
                continue
            b.s = max(0.0, min(wall.paths[k].length, b.s + ds))
            n_end += 1
            worst = max(worst, abs(ds))
    return n_side, n_end, worst


def fit_sections(sections, p):
    """Offset to the centreline, align, and choose each course's brick count.

    sections: [(pts [(x, y)], closed)] per course, bottom up, already cut from
    the Z-scaled surface.  Returns ([Path2], [count], [middle head joint]),
    where count is the course's middle-brick count m on open courses (see the
    module docstring) and the total brick count on closed courses.
    """
    secs = [(offset_polyline(pts, closed, p["face_offset"]), closed) for pts, closed in sections]
    secs = align_sections(secs)
    paths = [Path2(pts, closed) for pts, closed in secs]

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
    return paths, counts, joints
