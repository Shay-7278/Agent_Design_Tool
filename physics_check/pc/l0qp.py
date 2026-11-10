"""Step 4, l0_method = "qp": L0 repair as a sequential quadratic program.

All bricks move together to the layout closest to their targets (least
squares over the plan moves) that passes L0, instead of one brick at a time.

Variables: per brick, ds (along its course) and du (along its X axis, the
overhang direction).  Objective: sum of (s - target_s)^2 + (u - target_u)^2;
the target is the fitted position, or where L1 last moved the brick, so L0
does not undo an L1 repair.  Course 0 (the base) does not move; the end
bricks of open courses are pinned along the course (ds = 0) so the wall
ends follow the surface edge.

Constraints, linearised at the current layout (value + slope . move >= limit):
* contact:   bed overlap with the course below >= min_contact * bed area.
             Each pair's overlap is modelled as (length overlap) x (width
             overlap) in the upper brick's axes; bricks below that do not
             touch yet but could within the trust radius are included, so a
             brick can slide onto a new support.
* overhang:  |offset from the area-weighted support centre| <= max_overhang,
             with the support weights held fixed.
* balance:   the brick's centre lies at least min_balance inside its contact
             region with the course below, at each of its four ends along
             the brick's axes (wall.balance); an end set by a brick below
             moves with that brick.
* collision: clear gap to each nearby brick of the same course >= min_head,
             moving along the line between the two centres.
* limits:    every move <= the trust radius R; |s - fitted s|,
             |u - fitted u| <= max_move only if max_move > 0 (default 0 = no
             limit: the objective already keeps the deviation as small as
             the checks allow).

R starts at qp_trust, doubles (up to 4 x qp_trust) after a kept round whose
largest move reached R, and halves after a round that is undone.

The overlap is piecewise linear (kinks where bricks start or stop
touching), so the linear model only holds for small moves: each round
solves the QP within R, then re-checks the exact geometry (wall.check).  A
round that lowers the total violation without leaving any brick with no
support at all is kept; otherwise it is undone and R is halved.  Constraints that already fail get a slack variable at a high
cost per mm, so the QP always has a solution; a brick still failing when
the rounds stop has no passing position the QP could find and is flagged.

scipy (SLSQP) is imported inside ``_qp_round`` only.
"""

from __future__ import division

import math
import time

from .geom2d import overlap, overlap_poly, rect_gap
from .l0 import DIRS16, DIRS_LATERAL, smallest_move

PENALTY = 1e3   # objective cost per mm of remaining violation (slack)
MARGIN = 1.0    # mm kept inside each limit, for the linearisation error (brick
                # rotation along a curved course is not in the model)
SETTLE = 0.5    # bricks closer than this to their fitted position [mm] count as "passed"


def _sgn(v):
    return (v > 1e-9) - (v < -1e-9)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


class _Geo(object):
    """Poses and footprints of the current layout, computed once per round."""

    def __init__(self, wall):
        self.c, self.t, self.x, self.fp = {}, {}, {}, {}
        for b in wall.bricks():
            cx, cy, tx, ty, xx, xy = wall.pose(b)
            self.c[b], self.t[b], self.x[b] = (cx, cy), (tx, ty), (xx, xy)
            self.fp[b] = wall.footprint(b)

    def dist(self, a, b):
        return math.hypot(self.c[a][0] - self.c[b][0], self.c[a][1] - self.c[b][1])


def brick_violation(wall, b):
    """Amount by which brick b fails L0 [mm]: contact deficit / brickW +
    overhang excess + collision depth below min_head."""
    p = wall.p
    v = 0.0
    if b.k > 0:
        sup = wall.supports(b)
        area = sum(a for _c, a, _cen in sup)
        v += max(0.0, p["min_contact"] * b.length * wall.W - area) / wall.W
        ov = wall.overhang(b, sup=sup)
        if ov is not None:
            v += max(0.0, abs(ov) - p["max_overhang"])
        bal = wall.balance(b) if area > 0 else None
        if bal is not None:
            v += max(0.0, p["min_balance"] - bal)
    for _c, gap in wall.head_gaps(b):
        v += max(0.0, p["min_head"] - gap)
    return v


def _unsupported(wall):
    """Bricks above course 0 with no overlap at all with the course below."""
    return set(b for b in wall.bricks() if b.k > 0 and not wall.supports(b))


def violations(wall):
    """{brick: brick_violation} for every brick."""
    return dict((b, brick_violation(wall, b)) for b in wall.bricks())


def _balance_rows(wall, geo, b, R):
    """Balance rows of brick b: for each end of its contact region along its
    own axes (Y = along the course, X = across the wall), distance from the
    centre to that end >= min_balance.  An end set by a brick below moves
    with that brick; an end set by b's own edge does not move relative to b
    (no row)."""
    p = wall.p
    cb, tb, xb = geo.c[b], geo.t[b], geo.x[b]
    ends = {}    # key -> (signed position along the axis, brick below setting it)
    for c in wall.courses[b.k - 1]:
        if geo.dist(b, c) > wall.L + wall.W + 2.0 * R:
            continue
        for q in overlap_poly(geo.fp[b], geo.fp[c]):
            d = (q[0] - cb[0], q[1] - cb[1])
            for key, axis, sign in (("t+", tb, 1), ("t-", tb, -1), ("x+", xb, 1), ("x-", xb, -1)):
                v = _dot(d, axis)
                if key not in ends or sign * v > sign * ends[key][0]:
                    ends[key] = (v, c)
    rows = []
    for key, axis, sign in (("t+", tb, 1), ("t-", tb, -1), ("x+", xb, 1), ("x-", xb, -1)):
        if key not in ends:
            continue
        v, c = ends[key]
        dist = sign * v                                   # centre to this end
        own = 0.5 * (b.length if key[0] == "t" else wall.W)
        if abs(dist - own) < 0.5:                         # b's own edge
            continue
        tc, xc = geo.t[c], geo.x[c]
        rows.append(({(c, "s"): sign * _dot(axis, tc), (c, "u"): sign * _dot(axis, xc),
                      (b, "s"): -sign * _dot(axis, tb), (b, "u"): -sign * _dot(axis, xb)},
                     dist, p["min_balance"] + MARGIN))
    return rows


def _pinned(wall):
    """End bricks of open courses (pinned along the course)."""
    pins = set()
    for k, row in enumerate(wall.courses):
        if row and not wall.paths[k].closed:
            bs = sorted(row, key=lambda b: b.s)
            pins.add(bs[0])
            pins.add(bs[-1])
    return pins


def _rows(wall, geo, R):
    """Linearised constraints: [(coefs {(brick, 's'|'u'): slope}, value, limit)],
    meaning value + sum(slope * move) >= limit, all in mm."""
    p = wall.p
    W = wall.W
    rows = []
    for b in wall.bricks():
        if b.k == 0:
            continue
        cb, tb, xb = geo.c[b], geo.t[b], geo.x[b]
        # contact (divided by W, so in mm of brick length)
        co, area = {}, 0.0
        reach = wall.L + W + 2.0 * R
        for c in wall.courses[b.k - 1]:
            if geo.dist(b, c) > reach:
                continue
            a, _cen = overlap(geo.fp[b], geo.fp[c])
            area += a
            d = (geo.c[c][0] - cb[0], geo.c[c][1] - cb[1])
            dt, dx = _dot(d, tb), _dot(d, xb)
            tc, xc = geo.t[c], geo.x[c]
            # half extents of c along b's axes (c may be rotated against b,
            # e.g. where a wavy course crosses the one below)
            ct = 0.5 * (abs(_dot(tc, tb)) * c.length + abs(_dot(xc, tb)) * W)
            cx = 0.5 * (abs(_dot(tc, xb)) * c.length + abs(_dot(xc, xb)) * W)
            ln = 0.5 * b.length + ct - abs(dt)             # overlap along b's length
            wd = 0.5 * W + cx - abs(dx)                    # overlap along b's width
            lcap, wcap = 2.0 * min(0.5 * b.length, ct), 2.0 * min(0.5 * W, cx)
            if wd <= -2.0 * R or ln <= -2.0 * R or (wd <= 0 and ln <= 0):
                continue
            lp, wp = min(max(ln, 0.0), lcap), min(max(wd, 0.0), wcap)
            # d area / d dt and d dx; a pair that does not touch yet gets the
            # slope it will have once it does, so a brick can slide onto it
            gt = -_sgn(dt) * (wp if wd > 0 else 0.5 * wcap) if ln < lcap else 0.0
            gx = -_sgn(dx) * (lp if ln > 0 else 0.5 * lcap) if wd < wcap else 0.0
            for key, val in (((b, "s"), -gt), ((b, "u"), -gx),
                             ((c, "s"), gt * _dot(tb, tc) + gx * _dot(xb, tc)),
                             ((c, "u"), gt * _dot(tb, xc) + gx * _dot(xb, xc))):
                co[key] = co.get(key, 0.0) + val / W
        rows.append((co, area / W, p["min_contact"] * b.length + MARGIN))
        # overhang (both sides)
        sup = wall.supports(b)
        tot = sum(a for _c, a, _cen in sup)
        ov = wall.overhang(b, sup=sup) if tot > 0 else None
        if ov is not None:
            g = {(b, "u"): 1.0}
            for c, a, _cen in sup:
                w = a / tot
                g[(c, "s")] = g.get((c, "s"), 0.0) - w * _dot(xb, geo.t[c])
                g[(c, "u")] = g.get((c, "u"), 0.0) - w * _dot(xb, geo.x[c])
            lim = -p["max_overhang"] + MARGIN
            rows.append((g, ov, lim))                                   # ov >= -max
            rows.append((dict((k, -v) for k, v in g.items()), -ov, lim))  # -ov >= -max
        if tot > 0:
            rows.extend(_balance_rows(wall, geo, b, R))
    # collision: pairs in the same course
    for row in wall.courses:
        for i in range(len(row)):
            for j in range(i + 1, len(row)):
                bi, bj = row[i], row[j]
                dist = geo.dist(bi, bj)
                if dist > wall.L + W + p["max_head"] + 2.0 * R or dist < 1e-9:
                    continue
                gap = rect_gap(geo.fp[bi], geo.fp[bj])
                e = ((geo.c[bj][0] - geo.c[bi][0]) / dist, (geo.c[bj][1] - geo.c[bi][1]) / dist)
                rows.append(({(bj, "s"): _dot(e, geo.t[bj]), (bj, "u"): _dot(e, geo.x[bj]),
                              (bi, "s"): -_dot(e, geo.t[bi]), (bi, "u"): -_dot(e, geo.x[bi])},
                             gap, p["min_head"] + MARGIN))
    return rows


def _qp_round(wall, R, pins):
    """Solve one QP within trust radius R and apply the moves.

    Returns (number of variables, number of rows, largest move), or None if
    nothing could move (no row can become active within R)."""
    import numpy as np
    from scipy.optimize import minimize

    p = wall.p
    geo = _Geo(wall)
    rows = []
    for co, val, lim in _rows(wall, geo, R):
        co = dict((k, v) for k, v in co.items() if abs(v) > 1e-12)
        # keep rows that fail now or could start failing within R
        if val < lim or (co and val - lim < R * sum(abs(v) for v in co.values())):
            rows.append((co, val, lim))
    keys = []
    index = {}
    for co, _val, _lim in rows:
        for k in co:
            if k not in index:
                index[k] = len(keys)
                keys.append(k)
    if not keys:
        return None
    slack = [r for r, (_co, val, lim) in enumerate(rows) if val < lim]
    n, ns = len(keys), len(slack)
    A = np.zeros((len(rows), n + ns))
    rhs = np.zeros(len(rows))
    for r, (co, val, lim) in enumerate(rows):
        for k, v in co.items():
            A[r, index[k]] = v
        rhs[r] = lim - val
    x0 = np.zeros(n + ns)
    for q, r in enumerate(slack):
        A[r, n + q] = 1.0
        x0[n + q] = rhs[r] + 1e-9           # move 0 + slack = current violation: feasible

    off = np.zeros(n)                       # current - target
    bounds = []
    for i, (b, ax) in enumerate(keys):
        cur, fit, tgt = (b.s, b.s0, b.st) if ax == "s" else (b.u, b.u0, b.ut)
        to_tgt, to_fit = tgt - cur, fit - cur
        if ax == "s" and wall.paths[b.k].closed:     # shortest way round the course
            ln = wall.paths[b.k].length
            to_tgt = (to_tgt + 0.5 * ln) % ln - 0.5 * ln
            to_fit = (to_fit + 0.5 * ln) % ln - 0.5 * ln
        off[i] = -to_tgt
        if b.k == 0 or (ax == "s" and b in pins):     # course 0 fixed; ends pinned along
            bounds.append((0.0, 0.0))
            continue
        lo, hi = -R, R
        if p["max_move"] > 0:                        # optional cap on the deviation from the fit
            lo, hi = max(lo, to_fit - p["max_move"]), min(hi, to_fit + p["max_move"])
        bounds.append((min(lo, 0.0), max(hi, 0.0)))
    bounds += [(0.0, None)] * ns

    def fun(x):
        d = off + x[:n]
        return float(d.dot(d) + PENALTY * x[n:].sum())

    def jac(x):
        g = np.empty_like(x)
        g[:n] = 2.0 * (off + x[:n])
        g[n:] = PENALTY
        return g

    cons = {"type": "ineq", "fun": lambda x: A.dot(x) - rhs, "jac": lambda x: A}
    res = minimize(fun, x0, jac=jac, bounds=bounds, constraints=[cons], method="SLSQP",
                   options={"maxiter": 300, "ftol": 1e-9})
    step = float(np.abs(res.x[:n]).max())
    for i, (b, ax) in enumerate(keys):
        d = float(res.x[i])
        if abs(d) < 1e-9:
            continue
        if ax == "s":
            s = b.s + d
            b.s = s % wall.paths[b.k].length if wall.paths[b.k].closed else s
        else:
            b.u += d
    return n, len(rows), step


def _affected(wall, b, extra):
    """Bricks whose L0 result can change when b moves by up to extra [mm]:
    b, nearby bricks of its own course (collision) and of the course above
    (their contact, overhang and balance depend on b).  The course below is
    not affected: its checks only look further down."""
    cx, cy = wall.pose(b)[:2]
    reach = wall.L + wall.W + wall.p["max_head"] + extra
    out = [b]
    for k in (b.k, b.k + 1):
        if k >= len(wall.courses):
            continue
        for c in wall.courses[k]:
            if c is b:
                continue
            px, py = wall.pose(c)[:2]
            if math.hypot(px - cx, py - cy) <= reach:
                out.append(c)
    return out


def _polish(wall, pins, passes=3):
    """After the QP stalls: per failing brick, the smallest single move that
    makes it pass (l0.smallest_move, exact geometry), kept only if the
    violation of the bricks it affects goes down (so it cannot break the
    bricks around it).  Clears the sub-mm leftovers of the linearisation.
    Returns the number of moves."""
    moved = 0
    for _ in range(passes):
        bad = [b for b in wall.bricks() if wall.check(b)]
        if not bad:
            break
        for b in bad:
            if b.k == 0 or not wall.check(b):     # course 0 stays where the fit put it
                continue
            dirs = DIRS_LATERAL if b in pins else DIRS16
            res = smallest_move(wall, b, dirs, lambda s, u, b=b: not wall.check(b, s, u))
            if res is None:
                continue
            near = _affected(wall, b, res[0] + 1.0)
            before = sum(brick_violation(wall, c) for c in near)
            s0, u0 = b.s, b.u
            b.s, b.u = res[1], res[2]
            if sum(brick_violation(wall, c) for c in near) < before - 1e-6:
                moved += 1
            else:
                b.s, b.u = s0, u0
    return moved


def l0_repair_qp(wall, log):
    """Steps 3-4 as a sequential QP.  Returns (passed, fatal message) like
    l0.l0_repair."""
    p = wall.p
    t0 = time.time()
    R = p["qp_trust"]
    pins = _pinned(wall)
    per = violations(wall)
    viol = start = sum(per.values())
    unsup = _unsupported(wall)
    rounds = kept = 0
    r_max = 4.0 * p["qp_trust"]
    while viol > 1e-9 and rounds < p["qp_max_iter"] and R >= p["min_step"] - 1e-9:
        rounds += 1
        saved = [(b, b.s, b.u) for b in wall.bricks()]
        size = _qp_round(wall, R, pins)
        new_per = violations(wall) if size else per
        new = sum(new_per.values())
        # keep the round if the total went down and no brick lost all its
        # support (a brick pushed off the course below has no slope pulling
        # it back, so it would stay stranded)
        lost = size and bool(_unsupported(wall) - unsup)
        if size and new < viol - 1e-6 and not lost:
            per, viol = new_per, new
            unsup = _unsupported(wall)
            kept += 1
            if size[2] >= 0.9 * R:          # good step that was held back by R: allow more
                R = min(2.0 * R, r_max)
        else:
            for b, s, u in saved:
                b.s, b.u = s, u
            R *= 0.5
    polished, t_pol = 0, time.time()
    if viol > 1e-9:
        polished = _polish(wall, pins)
        viol = sum(violations(wall).values())
    t_pol = time.time() - t_pol
    if start > 1e-9:
        log.append("L0 QP: violation {:.1f} -> {:.1f} mm in {} round(s) ({} kept, final trust "
                   "radius {:.1f} mm), {} single-brick clean-up move(s), {:.1f} s ({:.1f} s "
                   "clean-up)".format(start, viol, rounds, kept, R, polished,
                                      time.time() - t0, t_pol))

    failing = 0
    for b in wall.bricks():
        f = wall.check(b)
        if f:
            failing += 1
            b.flag("L0 {}: no passing position found by the QP ({:.1f} mm short)".format(
                "/".join(sorted(f)), brick_violation(wall, b)))
        elif not b.flagged:
            b.status = "moved" if wall.deviation(b) > SETTLE else "passed"
    return failing == 0, None
