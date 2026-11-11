"""Step 4, l0_method = "qp": L0 repair as a sequential quadratic program.

All bricks move together to the layout closest to their targets that passes
L0, instead of one brick at a time.

Variables, per brick: ds (along its course), du (sideways, along the
course normal) and, if max_rotation > 0, dr (rotation in plan from the
course direction, about the brick centre, capped at +/- max_rotation).
Course 0 (the base) does not move; the end bricks of open courses are
pinned along the course (ds = 0) so the wall ends follow the surface edge,
unless they lack support (contact or balance fails), re-checked each round.

Objective (least squares, all in mm^2):
* deviation:  (s - target_s)^2 + (u - target_u)^2 per brick.  The target is
              the fitted position, or where L1 last moved the brick, so L0
              does not undo an L1 repair.
* smoothness: qp_smooth * (u_i - u_j)^2 for neighbouring bricks i, j of a
              course, so a deviation spreads along the course instead of
              jumping from one brick to the next.
* rotation:   (brickL/2)^2 * (r_i + slope_i)^2, slope_i = (u_next - u_prev) /
              (arc distance between them): each brick turns to follow the
              deviated course line instead of keeping the original course
              direction (which leaves a staircase of protruding corners).

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
             the brick's axes (wall.balance_ends); an end set by a brick
             below moves with that brick.
* collision: clear gap to each nearby brick of the same course >= min_head,
             moving along the line between the two centres.
* limits:    every move <= the trust radius R (a rotation: its brick ends
             move <= R); |s - fitted s|, |u - fitted u| <= max_move only if
             max_move > 0 (default 0 = no limit).
The slopes for ds and du are analytic; the slopes for dr are central finite
differences of the exact rotated footprints (H_ROT).

R starts at qp_trust, doubles (up to 4 x qp_trust) after a kept round whose
largest move reached R, and halves after a round that is undone.

The overlap is piecewise linear (kinks where bricks start or stop
touching), so the linear model only holds for small moves: each round
solves the QP within R, then re-checks the exact geometry (wall.check).  A
round that lowers the total violation without leaving any brick with no
support at all is kept; otherwise it is undone and R is halved.
Constraints that already fail get a slack variable at a high cost per mm,
so the QP always has a solution; a brick still failing when the rounds stop
has no passing position the QP could find and is flagged.

The QP is solved with the IPOPT build bundled with compas_cra (sparse
constraint matrix, exact constant Hessian), imported inside ``_qp_round``
only.  (The first version used scipy's SLSQP, which works on dense
matrices and took ~14 s per round on a 200-brick wall.)
"""

from __future__ import division

import math
import time

from .geom2d import overlap, overlap_poly, rect_gap
from .l0 import DIRS16, DIRS_LATERAL, smallest_move

PENALTY = 1e3   # objective cost per mm of remaining violation (slack)
MARGIN = 1.0    # mm kept inside each limit, for the linearisation error
SETTLE = 0.5    # bricks closer than this to their fitted position [mm] count as "passed"
H_ROT = 1e-3    # rotation step for the finite-difference slopes [rad]

# IPOPT (compas_cra's in-process build) for the QP: the constraints are
# linear and the Hessian is constant, which IPOPT can exploit.
IPOPT_OPTIONS = {
    "jac_c_constant": "yes",
    "jac_d_constant": "yes",
    "hessian_constant": "yes",
    "tol": 1e-8,
    "max_iter": 500,
    "mumps_scaling": 8,
}

ENDS = (("t+", "t", 1), ("t-", "t", -1), ("x+", "x", 1), ("x-", "x", -1))


def _sgn(v):
    return (v > 1e-9) - (v < -1e-9)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _turn(v, a):
    c, s = math.cos(a), math.sin(a)
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c)


class _Geo(object):
    """Poses and footprints of the current layout, computed once per round.

    t, x: brick axes (course axes turned by the brick's rotation);
    mt, mx: course axes, the directions ds and du move the brick in."""

    def __init__(self, wall):
        self.W = wall.W
        self.c, self.t, self.x, self.mt, self.mx, self.fp = {}, {}, {}, {}, {}, {}
        for b in wall.bricks():
            cx, cy, tx, ty, xx, xy = wall.pose(b)
            mtx, mty, mnx, mny = wall.course_axes(b)
            self.c[b], self.t[b], self.x[b] = (cx, cy), (tx, ty), (xx, xy)
            self.mt[b], self.mx[b] = (mtx, mty), (mnx, mny)
            self.fp[b] = wall.footprint(b)

    def dist(self, a, b):
        return math.hypot(self.c[a][0] - self.c[b][0], self.c[a][1] - self.c[b][1])

    def quad(self, q, dr):
        """(footprint, t, x) of brick q turned by dr about its centre."""
        t = _turn(self.t[q], dr) if dr else self.t[q]
        x = (t[1], -t[0])
        cx, cy = self.c[q]
        hl, hw = 0.5 * q.length, 0.5 * self.W
        fp = [(cx + t[0] * hl + x[0] * hw, cy + t[1] * hl + x[1] * hw),
              (cx + t[0] * hl - x[0] * hw, cy + t[1] * hl - x[1] * hw),
              (cx - t[0] * hl - x[0] * hw, cy - t[1] * hl - x[1] * hw),
              (cx - t[0] * hl + x[0] * hw, cy - t[1] * hl + x[1] * hw)]
        return fp, t, x


def _mv(geo, axis, q, sign=1.0):
    """Slopes of (position of q) . axis for q's ds and du."""
    return {(q, "s"): sign * _dot(axis, geo.mt[q]), (q, "u"): sign * _dot(axis, geo.mx[q])}


def _acc(co, d, scale=1.0):
    for k, v in d.items():
        co[k] = co.get(k, 0.0) + scale * v


def brick_violation(wall, b):
    """Amount by which brick b fails L0 [mm]: contact deficit / brickW +
    overhang excess + balance deficit + collision depth below min_head.

    A brick with no contact at all also adds its clear gap to the nearest
    brick below, so moving a stranded brick towards support counts as
    progress (without it the contact deficit is the same at any distance and
    the keep/undo test cannot see the brick coming back)."""
    p = wall.p
    v = 0.0
    if b.k > 0:
        sup = wall.supports(b)
        area = sum(a for _c, a, _cen in sup)
        v += max(0.0, p["min_contact"] * b.length * wall.W - area) / wall.W
        if not sup and wall.courses[b.k - 1]:
            fp = wall.footprint(b)
            v += max(0.0, min(rect_gap(fp, wall.footprint(c)) for c in wall.courses[b.k - 1]))
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


def _ends(geo, b, cands, dr_b=0.0, alt=None):
    """{end: (signed position along b's axis, brick below setting it)} of b's
    contact region, b turned by dr_b; alt = (c, footprint) replaces c's."""
    fpb, tb, xb = geo.quad(b, dr_b)
    cb = geo.c[b]
    axes = {"t": tb, "x": xb}
    ends = {}
    for c in cands:
        fpc = alt[1] if alt is not None and alt[0] is c else geo.fp[c]
        for q in overlap_poly(fpb, fpc):
            d = (q[0] - cb[0], q[1] - cb[1])
            for key, ax, sign in ENDS:
                v = _dot(d, axes[ax])
                if key not in ends or sign * v > sign * ends[key][0]:
                    ends[key] = (v, c)
    return ends


def _contact_row(wall, geo, b, cands, R):
    W = wall.W
    cb, tb, xb = geo.c[b], geo.t[b], geo.x[b]
    co, area, touching = {}, 0.0, []
    for c in cands:
        a, _cen = overlap(geo.fp[b], geo.fp[c])
        area += a
        if a > 0:
            touching.append(c)
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
        # dt = tb . (p_c - p_b), dx = xb . (p_c - p_b)
        for g, axis in ((gt, tb), (gx, xb)):
            if g:
                _acc(co, _mv(geo, axis, c), g / W)
                _acc(co, _mv(geo, axis, b), -g / W)

    def rot():
        out = {}
        fpp, fpm = geo.quad(b, H_ROT)[0], geo.quad(b, -H_ROT)[0]
        sb = 0.0
        for c in touching:
            sb += overlap(fpp, geo.fp[c])[0] - overlap(fpm, geo.fp[c])[0]
            cp, cm = geo.quad(c, H_ROT)[0], geo.quad(c, -H_ROT)[0]
            out[c] = (overlap(geo.fp[b], cp)[0] - overlap(geo.fp[b], cm)[0]) / (2.0 * H_ROT * W)
        out[b] = sb / (2.0 * H_ROT * W)
        return out

    return (co, area / W, wall.p["min_contact"] * b.length + MARGIN, rot)


def _overhang_rows(wall, geo, b, sup):
    p = wall.p
    tot = sum(a for _c, a, _cen in sup)
    ov = wall.overhang(b, sup=sup)
    if ov is None:
        return []
    xb, tb = geo.x[b], geo.t[b]
    g = {}
    _acc(g, _mv(geo, xb, b))
    sx = sy = 0.0
    for c, a, _cen in sup:
        w = a / tot
        _acc(g, _mv(geo, xb, c), -w)
        sx += w * geo.c[c][0]
        sy += w * geo.c[c][1]
    # turning b turns its X axis (d x / d r = t): d ov / d r_b = (c_b - S) . t_b
    d_rot = (geo.c[b][0] - sx) * tb[0] + (geo.c[b][1] - sy) * tb[1]
    lim = -p["max_overhang"] + MARGIN
    neg = dict((k, -v) for k, v in g.items())
    return [(g, ov, lim, lambda: {b: d_rot}),          # ov >= -max
            (neg, -ov, lim, lambda: {b: -d_rot})]      # -ov >= -max


def _balance_rows(wall, geo, b, cands):
    """Balance rows of brick b: for each end of its contact region along its
    own axes (Y = along the course, X = across the wall), distance from the
    centre to that end >= min_balance.  An end set by a brick below moves
    with that brick; an end set by b's own edge does not move relative to b
    (no row)."""
    p = wall.p
    ends = _ends(geo, b, cands)
    axes = {"t": geo.t[b], "x": geo.x[b]}
    rows = []
    for key, ax, sign in ENDS:
        if key not in ends:
            continue
        v, c = ends[key]
        dist = sign * v                                   # centre to this end
        own = 0.5 * (b.length if ax == "t" else wall.W)
        if abs(dist - own) < 0.5:                         # b's own edge
            continue
        co = {}
        _acc(co, _mv(geo, axes[ax], c, sign))
        _acc(co, _mv(geo, axes[ax], b, -sign))

        def rot(c=c, key=key, sign=sign, dist=dist):
            out = {}
            for q in (b, c):
                vals = []
                for h in (H_ROT, -H_ROT):
                    if q is b:
                        e = _ends(geo, b, cands, dr_b=h)
                    else:
                        e = _ends(geo, b, cands, alt=(c, geo.quad(c, h)[0]))
                    vals.append(sign * e[key][0] if key in e else dist)
                out[q] = (vals[0] - vals[1]) / (2.0 * H_ROT)
            return out

        rows.append((co, dist, p["min_balance"] + MARGIN, rot))
    return rows


def _collision_row(wall, geo, bi, bj, dist):
    gap = rect_gap(geo.fp[bi], geo.fp[bj])
    e = ((geo.c[bj][0] - geo.c[bi][0]) / dist, (geo.c[bj][1] - geo.c[bi][1]) / dist)
    co = {}
    _acc(co, _mv(geo, e, bj))
    _acc(co, _mv(geo, e, bi), -1.0)

    def rot():
        fi, fj = geo.fp[bi], geo.fp[bj]
        di = (rect_gap(geo.quad(bi, H_ROT)[0], fj) - rect_gap(geo.quad(bi, -H_ROT)[0], fj))
        dj = (rect_gap(fi, geo.quad(bj, H_ROT)[0]) - rect_gap(fi, geo.quad(bj, -H_ROT)[0]))
        return {bi: di / (2.0 * H_ROT), bj: dj / (2.0 * H_ROT)}

    return (co, gap, wall.p["min_head"] + MARGIN, rot)


def _rows(wall, geo, R):
    """Linearised constraints: [(coefs {(brick, 's'|'u'): slope}, value,
    limit, rot)], meaning value + sum(slope * move) >= limit, all in mm; rot()
    returns {brick: slope per radian of its rotation}."""
    p = wall.p
    W = wall.W
    rows = []
    for b in wall.bricks():
        if b.k == 0:
            continue
        reach = wall.L + W + 2.0 * R
        cands = [c for c in wall.courses[b.k - 1] if geo.dist(b, c) <= reach]
        rows.append(_contact_row(wall, geo, b, cands, R))
        sup = wall.supports(b)
        if sup:
            rows.extend(_overhang_rows(wall, geo, b, sup))
            rows.extend(_balance_rows(wall, geo, b, cands))
    for row in wall.courses:
        for i in range(len(row)):
            for j in range(i + 1, len(row)):
                bi, bj = row[i], row[j]
                dist = geo.dist(bi, bj)
                if dist > wall.L + W + p["max_head"] + 2.0 * R or dist < 1e-9:
                    continue
                rows.append(_collision_row(wall, geo, bi, bj, dist))
    return rows


def _pinned(wall):
    """End bricks of open courses (pinned along the course), except those that
    lack support (contact or balance fails, or no contact at all): an end brick
    whose course runs past the end of the course below can only get back onto
    it by sliding along the course.  Re-evaluated every round, so a brick
    that slid back onto support is pinned again where it now is."""
    pins = set()
    for k, row in enumerate(wall.courses):
        if row and not wall.paths[k].closed:
            bs = sorted(row, key=lambda b: b.s)
            for b in (bs[0], bs[-1]):
                if b.k == 0 or not (wall.check(b) & {"contact", "balance"}):
                    pins.add(b)
    return pins


def _wrap(v, ln):
    return (v + 0.5 * ln) % ln - 0.5 * ln


def _qp_round(wall, R, pins):
    """Solve one QP within trust radius R and apply the moves.

    Returns (number of variables, number of rows, largest move [mm], solve
    time [s]), or None if nothing could move (no row can become active
    within R)."""
    import numpy as np
    from compas_cra.nlp import NLPProblem, solve_nlp

    p = wall.p
    rotate = p["max_rotation"] > 0
    half_l = 0.5 * wall.L
    geo = _Geo(wall)

    # --- constraints: keep rows that fail now or could start failing within R
    rows = []
    for co, val, lim, rot in _rows(wall, geo, R):
        co = dict((k, v) for k, v in co.items() if abs(v) > 1e-12 and k[0].k > 0)
        reach = R * (sum(abs(v) for v in co.values()) + (1.0 if rotate else 0.0))
        if not (val < lim or val - lim < reach):
            continue
        if rotate:
            for q, sl in rot().items():
                if q.k > 0 and abs(sl) > 1e-9:
                    co[(q, "r")] = co.get((q, "r"), 0.0) + sl
        if co or val < lim:
            rows.append((co, val, lim))

    # --- variables: bricks in those rows plus their course neighbours
    order = [sorted(row, key=lambda b: b.s) for row in wall.courses]
    where = {}
    for k, row in enumerate(order):
        for i, b in enumerate(row):
            where[b] = (k, i)
    active = set(k[0] for co, _v, _l in rows for k in co)
    if not active:
        return None
    for b in list(active):
        k, i = where[b]
        row, closed = order[k], wall.paths[k].closed
        for j in (i - 1, i + 1):
            if closed or 0 <= j < len(row):
                nb = row[j % len(row)]
                if nb.k > 0:
                    active.add(nb)
    keys, index = [], {}
    for b in sorted(active, key=lambda b: where[b]):
        for ax in (("s", "u", "r") if rotate else ("s", "u")):
            index[(b, ax)] = len(keys)
            keys.append((b, ax))
    n = len(keys)

    # --- constraint matrix (sparse triplets) and slacks
    slack = [r for r, (_co, val, lim) in enumerate(rows) if val < lim]
    ns, m = len(slack), len(rows)
    jr, jc, jv = [], [], []
    rhs = np.zeros(m)
    for r, (co, val, lim) in enumerate(rows):
        for k, v in co.items():
            jr.append(r)
            jc.append(index[k])
            jv.append(v)
        rhs[r] = lim - val
    x0 = np.zeros(n + ns)
    for q, r in enumerate(slack):
        jr.append(r)
        jc.append(n + q)
        jv.append(1.0)
        x0[n + q] = rhs[r] + 1e-9           # move 0 + slack = current violation: feasible
    jr, jc, jv = np.array(jr), np.array(jc), np.array(jv, dtype=float)

    # --- objective: sum of weight * (linear expression)^2
    terms = []                              # ({var: coef}, constant, weight)

    def term(items, const, weight):
        co = {}
        for key, a in items:
            if key in index:
                co[index[key]] = co.get(index[key], 0.0) + a
        if co:
            terms.append((co, const, weight))

    for b in active:
        ln = wall.paths[b.k].length
        to_tgt = b.st - b.s
        if wall.paths[b.k].closed:
            to_tgt = _wrap(to_tgt, ln)
        term([((b, "s"), 1.0)], -to_tgt, 1.0)
        term([((b, "u"), 1.0)], b.u - b.ut, 1.0)
    for k, row in enumerate(order):
        closed = wall.paths[k].closed
        nr = len(row)
        pairs = [(row[i], row[(i + 1) % nr]) for i in range(nr if closed and nr > 2 else nr - 1)]
        for bi, bj in pairs:
            if bi in active or bj in active:
                term([((bi, "u"), 1.0), ((bj, "u"), -1.0)], bi.u - bj.u, p["qp_smooth"])
        if not rotate:
            continue
        for i, b in enumerate(row):
            if b not in active:
                continue
            prv = row[(i - 1) % nr] if (closed or i > 0) else b
            nxt = row[(i + 1) % nr] if (closed or i < nr - 1) else b
            if prv is nxt:
                continue
            d = nxt.s - prv.s
            if closed:
                d %= wall.paths[k].length
            if d < 1.0:
                continue
            # r_b + du/ds = 0: the brick follows the deviated course line
            term([((b, "r"), 1.0), ((nxt, "u"), 1.0 / d), ((prv, "u"), -1.0 / d)],
                 b.r + (nxt.u - prv.u) / d, half_l * half_l)

    tr, tcol, tv, tc, tw = [], [], [], [], []
    hess = {}
    for t, (co, const, w) in enumerate(terms):
        tc.append(const)
        tw.append(w)
        items = list(co.items())
        for i, a in items:
            tr.append(t)
            tcol.append(i)
            tv.append(a)
        for i, a in items:
            for j, b_ in items:
                if j <= i:
                    hess[(i, j)] = hess.get((i, j), 0.0) + 2.0 * w * a * b_
    tr, tcol, tv = np.array(tr), np.array(tcol), np.array(tv, dtype=float)
    tc, tw = np.array(tc, dtype=float), np.array(tw, dtype=float)
    hr = np.array([i for i, _j in hess], dtype=int)
    hc = np.array([j for _i, j in hess], dtype=int)
    hv = np.array(list(hess.values()), dtype=float)

    def residual(x):
        e = tc.copy()
        np.add.at(e, tr, tv * x[tcol])
        return e

    def fun(x):
        e = residual(x)
        return float((tw * e * e).sum() + PENALTY * x[n:].sum())

    def jac(x):
        e = residual(x)
        g = np.zeros_like(x)
        np.add.at(g, tcol, 2.0 * tv * (tw * e)[tr])
        g[n:] += PENALTY
        return g

    def cons(x):                            # A x, from the triplets
        g = np.zeros(m)
        np.add.at(g, jr, jv * x[jc])
        return g

    # --- bounds
    max_rot = math.radians(p["max_rotation"])
    lo_b, hi_b = [], []
    for b, ax in keys:
        if ax == "r":
            rr = R / half_l
            lo, hi = max(-rr, -max_rot - b.r), min(rr, max_rot - b.r)
        elif ax == "s" and b in pins:
            lo = hi = 0.0
        else:
            cur, fit = (b.s, b.s0) if ax == "s" else (b.u, b.u0)
            to_fit = fit - cur
            if ax == "s" and wall.paths[b.k].closed:
                to_fit = _wrap(to_fit, wall.paths[b.k].length)
            lo, hi = -R, R
            if p["max_move"] > 0:                    # optional cap on the deviation from the fit
                lo, hi = max(lo, to_fit - p["max_move"]), min(hi, to_fit + p["max_move"])
        lo_b.append(min(lo, 0.0))
        hi_b.append(max(hi, 0.0))
    lo_b += [0.0] * ns
    hi_b += [np.inf] * ns

    problem = NLPProblem(
        n=n + ns, x_l=lo_b, x_u=hi_b, g_l=rhs, g_u=np.full(m, np.inf), x0=x0,
        objective=fun, gradient=jac, constraints=cons,
        jac_rows=jr, jac_cols=jc, jacobian=lambda x: jv,
        hess_rows=hr, hess_cols=hc, hessian=lambda x, sigma, lam: sigma * hv)
    t_solve = time.time()
    res = solve_nlp(problem, backend="native", options=IPOPT_OPTIONS)
    t_solve = time.time() - t_solve

    step = 0.0
    for i, (b, ax) in enumerate(keys):
        d = float(res.x[i])
        if abs(d) < 1e-12:
            continue
        if ax == "s":
            s = b.s + d
            b.s = s % wall.paths[b.k].length if wall.paths[b.k].closed else s
            step = max(step, abs(d))
        elif ax == "u":
            b.u += d
            step = max(step, abs(d))
        else:
            b.r += d
            step = max(step, abs(d) * half_l)
    return n, m, step, t_solve


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
    per = violations(wall)
    viol = start = sum(per.values())
    unsup = _unsupported(wall)
    rounds = kept = 0
    r_max = 4.0 * p["qp_trust"]
    t_qp = 0.0
    while viol > 1e-9 and rounds < p["qp_max_iter"] and R >= p["min_step"] - 1e-9:
        rounds += 1
        saved = [(b, b.s, b.u, b.r) for b in wall.bricks()]
        size = _qp_round(wall, R, _pinned(wall))
        if size:
            t_qp += size[3]
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
            for b, s, u, r in saved:
                b.s, b.u, b.r = s, u, r
            R *= 0.5
    polished, t_pol = 0, time.time()
    if viol > 1e-9:
        polished = _polish(wall, _pinned(wall))
        viol = sum(violations(wall).values())
    t_pol = time.time() - t_pol
    if start > 1e-9:
        log.append("L0 QP: violation {:.1f} -> {:.1f} mm in {} round(s) ({} kept, final trust "
                   "radius {:.1f} mm), {} single-brick clean-up move(s), {:.1f} s ({:.1f} s "
                   "QP solves, {:.1f} s clean-up, rest geometry)".format(
                       start, viol, rounds, kept, R, polished, time.time() - t0, t_qp, t_pol))

    failing = 0
    for b in wall.bricks():
        f = wall.check(b)
        if f:
            failing += 1
            b.flag("L0 {}: no passing position found by the QP ({:.1f} mm short)".format(
                "/".join(sorted(f)), brick_violation(wall, b)))
        elif not b.flagged:
            moved = wall.deviation(b) > SETTLE or abs(b.r - b.r0) > math.radians(0.5)
            b.status = "moved" if moved else "passed"
    return failing == 0, None
