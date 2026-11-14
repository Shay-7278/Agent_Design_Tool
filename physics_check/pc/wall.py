"""Wall model: bricks placed on the fitted course centrelines, and the
per-brick L0 measures (step 3)."""

from __future__ import division

import math

from .geom2d import overlap, overlap_poly, rect_gap


class Brick(object):
    __slots__ = ("k", "i", "s", "u", "r", "s0", "u0", "r0", "st", "ut", "rt", "length",
                 "status", "notes")

    def __init__(self, k, i, s, length):
        self.k, self.i = k, i
        self.length = length    # along the course: brickL, or brickL / 2 for a half brick
        self.s = self.s0 = s
        self.u = self.u0 = 0.0
        self.r = self.r0 = 0.0  # rotation in plan from the course direction [rad], CCW +
        # target the QP L0 repair pulls the brick back to: the fitted
        # position, or where L1 last moved it (l0qp.py)
        self.st, self.ut, self.rt = s, 0.0, 0.0
        self.status = "passed"
        self.notes = []

    @property
    def flagged(self):
        return self.status == "flagged"

    def flag(self, note):
        self.status = "flagged"
        if note not in self.notes:
            self.notes.append(note)


class Wall(object):
    """Bricks placed on the fitted course centrelines.

    Each brick is (s, u): arc position along its course centreline and lateral
    offset along the brick's X axis.  Its orientation follows the course chord
    over the brick length.  counts[k] is what fit_sections chose for course k:
    the course's middle-brick count m on an open course, the total on a closed one.
    """

    def __init__(self, paths, z_base, p, counts):
        self.p = p
        self.paths = paths
        self.z_base = z_base
        self.L, self.W, self.H = p["brickL"], p["brickW"], p["brickH"]
        self.pitch = p["brickH"] + p["mortar_t"]
        self.courses = [[] for _ in paths]
        self.steps = [0.0 for _ in paths]
        self.widened = [False for _ in paths]   # joints allowed past max_head
        self.counts = list(counts)
        self.dropped = []                       # bricks removed by the L0 repair (l0qp.py)
        self.end_dev = {}                       # {course: (start, end) deviation [mm]} (place_bond)
        self.bond_change = {}                   # {course: middle bricks inserted (+) / removed (-)}
        self.end_cut = {}                       # {course: (start, end) end-fill pieces left out
                                                #  by end_support; a full -> half swap counts 0.5}
        for k, n in enumerate(counts):
            self.place_course(k, n)

    def place_course(self, k, n, widen=False):
        """Place course k from its count n and replace its bricks.

        Open course (n = middle-brick count m, layout of baseline.gh, see
        fit.py): even courses get a full brick flush with each end and m
        bricks centred in m equal cells of the middle curve; odd courses get a
        half brick flush with each end and m + 1 bricks on the cell
        boundaries.  fit.py picks m so the middle joints are within
        [min_head, max_head]; settle_counts may later drop a brick, so the
        joints are not capped at max_head (widened).

        Closed course (n = total): n bricks evenly spaced, odd courses
        shifted by offset_distance; widen=True only marks joints past
        max_head as allowed.
        """
        path = self.paths[k]
        L = self.L
        self.counts[k] = n
        if path.closed:
            step = path.length / n
            self.widened[k] = bool(widen) and step > L + self.p["max_head"]
            self.steps[k] = step
            off = self.p["offset_distance"] if k % 2 == 1 else 0.0
            self.courses[k] = [Brick(k, i, ((i + 0.5) * step + off) % path.length, L)
                               for i in range(n)]
            return

        g = self.p["head_joint"]
        odd = k % 2 == 1
        trim = L + g if odd else L + 0.5 * g        # = fit.end_trim(k, p)
        cell = (path.length - 2.0 * trim) / n
        self.steps[k] = cell
        self.widened[k] = True
        if odd:
            end_len = 0.5 * L
            middle = [trim + i * cell for i in range(n + 1)]
        else:
            end_len = L
            middle = [trim + (i + 0.5) * cell for i in range(n)]
        placed = ([(0.5 * end_len, end_len)] + [(s, L) for s in middle]
                  + [(path.length - 0.5 * end_len, end_len)])
        self.courses[k] = [Brick(k, i, s, ln) for i, (s, ln) in enumerate(placed)]

    def place_bond(self, k):
        """Place open course k over the head joints of course k - 1 (layout
        "bond"; course k - 1 must be placed and settled).

        Every brick of the bond is a full brick centred over a head joint of
        the course below (the joint's midpoint projected onto course k's
        line), so it bears on the two bricks either side of that joint, and
        each brick below carries two above: the running bond holds whatever
        the course lengths.  The middle bricks' contacts come first, so the
        bond bricks are not clipped to the surface edge: every joint below
        whose projection lands on the course line gets its brick, even if
        that brick hangs past the edge (by at most half a brick).  Joints
        that project past the course line's ends are left out.

        Each end gap (between the outermost bond brick and the surface edge)
        is then filled, packed against the bond: full bricks while one fits,
        then a half bat if one fits (each piece needs its length + min_head;
        joints at head_joint, or less down to min_head when the gap is
        tight).  A piece counts as fitting when it runs past the edge by at
        most end_overrun [mm]; otherwise the fill stays inside the edge (a
        gap that misses a full brick by a fraction of a millimetre would
        otherwise get a half bat and leave the end of the course open).  A
        leftover gap smaller than a half bat is left open.  The course gains
        or loses bricks only at its ends.

        With end_support on, each end piece must also bear on course k - 1
        by itself (min_contact, min_balance): where the course below stops
        short of the edge, the fill is cut back (a full brick becomes a half
        bat, or pieces are left out) so the end steps back over the open gap
        instead of hanging over it.  Pieces left out: self.end_cut[k].

        Records self.end_dev[k] = (start, end): signed distance of each
        course end from the surface edge [mm] (+ = open gap short of the
        edge, - = bond brick past it).  Returns False (and places nothing) if
        no joint below falls on the course line."""
        p = self.p
        L, path = self.L, self.paths[k]
        below = sorted(self.courses[k - 1], key=lambda b: b.s)
        ss = []
        for a, b in zip(below, below[1:]):
            ax, ay, atx, aty = self.pose(a)[:4]
            bx, by, btx, bty = self.pose(b)[:4]
            ea = (ax + atx * 0.5 * a.length, ay + aty * 0.5 * a.length)
            sb = (bx - btx * 0.5 * b.length, by - bty * 0.5 * b.length)
            ss.append(path.nearest((0.5 * (ea[0] + sb[0]), 0.5 * (ea[1] + sb[1])))[0])
        # nearest() clamps to the line's ends: a joint at s = 0 or s = length
        # lies beyond the course line
        ss = sorted(s for s in ss if 1e-6 < s < path.length - 1e-6)
        if not ss:
            return False

        lo_h, hi_h, hj = p["min_head"], p["max_head"], p["head_joint"]
        tol = p["end_overrun"] + 1e-6
        ss, change = self._bond_middle(k, ss, lo_h, hi_h, hj)

        def fill(gap):
            """(pieces outward from the bond, joint, leftover gap at the edge)
            for a gap [mm] between the bond's outermost brick and the edge.
            The leftover is negative when the last piece runs past the edge
            (by at most end_overrun)."""
            lens = []
            rest = gap
            while rest >= L + lo_h - tol:
                lens.append(L)
                rest -= L + lo_h
            if rest >= 0.5 * L + lo_h - tol:
                lens.append(0.5 * L)
            if not lens:
                return [], 0.0, gap
            g = max(lo_h, min(hj, (gap - sum(lens)) / len(lens)))
            return lens, g, gap - sum(lens) - len(lens) * g

        def spacing(gap, lens):
            g = max(lo_h, min(hj, (gap - sum(lens)) / len(lens))) if lens else 0.0
            return g, gap - sum(lens) - len(lens) * g

        def lay(start, sign, lens, g):
            """Centres of the pieces packed outward from the bond edge at
            start (sign -1 = towards s = 0)."""
            out, cur = [], start
            for ln in lens:
                cur += sign * g
                out.append((cur + sign * 0.5 * ln, ln))
                cur += sign * ln
            return out

        def bears(s, ln):
            """End piece at s rests on course k - 1 by itself: contact and
            balance as the L0 check (min_contact, min_balance)."""
            b = Brick(k, -1, s, ln)
            sup = self.supports(b)
            tot = sum(a for _c, a, _cen in sup)
            if tot < p["min_contact"] * ln * self.W:
                return False
            bal = self.balance(b)
            return bal is not None and bal >= p["min_balance"] - 1e-6

        def end(start, sign, gap):
            """(pieces, leftover, pieces left out) for one end.  With
            end_support on, the fill is cut back from the edge until every
            piece bears on the course below (a full brick that does not is
            first tried as a half bat): an open gap below is stepped back
            (racked) instead of bridged by a piece that would need holding."""
            lens, _g, dev = fill(gap)
            if not lens:
                return [], dev, 0
            cands = [lens]
            if p["end_support"]:
                for n in range(len(lens), -1, -1):
                    if n < len(lens):
                        cands.append(lens[:n])
                    if n and lens[n - 1] == L:
                        cands.append(lens[:n - 1] + [0.5 * L])
            for c in cands:
                g, dev = spacing(gap, c)
                pcs = lay(start, sign, c, g)
                if not p["end_support"] or all(bears(s, ln) for s, ln in pcs):
                    return pcs, dev, len(lens) - len(c) + 0.5 * sum(
                        1 for a, b in zip(lens, c) if a != b)
            return [], gap, len(lens)

        placed = [(s, L) for s in ss]
        pcs0, dev0, cut0 = end(ss[0] - 0.5 * L, -1, ss[0] - 0.5 * L)
        pcs1, dev1, cut1 = end(ss[-1] + 0.5 * L, 1, path.length - (ss[-1] + 0.5 * L))
        placed += pcs0 + pcs1
        placed.sort()
        if cut0 or cut1:
            self.end_cut[k] = (cut0, cut1)
        self.courses[k] = [Brick(k, i, s, ln) for i, (s, ln) in enumerate(placed)]
        self.counts[k] = len(placed)
        self.steps[k] = ((ss[-1] - ss[0]) / (len(ss) - 1)) if len(ss) > 1 else L + p["head_joint"]
        self.widened[k] = False
        self.end_dev[k] = (dev0, dev1)
        self.bond_change[k] = change
        return True

    def _bond_middle(self, k, ss, lo_h, hi_h, hj):
        """Middle bricks of a bond course: ss = brick centres over the joints
        below (arc positions, sorted).  Returns (centres, change).

        A course that grows (or shrinks) against the course below would put
        the growth into its head joints.  So the number of middle bricks is
        changed by `change` (+ inserted, - removed), the count whose mean
        head joint is closest to head_joint while staying >= min_head.  An
        inserted brick goes into the widest joint (centred in it); a removed
        brick is the one whose two joints are the tightest.  Then the centres
        are moved as little as possible (least squares, inserted bricks
        weighted less so the bond bricks stay over their joints) with every
        head joint in [min_head, max_head]: the fit of the middle bricks
        comes first, the bond gives way only around the changes.

        Head joints are the real gaps between the brick footprints: chord
        bricks on a curve lose joint on the concave side, so each joint's
        arc-length limits are shifted by its measured loss (re-measured
        after each solve)."""
        L = self.L
        n0 = len(ss)
        if n0 < 2:
            return list(ss), 0

        def loss(sa, sb):
            """arc-length joint minus footprint joint between centres sa < sb"""
            return (sb - sa - L) - rect_gap(self.footprint(Brick(k, 0, sa, L)),
                                            self.footprint(Brick(k, 0, sb, L)))

        losses = [loss(a, b) for a, b in zip(ss, ss[1:])]
        c_mean = sum(losses) / len(losses)
        tot = sum(b - a - L for a, b in zip(ss, ss[1:]))
        best = None
        for m in range(-(n0 - 2), n0 + 1):
            nj = n0 - 1 + m
            mean = (tot - m * L) / nj - c_mean          # mean footprint joint
            key = (mean < lo_h - 1e-9, abs(mean - hj))
            if best is None or key < best[0]:
                best = (key, m)
        change = best[1]

        items = [[s, 1.0] for s in ss]           # [target centre, weight]
        W_NEW = 0.25
        for _ in range(max(change, 0)):
            gaps = [items[i + 1][0] - items[i][0] for i in range(len(items) - 1)]
            i = max(range(len(gaps)), key=lambda q: gaps[q])
            items.insert(i + 1, [0.5 * (items[i][0] + items[i + 1][0]), W_NEW])
        for _ in range(max(-change, 0)):
            n = len(items)
            j = min(range(1, n - 1), key=lambda q: items[q + 1][0] - items[q - 1][0])
            del items[j]

        n = len(items)
        if n == 1:
            return [float(items[0][0])], change
        import numpy as np
        from scipy.optimize import lsq_linear
        # x = (s_0, d_1 .. d_n-1), s_i = s_0 + d_1 + .. + d_i, d = L + arc joint
        A = np.tril(np.ones((n, n)))
        w = np.array([it[1] for it in items])
        t = np.array([it[0] for it in items])
        s = t.copy()
        for _ in range(4):
            c = np.array([loss(a, b) for a, b in zip(s, s[1:])])
            lb = np.r_[-np.inf, L + lo_h + c]
            ub = np.r_[np.inf, L + hi_h + c]
            res = lsq_linear(A * w[:, None], t * w, bounds=(lb, ub))
            s_new = A.dot(res.x)
            done = np.max(np.abs(s_new - s)) < 0.1
            s = s_new
            if done:
                break
        return [float(v) for v in s], change

    def course_collides(self, k):
        """True if any two bricks of course k are closer than min_head
        (checks every pair: on tight curves non-neighbours can collide)."""
        row = self.courses[k]
        fps = [self.footprint(b) for b in row]
        for i in range(len(fps)):
            for j in range(i + 1, len(fps)):
                if rect_gap(fps[i], fps[j]) < self.p["min_head"] - 1e-6:
                    return True
        return False

    def head_joint(self, k):
        """Nominal (arc-length) head joint of course k as placed [mm]; on an
        open course, the joint between its middle bricks."""
        return self.steps[k] - self.L

    # -- poses ---------------------------------------------------------------
    def course_axes(self, b, s=None):
        """(tx, ty, nx, ny): the course direction at b (chord over the brick
        length) and its sideways normal.  s moves b along t, u along n."""
        s = b.s if s is None else s
        tx, ty = self.paths[b.k].tangent(s, 0.5 * b.length)
        return tx, ty, ty, -tx

    def pose(self, b, s=None, u=None, r=None):
        """(cx, cy, tx, ty, xx, xy): plan centre, run axis Y, overhang axis X.
        The axes are the course axes turned by the brick's rotation r."""
        s = b.s if s is None else s
        u = b.u if u is None else u
        r = b.r if r is None else r
        px, py = self.paths[b.k].point(s)
        tx, ty, nx, ny = self.course_axes(b, s)
        cx, cy = px + u * nx, py + u * ny
        if r:
            cr, sr = math.cos(r), math.sin(r)
            tx, ty = tx * cr - ty * sr, tx * sr + ty * cr
        return cx, cy, tx, ty, ty, -tx

    def footprint(self, b, s=None, u=None):
        cx, cy, tx, ty, xx, xy = self.pose(b, s, u)
        hl, hw = 0.5 * b.length, 0.5 * self.W
        return [
            (cx + tx * hl + xx * hw, cy + ty * hl + xy * hw),
            (cx + tx * hl - xx * hw, cy + ty * hl - xy * hw),
            (cx - tx * hl - xx * hw, cy - ty * hl - xy * hw),
            (cx - tx * hl + xx * hw, cy - ty * hl + xy * hw),
        ]

    def z_centre(self, k):
        return self.z_base + k * self.pitch + 0.5 * self.H

    def frame(self, b, fitted=False):
        s, u = (b.s0, b.u0) if fitted else (b.s, b.u)
        cx, cy, tx, ty, xx, xy = self.pose(b, s, u)
        return [[cx, cy, self.z_centre(b.k)], [xx, xy, 0.0], [tx, ty, 0.0]]

    def deviation(self, b):
        a = self.pose(b)
        f = self.pose(b, b.s0, b.u0)
        return math.hypot(a[0] - f[0], a[1] - f[1])

    def bricks(self):
        for row in self.courses:
            for b in row:
                yield b

    def drop(self, b, reason):
        """Remove brick b from its course (it keeps its (course, index) label;
        the other bricks keep theirs).  Recorded in self.dropped with its
        pose at the time."""
        self.courses[b.k].remove(b)
        b.status = "dropped"
        self.dropped.append((b, self.frame(b), reason))

    def move(self, b, s, u):
        if self.paths[b.k].closed:
            s = s % self.paths[b.k].length
        b.s, b.u = s, u
        if b.status != "flagged":
            b.status = "moved"

    # -- L0 measures ---------------------------------------------------------
    def supports(self, b, s=None, u=None):
        """[(brick below, overlap area, overlap centroid)] with area > 0."""
        if b.k == 0:
            return []
        cx, cy = self.pose(b, s, u)[:2]
        fp = self.footprint(b, s, u)
        reach = self.L + self.W
        out = []
        for c in self.courses[b.k - 1]:
            px, py = self.pose(c)[:2]
            if math.hypot(px - cx, py - cy) > reach:
                continue
            area, cen = overlap(fp, self.footprint(c))
            if area > 1e-6:
                out.append((c, area, cen))
        return out

    def overhang(self, b, s=None, u=None, sup=None):
        """Signed offset along the brick's X from the area-weighted centre of the
        bricks below it (None when nothing is below)."""
        sup = self.supports(b, s, u) if sup is None else sup
        tot = sum(a for _c, a, _cen in sup)
        if tot <= 0:
            return None
        sx = sum(self.pose(c)[0] * a for c, a, _cen in sup) / tot
        sy = sum(self.pose(c)[1] * a for c, a, _cen in sup) / tot
        cx, cy, _tx, _ty, xx, xy = self.pose(b, s, u)
        return (cx - sx) * xx + (cy - sy) * xy

    def balance(self, b, s=None, u=None):
        """How far b's centre lies inside its contact region with the course
        below [mm]: the smallest of its distances to the four ends of that
        region along b's own axes (Y along the course, X across the wall).
        The region spans all contact polygons together, so a brick bridging
        a joint is measured across both supports.  Negative = the centre is
        outside (the brick tips off on its own).  None for course 0 or a
        brick with no contact (the contact check fails then)."""
        ends = self.balance_ends(b, s, u)
        return min(ends.values()) if ends else None

    def balance_ends(self, b, s=None, u=None):
        """{'t+', 't-', 'x+', 'x-': distance from b's centre to that end of
        its contact region along its own axes [mm]}, or None (see balance)."""
        if b.k == 0:
            return None
        cx, cy, tx, ty, xx, xy = self.pose(b, s, u)
        fp = self.footprint(b, s, u)
        reach = self.L + self.W
        ts, xs = [], []
        for c in self.courses[b.k - 1]:
            px, py = self.pose(c)[:2]
            if math.hypot(px - cx, py - cy) > reach:
                continue
            for qx, qy in overlap_poly(fp, self.footprint(c)):
                ts.append((qx - cx) * tx + (qy - cy) * ty)
                xs.append((qx - cx) * xx + (qy - cy) * xy)
        if not ts:
            return None
        return {"t+": max(ts), "t-": -min(ts), "x+": max(xs), "x-": -min(xs)}

    def head_gaps(self, b, s=None, u=None):
        """[(other brick, clear gap)] for nearby bricks in the same course."""
        cx, cy = self.pose(b, s, u)[:2]
        fp = self.footprint(b, s, u)
        reach = self.L + self.W + self.p["max_head"]
        out = []
        for c in self.courses[b.k]:
            if c is b:
                continue
            px, py = self.pose(c)[:2]
            if math.hypot(px - cx, py - cy) > reach:
                continue
            out.append((c, rect_gap(fp, self.footprint(c))))
        return out

    def check(self, b, s=None, u=None):
        """L0 failures of one brick at (s, u): subset of
        {'contact', 'overhang', 'balance', 'collision'}."""
        p = self.p
        fails = set()
        if b.k > 0:
            sup = self.supports(b, s, u)
            tot = sum(a for _c, a, _cen in sup)
            if tot < p["min_contact"] * b.length * self.W:
                fails.add("contact")
            ov = self.overhang(b, s, u, sup)
            if ov is not None and abs(ov) > p["max_overhang"] + 1e-6:
                fails.add("overhang")
            bal = self.balance(b, s, u) if tot > 0 else None
            if bal is not None and bal < p["min_balance"] - 1e-6:
                fails.add("balance")
        for _c, gap in self.head_gaps(b, s, u):
            if gap < p["min_head"] - 1e-6:
                fails.add("collision")
                break
        return fails
