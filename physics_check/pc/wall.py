"""Wall model: bricks placed on the fitted course centrelines, and the
per-brick L0 measures (step 3)."""

from __future__ import division

import math

from .geom2d import overlap, overlap_poly, rect_gap


class Brick(object):
    __slots__ = ("k", "i", "s", "u", "s0", "u0", "st", "ut", "length", "status", "notes")

    def __init__(self, k, i, s, length):
        self.k, self.i = k, i
        self.length = length    # along the course: brickL, or brickL / 2 for a half brick
        self.s = self.s0 = s
        self.u = self.u0 = 0.0
        # target the QP L0 repair pulls the brick back to: the fitted
        # position, or where L1 last moved it (l0qp.py)
        self.st, self.ut = s, 0.0
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
    def pose(self, b, s=None, u=None):
        """(cx, cy, tx, ty, xx, xy): plan centre, run axis Y, overhang axis X."""
        s = b.s if s is None else s
        u = b.u if u is None else u
        path = self.paths[b.k]
        cx, cy = path.point(s)
        tx, ty = path.tangent(s, 0.5 * b.length)
        xx, xy = ty, -tx
        return cx + u * xx, cy + u * xy, tx, ty, xx, xy

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
        return min(-min(ts), max(ts), -min(xs), max(xs))

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
