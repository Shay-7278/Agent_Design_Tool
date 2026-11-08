"""Wall model: bricks placed on the fitted course centrelines, and the
per-brick L0 measures (step 3)."""

from __future__ import division

import math

from .geom2d import overlap, rect_gap


class Brick(object):
    __slots__ = ("k", "i", "s", "u", "s0", "u0", "status", "notes")

    def __init__(self, k, i, s):
        self.k, self.i = k, i
        self.s = self.s0 = s
        self.u = self.u0 = 0.0
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
    over the brick length.
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
        for k, n in enumerate(counts):
            self.place_course(k, n)

    def place_course(self, k, n, widen=False):
        """Place n bricks evenly along course k (running bond: odd courses
        shifted by offset_distance).  On an open course the spacing is capped
        at brickL + max_head and the bricks are centred, so leftover length
        stays at the ends; with widen=True the bricks span the full course
        and the head joints may exceed max_head.  Replaces the course's
        bricks."""
        path = self.paths[k]
        step = path.length / n
        start = 0.0
        self.widened[k] = bool(widen) and step > self.L + self.p["max_head"]
        if not path.closed and not widen and step > self.L + self.p["max_head"]:
            # open course: keep joints <= max_head, leave the rest at the ends
            step = self.L + self.p["max_head"]
            start = 0.5 * (path.length - n * step)
        self.steps[k] = step
        off = self.p["offset_distance"] if k % 2 == 1 else 0.0
        row = []
        for i in range(n):
            s = start + (i + 0.5) * step + off
            if path.closed:
                s = s % path.length
            row.append(Brick(k, i, s))
        self.courses[k] = row

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
        """Nominal (arc-length) head joint of course k as placed [mm]."""
        return self.steps[k] - self.L

    # -- poses ---------------------------------------------------------------
    def pose(self, b, s=None, u=None):
        """(cx, cy, tx, ty, xx, xy): plan centre, run axis Y, overhang axis X."""
        s = b.s if s is None else s
        u = b.u if u is None else u
        path = self.paths[b.k]
        cx, cy = path.point(s)
        tx, ty = path.tangent(s, 0.5 * self.L)
        xx, xy = ty, -tx
        return cx + u * xx, cy + u * xy, tx, ty, xx, xy

    def footprint(self, b, s=None, u=None):
        cx, cy, tx, ty, xx, xy = self.pose(b, s, u)
        hl, hw = 0.5 * self.L, 0.5 * self.W
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

    def thickness(self, b, s=None, u=None):
        """Wall thickness envelope at this brick: brickW + |lateral offset of the
        brick centre from the course-0 centreline|."""
        cx, cy = self.pose(b, s, u)[:2]
        _s, lat = self.paths[0].nearest((cx, cy))
        return self.W + abs(lat)

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
        {'contact', 'overhang', 'thickness', 'collision'}."""
        p = self.p
        fails = set()
        if b.k > 0:
            sup = self.supports(b, s, u)
            tot = sum(a for _c, a, _cen in sup)
            if tot < p["min_contact"] * self.L * self.W:
                fails.add("contact")
            ov = self.overhang(b, s, u, sup)
            if ov is not None and abs(ov) > p["max_overhang"] + 1e-6:
                fails.add("overhang")
        if self.thickness(b, s, u) > p["thickness_cap"] + 1e-6:
            fails.add("thickness")
        for _c, gap in self.head_gaps(b, s, u):
            if gap < p["min_head"] - 1e-6:
                fails.add("collision")
                break
        return fails
