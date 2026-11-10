"""2D plan geometry: polygon clipping / overlap, gaps, polylines."""

from __future__ import division

import math

from .params import EPS


def poly_signed_area(poly):
    a = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return 0.5 * a


def _clip(subject, clip):
    """Sutherland-Hodgman: convex subject clipped by convex clip polygon."""
    if poly_signed_area(clip) < 0:
        clip = clip[::-1]
    out = list(subject)
    n = len(clip)
    for i in range(n):
        ax, ay = clip[i]
        bx, by = clip[(i + 1) % n]
        inp = out
        out = []
        if not inp:
            break

        def inside(pt):
            return (bx - ax) * (pt[1] - ay) - (by - ay) * (pt[0] - ax) >= -EPS

        def cross(p1, p2):
            x1, y1 = p1
            x2, y2 = p2
            dx, dy = x2 - x1, y2 - y1
            den = (bx - ax) * dy - (by - ay) * dx
            if abs(den) < EPS:
                return p2
            t = ((x1 - ax) * dy - (y1 - ay) * dx) / den
            return (ax + t * (bx - ax), ay + t * (by - ay))

        prev = inp[-1]
        for cur in inp:
            if inside(cur):
                if not inside(prev):
                    out.append(cross(prev, cur))
                out.append(cur)
            elif inside(prev):
                out.append(cross(prev, cur))
            prev = cur
    return out


def overlap(poly_a, poly_b):
    """(area, centroid) of the intersection of two convex polygons."""
    pts = _clip(poly_a, poly_b)
    if len(pts) < 3:
        return 0.0, None
    a = poly_signed_area(pts)
    if abs(a) < EPS:
        return 0.0, None
    cx = cy = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        c = x1 * y2 - x2 * y1
        cx += (x1 + x2) * c
        cy += (y1 + y2) * c
    return abs(a), (cx / (6.0 * a), cy / (6.0 * a))


def overlap_poly(poly_a, poly_b):
    """Vertices of the intersection of two convex polygons ([] if none)."""
    pts = _clip(poly_a, poly_b)
    if len(pts) < 3 or abs(poly_signed_area(pts)) < 1e-6:
        return []
    return pts


def _pt_seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < EPS else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    qx, qy = a[0] + t * dx, a[1] + t * dy
    return math.hypot(p[0] - qx, p[1] - qy)


def rect_gap(poly_a, poly_b):
    """Clear distance between two convex polygons; negative if they overlap."""
    area, _ = overlap(poly_a, poly_b)
    if area > 1e-6:
        return -math.sqrt(area)
    best = float("inf")
    for P, Q in ((poly_a, poly_b), (poly_b, poly_a)):
        n = len(Q)
        for p in P:
            for i in range(n):
                best = min(best, _pt_seg_dist(p, Q[i], Q[(i + 1) % n]))
    return best


def offset_polyline(pts, closed, d):
    """Offset a plan polyline by d along its +X normal (tangent x Z)."""
    if abs(d) < EPS:
        return list(pts)
    n = len(pts)
    out = []
    for i in range(n):
        if closed:
            a, b = pts[i - 1], pts[(i + 1) % n]
        else:
            a, b = pts[max(i - 1, 0)], pts[min(i + 1, n - 1)]
        tx, ty = b[0] - a[0], b[1] - a[1]
        ln = math.hypot(tx, ty) or 1.0
        tx, ty = tx / ln, ty / ln
        out.append((pts[i][0] + d * ty, pts[i][1] - d * tx))
    return out


class Path2(object):
    """Plan polyline parametrised by arc length (open: extrapolated past the ends)."""

    def __init__(self, pts, closed):
        pts = [(float(x), float(y)) for x, y in pts]
        if closed and len(pts) > 2 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 1e-6:
            pts = pts[:-1]
        self.pts = pts
        self.closed = bool(closed)
        segs = list(zip(pts[:-1], pts[1:]))
        if self.closed:
            segs.append((pts[-1], pts[0]))
        self.segs = segs
        self.cum = [0.0]
        for a, b in segs:
            self.cum.append(self.cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = self.cum[-1]

    def _locate(self, s):
        if self.closed:
            s = s % self.length
        lo, hi = 0, len(self.segs) - 1
        if s <= 0:
            return 0, s
        if s >= self.length:
            return hi, s - self.cum[hi]
        while lo < hi:
            mid = (lo + hi) // 2
            if self.cum[mid + 1] < s:
                lo = mid + 1
            else:
                hi = mid
        return lo, s - self.cum[lo]

    def point(self, s):
        i, t = self._locate(s)
        a, b = self.segs[i]
        ln = self.cum[i + 1] - self.cum[i]
        if ln < EPS:
            return a
        return (a[0] + (b[0] - a[0]) * t / ln, a[1] + (b[1] - a[1]) * t / ln)

    def tangent(self, s, half):
        """Unit chord direction between s - half and s + half."""
        a, b = self.point(s - half), self.point(s + half)
        tx, ty = b[0] - a[0], b[1] - a[1]
        ln = math.hypot(tx, ty)
        if ln < EPS:
            i, _ = self._locate(s)
            a, b = self.segs[i]
            tx, ty = b[0] - a[0], b[1] - a[1]
            ln = math.hypot(tx, ty) or 1.0
        return tx / ln, ty / ln

    def nearest(self, p):
        """(s, signed lateral distance along +X) of the closest point to p."""
        best = (float("inf"), 0.0, 0.0)
        for i, (a, b) in enumerate(self.segs):
            dx, dy = b[0] - a[0], b[1] - a[1]
            l2 = dx * dx + dy * dy
            t = 0.0 if l2 < EPS else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
            qx, qy = a[0] + t * dx, a[1] + t * dy
            dist = math.hypot(p[0] - qx, p[1] - qy)
            if dist < best[0]:
                ln = math.sqrt(l2) or 1.0
                lat = (p[0] - qx) * (dy / ln) - (p[1] - qy) * (dx / ln)
                best = (dist, self.cum[i] + t * math.sqrt(l2), lat)
        return best[1], best[2]
