"""Steps 3-4: L0 check and repair (smallest move that passes, head-joint
redistribution)."""

from __future__ import division

import math

from .geom2d import rect_gap

DIRS16 = [(math.cos(2 * math.pi * j / 16.0), math.sin(2 * math.pi * j / 16.0)) for j in range(16)]
DIRS_LATERAL = [(0.0, 1.0), (0.0, -1.0)]


def smallest_move(wall, b, dirs, ok):
    """Smallest (distance, s, u) along any of dirs (ds, du per mm) where ok(s, u)."""
    p = wall.p
    step, max_move = p["move_step"], p["max_move"]
    best = None
    for ds, du in dirs:
        prev, d, found = 0.0, step, None
        while d <= max_move + 1e-9:
            if best is not None and d >= best[0]:
                break
            if ok(b.s + ds * d, b.u + du * d):
                found = d
                break
            prev, d = d, d + step
        if found is None:
            continue
        lo, hi = prev, found
        for _ in range(8):
            mid = 0.5 * (lo + hi)
            if ok(b.s + ds * mid, b.u + du * mid):
                hi = mid
            else:
                lo = mid
        if best is None or hi < best[0]:
            best = (hi, b.s + ds * hi, b.u + du * hi)
    return best


def _course_order(wall, k):
    return sorted(wall.courses[k], key=lambda b: b.s)


def repair_spacing(wall, k):
    """Push colliding bricks of course k apart along the course, taking the
    space from the other head joints.  Returns None or an error message."""
    p = wall.p
    path = wall.paths[k]
    closed = path.closed
    for _it in range(60):
        bs = _course_order(wall, k)
        n = len(bs)
        nj = n if closed else n - 1
        fps = [wall.footprint(b) for b in bs]
        viol = []
        for i in range(n):
            for j in range(i + 1, n):
                g = rect_gap(fps[i], fps[j])
                if g < p["min_head"] - 1e-6:
                    viol.append((i, j, p["min_head"] - g))
        if not viol:
            break
        adj = [rect_gap(fps[q], fps[(q + 1) % n]) for q in range(nj)]
        add = [0.0] * nj
        for i, j, d in viol:
            joints = list(range(i, j))
            if closed and (j - i) > n / 2.0:
                joints = list(range(j, n)) + list(range(0, i))
            per = (d + 0.25) / len(joints)
            for q in joints:
                add[q] = max(add[q], per)
        need = sum(add)
        slack = [0.0 if add[q] > 0 else max(0.0, adj[q] - p["min_head"]) for q in range(nj)]
        total = sum(slack)
        if total < need:
            return ("course {}: cannot keep every head joint >= min_head ({:.0f} mm) with "
                    "its {} bricks".format(k, p["min_head"], n))
        for q in range(nj):
            add[q] -= need * slack[q] / total
        e = [bs[q + 1].s - bs[q].s if q < n - 1 else bs[0].s + path.length - bs[q].s
             for q in range(nj)]
        new_s = [bs[0].s]
        for q in range(n - 1):
            new_s.append(new_s[-1] + e[q] + add[q])
        if not closed:
            shift = (sum(b.s for b in bs) - sum(new_s)) / n
            new_s = [s + shift for s in new_s]
        for b, s in zip(bs, new_s):
            if abs(s - b.s) > 1e-9:
                wall.move(b, s, b.u)
    else:
        return "course {}: head-joint redistribution did not converge".format(k)
    bs = _course_order(wall, k)
    n = len(bs)
    nj = n if closed else n - 1
    for q in range(nj):
        g = rect_gap(wall.footprint(bs[q]), wall.footprint(bs[(q + 1) % n]))
        if g > p["max_head"] + 1e-6 and not wall.widened[k]:
            return ("course {}: head joint {:.1f} mm exceeds max_head {:.0f} mm after "
                    "redistribution".format(k, g, p["max_head"]))
    return None


def settle_counts(wall, log):
    """Part of the fit (step 2): make every course collision-free before the
    checks.  Chord bricks on a curve lose head joint on the concave side, so a
    colliding course first has its head joints redistributed within
    [min_head, max_head]; if that cannot work, it drops a brick and is
    re-spaced over its full length, so its ends stay in line with the courses
    above and below (its head joints may then exceed max_head; reported).
    The settled positions are the fitted reference."""
    for k in range(len(wall.courses)):
        n0 = n = len(wall.courses[k])
        while wall.course_collides(k) and repair_spacing(wall, k) and n > 1:
            n -= 1
            wall.place_course(k, n, widen=True)
        for b in wall.courses[k]:
            b.s0, b.u0, b.status = b.s, b.u, "passed"
        if n != n0:
            msg = "course {}: {} -> {} bricks (curvature: head joints < min_head)".format(k, n0, n)
            if wall.widened[k]:
                msg += "; head joints widened to {:.1f} mm (> max_head {:.0f} mm)".format(
                    wall.head_joint(k), wall.p["max_head"])
            log.append(msg)


def l0_failures(wall):
    return {b: f for b in wall.bricks() for f in [wall.check(b)] if f}


def l0_repair(wall, log):
    """Steps 3-4, repeated until clean.  Returns (passed, fatal message)."""
    p = wall.p
    for it in range(p["l0_max_iter"]):
        fails = l0_failures(wall)
        live = {b: f for b, f in fails.items() if not b.flagged}
        if not live:
            return not fails, None
        log.append("L0 pass {}: {} brick(s) failing".format(it + 1, len(live)))
        for k in range(len(wall.courses)):
            if any("collision" in f for b, f in live.items() if b.k == k):
                err = repair_spacing(wall, k)
                if err:
                    return False, err
            for b in wall.courses[k]:
                if b.flagged:
                    continue
                f = wall.check(b)
                if not f:
                    continue
                dirs = DIRS_LATERAL if f <= {"overhang", "thickness"} else DIRS16
                res = smallest_move(wall, b, dirs, lambda s, u, b=b: not wall.check(b, s, u))
                if res is None:
                    b.flag("L0 {} unfixable within limits".format("/".join(sorted(f))))
                else:
                    wall.move(b, res[1], res[2])
    fails = l0_failures(wall)
    return not fails, None
