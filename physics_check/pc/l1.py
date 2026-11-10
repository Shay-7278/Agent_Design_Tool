"""Step 6: move bricks above failing interfaces (top-down), or reduce the
overhang of suspected sliding bricks when the CRA solve does not converge."""

from __future__ import division

import math

from .params import EPS, MM, move_cap


def _toward(wall, b, dir_xy, dist, label):
    """Move b by dist along the plan direction dir_xy, shrinking the step until
    L0 still passes; flag b if even min_step fails.  Returns True if moved."""
    p = wall.p
    _cx, _cy, tx, ty, xx, xy = wall.pose(b)
    ln = math.hypot(*dir_xy)
    if ln < EPS:
        b.flag(label + ": no move direction")
        return False
    dx, dy = dir_xy[0] / ln, dir_xy[1] / ln
    ds, du = dx * tx + dy * ty, dx * xx + dy * xy
    d = min(max(dist, p["min_step"]), move_cap(p))
    while d >= p["min_step"] - 1e-9:
        s, u = b.s + ds * d, b.u + du * d
        if not wall.check(b, s, u):
            wall.move(b, s, u)
            b.st, b.ut = b.s, b.u      # the QP L0 repair keeps this move (l0qp.py)
            return True
        d *= 0.5
    b.flag(label + ": no passing position within the limits")
    return False


def _toward_support(wall, b):
    sup = wall.supports(b)
    tot = sum(a for _c, a, _cen in sup)
    cx, cy = wall.pose(b)[:2]
    if tot <= 0:
        return None
    sx = sum(wall.pose(c)[0] * a for c, a, _cen in sup) / tot
    sy = sum(wall.pose(c)[1] * a for c, a, _cen in sup) / tot
    return (sx - cx, sy - cy)


def l1_repair(wall, cra, log):
    """Step 6 moves.  Returns the number of bricks moved."""
    p = wall.p
    moved = 0
    if cra["state"] == "geometry":
        for b in cra["floating"]:
            b.flag("L1: block has no load path to course 0 (geometry)")
        log.append("L1: {} floating brick(s) - geometry problem".format(len(cra["floating"])))
        return 0
    if cra["state"] == "failed":
        # Likely sliding: tension cannot help.  The solve does not say which
        # bricks slide, so reduce the overhang of the most overhanging ones.
        ovs = [(abs(wall.overhang(b) or 0.0), b) for b in wall.bricks() if b.k > 0 and not b.flagged]
        worst = max([o for o, _b in ovs] or [0.0])
        if worst <= 1e-3:
            log.append("L1 failed with no overhang to reduce ({})".format(cra.get("error")))
            return 0
        sus = [b for o, b in ovs if o >= 0.9 * worst]
        log.append("L1 did not converge ({}); suspected sliding, reducing overhang of {} brick(s)"
                   .format(cra.get("error"), len(sus)))
        for b in sorted(sus, key=lambda b: -b.k):
            if "L1: suspected sliding" not in b.notes:
                b.notes.append("L1: suspected sliding")
            d = _toward_support(wall, b)
            if d and _toward(wall, b, d, p["slide_step"], "L1 sliding"):
                moved += 1
        return moved

    failing = [r for r in cra["interfaces"] if r["fails"]]
    # one move per brick: its worst interface, top-down
    worst = {}
    for r in failing:
        b = r["upper"]
        if b.k == 0 or b.flagged:
            continue
        if b not in worst or r["ratio"] > worst[b]["ratio"]:
            worst[b] = r
    for b in sorted(worst, key=lambda b: -b.k):
        r = worst[b]
        note = "L1: sigma_t/(f_t/SF) = {:.2f}".format(r["ratio"])
        b.notes.append(note)
        allow = p["f_t"] / p["SF"]
        if r["cop"] is not None and r["P"] > 0:
            ex = (r["cop"][0] - r["centroid"][0]) / MM
            ey = (r["cop"][1] - r["centroid"][1]) / MM
            e = math.hypot(ex, ey)
            pa = r["P"] / r["A"]
            e_ok = e * (allow + pa) / (r["sigma_t"] + pa) if r["sigma_t"] + pa > 0 else 0.0
            # the overlap centroid follows the brick by ~half the move
            dist = 2.0 * (e - e_ok)
            direction = (-ex, -ey)
        else:
            dist = p["slide_step"]
            direction = _toward_support(wall, b) or (0.0, 0.0)
        if _toward(wall, b, direction, dist, "L1 tension"):
            moved += 1
    return moved
