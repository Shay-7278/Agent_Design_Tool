"""Construction-stage check: is the wall stable after each finished course?

L1 (cra.run_cra) solves the finished wall.  A finished wall that passes can
still be unstable while it is built: a brick can be held down by the course
above it, and the mortar has no tensile strength until it sets.  This module
solves the partial wall made of courses 0..K for every K with no tension
(wet mortar) and lists the bricks that need a robot arm to hold them at each
stage.  p["stage_solver"]: "lp" (default) solves a small linear program on
the L0 contact polygons (_LP, about 0.05 s per stage); "rbe" calls compas_cra
(run_cra, a few seconds per stage) and honours p["stage_f_t"].

Per stage K:
* need: the upper bricks of the interfaces that fail (tension over
  stage_f_t / SF).  These are the bricks to hold.
* held_ok: the partial wall re-solved with every brick in need fixed (held by
  an arm) passes.  False means holding these bricks is not enough.
* one_arm: when several bricks are in need, the first single brick (top
  courses first, at most stage_max_tries tried) whose holding alone makes the
  stage pass.  None when no single brick does it.

A brick is held from the stage where it first needs support until the first
later stage that passes without it (usually after the course above is laid).
Stages are whole courses: the states while a course is being laid, brick by
brick, are not checked.
"""

from __future__ import division

import time

from .cra import run_cra
from .params import G, MM


def _key(b):
    return [b.k, b.i]


def _ok(res):
    return res["state"] == "solved" and not res["need"]


class _LP(object):
    """No-tension equilibrium as a linear program (stage_solver = "lp").

    The bricks only turn in plan, so every contact is horizontal and the only
    load is gravity: no horizontal force is needed anywhere, and rigid-block
    equilibrium reduces to vertical forces at the corners of each contact
    polygon (the same polygons L0 uses).  Per free brick: sum of vertical
    forces = weight, and their moments about the brick centre = 0 (3 rows).
    Each corner force is fp - fn with fp, fn >= 0; the LP minimises the total
    tension sum(fn).  Zero tension = stable as a dry stack (what RBE checks,
    friction never matters here); interfaces with fn > 0 need tension, and
    their upper bricks need holding.  Only f_t = 0 is checked (any tension
    fails).
    """

    def __init__(self, wall):
        from .geom2d import overlap_poly
        p = wall.p
        self.wall = wall
        self.weight = {}
        self.centre = {}
        self.itfs = []          # (upper, lower, [(x, y)] in m)
        for b in wall.bricks():
            vol = (wall.W * MM) * (b.length * MM) * ((wall.H + p["mortar_t"]) * MM)
            self.weight[b] = p["density"] * vol * G
            self.centre[b] = tuple(c * MM for c in wall.pose(b)[:2])
            if b.k == 0:
                continue
            fp = wall.footprint(b)
            for c, _a, _cen in wall.supports(b):
                poly = overlap_poly(fp, wall.footprint(c))
                if poly:
                    self.itfs.append((b, c, [(x * MM, y * MM) for x, y in poly]))

    def solve(self, nodes, held=()):
        """{"state": "solved" | "failed", "need": [upper bricks of tensioned interfaces]}"""
        import numpy as np
        from scipy.optimize import linprog
        from scipy.sparse import coo_matrix

        nodes = set(nodes)
        held = set(held)
        free = [b for b in nodes if b.k > 0 and b not in held]
        row = dict((b, 3 * n) for n, b in enumerate(free))
        itfs = [t for t in self.itfs if t[0] in nodes and t[1] in nodes]
        ri, ci, vals = [], [], []
        col = 0
        owner = []              # interface index of each corner
        for n, (up, lo, poly) in enumerate(itfs):
            for x, y in poly:
                # corner force f (compression +) pushes up on `up`, down on `lo`
                for b, sgn in ((up, 1.0), (lo, -1.0)):
                    if b in row:
                        cx, cy = self.centre[b]
                        r = row[b]
                        for dr, v in ((0, sgn), (1, sgn * (x - cx)), (2, sgn * (y - cy))):
                            # fp at col, fn at col + 1 (f = fp - fn)
                            ri += [r + dr, r + dr]
                            ci += [col, col + 1]
                            vals += [v, -v]
                owner.append(n)
                col += 2
        nv = col
        if not free:
            return {"state": "solved", "need": []}
        A = coo_matrix((vals, (ri, ci)), shape=(3 * len(free), nv)).tocsr()
        rhs = np.zeros(3 * len(free))
        for b, r in row.items():
            rhs[r] = self.weight[b]
        cost = np.zeros(nv)
        cost[1::2] = 1.0
        res = linprog(cost, A_eq=A, b_eq=rhs, bounds=(0, None), method="highs")
        if res.status != 0:
            return {"state": "failed", "need": [], "error": res.message}
        tol = 1e-6 * max(self.weight.values())
        fn = res.x[1::2]
        bad = set(owner[j] for j in range(len(fn)) if fn[j] > tol)
        return {"state": "solved", "need": sorted(set(itfs[n][0] for n in bad),
                                                  key=lambda b: (-b.k, b.i))}


class _RBE(object):
    """The same question asked of compas_cra (stage_solver = "rbe" / "cra")."""

    def __init__(self, wall):
        self.wall = wall
        self.allow = wall.p["stage_f_t"] / wall.p["SF"]

    def solve(self, nodes, held=()):
        res = run_cra(self.wall, nodes=nodes, held=held, allow=self.allow)
        out = {"state": res["state"], "need": [], "error": res.get("error")}
        if res["state"] == "solved":
            out["need"] = sorted(set(r["upper"] for r in res["interfaces"] if r["fails"]),
                                 key=lambda b: (-b.k, b.i))
        elif res["state"] == "geometry":
            out["need"] = sorted(res.get("floating", []), key=lambda b: (-b.k, b.i))
        return out


def stage_check(wall, log):
    """Solve courses 0..K for K = 1..top.  Returns (summary dict, solve time [s])."""
    p = wall.p
    use_lp = p["stage_solver"] == "lp"
    solver = _LP(wall) if use_lp else _RBE(wall)
    bricks = list(wall.bricks())
    top = max(b.k for b in bricks) if bricks else 0
    stages = []
    t_cra = 0.0
    held_since = {}    # brick -> first stage it needs holding
    spans = []         # [brick, from stage, to stage (last stage held) or None]
    for K in range(1, top + 1):
        nodes = [b for b in bricks if b.k <= K]
        t0 = time.time()
        res = solver.solve(nodes)
        st = {"course": K, "state": res["state"], "need": [], "held_ok": None, "one_arm": None,
              "arms": 0}
        need = res["need"]
        if need:
            st["held_ok"] = _ok(solver.solve(nodes, held=need))
            if len(need) == 1:
                st["one_arm"] = _key(need[0]) if st["held_ok"] else None
            else:
                for b in need[:max(0, p["stage_max_tries"])]:
                    if _ok(solver.solve(nodes, held=[b])):
                        st["one_arm"] = _key(b)
                        break
            st["arms"] = 1 if st["one_arm"] else len(need)
        st["need"] = [_key(b) for b in need]
        st["time"] = time.time() - t0
        t_cra += st["time"]
        stages.append(st)

        # hold spans: a brick stays held until a stage no longer needs it
        now = set(need)
        for b in list(held_since):
            if b not in now:
                spans.append([b, held_since.pop(b), K - 1])
        for b in need:
            held_since.setdefault(b, K)

        if res["state"] in ("failed", "timeout"):
            log.append("stage {}: {} solve {} ({}); bricks to hold unknown".format(
                K, p["stage_solver"], res["state"], res.get("error") or "cra_timeout"))
    for b, k0 in held_since.items():
        spans.append([b, k0, None])      # still needed when the wall is finished

    unknown = [s["course"] for s in stages if s["state"] in ("failed", "timeout")]
    needy = [s for s in stages if s["need"]]
    multi = [s for s in needy if not s["one_arm"]]
    log.append("construction stages ({}, courses 0..K, f_t = {:.0f} kPa; bricks as course/index): "
               "{} stage(s), {} need "
               "holding, {} need more than one held brick, {:.2f} s{}".format(
                   p["stage_solver"], 0.0 if use_lp else p["stage_f_t"] / 1e3, len(stages),
                   len(needy), len(multi), t_cra,
                   "; solve did not finish at stage(s) " + " ".join(str(k) for k in unknown)
                   if unknown else ""))
    for s in needy:
        log.append("  stage {}: hold {} brick(s) {}; {}".format(
            s["course"], len(s["need"]), " ".join("{}/{}".format(k, i) for k, i in s["need"]),
            "one arm: hold {}/{}".format(*s["one_arm"]) if s["one_arm"] else
            ("holding all of them is stable" if s["held_ok"] else
             "NOT stable even with all of them held")))
    spans.sort(key=lambda t: (t[1], t[0].k, t[0].i))
    for b, k0, k1 in spans:
        log.append("  brick {}/{}: hold from stage {} {}".format(
            b.k, b.i, k0, "until the wall is finished (unstable as built)" if k1 is None else
            "through stage {} (released once course {} is laid)".format(k1, k1 + 1)))
    summary = {
        "f_t": p["stage_f_t"],
        "stages": stages,
        "holds": [{"brick": _key(b), "from": k0, "to": k1} for b, k0, k1 in spans],
        "max_arms": max([s["arms"] for s in stages] or [0]),
        "unknown": unknown,
    }
    return summary, t_cra
