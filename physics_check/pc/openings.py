"""Opening-aware course placement and constrained L0/L1 repair.

Rectangles use a center fraction on each open course and an arc-length width.
Lintels are ideal concealed supports with end bearing, never fixed upper bricks.
Their capacity is assumed, not designed by this program.
"""
import math
import time

from .wall import Wall, Brick
from .geom2d import Path2, overlap, overlap_poly, rect_gap, poly_signed_area
from .params import resolve_params, MM


def validate_openings(openings):
    if not isinstance(openings, list) or len(openings) > 1:
        raise ValueError("This version supports one rectangular opening; the document stores a list for future expansion.")
    out = []
    for raw in openings:
        required = {"id", "center_fraction", "width_mm", "height_mm", "sill_mm", "lintel", "bearing_mm", "allow_custom", "min_piece_mm"}
        if not isinstance(raw, dict) or set(raw) != required or not isinstance(raw["id"], str) or not raw["id"]:
            raise ValueError("Invalid opening fields")
        o = dict(raw)
        for name, lo, hi in (("center_fraction", 0, 1), ("width_mm", 50, 100000), ("height_mm", 50, 100000),
                             ("sill_mm", 0, 100000), ("bearing_mm", 50, 2000), ("min_piece_mm", 20, 190)):
            v = o[name]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
                raise ValueError(f"{name} must be between {lo} and {hi}")
            o[name] = float(v)
        if type(o["lintel"]) is not bool or type(o["allow_custom"]) is not bool:
            raise ValueError("lintel and allow_custom must be boolean")
        out.append(o)
    return out


def effective_opening(o, p, n_courses):
    pitch = p["brickH"] + p["mortar_t"]
    low = int(math.floor(o["sill_mm"]/pitch + 1e-9))
    high = int(math.ceil((o["sill_mm"]+o["height_mm"])/pitch - 1e-9))
    needed = high + 1
    if needed > n_courses:
        raise ValueError(f"Opening needs a wall of at least {needed*pitch-p['mortar_t']:.0f} mm including head clearance; enlarge the wall or reduce the opening.")
    return {**o, "bottom_course": low, "head_course": high,
            "effective_sill_mm": low*pitch, "effective_head_mm": high*pitch,
            "effective_height_mm": (high-low)*pitch}


def fit_interval(lo, hi, k, p, allow_custom=True, minimum=50):
    """Fit the interval before instantiating bricks; prefer full/half pieces.

All interior pieces are full length. Custom pieces are confined to interval
ends. Candidate layouts share a global course parity/phase preference.
"""
    span, L = hi-lo, p["brickL"]
    if span < minimum-1e-6 and abs(span-L/2)>1e-5:
        raise ValueError(f"Placement region {span:.1f} mm is shorter than the minimum piece {minimum:g} mm")
    best = None
    def offer(lens, gap):
        nonlocal best
        if any((x < minimum-1e-6 and abs(x-L/2)>1e-5) or x > L+1e-6 for x in lens): return
        custom = sum(min(abs(x-L), abs(x-L/2)) > 1e-5 for x in lens)
        if custom and not allow_custom: return
        cur, pieces = lo, []
        for ln in lens:
            pieces.append((cur+ln/2, ln))
            cur += ln+gap
        pitch = L+p["head_joint"]
        phase = (p["offset_distance"] if k % 2 else 0) + L/2
        bond = sum(abs((s-phase+pitch/2) % pitch-pitch/2) for s, _ in pieces)/len(pieces)
        score = (custom, bond, abs(gap-p["head_joint"]), sum(abs(x-L/2) < 1e-5 for x in lens))
        if best is None or score < best[0]: best = (score, pieces, gap)
    if span <= L+1e-6: offer([span], 0)
    for n in range(2, int(span/(L+p["min_head"]))+4):
        for a in (L, L/2):
            for b in (L, L/2):
                lens = [a]+[L]*(n-2)+[b]
                gap = (span-sum(lens))/(n-1)
                if p["min_head"]-1e-6 <= gap <= p["max_head"]+1e-6:
                    offer(lens, gap)
        if allow_custom:
            for gap in (p["head_joint"], p["min_head"], p["max_head"]):
                for a in (L, L/2):
                    remainder = span-(n-2)*L-a-(n-1)*gap
                    offer([a]+[L]*(n-2)+[remainder], gap)
                    offer([remainder]+[L]*(n-2)+[a], gap)
                end = (span-(n-2)*L-(n-1)*gap)/2
                offer([end]+[L]*(n-2)+[end], gap)
    if best is None:
        raise ValueError(f"Cannot fit region {span:.1f} mm with the allowed pieces/joints. Enable custom pieces, reduce minimum length, or adjust the opening.")
    return best[1], best[2]


class OpeningWall(Wall):
    preserve_regions = True

    def movable(self, b):
        return b.k > 0 and self.kinds[b] != "lintel"

    def cra_contacts(self, nodes):
        indices = {b: i for i, b in enumerate(nodes)}
        out = []
        for b in nodes:
            for c in self.support_candidates(b):
                if c not in indices: continue
                z = self.z_base + b.k*self.pitch - self.p["mortar_t"]/2
                if self.kinds[b] == "lintel" or self.kinds[c] == "lintel":
                    z = self.z_base + b.k*self.pitch
                for poly in self.contact_polygons(b, c):
                    out.append((indices[c], indices[b], [[x*MM, y*MM, z*MM] for x, y in poly]))
        return out

    def boundary_margins(self, b):
        if self.kinds[b] == "lintel": return (0., 0.)
        _, lo, hi = self.regions[b]
        ss = [self.paths[b.k].nearest(q)[0] for q in self.footprint(b)]
        return min(ss)-lo+1e-5, hi-max(ss)+1e-5

    def hard_constraints_ok(self):
        return all(min(self.boundary_margins(b)) >= -1e-7 for b in self.bricks())

    def constraint_metrics(self, b, neighbors):
        """Exact signed L0 margins in mm, including opening and bearing limits."""
        p = self.p
        metrics = list(self.boundary_margins(b))
        if b.k > 0:
            sup = self.supports(b)
            if self.kinds[b] == "lintel":
                left, right, bearing = self.lintel_opening[b]
                for side in (lambda c: c.s < left, lambda c: c.s > right):
                    metrics.append(sum(a for c, a, _ in sup if side(c))/self.W-p["min_contact"]*bearing)
            else:
                metrics.append(sum(a for _, a, _ in sup)/self.W-p["min_contact"]*b.length)
            ov = self.overhang(b, sup=sup)
            metrics.extend([p["max_overhang"]-(ov or 0), p["max_overhang"]+(ov or 0)])
            ends = self.balance_ends(b)
            metrics.extend([(ends[key] if ends else 0)-p["min_balance"] for key in ("t+", "t-", "x+", "x-")])
        for c in neighbors:
            metrics.append(rect_gap(self.footprint(b), self.footprint(c))-p["min_head"])
        return metrics

    def violation(self, b):
        neighbors = [c for c in self.courses[b.k] if c is not b and self.kinds[c] != "lintel"] if self.kinds[b] != "lintel" else []
        return sum(max(0., -v) for v in self.constraint_metrics(b, neighbors))

    def constraint_rows(self, R):
        """Linearize exact clipped contacts for the existing trust-region QP.

        The ordinary analytic rows assume uninterrupted courses. Finite
        differences here include the plate and disjoint masonry intervals.
        """
        rows = []
        for b in self.bricks():
            cx, cy = self.pose(b)[:2]
            reach = b.length + self.L + self.W + 2*R
            near = lambda c: math.hypot(self.pose(c)[0]-cx, self.pose(c)[1]-cy) <= reach
            neighbors = [c for c in self.courses[b.k] if c is not b and self.kinds[c] != "lintel" and near(c)] if self.kinds[b] != "lintel" else []
            deps = set([b]+neighbors+self.support_candidates(b))
            base = self.constraint_metrics(b, neighbors)
            co = [{} for _ in base]; rot = [{} for _ in base]
            for c in deps:
                if not self.movable(c) or (self.kinds[b] != "lintel" and not near(c)): continue
                for ax, h in (("s", .05), ("u", .05), ("r", 1e-4)):
                    if ax == "r" and self.p["max_rotation"] <= 0: continue
                    saved = getattr(c, ax)
                    try:
                        setattr(c, ax, saved+h); plus = self.constraint_metrics(b, neighbors)
                        setattr(c, ax, saved-h); minus = self.constraint_metrics(b, neighbors)
                    finally:
                        setattr(c, ax, saved)
                    for i, (vp, vm) in enumerate(zip(plus, minus)):
                        slope = (vp-vm)/(2*h)
                        if abs(slope) > 1e-9:
                            if ax == "r": rot[i][c] = slope
                            else: co[i][(c, ax)] = slope
            for i, val in enumerate(base):
                rows.append((co[i], val, 0., lambda d=rot[i]: d))
        return rows

    def __init__(self, paths, z_base, p, openings):
        self.opening_setup = True
        self.regions, self.kinds, self.lintel_opening = {}, {}, {}
        self.openings = [effective_opening(o, p, len(paths)) for o in validate_openings(openings)]
        super().__init__(paths, z_base, p, [1]*len(paths))
        self.opening_setup = False
        self.place_regions()

    def place_course(self, k, n, widen=False):
        # Base initialization must not place a continuous row across the opening.
        if self.opening_setup: return
        raise ValueError("Opening layouts must be regenerated by regions")

    def fit_region(self, a, b, k, o):
        """fit_interval spaces pieces by arc length along the course line; on
        the inside of a curve the real gap between the rectangular footprints
        is smaller.  Refit with a larger minimum joint until the real gaps
        reach min_head (as the plain bond layout measures them)."""
        p = dict(self.p)
        best = None
        for _ in range(6):
            try:
                pieces, gap = fit_interval(a, b, k, p, o["allow_custom"], o["min_piece_mm"])
            except ValueError:
                if best is None:
                    raise
                break
            feet = [self.footprint(Brick(k, -1, s, ln)) for s, ln in pieces]
            real = min([rect_gap(f, g) for f, g in zip(feet, feet[1:])] or [float("inf")])
            short = self.p["min_head"] - real
            if best is None or short < best[0]:
                best = (short, pieces, gap)
            if short <= 1e-6:
                break
            p["min_head"] += short + 0.5
            p["head_joint"] = max(p["head_joint"], p["min_head"])
            p["max_head"] = max(p["max_head"], p["min_head"])
        return best[1], best[2]

    def place_regions(self):
        o = self.openings[0]
        for k, path in enumerate(self.paths):
            if path.closed: raise ValueError("Openings currently require an open wall, not a closed loop")
            center = o["center_fraction"]*path.length
            lo, hi = center-o["width_mm"]/2, center+o["width_mm"]/2
            clearance = o["bearing_mm"]+self.p["min_head"] if o["lintel"] else o["min_piece_mm"]
            if lo < clearance or path.length-hi < clearance:
                raise ValueError(f"Opening does not leave enough jamb/bearing width on course {k}")
            available = [(0., path.length)]
            lintel = None
            if o["bottom_course"] <= k < o["head_course"]:
                available = [(0., lo), (hi, path.length)]
            elif k == o["head_course"] and o["lintel"]:
                a, b = lo-o["bearing_mm"], hi+o["bearing_mm"]
                lintel = (a, b)
            self.courses[k] = []
            gaps = []
            for region, (a, b) in enumerate(available):
                pieces, gap = self.fit_region(a, b, k, o)
                gaps.append(gap)
                for s, length in pieces:
                    brick = Brick(k, len(self.courses[k]), s, length)
                    self.courses[k].append(brick)
                    self.regions[brick] = (region, a, b)
                    self.kinds[brick] = "full" if abs(length-self.L)<1e-5 else "half" if abs(length-self.L/2)<1e-5 else "custom"
            if lintel:
                a, b = lintel
                brick = Brick(k, len(self.courses[k]), (a+b)/2, b-a)
                self.courses[k].append(brick)
                self.regions[brick] = (-1, a, b)
                self.kinds[brick] = "lintel"
                self.lintel_opening[brick] = (lo, hi, o["bearing_mm"])
                brick.notes.append("Hidden, massless ideal lintel; capacity and deflection assumed; installed before head-course masonry.")
            self.counts[k] = len(self.courses[k])
            self.steps[k] = self.L + sum(gaps)/len(gaps)
        self.boundary_adjustments = []
        for b in self.bricks():
            if self.kinds[b] == "lintel": continue
            _, lo, hi = self.regions[b]
            def fits(length):
                b.length = length
                ss = [self.paths[b.k].nearest(point)[0] for point in self.footprint(b)]
                return min(ss) >= lo-1e-5 and max(ss) <= hi+1e-5
            original = b.length
            if fits(original): continue
            if o["allow_custom"]:
                lower, upper = o["min_piece_mm"], original
                if lower > upper or not fits(lower):
                    raise ValueError(f"Course {b.k}: even a {lower:g} mm piece crosses the opening boundary. Adjust the opening or minimum piece length.")
                for _ in range(35):
                    mid = (lower+upper)/2
                    if fits(mid): lower = mid
                    else: upper = mid
                b.length = lower
                self.kinds[b] = "custom"
            elif original >= self.L/2 and fits(self.L/2):
                self.kinds[b] = "half"
            else:
                raise ValueError(f"Course {b.k}: enable custom jamb pieces to fit this curved opening edge.")
            b.notes.append(f"Boundary fit: shortened {original:.3f} to {b.length:.3f} mm to avoid the opening.")
            self.boundary_adjustments.append({"course": b.k, "index": b.i, "original_length_mm": original, "length_mm": b.length})

    def frame(self, b, fitted=False):
        frame = super().frame(b, fitted)
        if self.kinds.get(b) == "lintel":
            frame[0][2] = self.z_base + b.k*self.pitch
        return frame

    def analysis_volume(self, b):
        return 0. if self.kinds[b] == "lintel" else self.W*b.length*self.pitch

    def support_candidates(self, b):
        if b.k == 0: return []
        lintels = [c for c in self.courses[b.k] if self.kinds[c] == "lintel"]
        if self.kinds[b] != "lintel" and lintels:
            # Use the concealed plate when the brick overlaps it. Other head
            # bricks retain their direct jamb contacts; no plate is grounded.
            if any(overlap(self.footprint(b), self.footprint(c))[0] > 1e-6 for c in lintels):
                return lintels + self.courses[b.k-1]
        return [c for c in self.courses[b.k-1] if self.kinds[c] != "lintel"]

    def contact_polygons(self, b, c, s=None, u=None):
        poly = overlap_poly(self.footprint(b, s, u), self.footprint(c))
        if not poly: return []
        plates = [v for v in self.courses[b.k] if self.kinds[v] == "lintel"]
        if self.kinds[b] == "lintel" or self.kinds[c] == "lintel" or not plates:
            return [poly]
        # Subtract the plate footprint from direct masonry contacts. This
        # avoids counting the same bearing area through two parallel paths.
        clip = self.footprint(plates[0])
        if poly_signed_area(clip) < 0: clip = clip[::-1]
        def halfplane(points, a, z, keep_inside):
            out = []
            def side(p): return (z[0]-a[0])*(p[1]-a[1])-(z[1]-a[1])*(p[0]-a[0])
            if not points: return out
            prev = points[-1]; dp = side(prev)
            for cur in points:
                dc = side(cur)
                pin = dp >= 0 if keep_inside else dp <= 0
                cin = dc >= 0 if keep_inside else dc <= 0
                if pin != cin:
                    t = dp/(dp-dc)
                    out.append((prev[0]+t*(cur[0]-prev[0]), prev[1]+t*(cur[1]-prev[1])))
                if cin: out.append(cur)
                prev, dp = cur, dc
            return out
        pieces, remaining = [], poly
        for a, z in zip(clip, clip[1:]+clip[:1]):
            piece = halfplane(remaining, a, z, False)
            if len(piece) >= 3 and abs(poly_signed_area(piece)) > 1e-6: pieces.append(piece)
            remaining = halfplane(remaining, a, z, True)
        return pieces

    def supports(self, b, s=None, u=None):
        if b.k == 0: return []
        out = []
        for c in self.support_candidates(b):
            for poly in self.contact_polygons(b, c, s, u):
                area, centroid = overlap(poly, poly)
                if area > 1e-6: out.append((c, area, centroid))
        return out

    def balance_ends(self, b, s=None, u=None):
        if b.k == 0: return None
        cx, cy, tx, ty, xx, xy = self.pose(b, s, u)
        ts, xs = [], []
        for c in self.support_candidates(b):
            for poly in self.contact_polygons(b, c, s, u):
                for x, y in poly:
                    ts.append((x-cx)*tx+(y-cy)*ty)
                    xs.append((x-cx)*xx+(y-cy)*xy)
        return {"t+": max(ts), "t-": -min(ts), "x+": max(xs), "x-": -min(xs)} if ts else None

    def head_gaps(self, b, s=None, u=None):
        if self.kinds[b] == "lintel": return []
        return [(c, rect_gap(self.footprint(b, s, u), self.footprint(c))) for c in self.courses[b.k] if c is not b and self.kinds[c] != "lintel"]

    def check(self, b, s=None, u=None):
        fails = super().check(b, s, u)
        if self.p["max_move"] > 0 and (abs((b.s if s is None else s)-b.s0) > self.p["max_move"]+1e-6 or abs((b.u if u is None else u)-b.u0) > self.p["max_move"]+1e-6):
            fails.add("movement_limit")
        _, lo, hi = self.regions[b]
        ss = [self.paths[b.k].nearest(point)[0] for point in self.footprint(b, s, u)]
        if self.kinds[b] != "lintel" and (min(ss)<lo-1e-5 or max(ss)>hi+1e-5): fails.add("opening_boundary")
        if self.kinds[b] == "lintel":
            # Require actual contact on both jambs; do not ground/fix this block.
            left, right, bearing = self.lintel_opening[b]
            contacts = self.supports(b, s, u)
            areas = [sum(area for c, area, _ in contacts if c.s < left),
                     sum(area for c, area, _ in contacts if c.s > right)]
            if min(areas) < self.p["min_contact"]*bearing*self.W:
                fails.add("lintel_bearing")
            else:
                fails.discard("contact")  # bearing regions, not entire beam span
        return fails


def run_opening_check(job, use_cra=True):
    from .cra import run_cra
    from .stages import stage_check
    from .l0qp import l0_repair_qp
    from .l1 import l1_repair
    from .loop import report, _finite, log_fit
    from .fit import FitError, fit_sections
    start = time.time()
    p = resolve_params(job.get("params", {}))
    result = {"units": "mm", "brickL": p["brickL"], "brickW": p["brickW"], "brickH": p["brickH"],
              "mortar_t": p["mortar_t"], "params": p, "bricks": [], "interfaces": [], "frames": [],
              "dropped": [], "stages": None, "use_cra": bool(use_cra), "solver": p["solver"],
              "opening_assumptions": "Opening-aware L0 trust-region QP and L1-guided repair preserve exclusion boundaries and the selected CRA/RBE solver, friction and tensile limits. Region piece counts are retained during repair; automatic dropping/rebonding across openings is disabled. Concealed lintels are hidden, massless ideal rigid supports with explicit jamb contacts, never grounded. Capacity, self-weight, deflection and metal section design are assumed, not checked. Curved lintels use a straight chord proxy. Construction stages use the configured stage solver; installation within a course is not checked.",
              "log": [], "cra_time": 0., "ok": False, "l0_pass": False, "l1_pass": False, "l1_state": "skipped", "fatal": None}
    try:
        # Same course lines as the plain wall: fit_sections clamps how far each
        # course line may sit sideways from the one below (max_overhang).  On
        # a leaning surface the raw section curves stack bricks whose
        # sideways balance can only be fixed by moving whole columns, which
        # the single-brick L0 repair cannot do.
        sections = [(s["points"], s.get("closed", False)) for s in job["sections"]]
        paths, _counts, _joints, info = fit_sections(sections, p)
        log_fit(paths, info, p, result["log"])
        wall = OpeningWall(paths, float(job.get("z_base", 0)), p, job["openings"])
        result["openings"] = wall.openings
        result["boundary_adjustments"] = wall.boundary_adjustments
        cra = {"state": "skipped", "interfaces": []}
        l0_ok = l1_ok = False
        result["equilibrium_method"] = p["solver"] + "_explicit_opening_contacts"
        result["optimization_history"] = []
        for iteration in range(1, p["max_iter"]+1):
            l0_ok, fatal = l0_repair_qp(wall, result["log"])
            cra = {"state": "skipped", "interfaces": []}
            step = {"iteration": iteration, "l0_pass": l0_ok, "l1_state": "skipped", "l1_moves": 0}
            result["optimization_history"].append(step)
            if fatal or not l0_ok:
                result["fatal"] = fatal
                result["log"].append("stop: opening-constrained L0 repair exhausted; L1 requires valid geometry")
                break
            if not use_cra:
                result["log"].append("L0 repair complete; geometry-only mode explicitly skips L1")
                break
            t = time.time()
            cra = run_cra(wall)
            result["cra_time"] += time.time()-t
            step["l1_state"] = cra["state"]
            step["solver_error"] = cra.get("error")
            step["solver_diagnostics"] = cra.get("solver")
            step["failed_interfaces"] = sum(bool(i["fails"]) for i in cra["interfaces"])
            result["log"].append(f"iteration {iteration}: L0 passed; {p['solver']} {cra['state']}, {step['failed_interfaces']} failing interfaces")
            l1_ok = cra["state"] == "solved" and step["failed_interfaces"] == 0
            if l1_ok:
                result["log"].append("L1 passed with explicit concealed-lintel load paths")
                break
            if cra["state"] == "timeout" or iteration == p["max_iter"]:
                result["log"].append("stop: solver timeout or iteration limit; final result remains checked at its reported geometry")
                break
            # Flags from previous trials must not permanently exclude bricks
            # that have since passed L0 from the L1 feedback search.
            for b in wall.bricks():
                if not wall.check(b): b.status = "passed"
            step["l1_moves"] = l1_repair(wall, cra, result["log"])
            if not step["l1_moves"]:
                result["log"].append("stop: L1 found no further permitted moves")
                break
        result["iterations"] = len(result["optimization_history"])
        failures = {b: wall.check(b) for b in wall.bricks()}
        if p["stage_check"] and use_cra and l0_ok and cra["state"] == "solved":
            result["stages"], secs = stage_check(wall, result["log"])
            result["cra_time"] += secs
        bad = {i["upper"] for i in cra["interfaces"] if i["fails"]}
        l1_ok = cra["state"] == "solved" and not bad
        for b in wall.bricks():
            result["bricks"].append({"course": b.k, "index": b.i, "status": "failed" if failures[b] or b in bad else "moved" if wall.deviation(b) > .5 or abs(b.r-b.r0) > math.radians(.5) else "passed",
                "length": b.length, "kind": wall.kinds[b], "hidden": wall.kinds[b] == "lintel", "region": list(wall.regions[b]),
                "frame": wall.frame(b), "fitted_frame": wall.frame(b, fitted=True), "deviation": wall.deviation(b), "rotation": math.degrees(b.r-b.r0),
                "l0_fail": sorted(failures[b]), "notes": b.notes})
        for i in cra["interfaces"]:
            result["interfaces"].append({"upper": [i["upper"].k, i["upper"].i], "lower": [i["lower"].k, i["lower"].i],
                "centroid": [c/MM for c in i["centroid"]], "sigma_t": _finite(i["sigma_t"]) if i["sigma_t"] is not None else None, "P": i["P"],
                "ratio": _finite(i["ratio"]) if i["ratio"] is not None else None, "tensioned": i["tensioned"], "fails": i["fails"]})
        result.update(ok=bool(l0_ok and l1_ok), l0_pass=l0_ok, l1_pass=l1_ok, l1_state=cra["state"],
                      bricks_per_course=wall.counts, head_joints=[wall.head_joint(k) for k in range(len(paths))])
        result["custom_piece_schedule"] = [{"course": b["course"], "index": b["index"], "length_mm": b["length"]} for b in result["bricks"] if b["kind"] == "custom"]
        result["frames"] = [b["frame"] for b in result["bricks"]]
    except (ValueError, RuntimeError, FitError) as exc:
        result["fatal"] = str(exc)
    result["log"].append(result["opening_assumptions"])
    result["time"] = time.time()-start
    result["report"] = report(result)
    return result
