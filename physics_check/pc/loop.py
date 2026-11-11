"""Step 7: the outer loop (fit -> L0 -> L1 -> repeat) and the text report."""

from __future__ import division

import math
import time

from .cra import run_cra
from .fit import FitError, fit_sections, surface_lean
from .l0 import l0_repair, settle_counts
from .l0qp import l0_repair_qp
from .l1 import l1_repair
from .params import DEFAULTS, MM, resolve_params
from .wall import Wall


def run_check(job, use_cra=True):
    """Run steps 2-7 on a job dict.

    job = {"z_base": float [mm],
           "params": {name: value} (any of params.DEFAULTS; missing/None -> default),
           "sections": [{"points": [[x, y], ...], "closed": bool}, ...] bottom up,
           "use_cra": bool (optional, default True)}

    CRA (L1) is skipped when use_cra is False or job["use_cra"] is False.

    Returns the result dict written to result.json (its "report" key holds the
    text summary; "time" / "cra_time" hold the total / CRA wall-clock time [s]).
    """
    t_start = time.time()
    use_cra = bool(use_cra) and job.get("use_cra", True) is not False
    cra_time = 0.0
    given = job.get("params") or {}
    p = resolve_params(given)
    log = []
    unknown = sorted(k for k in given if k not in DEFAULTS)
    if unknown:
        log.append("ignored unknown params: " + ", ".join(unknown))
    sections = [(sec["points"], sec.get("closed", False)) for sec in job["sections"]]
    result = {"units": "mm", "brickL": p["brickL"], "brickW": p["brickW"],
              "brickH": p["brickH"], "mortar_t": p["mortar_t"], "params": p}
    try:
        paths, counts, _joints, info = fit_sections(sections, p)
        shifts = ["{} ({} -> {})".format(k, counts[k - 1], counts[k]) for k in range(1, len(paths))
                  if not paths[k].closed and not paths[k - 1].closed and counts[k] != counts[k - 1]]
        if shifts:
            log.append("middle-brick count changes (joints not over the bricks below) at course(s): "
                       + ", ".join(shifts))
        lean, need = surface_lean(info["surface"], p["max_overhang"])
        log.append("surface lean per course, max sideways offset from the course below [mm, signed]: "
                   + " ".join("{:+.0f}".format(v) for v in lean[1:]))
        steep = [k for k in range(1, len(lean)) if abs(lean[k]) > p["max_overhang"]]
        if steep:
            log.append("surface leans more than max_overhang ({:.0f} mm) at course(s): {}".format(
                p["max_overhang"], " ".join(str(k) for k in steep)))
            k_worst = max(range(len(need)), key=lambda k: need[k])
            capped = 0 < p["max_move"] < need[k_worst]
            log.append("estimated deviation from the surface forced by max_overhang: up to {:.0f} mm "
                       "at course {}{}".format(
                           need[k_worst], k_worst,
                           "; above max_move ({:.0f} mm), L0 cannot pass there".format(p["max_move"])
                           if capped else ""))
        moved = [k for k in range(len(paths)) if info["moves"][k] > 0.5 or info["trims"][k] > 0.5]
        if moved:
            k_worst = max(moved, key=lambda k: info["moves"][k])
            log.append("course lines moved over the course below (the fit and the deviations "
                       "below are measured from the moved lines): {} course(s), up to {:.0f} mm "
                       "sideways at course {}{}".format(
                           len(moved), info["moves"][k_worst], k_worst,
                           "; open ends trimmed at course(s) " + " ".join(
                               "{} ({:.0f} mm)".format(k, info["trims"][k]) for k in moved
                               if info["trims"][k] > 0.5)
                           if any(info["trims"][k] > 0.5 for k in moved) else ""))
    except FitError as err:
        result.update({"ok": False, "fatal": str(err), "log": log + [str(err)], "frames": [],
                       "bricks": []})
        result["report"] = report(result)
        return result
    wall = Wall(paths, float(job.get("z_base", 0.0)), p, counts)
    settle_counts(wall, log)
    counts = [len(row) for row in wall.courses]
    joints = [wall.head_joint(k) for k in range(len(paths))]
    log.append("fit: {} courses, {}-{} bricks per course, mean head joints {:.1f}-{:.1f} mm".format(
        len(paths), min(counts), max(counts), min(joints), max(joints)))
    log.append("bricks per course (bottom up): " + " ".join(str(n) for n in counts))

    fatal = None
    l0_ok = l1_ok = False
    cra = {"state": "skipped", "interfaces": []}
    it = 0
    for it in range(1, p["max_iter"] + 1):
        l0_ok, fatal = (l0_repair_qp if p["l0_method"] == "qp" else l0_repair)(wall, log)
        if fatal:
            log.append("stop: " + fatal)
            break
        if not l0_ok:
            # L1 only checks a geometrically valid wall: CRA on bricks that
            # still fail L0 (flagged) rarely converges and runs to the timeout
            n_bad = sum(1 for b in wall.bricks() if wall.check(b))
            log.append("iteration {}: L0 fail ({} brick(s) still failing); CRA skipped".format(
                it, n_bad))
            log.append("stop: L0 did not pass; L1 runs only on a wall that passes L0")
            cra = {"state": "skipped", "interfaces": []}   # drop results of an older geometry
            l1_ok = False
            break
        if not use_cra:
            log.append("iteration {}: L0 pass; CRA skipped".format(it))
            break
        t_cra = time.time()
        cra = run_cra(wall)
        t_cra = time.time() - t_cra
        cra_time += t_cra
        if cra["state"] == "timeout":
            log.append("iteration {}: L0 pass, CRA timed out after {:.0f} s; L1 not checked".format(
                it, p["cra_timeout"]))
            log.append("stop: CRA solve exceeded cra_timeout (raise it, or 0 = no limit)")
            break
        n_fail = sum(1 for r in cra["interfaces"] if r["fails"])
        n_ten = sum(1 for r in cra["interfaces"] if r["tensioned"])
        sol = cra.get("solver") or {}
        log.append("iteration {}: L0 pass, {} {} in {:.1f} s ({} interfaces, {} need tension, {} over f_t/SF){}".format(
            it, "RBE" if p["solver"] == "rbe" else "CRA", cra["state"], t_cra,
            cra.get("n_interfaces", 0), n_ten, n_fail,
            "; IPOPT {} after {} iterations".format(sol["status"], sol["iterations"]) if sol else ""))
        if cra["state"] == "solved" and n_fail == 0:
            l1_ok = True
            log.append("L1 pass: " + ("stable as a dry stack" if n_ten == 0 else
                                      "tension within f_t/SF"))
            break
        moved = l1_repair(wall, cra, log)
        if moved == 0:
            log.append("stop: no brick could be moved further")
            break
    else:
        log.append("stop: max_iter = {} reached".format(p["max_iter"]))

    # bricks still failing when the loop stopped are "failed", whatever they
    # did before (a fatal stop or max_iter can leave them unrepaired)
    l1_fail = set(r["upper"] for r in cra["interfaces"] if r["fails"])
    if wall.dropped:
        counts = [len(row) for row in wall.courses]
        log.append("bricks per course after dropping {} brick(s) (bottom up): {}".format(
            len(wall.dropped), " ".join(str(n) for n in counts)))
    dropped = [{"course": b.k, "index": b.i, "length": b.length, "frame": frame,
                "fitted_frame": wall.frame(b, fitted=True), "reason": reason}
               for b, frame, reason in wall.dropped]
    bricks = []
    for b in wall.bricks():
        l0_fail = sorted(wall.check(b))
        status = b.status
        if status != "flagged" and (l0_fail or b in l1_fail):
            status = "failed"
        bricks.append({
            "course": b.k, "index": b.i, "status": status, "length": b.length,
            "frame": wall.frame(b), "fitted_frame": wall.frame(b, fitted=True),
            "deviation": wall.deviation(b), "rotation": math.degrees(b.r - b.r0),
            "notes": b.notes,
            "l0_fail": l0_fail,
        })
    interfaces = []
    for r in cra["interfaces"]:
        interfaces.append({
            "upper": [r["upper"].k, r["upper"].i], "lower": [r["lower"].k, r["lower"].i],
            "centroid": [c / MM for c in r["centroid"]], "sigma_t": _finite(r["sigma_t"]),
            "P": r["P"], "ratio": _finite(r["ratio"]), "tensioned": r["tensioned"],
            "fails": r["fails"],
        })
    result.update({
        "ok": bool(l0_ok and l1_ok and not fatal), "fatal": fatal, "iterations": it,
        "l0_pass": l0_ok, "l1_state": cra["state"], "l1_pass": l1_ok,
        "bricks_per_course": counts, "head_joints": joints, "log": log, "bricks": bricks,
        "dropped": dropped, "interfaces": interfaces,
        "frames": [b["frame"] for b in bricks],     # export_frames_json / cra_check format
        "use_cra": use_cra, "solver": p["solver"], "cra_time": cra_time,
        "time": time.time() - t_start,
    })
    result["report"] = report(result)
    return result


def _finite(v):
    """v, or None when it is infinite / NaN (JSON has no Infinity: strict
    readers reject it).  Works in IronPython 2.7 (no math.isfinite)."""
    return v if v == v and abs(v) != float("inf") else None


def report(result):
    lines = list(result.get("log", []))
    br = result.get("bricks", [])
    counts = {}
    for b in br:
        counts[b["status"]] = counts.get(b["status"], 0) + 1
    if result.get("dropped"):
        counts["dropped"] = len(result["dropped"])
    lines.append("bricks: " + ", ".join("{} {}".format(v, k) for k, v in sorted(counts.items())))
    if br:
        worst = max(br, key=lambda b: b["deviation"])
        lines.append("max deviation from fit: {:.1f} mm (course {} brick {})".format(
            worst["deviation"], worst["course"], worst["index"]))
        turned = max(br, key=lambda b: abs(b.get("rotation", 0.0)))
        if abs(turned.get("rotation", 0.0)) > 0.05:
            lines.append("max rotation from the course direction: {:.1f} deg (course {} brick {})"
                         .format(turned["rotation"], turned["course"], turned["index"]))
    for b in br:
        if b["status"] in ("flagged", "failed"):
            lines.append("  course {} brick {}: {} {} {}".format(
                b["course"], b["index"], b["status"], ",".join(b["l0_fail"]), "; ".join(b["notes"])))
    for d in result.get("dropped", []):
        lines.append("  course {} brick {}: dropped ({})".format(d["course"], d["index"], d["reason"]))
    bad = [i for i in result.get("interfaces", []) if i["fails"]]
    for i in bad:
        if i["ratio"] is not None:
            lines.append("  interface {}->{}: sigma_t/(f_t/SF) = {:.2f}".format(
                i["lower"], i["upper"], i["ratio"]))
        elif i["sigma_t"] is not None:      # f_t = 0 (dry stack)
            lines.append("  interface {}->{}: sigma_t = {:.1f} kPa with f_t/SF = 0 (dry stack: "
                         "any tension fails)".format(i["lower"], i["upper"], i["sigma_t"] / 1e3))
        else:
            lines.append("  interface {}->{}: tension on a degenerate contact (no stress "
                         "computed)".format(i["lower"], i["upper"]))
    if "time" in result:
        lines.append("solver time: {:.1f} s total, {:.1f} s in L1 solves ({})".format(
            result["time"], result.get("cra_time", 0.0),
            result.get("solver", "cra_penalty") if result.get("use_cra", True) else "off"))
    lines.append("RESULT: " + ("PASS" if result.get("ok") else "FAIL"))
    return "\n".join(lines)
