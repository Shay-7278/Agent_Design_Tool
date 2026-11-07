"""Step 7: the outer loop (fit -> L0 -> L1 -> repeat) and the text report."""

from __future__ import division

from .cra import run_cra
from .fit import FitError, fit_sections
from .l0 import l0_repair
from .l1 import l1_repair
from .params import DEFAULTS, MM, resolve_params
from .wall import Wall


def run_check(job, use_cra=True):
    """Run steps 2-7 on a job dict.

    job = {"z_base": float [mm],
           "params": {name: value} (any of params.DEFAULTS; missing/None -> default),
           "sections": [{"points": [[x, y], ...], "closed": bool}, ...] bottom up}

    Returns the result dict written to result.json (its "report" key holds the
    text summary).
    """
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
        paths, factors = fit_sections(sections, p)
    except FitError as err:
        result.update({"ok": False, "fatal": str(err), "log": log + [str(err)], "frames": [],
                       "bricks": []})
        result["report"] = report(result)
        return result
    wall = Wall(paths, float(job.get("z_base", 0.0)), p)
    log.append("fit: {} courses, stretch factors {:.3f}-{:.3f}".format(
        len(paths), min(factors), max(factors)))

    fatal = None
    l0_ok = l1_ok = False
    cra = {"state": "skipped", "interfaces": []}
    it = 0
    for it in range(1, p["max_iter"] + 1):
        l0_ok, fatal = l0_repair(wall, log)
        if fatal:
            log.append("stop: " + fatal)
            break
        if not use_cra:
            log.append("iteration {}: L0 {}; CRA skipped".format(it, "pass" if l0_ok else "fail"))
            break
        cra = run_cra(wall)
        n_fail = sum(1 for r in cra["interfaces"] if r["fails"])
        n_ten = sum(1 for r in cra["interfaces"] if r["tensioned"])
        log.append("iteration {}: L0 {}, CRA {} ({} interfaces, {} need tension, {} over f_t/SF)".format(
            it, "pass" if l0_ok else "fail", cra["state"], cra.get("n_interfaces", 0), n_ten, n_fail))
        if cra["state"] == "solved" and n_fail == 0:
            l1_ok = True
            log.append("L1 pass: " + ("stable as a dry stack" if n_ten == 0 else
                                      "tension within f_t/SF"))
            if l0_ok:
                break
        moved = l1_repair(wall, cra, log) if not l1_ok else 0
        if l1_ok:
            break
        if moved == 0:
            log.append("stop: no brick could be moved further")
            break
    else:
        log.append("stop: max_iter = {} reached".format(p["max_iter"]))

    bricks = []
    for b in wall.bricks():
        bricks.append({
            "course": b.k, "index": b.i, "status": b.status,
            "frame": wall.frame(b), "fitted_frame": wall.frame(b, fitted=True),
            "deviation": wall.deviation(b), "notes": b.notes,
            "l0_fail": sorted(wall.check(b)),
        })
    interfaces = []
    for r in cra["interfaces"]:
        interfaces.append({
            "upper": [r["upper"].k, r["upper"].i], "lower": [r["lower"].k, r["lower"].i],
            "centroid": [c / MM for c in r["centroid"]], "sigma_t": r["sigma_t"],
            "P": r["P"], "ratio": r["ratio"], "tensioned": r["tensioned"], "fails": r["fails"],
        })
    result.update({
        "ok": bool(l0_ok and l1_ok and not fatal), "fatal": fatal, "iterations": it,
        "l0_pass": l0_ok, "l1_state": cra["state"], "l1_pass": l1_ok,
        "stretch_factors": factors, "log": log, "bricks": bricks, "interfaces": interfaces,
        "frames": [b["frame"] for b in bricks],     # export_frames_json / cra_check format
    })
    result["report"] = report(result)
    return result


def report(result):
    lines = list(result.get("log", []))
    br = result.get("bricks", [])
    counts = {}
    for b in br:
        counts[b["status"]] = counts.get(b["status"], 0) + 1
    lines.append("bricks: " + ", ".join("{} {}".format(v, k) for k, v in sorted(counts.items())))
    if br:
        lines.append("max deviation from fit: {:.1f} mm".format(max(b["deviation"] for b in br)))
    for b in br:
        if b["status"] == "flagged" or b["l0_fail"]:
            lines.append("  course {} brick {}: {} {} {}".format(
                b["course"], b["index"], b["status"], ",".join(b["l0_fail"]), "; ".join(b["notes"])))
    bad = [i for i in result.get("interfaces", []) if i["fails"]]
    for i in bad:
        lines.append("  interface {}->{}: sigma_t/(f_t/SF) = {:.2f}".format(i["lower"], i["upper"], i["ratio"]))
    lines.append("RESULT: " + ("PASS" if result.get("ok") else "FAIL"))
    return "\n".join(lines)
