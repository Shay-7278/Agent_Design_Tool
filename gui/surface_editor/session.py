"""Document revisions, tool dispatch and analysis provenance (no Qt dependency)."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import uuid

import numpy as np
import rhino3dm

from .model import NurbsModel
from .operations import apply_operations, local_frame
from . import refit, wall_view


def surface_hash(model):
    s = model._srf
    data = {"points": model.control_points().tolist(), "weights": model.weights().tolist(),
            "degree": model.degree, "knots_u": list(s.KnotsU), "knots_v": list(s.KnotsV),
            "domains": [[s.Domain(k).T0, s.Domain(k).T1] for k in (0, 1)]}
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()


def clone_model(model):
    # This detached geometry is safe for a worker to evaluate while the live
    # document changes. No background thread touches the live model or VTK.
    surface = rhino3dm.CommonObject.Decode(model._srf.Encode())
    return NurbsModel(model.path, rhino3dm.File3dm(), None, rhino3dm.ObjectAttributes(), surface)


class DesignSession:
    def __init__(self, editor, results_dir):
        self.editor = editor
        self.results_dir = Path(results_dir)
        self.revision = 0
        self._points = editor.cps.copy()
        self.origin, self.frame = local_frame(editor.cps)
        self.result = wall_view.load_result(editor._wall_result_path) if editor._wall_result_path else None
        self.running = None
        self.last_error = None
        self.changed = lambda: None
        self.start_analysis = None
        editor.on_surface_changed = self.observe_change

    def observe_change(self):
        if not np.array_equal(self._points, self.editor.cps):
            self.revision += 1
            self._points = self.editor.cps.copy()
            self.changed()

    def analysis_status(self):
        if self.result is None:
            return "not analyzed"
        meta = self.result.get("design_context")
        if not meta:
            return "unknown surface version"
        if meta.get("surface_hash") != surface_hash(self.editor.model) or meta.get("solver") != self.editor.solver:
            return "outdated"
        return "current"

    def state(self):
        e = self.editor
        return {"revision": self.revision, "units": "mm", "grid_shape": list(e.model.shape),
                "degree": list(e.model.degree), "local_origin": self.origin.tolist(),
                "local_axes_world": {name: self.frame[:, k].tolist() for k, name in enumerate(("width", "depth", "height"))},
                "world_bounds": [e.cps.min(axis=(0, 1)).tolist(), e.cps.max(axis=(0, 1)).tolist()],
                "control_points_world": e.cps.tolist(), "analysis_status": self.analysis_status(),
                "running_analysis": self.running, "undo_available": bool(e.undo_stack),
                "limits": "Fixed control grid; edits deform the control cage, not exact target surfaces. UV follows grid indices."}

    def check_revision(self, expected):
        if type(expected) is not int or expected != self.revision:
            raise ValueError(f"Surface changed: expected revision {expected}, current {self.revision}. Inspect again.")

    def execute(self, name, args):
        fields = {"get_surface_state": set(), "edit_surface": {"expected_revision", "operations"},
                  "undo_edit": {"expected_revision"}, "analyze_wall": {"mode"}, "get_analysis_summary": set()}
        if name not in fields or not isinstance(args, dict) or set(args) != fields[name]:
            raise ValueError("Unknown tool or invalid arguments")
        if name == "get_surface_state":
            return self.state()
        if name == "edit_surface":
            self.check_revision(args["expected_revision"])
            candidate = apply_operations(self.editor.cps, args["operations"], self.origin, self.frame)
            check = clone_model(self.editor.model)
            check.set_control_points(candidate)
            if not check._srf.IsValid or not np.isfinite(check.evaluate(15, 15)).all():
                raise ValueError("Edit produced invalid surface geometry")
            self.editor.apply_control_points(candidate)
            return {"revision": self.revision, "applied": True, "analysis_status": self.analysis_status(),
                    "note": "Control-cage deformation applied; structural performance has not been checked."}
        if name == "undo_edit":
            self.check_revision(args["expected_revision"])
            if not self.editor.undo_stack:
                raise ValueError("Nothing to undo")
            self.editor.undo()
            return {"revision": self.revision, "undone": True}
        if name == "analyze_wall":
            if args["mode"] not in ("fast", "full"):
                raise ValueError("mode must be fast or full")
            if self.start_analysis is None:
                raise ValueError("Analysis runner is unavailable")
            return self.start_analysis(args["mode"])
        return self.summary()

    def prepare_analysis(self, mode):
        if self.running:
            raise ValueError("An analysis is already running")
        if mode not in ("fast", "full"):
            raise ValueError("mode must be fast or full")
        snapshot = clone_model(self.editor.model)
        params = dict(self.result.get("params", {})) if self.result else {}
        params["solver"] = self.editor.solver
        meta = {"run_id": uuid.uuid4().hex, "revision": self.revision,
                "surface_hash": surface_hash(snapshot), "solver": self.editor.solver, "mode": mode}
        self.running = meta.copy()
        self.last_error = None
        self.changed()
        return snapshot, params, meta

    def finish_analysis(self, path=None, error=None):
        self.running = None
        self.last_error = error
        if path is not None:
            self.result = wall_view.load_result(path)
            self.editor.show_wall(path)
        self.changed()

    def summary(self):
        out = {"freshness": self.analysis_status(), "running": self.running, "last_error": self.last_error}
        if not self.result:
            return out
        r = self.result
        bricks = r.get("bricks", [])
        interfaces = r.get("interfaces", [])
        stages = r.get("stages")
        out.update({"context": r.get("design_context"), "solver": r.get("solver"),
                    "parameters": r.get("params", {}),
                    "geometry_check": r.get("l0_pass"),
                    "equilibrium_state": r.get("l1_state", "not checked"),
                    "equilibrium_pass": r.get("l1_pass") if r.get("use_cra") else None,
                    "overall_solver_ok": r.get("ok"), "fatal": r.get("fatal"),
                    "brick_count": len(bricks), "brick_status_counts": dict(Counter(b.get("status", "unknown") for b in bricks)),
                    "dropped_count": len(r.get("dropped", [])),
                    "largest_deviations": [{k: b.get(k) for k in ("course", "index", "deviation", "rotation", "l0_fail", "notes")}
                                           for b in sorted(bricks, key=lambda b: b.get("deviation", 0), reverse=True)[:10]],
                    "tensioned_interface_count": sum(bool(i.get("tensioned")) for i in interfaces),
                    "failed_interface_count": sum(bool(i.get("fails")) for i in interfaces),
                    "problem_interfaces": [i for i in interfaces if i.get("fails") or i.get("tensioned")][:20],
                    "construction": stages if stages is not None else {"state": "not checked"},
                    "log_tail": r.get("log", [])[-12:],
                    "interpretation": "overall_solver_ok does not include construction stages. Fast mode does not check equilibrium. Results apply only to the captured geometry and assumptions."})
        return out


def run_analysis(snapshot, params, meta, results_dir):
    """Worker-only; unique files plus geometry provenance accompany every result."""
    use_cra = meta["mode"] == "full"
    job = refit.build_job(snapshot, params, use_cra=use_cra)
    job["design_context"] = meta
    path, seconds = refit.run_solver(job, Path(results_dir) / ("ai_" + meta["run_id"]), use_cra=use_cra)
    result = wall_view.load_result(path)
    result["design_context"] = meta
    path.write_text(json.dumps(result, allow_nan=False), encoding="utf-8")
    return path, seconds
