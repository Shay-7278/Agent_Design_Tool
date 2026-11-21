"""PyVista GUI: edit a NURBS surface by dragging its control points.

Interaction
-----------
- Drag an orange sphere to move that control point. Only the degree+1
  surrounding spans change (locality of NURBS).
- Sliders on the right scale the surface about its original centroid
  (affine map on CPs == exact scale of the surface).
- Keys:
    x / y / z   constrain drags to that world axis
    g           free drag (no constraint)
    u           undo
    b           reset all edits back to the loaded state
    s           save (overwrites the .3dm; writes <name>.3dm.orig first)
    a           re-fit the brick wall to the current surface (runs the solver)
    l           L1 solver for 'a': rbe (start) / cra_penalty
    r           reset camera,  q  quit

Run from the gui/ folder:
    python -m surface_editor [path.3dm] [-r result.json]

The fitted brick wall from the latest analysis (newest *_result.json in
../results) is overlaid automatically; hide/show it with 'v'.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import numpy as np
import pyvista as pv

from .model import NurbsModel
from . import refit, wall_view

AXIS_IDX = {"x": 0, "y": 1, "z": 2}
L1_SOLVERS = ("rbe", "cra_penalty")  # pc/params.py SOLVERS; first = start value
UNDO_CAP = 64
ENGAGE_GAP = 0.6  # seconds of silence before a new drag counts as a new gesture


def _project_root() -> Path:
    """optionA_test/ (the parent of this package's gui/ folder)."""
    return Path(__file__).resolve().parent.parent.parent


class SurfaceEditor:
    def __init__(
        self,
        path: str | Path,
        res: int = 60,
        off_screen: bool = False,
        result_path: str | Path | None = None,
    ) -> None:
        self.res = res
        self.model = NurbsModel.load(path)
        self.cps = self.model.control_points()  # (n, m, 3) source of truth
        self.initial = self.cps.copy()
        self.pivot = self.cps.reshape(-1, 3).mean(axis=0)
        self.n, self.m = self.cps.shape[:2]

        self.undo_stack: list[np.ndarray] = []
        self.axis_lock: str | None = None
        self.soft = 0.0  # 0 = move single CP only
        self.scales = np.array([1.0, 1.0, 1.0])
        self.drag_index: int | None = None
        self.last_event = time.perf_counter()
        self._guard = False  # re-entrancy guard for SetCenter from callbacks
        # Overrides the solver copied from a previous result: those store every
        # resolved param, so a default change would otherwise never reach 'a'.
        self.solver = L1_SOLVERS[0]

        self._build_scene(off_screen)
        self.wall_actors: list = []
        self.wall_visible = True
        self.wall_label = ""
        self._wall_result_path: Path | None = None
        self._refit_busy = False
        self._refit_done = False
        self._refit_error: str | None = None
        self._refit_result_path: Path | None = None
        self._refit_seconds = 0.0
        if result_path is not None:
            self.show_wall(result_path)
        else:
            latest = wall_view.find_latest_result(_project_root())
            if latest is not None:
                self.show_wall(latest)

    # ----------------------------------------------------------------- scene
    def _build_scene(self, off_screen: bool) -> None:
        p = pv.Plotter(off_screen=off_screen, window_size=(1500, 950), title="NURBS Surface Editor")

        # shaded surface (StructuredGrid, points updated in place each frame)
        pts = self.model.evaluate(self.res, self.res)
        grid = pv.StructuredGrid(pts[..., 0], pts[..., 1], pts[..., 2])
        # whatever flatten order StructuredGrid used, keep using the same one
        self.flat_order = "C" if np.allclose(grid.points, pts.reshape(-1, 3)) else "F"
        p.add_mesh(
            grid,
            color="lightsteelblue",
            opacity=0.7,
            smooth_shading=True,
            show_edges=False,
            render_lines_as_tubes=False,
        )
        self.grid = grid

        # control polygon
        poly = pv.PolyData(self.cps.reshape(-1, 3))
        segs: list[int] = []
        for i in range(self.n):
            for j in range(self.m):
                if i + 1 < self.n:
                    segs += [2, self._flat(i, j), self._flat(i + 1, j)]
                if j + 1 < self.m:
                    segs += [2, self._flat(i, j), self._flat(i, j + 1)]
        poly.lines = np.array(segs)
        p.add_mesh(poly, color="dimgray", line_width=2, render_lines_as_tubes=True)
        self.cp_poly = poly

        # draggable CP handles
        diag = np.linalg.norm(
            self.cps.reshape(-1, 3).max(axis=0) - self.cps.reshape(-1, 3).min(axis=0)
        )
        radius = max(diag * 0.012, 1.0)
        p.add_sphere_widget(
            self.on_drag,
            center=self.cps.reshape(-1, 3),
            radius=radius,
            color="darkorange",
            selected_color="magenta",
            test_callback=False,
            interaction_event="always",
            pass_widget=True,
        )

        # scale sliders (right edge, stacked)
        self.sliders = []
        for k, (label, lo, hi, cb) in enumerate(
            [
                ("scale X", 0.5, 1.5, self._mk_scale_cb(0)),
                ("scale Y", 0.5, 1.5, self._mk_scale_cb(1)),
                ("scale Z", 0.5, 1.5, self._mk_scale_cb(2)),
                ("soft pull", 0.0, 1.0, self._soft_cb),
            ]
        ):
            y = 0.92 - 0.09 * k
            w = p.add_slider_widget(
                cb,
                rng=[lo, hi],
                value=1.0 if label.startswith("scale") else 0.0,
                title=label,
                pointa=(0.84, y),
                pointb=(0.99, y),
                interaction_event="always",
                style="modern",
            )
            self.sliders.append(w)

        # keys
        p.add_key_event("x", lambda: self._set_lock("x"))
        p.add_key_event("y", lambda: self._set_lock("y"))
        p.add_key_event("z", lambda: self._set_lock("z"))
        p.add_key_event("g", lambda: self._set_lock(None))
        p.add_key_event("u", self.undo)
        p.add_key_event("b", self.reset)
        p.add_key_event("s", self.save)
        p.add_key_event("v", self.toggle_wall)
        p.add_key_event("a", self.start_refit)
        p.add_key_event("l", self.toggle_solver)

        p.add_text(
            "drag orange spheres = move CP\n"
            "x/y/z: axis lock   g: free\n"
            "u: undo   b: reset edits\n"
            "v: toggle fitted wall\n"
            "a: re-fit bricks to current surface\n"
            "l: L1 solver rbe / cra_penalty\n"
            "s: save .3dm   r: reset view   q: quit",
            position="upper_left",
            font_size=11,
            name="help",
        )
        self.plotter = p
        self._status(f"loaded (L1 solver: {self.solver})")

    # --------------------------------------------------------------- helpers
    def _flat(self, i: int, j: int) -> int:
        return i * self.m + j

    def _status(self, msg: str) -> None:
        self.plotter.add_text(msg, position="lower_left", font_size=11, name="status")

    def _set_lock(self, axis: str | None) -> None:
        self.axis_lock = axis
        self._status(f"drag axis: {axis or 'free'}")

    def _push_undo(self) -> None:
        self.undo_stack.append(self.cps.copy())
        if len(self.undo_stack) > UNDO_CAP:
            self.undo_stack.pop(0)

    def _sync_scene(self) -> None:
        """Push self.cps into model, surface mesh, control polygon and handles."""
        self.model.set_control_points(self.cps)
        pts = self.model.evaluate(self.res, self.res)
        self.grid.points = pts.reshape(-1, 3, order=self.flat_order)
        self.cp_poly.points = self.cps.reshape(-1, 3)
        if not self._guard:
            self._guard = True
            try:
                for w, c in zip(self.plotter.widgets.sphere_widgets, self.cps.reshape(-1, 3)):
                    w.SetCenter(tuple(float(v) for v in c))
            finally:
                self._guard = False

    # -------------------------------------------------------------- dragging
    def on_drag(self, point, index, widget) -> None:
        if self._guard:
            return
        now = time.perf_counter()
        if index != self.drag_index or now - self.last_event > ENGAGE_GAP:
            self._push_undo()  # snapshot once per drag gesture
        self.drag_index = index
        self.last_event = now

        i, j = divmod(int(index), self.m)
        new = np.asarray(point, dtype=float)
        orig = self.cps[i, j].copy()

        if self.axis_lock:
            axis = np.zeros(3)
            axis[AXIS_IDX[self.axis_lock]] = 1.0
            new = orig + axis * float(np.dot(new - orig, axis))

        delta = new - orig
        self.cps[i, j] = new
        if self.soft > 1e-3:
            ii, jj = np.meshgrid(np.arange(self.n), np.arange(self.m), indexing="ij")
            d2 = (ii - i) ** 2 + (jj - j) ** 2
            sigma = 0.4 + self.soft * 0.8  # tight: a 6x6 grid has no far CPs
            w = np.exp(-d2 / (2.0 * sigma**2))
            w[i, j] = 0.0  # center already moved
            self.cps += delta[None, None, :] * w[..., None]

        self._sync_scene()

    # ----------------------------------------------------------------- scale
    def _mk_scale_cb(self, axis: int):
        def cb(value):
            if self._guard:
                return
            v = float(value)
            if abs(v - self.scales[axis]) < 1e-9:
                return
            now = time.perf_counter()
            if now - self.last_event > ENGAGE_GAP:
                self._push_undo()
            self.last_event = now
            ratio = v / self.scales[axis]
            self.scales[axis] = v
            self.cps = (self.cps - self.pivot) * [ratio if k == axis else 1.0 for k in range(3)] + self.pivot
            self._sync_scene()

        return cb

    def _soft_cb(self, value):
        self.soft = float(value)

    # ------------------------------------------------------------- undo/reset
    def undo(self) -> None:
        if not self.undo_stack:
            self._status("nothing to undo")
            return
        self.cps = self.undo_stack.pop()
        self._reset_scales()
        self._sync_scene()
        self._status(f"undo ({len(self.undo_stack)} left)")

    def reset(self) -> None:
        self._push_undo()
        self.cps = self.initial.copy()
        self._reset_scales()
        self._sync_scene()
        self._status("reset to loaded state")

    def _reset_scales(self) -> None:
        self.scales[:] = 1.0
        for k, w in enumerate(self.sliders[:3]):
            rep = w.GetRepresentation()
            self._guard = True
            try:
                rep.SetValue(1.0)
            finally:
                self._guard = False

    # ------------------------------------------------------------------ save
    def save(self) -> None:
        try:
            out = self.model.save()
        except Exception as exc:  # noqa: BLE001
            self._status(f"save FAILED: {exc}")
            return
        self._status(f"saved -> {out}")

    # ------------------------------------------------------------------- wall
    def show_wall(self, result_path: str | Path) -> None:
        """Overlay the fitted brick wall from a physics_check result JSON."""
        for a in self.wall_actors:
            self.plotter.remove_actor(a)
        self.wall_actors = []
        result = wall_view.load_result(result_path)
        self._wall_result_path = Path(result_path)
        result = wall_view.load_result(result_path)
        for status, mesh in wall_view.build_wall_meshes(result):
            a = self.plotter.add_mesh(
                mesh,
                color=wall_view.STATUS_COLORS.get(status, "gray"),
                opacity=0.95,
                name=f"wall_{status}",
                smooth_shading=True,
            )
            self.wall_actors.append(a)
        markers = wall_view.interface_markers(result)
        if markers is not None:
            a = self.plotter.add_mesh(
                markers.glyph(geom=pv.Sphere(radius=15.0), orient=False),
                color=wall_view.MARKER_COLOR,
                name="wall_tension",
            )
            self.wall_actors.append(a)
        self.wall_visible = True
        self.wall_label = wall_view.result_summary(result, result_path)
        self._status(f"wall: {self.wall_label}")

    def toggle_wall(self) -> None:
        if not self.wall_actors:
            self._status("no wall loaded")
            return
        self.wall_visible = not self.wall_visible
        for a in self.wall_actors:
            if a is not None:
                a.SetVisibility(self.wall_visible)
        self.plotter.render()
        self._status(f"wall {'shown' if self.wall_visible else 'hidden'}: {self.wall_label}")

    # ------------------------------------------------------------------ refit
    def start_refit(self) -> None:
        """Re-fit the brick wall to the CURRENT surface (key 'a').

        Sections are extracted synchronously (~0.5 s), the solver runs in a
        background thread (~20-90 s), a timer swaps in the new wall when done.
        """
        if self._refit_busy:
            self._status("fit already running...")
            return
        src = self._wall_result_path or wall_view.find_latest_result(_project_root())
        # No earlier result: an empty dict makes the solver use pc/params.py DEFAULTS.
        params = wall_view.load_result(src)["params"] if src is not None else {}
        params = {**params, "solver": self.solver}
        source = Path(src).stem if src is not None else "solver defaults"
        self._status("extracting sections from current surface...")
        try:
            job = refit.build_job(self.model, params, use_cra=True)
        except Exception as exc:  # noqa: BLE001
            self._refit_error = str(exc)
            self._status(f"refit FAILED: {exc}")
            return
        self._refit_busy = True
        self._refit_done = False
        self._refit_error = None
        self._status(f"fitting bricks to current surface "
                     f"(L1 {self.solver}, params from {source})...")

        def worker():
            try:
                path, secs = refit.run_solver(job, _project_root() / "results", use_cra=True)
                self._refit_result_path = path
                self._refit_seconds = secs
            except Exception as exc:  # noqa: BLE001
                self._refit_error = str(exc)
            finally:
                self._refit_done = True

        threading.Thread(target=worker, daemon=True).start()

    def toggle_solver(self) -> None:
        i = L1_SOLVERS.index(self.solver)
        self.solver = L1_SOLVERS[(i + 1) % len(L1_SOLVERS)]
        self._status(f"L1 solver for next 'a': {self.solver}")

    def _poll_refit(self, _step=None) -> None:
        """Timer callback (GUI thread): swap in the new wall when the solver ends."""
        if not (self._refit_busy and self._refit_done):
            return
        self._refit_busy = False
        self._refit_done = False
        if self._refit_error is not None:
            self._status(f"refit FAILED: {self._refit_error[:120]}")
            return
        self.show_wall(self._refit_result_path)
        result = wall_view.load_result(self._refit_result_path)
        state = "PASS" if result.get("ok") else "FAIL"
        self._status(f"refit done in {self._refit_seconds:.0f} s: "
                     f"{len(result.get('bricks', []))} bricks, {state}")

    # ------------------------------------------------------------------- run
    def show(self) -> None:
        self.plotter.add_axes()
        self.plotter.camera_position = "iso"
        # A timer created before show() never fires on Windows (pyvista 0.49),
        # so start the refit poll timer after the window's first render.
        rw = self.plotter.render_window
        obs = None

        def start_poll_timer(_obj, _event):
            rw.RemoveObserver(obs)
            self.plotter.add_timer_event(max_steps=10**9, duration=500,
                                          callback=self._poll_refit)

        obs = rw.AddObserver("EndEvent", start_poll_timer)
        self.plotter.show()


def default_file() -> Path:
    here = Path(__file__).resolve().parent          # gui/surface_editor
    root = here.parent.parent                       # optionA_test/
    candidate = root / "Reference Surface_1.3dm"
    if candidate.exists():
        return candidate
    for base in (root, here.parent, Path.cwd()):
        matches = [p for p in sorted(base.glob("*.3dm")) if p.suffix == ".3dm"]
        if matches:
            return matches[0]
    raise SystemExit("no .3dm found; pass a path: python -m surface_editor <file.3dm>")


def main(argv: list[str] | None = None) -> None:
    argv = sys.argv[1:] if argv is None else argv
    result_path: str | None = None
    surface_args: list[str] = []
    it = iter(argv)
    for a in it:
        if a in ("-r", "--result"):
            try:
                result_path = next(it)
            except StopIteration:
                raise SystemExit("--result needs a path") from None
        else:
            surface_args.append(a)
    path = Path(surface_args[0]) if surface_args else default_file()
    editor = SurfaceEditor(path, result_path=result_path)
    editor.show()


if __name__ == "__main__":
    main()
